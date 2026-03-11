#!/usr/bin/env python3
"""
Hand-eye calibration verification node.

Verification invariant:
- eye_in_hand: base_link -> marker_frame should be stable (marker fixed in base).
- eye_to_hand: end_effector_link -> marker_frame should be stable (marker fixed on gripper).

Optional estimation:
- eye_to_hand: estimates `T_base<-camera` (base_link -> camera_*) from streaming TF samples
  (robot kinematics + ArUco PnP TF) and prints the 4x4 matrix periodically.

Frame / transform conventions used in this node:
- TF lookup in ROS returns a transform that maps points from `source` into `target`:
    p_target = T_target<-source * p_source
- We write `A <- B` to mean "transform from frame B into frame A".
- OpenCV `cv2.calibrateHandEye` interface (as used here):
    Inputs:  (robot) gripper->base, (vision) target->camera
    Output:  camera->gripper  (eye_in_hand canonical form)
  For eye_to_hand, we invert the robot motion (base->gripper) so that OpenCV's output
  corresponds to camera->base, i.e. `T_base<-camera`.
"""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener


def _quat_angle_deg(q_xyzw: np.ndarray) -> float:
    # For a unit quaternion, relative rotation angle is 2*acos(|w|).
    w = float(q_xyzw[3])
    w = max(-1.0, min(1.0, abs(w)))
    return math.degrees(2.0 * math.acos(w))


def _mean_quaternion(quats_xyzw: np.ndarray) -> np.ndarray:
    # Average quaternions with hemisphere alignment, then normalize.
    if quats_xyzw.shape[0] == 0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    ref = quats_xyzw[0]
    aligned = quats_xyzw.copy()
    for i in range(aligned.shape[0]):
        if float(np.dot(aligned[i], ref)) < 0.0:
            aligned[i] = -aligned[i]

    q = np.sum(aligned, axis=0)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__('calibration_verifier')

        self.declare_parameter('calibration_type', 'eye_in_hand')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'gripper_center')
        # ArUco/PnP publishes marker pose in the camera optical frame (REP-103 optical convention).
        # We estimate `base_link <- camera_optical_frame`, then optionally convert to `camera_link`.
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter('marker_frame', 'aruco_marker_frame')
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('window_size', 100)
        self.declare_parameter('min_samples', 30)
        self.declare_parameter('report_period_sec', 2.0)
        # Extra feature:
        # - eye_to_hand: estimate and print `base_link <- camera_link` matrix from streaming TF samples.
        # - eye_in_hand: this is ignored (verification invariant is different).
        self.declare_parameter('estimate_hand_eye', True)
        self.declare_parameter('estimate_method', 'daniilidis')
        self.declare_parameter('estimate_output_file', '')

        self.calibration_type = str(self.get_parameter('calibration_type').value).strip()
        self.base_link = str(self.get_parameter('base_link').value).strip()
        self.end_effector_link = str(self.get_parameter('end_effector_link').value).strip()
        self.camera_optical_frame = str(self.get_parameter('camera_optical_frame').value).strip()
        self.camera_link_frame = str(self.get_parameter('camera_link_frame').value).strip()
        self.marker_frame = str(self.get_parameter('marker_frame').value).strip()
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.report_period_sec = float(self.get_parameter('report_period_sec').value)
        self.estimate_hand_eye = bool(self.get_parameter('estimate_hand_eye').value)
        self.estimate_method = str(self.get_parameter('estimate_method').value).strip().lower()
        self.estimate_output_file = str(self.get_parameter('estimate_output_file').value).strip()

        self.invariant_frame = self.base_link
        if self.calibration_type == 'eye_to_hand':
            self.invariant_frame = self.end_effector_link

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.samples_t = deque(maxlen=max(1, self.window_size))
        self.samples_q = deque(maxlen=max(1, self.window_size))

        # Estimation samples for OpenCV (absolute poses).
        # We collect the same raw observations as `hand_eye_calibration.py`:
        # - Robot: gripper -> base
        # - Vision: target(marker) -> camera(optical)
        self._samples_R_gripper2base = deque(maxlen=max(1, self.window_size))
        self._samples_t_gripper2base = deque(maxlen=max(1, self.window_size))
        self._samples_R_target2cam = deque(maxlen=max(1, self.window_size))
        self._samples_t_target2cam = deque(maxlen=max(1, self.window_size))

        self._last_report_time = self.get_clock().now()
        self._last_estimate_save_time = self.get_clock().now()

        self.get_logger().info('=' * 60)
        self.get_logger().info('Hand-eye calibration verification started')
        self.get_logger().info(f'calibration_type: {self.calibration_type}')
        self.get_logger().info(f'invariant_frame: {self.invariant_frame}')
        self.get_logger().info(f'marker_frame: {self.marker_frame}')
        self.get_logger().info(f'camera_optical_frame: {self.camera_optical_frame}')
        self.get_logger().info(f'camera_link_frame: {self.camera_link_frame}')
        if self.calibration_type == 'eye_in_hand':
            self.get_logger().info(f'suggested check: tf2_echo {self.base_link} {self.marker_frame}')
        else:
            self.get_logger().info(f'suggested check: tf2_echo {self.end_effector_link} {self.marker_frame}')
        if self.calibration_type == 'eye_to_hand' and self.estimate_hand_eye:
            self.get_logger().info(
                f'estimation enabled: prints estimated {self.base_link} <- {self.camera_link_frame} matrix'
            )
        self.get_logger().info('=' * 60)

        period = 0.1
        if self.sample_rate_hz > 0.0:
            period = 1.0 / self.sample_rate_hz
        self.create_timer(period, self._tick)

    @staticmethod
    def _tf_to_rt(tf_msg) -> tuple[np.ndarray, np.ndarray]:
        # Convert a geometry_msgs TransformStamped into (R, t).
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        tv = np.array([t.x, t.y, t.z], dtype=float)
        return Rm, tv

    @staticmethod
    def _rt_to_matrix(Rm: np.ndarray, tv: np.ndarray) -> np.ndarray:
        # Build a homogeneous 4x4 matrix from (R, t).
        T = np.eye(4, dtype=float)
        T[:3, :3] = Rm
        T[:3, 3] = tv
        return T

    @staticmethod
    def _invert_rt(Rm: np.ndarray, tv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # Invert a rigid transform represented as (R, t).
        R_inv = Rm.T
        t_inv = -R_inv @ tv.reshape(3, 1)
        return R_inv, t_inv.flatten()

    @staticmethod
    def _calib_method(method_name: str) -> int:
        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        }
        return method_map.get(method_name, cv2.CALIB_HAND_EYE_DANIILIDIS)

    def _maybe_collect_estimation_sample(self):
        if self.calibration_type != 'eye_to_hand' or not self.estimate_hand_eye:
            return

        try:
            # Robot: gripper -> base (TF: base_link <- gripper_center)
            tf_g2b = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector_link,
                rclpy.time.Time(),
            )

            # Vision: target(marker) -> camera(optical) from ArUco TF
            # (TF: camera_optical_frame <- marker_frame)
            tf_t2c = self.tf_buffer.lookup_transform(
                self.camera_optical_frame,
                self.marker_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return

        R_g2b, t_g2b = self._tf_to_rt(tf_g2b)
        R_t2c, t_t2c = self._tf_to_rt(tf_t2c)

        self._samples_R_gripper2base.append(R_g2b)
        self._samples_t_gripper2base.append(t_g2b.reshape(3, 1))
        self._samples_R_target2cam.append(R_t2c)
        self._samples_t_target2cam.append(t_t2c.reshape(3, 1))

    def _estimate_base_from_camera(self) -> tuple[np.ndarray, str] | tuple[None, str]:
        if len(self._samples_R_gripper2base) < self.min_samples:
            return None, f'need >= {self.min_samples} samples'

        # Prepare OpenCV inputs (same mapping as `hand_eye_calibration.py`)
        R_robot = []
        t_robot = []
        for R_g2b, t_g2b in zip(self._samples_R_gripper2base, self._samples_t_gripper2base):
            # eye_to_hand: invert gripper->base into base->gripper
            R_b2g, t_b2g = self._invert_rt(R_g2b, t_g2b.flatten())
            R_robot.append(R_b2g)
            t_robot.append(t_b2g.reshape(3, 1))

        R_target2cam = list(self._samples_R_target2cam)
        t_target2cam = list(self._samples_t_target2cam)

        method = self._calib_method(self.estimate_method)
        try:
            # With the eye_to_hand mapping above, OpenCV output corresponds to camera->base.
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_robot, t_robot, R_target2cam, t_target2cam, method=method
            )
        except Exception as e:
            return None, f'calibrateHandEye failed: {e}'

        # Homogeneous matrix for `base_link <- camera_optical_frame`.
        T_base_from_cam_optical = self._rt_to_matrix(R_cam2base, t_cam2base.flatten())
        return T_base_from_cam_optical, 'ok'

    def _optical_to_link(self, T_base_from_cam_optical: np.ndarray) -> tuple[np.ndarray | None, str]:
        if self.camera_link_frame == self.camera_optical_frame:
            return T_base_from_cam_optical, 'ok'

        try:
            # TF internal camera extrinsics.
            # `camera_link <- camera_optical_frame` maps optical-frame points into camera_link.
            tf_link_opt = self.tf_buffer.lookup_transform(
                self.camera_link_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
            )
        except Exception as e:
            return None, f'missing TF {self.camera_link_frame} -> {self.camera_optical_frame}: {e}'

        R_link_opt, t_link_opt = self._tf_to_rt(tf_link_opt)
        T_link_from_opt = self._rt_to_matrix(R_link_opt, t_link_opt)
        T_opt_from_link = np.linalg.inv(T_link_from_opt)

        # We need optical<-link, so invert link<-optical.
        # base<-link = base<-optical @ optical<-link
        return T_base_from_cam_optical @ T_opt_from_link, 'ok'

    def _format_matrix(self, T: np.ndarray) -> str:
        return np.array2string(T, formatter={'float_kind': lambda x: f'{x: .6f}'})

    def _compare_to_current_tf(self, T_base_from_cam_link: np.ndarray) -> str:
        # Compare the estimated matrix against the currently published TF (if any),
        # e.g. a static_transform_publisher created from YAML.
        try:
            tf_ref = self.tf_buffer.lookup_transform(
                self.base_link,
                self.camera_link_frame,
                rclpy.time.Time(),
            )
        except Exception as e:
            return f'no reference TF ({self.base_link} <- {self.camera_link_frame}): {e}'

        R_ref, t_ref = self._tf_to_rt(tf_ref)
        T_ref = self._rt_to_matrix(R_ref, t_ref)

        dT = np.linalg.inv(T_ref) @ T_base_from_cam_link
        dt = float(np.linalg.norm(dT[:3, 3]))
        dang = float(R.from_matrix(dT[:3, :3]).magnitude()) * 180.0 / math.pi
        return f'delta vs TF: trans={dt:.4f}m rot={dang:.2f}deg'

    def _maybe_save_estimate(self, T_base_from_cam_link: np.ndarray):
        if not self.estimate_output_file:
            return

        now = self.get_clock().now()
        # Avoid writing too frequently if running at high rate.
        if (now - self._last_estimate_save_time).nanoseconds < int(2e9):
            return
        self._last_estimate_save_time = now

        try:
            rot = R.from_matrix(T_base_from_cam_link[:3, :3]).as_quat()
            t = T_base_from_cam_link[:3, 3]
            payload = {
                'hand_eye_calibration': {
                    'type': 'eye_to_hand',
                    'frame_id': self.base_link,
                    'child_frame_id': self.camera_link_frame,
                    'transform_description': 'estimated from verifier',
                    'transform': {
                        'translation': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
                        'rotation': {'quaternion': {'x': float(rot[0]), 'y': float(rot[1]), 'z': float(rot[2]), 'w': float(rot[3])}},
                        'matrix_4x4': T_base_from_cam_link.tolist(),
                    },
                }
            }
            import yaml
            with open(self.estimate_output_file, 'w') as f:
                yaml.dump(payload, f, default_flow_style=False, allow_unicode=True)
        except Exception as e:
            self.get_logger().warn(f'failed to save estimate_output_file: {e}')

    def _tick(self):
        self._maybe_collect_estimation_sample()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.marker_frame,
                self.invariant_frame,
                rclpy.time.Time(),
            )
        except Exception:
            # Throttle: don't spam logs if TF isn't ready.
            now = self.get_clock().now()
            if (now - self._last_report_time).nanoseconds > int(2e9):
                self.get_logger().warn(
                    f'Waiting for TF: {self.invariant_frame} -> {self.marker_frame}'
                )
                self._last_report_time = now
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        t_vec = np.array([t.x, t.y, t.z], dtype=float)
        q_xyzw = np.array([q.x, q.y, q.z, q.w], dtype=float)

        # Normalize quaternion to avoid drift from upstream publishers.
        q_norm = float(np.linalg.norm(q_xyzw))
        if q_norm > 1e-12:
            q_xyzw = q_xyzw / q_norm

        self.samples_t.append(t_vec)
        self.samples_q.append(q_xyzw)

        if len(self.samples_t) < self.min_samples:
            return

        now = self.get_clock().now()
        if (now - self._last_report_time).nanoseconds < int(self.report_period_sec * 1e9):
            return
        self._last_report_time = now

        t_arr = np.stack(list(self.samples_t), axis=0)
        q_arr = np.stack(list(self.samples_q), axis=0)

        t_mean = np.mean(t_arr, axis=0)
        q_mean = _mean_quaternion(q_arr)

        trans_err = np.linalg.norm(t_arr - t_mean[None, :], axis=1)

        # Rotation error: angle between mean quaternion and each sample.
        q_mean_inv = R.from_quat(q_mean).inv()
        rot_err_deg = []
        for q_s in q_arr:
            q_rel = (q_mean_inv * R.from_quat(q_s)).as_quat()
            rot_err_deg.append(_quat_angle_deg(q_rel))
        rot_err_deg = np.array(rot_err_deg, dtype=float)

        self.get_logger().info(
            f'[verify] n={len(self.samples_t)} '
            f'trans_err(m): mean={float(np.mean(trans_err)):.4f} std={float(np.std(trans_err)):.4f} max={float(np.max(trans_err)):.4f} | '
            f'rot_err(deg): mean={float(np.mean(rot_err_deg)):.2f} std={float(np.std(rot_err_deg)):.2f} max={float(np.max(rot_err_deg)):.2f}'
        )

        if self.calibration_type == 'eye_to_hand' and self.estimate_hand_eye:
            T_base_from_cam_optical, status = self._estimate_base_from_camera()
            if T_base_from_cam_optical is None:
                self.get_logger().info(f'[estimate] skipped: {status}')
                return

            T_base_from_cam_link, status2 = self._optical_to_link(T_base_from_cam_optical)
            if T_base_from_cam_link is None:
                self.get_logger().info(f'[estimate] optical ok, link convert failed: {status2}')
                self.get_logger().info(
                    f'[estimate] {self.base_link} <- {self.camera_optical_frame}:\n{self._format_matrix(T_base_from_cam_optical)}'
                )
                return

            self.get_logger().info(
                f'[estimate] {self.base_link} <- {self.camera_link_frame} ({self.estimate_method}): '
                f'{self._compare_to_current_tf(T_base_from_cam_link)}\n{self._format_matrix(T_base_from_cam_link)}'
            )
            self._maybe_save_estimate(T_base_from_cam_link)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
