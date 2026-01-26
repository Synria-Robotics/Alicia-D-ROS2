"""
Plane Estimation Utilities for A4 Paper Reference Method

This module provides functions for:
- A4 paper detection in images
- Plane estimation from detected rectangles
- Pixel to 3D point conversion using plane intersection

Author: Synria Robotics
Date: 2026-01
"""

import cv2
import numpy as np


def order_points_clockwise(pts):
    """
    Order points in clockwise order starting from top-left: TL, TR, BR, BL
    
    Args:
        pts: Array of 4 points (4x2)
        
    Returns:
        Ordered points array (4x2)
    """
    pts = np.array(pts, dtype=np.float32)
    
    # Calculate center
    center = pts.mean(axis=0)
    
    # Calculate angles from center
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    
    # Sort by angle
    order = np.argsort(angles)
    ordered = pts[order]
    
    # Find top-left (smallest sum of x + y)
    sums = ordered.sum(axis=1)
    tl_idx = np.argmin(sums)
    
    # Rotate to start from top-left
    ordered = np.roll(ordered, -tl_idx, axis=0)
    
    return ordered


def detect_a4_rectangle(image_bgr, min_area=20000):
    """
    Detect A4 paper (white rectangle) in image.
    
    Args:
        image_bgr: BGR image
        min_area: Minimum contour area to consider
        
    Returns:
        4 corners ordered as TL, TR, BR, BL or None if not found
    """
    # Convert to HSV
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    
    # White paper: low saturation, high value
    mask_white = cv2.inRange(hsv, 
                             np.array([0, 0, 180], np.uint8), 
                             np.array([179, 70, 255], np.uint8))
    
    # Also use grayscale threshold
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, mask_gray = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    
    # Combine masks
    mask = cv2.bitwise_and(mask_white, mask_gray)
    
    # Morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Find largest quadrilateral
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        
        if len(approx) == 4:
            corners = approx.reshape(-1, 2).astype(np.float32)
            return order_points_clockwise(corners)
    
    return None


def estimate_plane_from_rectangle(corners_img, rect_w_m, rect_h_m, K, dist):
    """
    Estimate 3D plane from rectangle corners using solvePnP.
    
    Args:
        corners_img: 4 image points (pixels), ordered TL, TR, BR, BL
        rect_w_m: Rectangle width in meters
        rect_h_m: Rectangle height in meters
        K: Camera intrinsic matrix (3x3)
        dist: Distortion coefficients
        
    Returns:
        Tuple (normal, point, corners_3d) or None if failed
        - normal: Plane normal vector in camera frame
        - point: A point on the plane in camera frame
        - corners_3d: 3D positions of corners in camera frame
    """
    # Check corner pixel dimensions to determine orientation
    corners_img = np.array(corners_img, dtype=np.float32)
    w_px = np.linalg.norm(corners_img[1] - corners_img[0])
    h_px = np.linalg.norm(corners_img[2] - corners_img[1])
    
    # Swap dimensions if needed
    if h_px > w_px:
        rect_w_m, rect_h_m = rect_h_m, rect_w_m
    
    # 3D object points on Z=0 plane
    obj_points = np.array([
        [0, 0, 0],
        [rect_w_m, 0, 0],
        [rect_w_m, rect_h_m, 0],
        [0, rect_h_m, 0]
    ], dtype=np.float32)
    
    img_points = corners_img.reshape(-1, 1, 2)
    
    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    
    if not success:
        return None
    
    # Convert rotation vector to matrix
    R_mat, _ = cv2.Rodrigues(rvec)
    
    # Plane normal (Z-axis of plane in camera frame)
    normal = R_mat[:, 2]
    
    # Transform corners to camera frame
    corners_3d = (R_mat @ obj_points.T).T + tvec.flatten()
    
    # Center point on plane
    center_3d = corners_3d.mean(axis=0)
    
    return (normal, center_3d, corners_3d)


def pixel_to_plane_point(u, v, K, normal, plane_point):
    """
    Convert pixel coordinates to 3D point on plane.
    
    Uses ray-plane intersection to find the 3D point.
    
    Args:
        u, v: Pixel coordinates
        K: Camera intrinsic matrix (3x3)
        normal: Plane normal vector
        plane_point: A point on the plane
        
    Returns:
        3D point in camera frame or None if no intersection
    """
    # Camera ray direction
    K_inv = np.linalg.inv(K)
    ray = K_inv @ np.array([u, v, 1.0])
    ray = ray / np.linalg.norm(ray)
    
    # Plane equation: n · (P - P0) = 0
    # Ray: P = t * ray (from camera origin)
    # Solve: n · (t * ray - P0) = 0
    # => t = (n · P0) / (n · ray)
    
    n_dot_ray = np.dot(normal, ray)
    if abs(n_dot_ray) < 1e-6:
        return None  # Ray parallel to plane
    
    t = np.dot(normal, plane_point) / n_dot_ray
    
    if t < 0:
        return None  # Plane behind camera
    
    return t * ray


def pixel_to_plane_point_with_offset(u, v, K, normal, plane_point, height_offset=0.0):
    """
    Convert pixel coordinates to 3D point above/below plane.
    
    Args:
        u, v: Pixel coordinates
        K: Camera intrinsic matrix (3x3)
        normal: Plane normal vector
        plane_point: A point on the plane
        height_offset: Offset along normal (positive = above plane)
        
    Returns:
        3D point in camera frame or None if no intersection
    """
    point = pixel_to_plane_point(u, v, K, normal, plane_point)
    
    if point is None:
        return None
    
    # Add height offset along normal
    return point + height_offset * normal
