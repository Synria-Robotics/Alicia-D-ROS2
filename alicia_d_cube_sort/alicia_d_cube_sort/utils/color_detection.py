"""
Color Detection Utilities

This module provides functions for HSV-based color detection with
adaptive thresholds and image preprocessing.

Author: Synria Robotics
Date: 2026-01
"""

import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional


# Default color ranges (HSV)
DEFAULT_COLOR_RANGES = {
    'green': {'lower': [35, 80, 80], 'upper': [85, 255, 255]},
    'blue': {'lower': [90, 80, 80], 'upper': [130, 255, 255]},
    'red': {'lower': [0, 80, 80], 'upper': [10, 255, 255]},  # Note: red wraps around
    'red2': {'lower': [170, 80, 80], 'upper': [180, 255, 255]},  # Second red range
}


def preprocess_image(image: np.ndarray, 
                    clahe_clip_limit: float = 2.0,
                    clahe_tile_size: Tuple[int, int] = (8, 8)) -> np.ndarray:
    """
    Preprocess image for color detection.
    
    Applies gamma correction for low light and CLAHE for contrast enhancement.
    
    Args:
        image: BGR image
        clahe_clip_limit: CLAHE clip limit
        clahe_tile_size: CLAHE tile grid size
        
    Returns:
        Preprocessed HSV image
    """
    # Get brightness level
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    brightness = np.mean(gray)
    
    # Apply gamma correction for low light
    if brightness < 127.5:
        gamma = 1.7 if brightness >= 76.5 else 2.0
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 
                         for i in np.arange(256)]).astype('uint8')
        image = cv2.LUT(image, table)
    
    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # Apply CLAHE to saturation and value channels
    clahe = cv2.createCLAHE(clipLimit=clahe_clip_limit, tileGridSize=clahe_tile_size)
    v = clahe.apply(v)
    s = clahe.apply(s)
    
    return cv2.merge([h, s, v])


def detect_color(hsv_image: np.ndarray,
                 color_range: Dict,
                 min_area: int = 150,
                 max_area: int = 10000,
                 min_solidity: float = 0.8,
                 aspect_ratio_range: Tuple[float, float] = (0.7, 1.4)) -> List[Dict]:
    """
    Detect objects of a specific color in HSV image.
    
    Args:
        hsv_image: HSV image
        color_range: Dictionary with 'lower' and 'upper' HSV bounds
        min_area: Minimum contour area
        max_area: Maximum contour area
        min_solidity: Minimum solidity ratio
        aspect_ratio_range: (min, max) aspect ratio
        
    Returns:
        List of detections, each with 'center_px', 'box', 'area'
    """
    lower = np.array(color_range['lower'], dtype=np.uint8)
    upper = np.array(color_range['upper'], dtype=np.uint8)
    
    # Create mask
    mask = cv2.inRange(hsv_image, lower, upper)
    
    # Clean up mask
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)
    
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue
        
        # Check solidity
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        solidity = area / hull_area
        if solidity < min_solidity:
            continue
        
        # Get bounding box
        rect = cv2.minAreaRect(hull)
        (cx, cy), (w, h), angle = rect
        
        # Check aspect ratio
        if h > 0 and w > 0:
            aspect = max(w, h) / min(w, h)
            if aspect < aspect_ratio_range[0] or aspect > aspect_ratio_range[1]:
                continue
        
        box = cv2.boxPoints(rect).astype(int)
        
        detections.append({
            'center_px': (float(cx), float(cy)),
            'box': box,
            'area': area,
            'angle': angle
        })
    
    return detections


def detect_all_colors(image: np.ndarray,
                      color_ranges: Optional[Dict] = None,
                      **kwargs) -> Dict[str, List[Dict]]:
    """
    Detect all specified colors in image.
    
    Args:
        image: BGR image
        color_ranges: Dictionary mapping color names to HSV ranges
        **kwargs: Additional arguments passed to detect_color()
        
    Returns:
        Dictionary mapping color names to lists of detections
    """
    if color_ranges is None:
        color_ranges = DEFAULT_COLOR_RANGES
    
    # Preprocess image
    hsv = preprocess_image(image, 
                           kwargs.pop('clahe_clip_limit', 2.0),
                           kwargs.pop('clahe_tile_size', (8, 8)))
    
    results = {}
    
    for color_name, ranges in color_ranges.items():
        if color_name == 'red2':
            continue  # Handle with 'red'
        
        detections = detect_color(hsv, ranges, **kwargs)
        
        # Handle red wrap-around
        if color_name == 'red' and 'red2' in color_ranges:
            detections.extend(detect_color(hsv, color_ranges['red2'], **kwargs))
        
        results[color_name] = detections
    
    return results


def draw_detections(image: np.ndarray,
                    detections: Dict[str, List[Dict]],
                    color_map: Optional[Dict[str, Tuple[int, int, int]]] = None) -> np.ndarray:
    """
    Draw detection boxes on image.
    
    Args:
        image: BGR image
        detections: Dictionary mapping color names to detections
        color_map: Dictionary mapping color names to BGR colors
        
    Returns:
        Image with drawn detections
    """
    if color_map is None:
        color_map = {
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'red': (0, 0, 255),
        }
    
    output = image.copy()
    
    for color_name, dets in detections.items():
        draw_color = color_map.get(color_name, (0, 255, 0))
        
        for det in dets:
            box = det['box']
            cx, cy = det['center_px']
            
            cv2.polylines(output, [box], True, draw_color, 2)
            cv2.circle(output, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(output, color_name, 
                       (box[0][0], box[0][1] - 6),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1)
    
    return output
