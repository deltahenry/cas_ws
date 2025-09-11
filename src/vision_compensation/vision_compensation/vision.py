#!/usr/bin/env python3

import cv2
import numpy as np
import os
import yaml
from typing import Tuple, Optional, List

# Return codes (match C++ implementation)
class ReturnCodes:
    OK = 0
    ERR_READ_IMAGE = -10
    ERR_CHANNEL_MISSING = -11
    ERR_RIGHT_SIDE_DETECT = -2
    ERR_LEFT_SIDE_DETECT = -3
    ERR_FOUR_POINTS_NOT_FOUND = -4
    ERR_LINE_INTERSECTION = -5

class BlobEdgePipeline:
    """Configuration class for vision pipeline parameters"""
    
    def __init__(self, config_path: Optional[str] = None):
        # Default parameters
        self.MinAreaLR = 10000
        self.LeftRightThreshold = 165
        self.MinAreaTB = 150
        self.TopBottomThreshold = 150
        self.dilateIterationsLR = 0
        self.TrimPercent = 15
        self.BandScale = 0.98
        self.MinSamplesForFit = 10
        
        # Load from config file if provided
        if config_path and os.path.exists(config_path):
            self.load_from_yaml(config_path)
    
    def load_from_yaml(self, config_path: str):
        """Load parameters from YAML configuration file"""
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)

                pipeline_config = config.get('blob_edge_pipeline', {})
                
                self.MinAreaLR = pipeline_config.get('MinAreaLR', self.MinAreaLR)
                self.LeftRightThreshold = pipeline_config.get('LeftRightThreshold', self.LeftRightThreshold)
                self.MinAreaTB = pipeline_config.get('MinAreaTB', self.MinAreaTB)
                self.TopBottomThreshold = pipeline_config.get('TopBottomThreshold', self.TopBottomThreshold)
                self.dilateIterationsLR = pipeline_config.get('dilateIterationsLR', self.dilateIterationsLR)
                self.TrimPercent = pipeline_config.get('TrimPercent', self.TrimPercent)
                self.BandScale = pipeline_config.get('BandScale', self.BandScale)
                self.MinSamplesForFit = pipeline_config.get('MinSamplesForFit', self.MinSamplesForFit)
                
        except Exception as e:
            print(f"Error loading config from {config_path}: {e}")
            print("Using default parameters")

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def clamp_point(point, width, height):
    x, y = point
    return (clamp(x, 0, width - 1), clamp(y, 0, height - 1))

def median_of(values):
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    n = len(sorted_vals)
    if n % 2 == 1:
        return sorted_vals[n // 2]
    else:
        return 0.5 * (sorted_vals[n // 2 - 1] + sorted_vals[n // 2])

def cross_2d(ax, ay, bx, by):
    return ax * by - ay * bx

def line_line_intersection(p1, p2, p3, p4):
    EPS = 1e-9
    
    r = (p2[0] - p1[0], p2[1] - p1[1])
    s = (p4[0] - p3[0], p4[1] - p3[1])
    qp = (p3[0] - p1[0], p3[1] - p1[1])
    
    rxs = cross_2d(r[0], r[1], s[0], s[1])

    qpxs = cross_2d(qp[0], qp[1], s[0], s[1])
    
    if abs(rxs) < EPS:
        return None
        
    t = qpxs / rxs
    inter = (p1[0] + t * r[0], p1[1] + t * r[1])
    return inter

def line_across_image(vx, vy, x0, y0, width, height):
    pts = []
    EPS = 1e-9
    
    if abs(vx) > EPS:
        # x = 0
        t = (0 - x0) / vx
        y = y0 + t * vy
        if 0 <= y <= height - 1:
            pts.append((0, int(round(y))))
        
        # x = width - 1
        t = ((width - 1) - x0) / vx
        y = y0 + t * vy
        if 0 <= y <= height - 1:
            pts.append((width - 1, int(round(y))))
    
    if abs(vy) > EPS:
        # y = 0
        t = (0 - y0) / vy
        x = x0 + t * vx
        if 0 <= x <= width - 1:
            pts.append((int(round(x)), 0))
        
        # y = height - 1
        t = ((height - 1) - y0) / vy
        x = x0 + t * vx
        if 0 <= x <= width - 1:
            pts.append((int(round(x)), height - 1))
    
    if len(pts) < 2:
        p1 = clamp_point((int(round(x0 - 100 * vx)), int(round(y0 - 100 * vy))), width, height)
        p2 = clamp_point((int(round(x0 + 100 * vx)), int(round(y0 + 100 * vy))), width, height)
        return [p1, p2]
    
    # Find the two points with maximum distance
    max_dist = -1
    a, b = pts[0], pts[1]
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dx = pts[i][0] - pts[j][0]
            dy = pts[i][1] - pts[j][1]
            dist_sq = dx * dx + dy * dy
            if dist_sq > max_dist:
                max_dist = dist_sq
                a, b = pts[i], pts[j]
    
    return [a, b]

def order_corners(pts4):
    if len(pts4) != 4:
        raise ValueError("order_corners: need exactly 4 points")
    
    # Sort by y coordinate
    sorted_pts = sorted(pts4, key=lambda p: p[1])
    top_pair = sorted_pts[:2]
    bottom_pair = sorted_pts[2:]
    
    # Sort each pair by x coordinate
    top_pair.sort(key=lambda p: p[0])
    bottom_pair.sort(key=lambda p: p[0])
    
    top_left = top_pair[0]
    top_right = top_pair[1]
    bottom_left = bottom_pair[0]
    bottom_right = bottom_pair[1]
    
    return top_left, top_right, bottom_left, bottom_right

def mask_between_two_lines(img_size, l1a, l1b, r1a, r1b):
    def build_param(a, b):
        vx = float(b[0] - a[0])
    
        vy = float(b[1] - a[1])
        n = np.sqrt(vx * vx + vy * vy)
        if n < 1e-6:
            vx, vy = 1.0, 0.0
        else:
            vx, vy = vx / n, vy / n
        x0, y0 = float(a[0]), float(a[1])
        return vx, vy, x0, y0
    
    height, width = img_size
    
    lvx, lvy, lx0, ly0 = build_param(l1a, l1b)
    rvx, rvy, rx0, ry0 = build_param(r1a, r1b)
    
    l_ends = line_across_image(lvx, lvy, lx0, ly0, width, height)
    r_ends = line_across_image(rvx, rvy, rx0, ry0, width, height)
    
    poly = np.array([l_ends[0], l_ends[1], r_ends[1], r_ends[0]], dtype=np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, poly, 255)
    return mask

def find_blobs(binary):
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    
    blobs = []
    for i in range(1, num_labels):  # Skip background label 0
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        cx = centroids[i, 0]
        cy = centroids[i, 1]
        
        mask = (labels == i).astype(np.uint8) * 255
        
        blob_info = {
            'label': i,
            'area': area,
            'bbox': (x, y, w, h),
            'centroid': (cx, cy),
            'mask': mask
        }
        blobs.append(blob_info)
    
    return blobs

def robust_edge_fit_from_contour(contour, side='RIGHT'):
    if len(contour) < 5:
        return None
    
    x, y, w, h = cv2.boundingRect(contour)
    rect = (x, y, w, h)
    
    band = max(6, int(round(w * 0.12)))
    left_x = x
    right_x = x + w
    
    if side == 'RIGHT':
        threshold_x = right_x - band
        sel = [p[0] for p in contour if p[0][0] >= threshold_x]
        if len(sel) < 5:
            sel = sorted(contour, key=lambda p: p[0][0], reverse=True)
            if len(sel) > 50:
                sel = sel[:50]
    else:  # LEFT
        threshold_x = left_x + band
        sel = [p[0] for p in contour if p[0][0] <= threshold_x]
        if len(sel) < 5:
            sel = sorted(contour, key=lambda p: p[0][0])
            if len(sel) > 50:
                sel = sel[:50]
    
    if len(sel) < 5:
        return None
    
    # Convert to numpy array for fitLine
    points = np.array(sel, dtype=np.float32).reshape(-1, 1, 2)
    line = cv2.fitLine(points, cv2.DIST_FAIR, 0, 0.01, 0.01)
    vx, vy, x0, y0 = line.flatten()
    
    # Calculate distances for robust fitting
    dists = []
    for p in sel:
        dist = abs(vy * (p[0] - x0) - vx * (p[1] - y0))
        dists.append(dist)
    
    med = median_of(dists)
    mad_vals = [abs(d - med) for d in dists]
    mad = median_of(mad_vals) + 1e-6
    k = 2.5
    
    # Filter inliers
    inliers = []
    for i, p in enumerate(sel):
        if abs(dists[i] - med) <= k * mad:
            inliers.append(p)
    
    # Refit with inliers if we have enough
    if len(inliers) >= 5:
        points = np.array(inliers, dtype=np.float32).reshape(-1, 1, 2)
        line = cv2.fitLine(points, cv2.DIST_FAIR, 0, 0.01, 0.01)
        vx, vy, x0, y0 = line.flatten()
    
    return vx, vy, x0, y0, rect

def find_out_battery_frame_corner(img_gray, thr1, thr2, thr3, area_min):
    if img_gray is None or len(img_gray.shape) != 2:
        return ReturnCodes.ERR_READ_IMAGE, None
    
    # Thresholding
    _, bin1 = cv2.threshold(img_gray, thr1, 255, cv2.THRESH_BINARY)
    _, bin2 = cv2.threshold(img_gray, thr2, 255, cv2.THRESH_BINARY)
    _, bin3 = cv2.threshold(img_gray, thr3, 255, cv2.THRESH_BINARY)
    
    # Morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bin1 = cv2.erode(bin1, kernel, iterations=8)
    bin2 = cv2.erode(bin2, kernel, iterations=8)
    
    # Right side from bin1 (largest blob)
    blobs1 = find_blobs(bin1)
    large_blobs1 = [b for b in blobs1 if b['area'] > 10000]
    if not large_blobs1:
        return ReturnCodes.ERR_RIGHT_SIDE_DETECT, None
    
    large_blobs1.sort(key=lambda b: b['area'], reverse=True)
    max_blob1 = large_blobs1[0]
    
    mask1 = max_blob1['mask'].copy()
    mask1 = cv2.dilate(mask1, kernel, iterations=5)
    
    # Find contours
    contours1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours1:
        return ReturnCodes.ERR_RIGHT_SIDE_DETECT, None
    
    # Find largest contour
    best_contour1 = max(contours1, key=cv2.contourArea)
    
    # Robust edge fit for right side
    edge_result = robust_edge_fit_from_contour(best_contour1, 'RIGHT')
    if edge_result is None:
        return ReturnCodes.ERR_RIGHT_SIDE_DETECT, None
    
    vx1, vy1, x01, y01, rect1 = edge_result
    line_right = line_across_image(vx1, vy1, x01, y01, img_gray.shape[1], img_gray.shape[0])
    left_line_1 = (line_right[0][0] + 20, line_right[0][1])
    left_line_2 = (line_right[1][0] + 20, line_right[1][1])
    
    # Left side from bin2 (second-largest blob)
    blobs2 = find_blobs(bin2)
    large_blobs2 = [b for b in blobs2 if b['area'] > 10000]
    if len(large_blobs2) < 2:
        return ReturnCodes.ERR_LEFT_SIDE_DETECT, None
    
    large_blobs2.sort(key=lambda b: b['area'], reverse=True)
    second_blob2 = large_blobs2[1]  # Second largest
    
    mask2 = second_blob2['mask'].copy()
    mask2 = cv2.dilate(mask2, kernel, iterations=5)
    
    contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours2:
        return ReturnCodes.ERR_LEFT_SIDE_DETECT, None
    
    best_contour2 = max(contours2, key=cv2.contourArea)
    
    # Robust edge fit for left side
    edge_result = robust_edge_fit_from_contour(best_contour2, 'LEFT')
    if edge_result is None:
        return ReturnCodes.ERR_LEFT_SIDE_DETECT, None
    
    vx2, vy2, x02, y02, rect2 = edge_result
    line_left = line_across_image(vx2, vy2, x02, y02, img_gray.shape[1], img_gray.shape[0])
    right_line_1 = (line_left[0][0] - 20, line_left[0][1])
    right_line_2 = (line_left[1][0] - 20, line_left[1][1])
    
    # Mask between two lines
    band_mask = mask_between_two_lines(img_gray.shape, left_line_1, left_line_2, right_line_1, right_line_2)
    temp = cv2.bitwise_and(bin3, band_mask)
    
    # Find feature blobs
    blobs3 = find_blobs(temp)
    points = []
    for blob in blobs3:
        if blob['area'] > area_min:
            cx, cy = blob['centroid']
            points.append((int(round(cx)), int(round(cy))))
            if len(points) == 4:
                break
    
    if len(points) < 4:
        return ReturnCodes.ERR_FOUR_POINTS_NOT_FOUND, None
    
    # Order corners
    tl, tr, bl, br = order_corners(points)
    
    # Line intersections
    p_lt = line_line_intersection(left_line_1, left_line_2, tl, tr)
    if p_lt is None:
        return ReturnCodes.ERR_LINE_INTERSECTION, None
    
    p_lb = line_line_intersection(left_line_1, left_line_2, bl, br)
    if p_lb is None:
        return ReturnCodes.ERR_LINE_INTERSECTION, None
    
    p_rt = line_line_intersection(right_line_1, right_line_2, tl, tr)
    if p_rt is None:
        return ReturnCodes.ERR_LINE_INTERSECTION, None
    
    p_rb = line_line_intersection(right_line_1, right_line_2, bl, br)
    if p_rb is None:
        return ReturnCodes.ERR_LINE_INTERSECTION, None
    
    point_corners = [p_lt, p_lb, p_rt, p_rb]
    return ReturnCodes.OK, point_corners

def draw_corners(vis_image, corners_ok, corners_points, color=(255, 0, 255)):
    """Draw corner points on visualization image"""
    if vis_image is None:
        return
    
    labels = ['LT', 'LB', 'RT', 'RB']
    for i, (is_ok, point) in enumerate(zip(corners_ok, corners_points)):
        if is_ok and point is not None:
            x, y = int(point[0]), int(point[1])
            # Draw filled circle
            cv2.circle(vis_image, (x, y), 7, color, -1)
            # Draw label
            cv2.putText(vis_image, labels[i], (x + 6, y - 6), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def draw_avg_dot(vis_image, avg_point, color, tag):
    """Draw average point with label"""
    if vis_image is None or avg_point is None:
        return
    
    x, y = int(round(avg_point[0])), int(round(avg_point[1]))
    # Draw filled circle
    cv2.circle(vis_image, (x, y), 9, color, -1)
    # Draw outer circle
    cv2.circle(vis_image, (x, y), 12, color, 2)
    # Draw label
    cv2.putText(vis_image, tag, (x + 8, y - 8), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

def draw_arrow_and_delta(vis_image, from_avg, to_avg, dx, dy, color_from, color_to, color_text):
    """Draw arrow from current to golden position with delta text"""
    if vis_image is None or from_avg is None or to_avg is None:
        return
    
    from_pt = (int(round(from_avg[0])), int(round(from_avg[1])))
    to_pt = (int(round(to_avg[0])), int(round(to_avg[1])))
    
    # Draw arrow from current to golden (yellow arrow)
    try:
        cv2.arrowedLine(vis_image, from_pt, to_pt, (0, 255, 255), 3, tipLength=0.2)
    except:
        cv2.line(vis_image, from_pt, to_pt, (0, 255, 255), 3)
    
    # Draw delta text at midpoint
    mid_x = (from_pt[0] + to_pt[0]) // 2
    mid_y = (from_pt[1] + to_pt[1]) // 2
    text = f"ΔX={dx:.2f}, ΔY={dy:.2f}"
    cv2.putText(vis_image, text, (mid_x + 10, mid_y - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_text, 2)

def compensate_cabinet_with_visualization(image_golden_path: str, image_now: np.ndarray) -> Tuple[float, float, np.ndarray, np.ndarray]:
    """
    Compare golden sample with current image and return compensation values plus visualization images.
    
    Args:
        image_golden_path: Path to golden image file
        image_now: Current image as numpy array
        
    Returns:
        Tuple of (x, z, vis_golden, vis_current) - compensation values and visualization images
    """
    # Read golden image
    color_golden = cv2.imread(image_golden_path, cv2.IMREAD_COLOR)
    if color_golden is None:
        raise ValueError(f"Could not read golden image: {image_golden_path}")
    
    if image_now is None or image_now.size == 0:
        raise ValueError("Current image is empty")
    
    # Create visualization images (BGR color)
    vis_golden = color_golden.copy()
    if len(image_now.shape) == 3:
        vis_current = image_now.copy()
    else:
        vis_current = cv2.cvtColor(image_now, cv2.COLOR_GRAY2BGR)
    
    # Split channels
    ch_g = cv2.split(color_golden)
    ch_c = cv2.split(image_now)
    
    if len(ch_g) < 3 or len(ch_c) < 3:
        raise ValueError("Images must have at least 3 channels")
    
    # Use red channel (index 2)
    thr1, thr2, thr3 = 150, 110, 150
    area_min = 350
    
    # Find corners in both images
    rc1, pts_golden = find_out_battery_frame_corner(ch_g[2], thr1, thr2, thr3, area_min)
    if rc1 != ReturnCodes.OK:
        raise RuntimeError(f"Failed to find corners in golden image, error code: {rc1}")
    
    rc2, pts_current = find_out_battery_frame_corner(ch_c[2], thr1, thr2, thr3, area_min)
    if rc2 != ReturnCodes.OK:
        raise RuntimeError(f"Failed to find corners in current image, error code: {rc2}")
    
    # Calculate average positions
    avg_x_golden = sum(p[0] for p in pts_golden) / 4.0
    avg_y_golden = sum(p[1] for p in pts_golden) / 4.0
    avg_golden = (avg_x_golden, avg_y_golden)
    
    avg_x_current = sum(p[0] for p in pts_current) / 4.0
    avg_y_current = sum(p[1] for p in pts_current) / 4.0
    avg_current = (avg_x_current, avg_y_current)
    
    # Calculate raw offsets (before calibration)
    x_out = avg_x_golden - avg_x_current
    z_out = avg_y_golden - avg_y_current
    
    # Colors (BGR format)
    col_corner = (255, 0, 255)    # Magenta
    col_avg_golden = (255, 255, 0)  # Cyan 
    col_avg_current = (0, 0, 255)   # Red
    col_text = (255, 255, 255)      # White
    
    # Draw on golden image
    corners_ok_g = [True] * 4  # Assume all corners were found
    draw_corners(vis_golden, corners_ok_g, pts_golden, col_corner)
    draw_avg_dot(vis_golden, avg_golden, col_avg_golden, "G")
    
    # Draw on current image  
    corners_ok_c = [True] * 4  # Assume all corners were found
    draw_corners(vis_current, corners_ok_c, pts_current, col_corner)
    draw_avg_dot(vis_current, avg_current, col_avg_current, "C")
    
    # Draw arrow from current to golden on current image
    draw_arrow_and_delta(vis_current, avg_current, avg_golden, x_out, z_out, 
                        col_avg_current, col_avg_golden, col_text)
    
    return x_out, z_out, vis_golden, vis_current

def compensate_cabinet_test_with_visualization(image_golden_path: str, image_test_path: str) -> Tuple[float, float, np.ndarray, np.ndarray]:
    """
    Compare golden sample with test image file and return compensation values plus visualization images.
    
    Args:
        image_golden_path: Path to golden image file
        image_test_path: Path to test image file
        
    Returns:
        Tuple of (x, z, vis_golden, vis_current) - compensation values and visualization images
    """
    # Read test image
    color_test = cv2.imread(image_test_path, cv2.IMREAD_COLOR)
    if color_test is None:
        raise ValueError(f"Could not read test image: {image_test_path}")
    
    return compensate_cabinet_with_visualization(image_golden_path, color_test)

# Legacy functions for backwards compatibility
def compensate_cabinet(image_golden_path: str, image_now: np.ndarray) -> Tuple[float, float]:
    """
    Compare golden sample with current image and return compensation values.
    
    Args:
        image_golden_path: Path to golden image file
        image_now: Current image as numpy array
        
    Returns:
        Tuple of (x, z) compensation values
    """
    x, z, _, _ = compensate_cabinet_with_visualization(image_golden_path, image_now)
    return x, z

def compensate_cabinet_test(image_golden_path: str, image_test_path: str) -> Tuple[float, float]:
    """
    Compare golden sample with test image file and return compensation values.
    
    Args:
        image_golden_path: Path to golden image file
        image_test_path: Path to test image file
        
    Returns:
        Tuple of (x, z) compensation values
    """
    x, z, _, _ = compensate_cabinet_test_with_visualization(image_golden_path, image_test_path)
    return x, z