#!/usr/bin/env python3

import cv2
import numpy as np
import os
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

def compensate_cabinet(image_golden_path: str, image_now: np.ndarray) -> Tuple[float, float]:
    """
    Compare golden sample with current image and return compensation values.
    
    Args:
        image_golden_path: Path to golden image file
        image_now: Current image as numpy array
        
    Returns:
        Tuple of (x, z) compensation values
    """
    # Read golden image
    color_golden = cv2.imread(image_golden_path, cv2.IMREAD_COLOR)
    if color_golden is None:
        raise ValueError(f"Could not read golden image: {image_golden_path}")
    
    if image_now is None or image_now.size == 0:
        raise ValueError("Current image is empty")
    
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
    
    avg_x_current = sum(p[0] for p in pts_current) / 4.0
    avg_y_current = sum(p[1] for p in pts_current) / 4.0
    
    # Apply calibration coefficients
    x_out = (avg_x_golden - avg_x_current) * 0.53
    z_out = (avg_y_golden - avg_y_current) * 0.18
    
    return x_out, z_out

def compensate_cabinet_test(image_golden_path: str, image_test_path: str) -> Tuple[float, float]:
    """
    Compare golden sample with test image file and return compensation values.
    
    Args:
        image_golden_path: Path to golden image file
        image_test_path: Path to test image file
        
    Returns:
        Tuple of (x, z) compensation values
    """
    # Read test image
    color_test = cv2.imread(image_test_path, cv2.IMREAD_COLOR)
    if color_test is None:
        raise ValueError(f"Could not read test image: {image_test_path}")
    
    return compensate_cabinet(image_golden_path, color_test)