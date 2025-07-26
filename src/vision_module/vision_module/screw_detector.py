import cv2
import numpy as np
import pyrealsense2 as rs

class ScrewDetector:
    def __init__(self, templates, roi_boxes=None, scales=None, match_threshold=0.65):
        self.templates = templates
        self.roi_boxes = roi_boxes if roi_boxes else [
            (362, 266, 397, 399),
            (902, 261, 938, 408)
        ]
        self.scales = scales if scales else [0.8, 0.9, 1.0, 1.1, 1.2]
        self.match_threshold = match_threshold

    def detect(self, color_frame, depth_frame):
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        all_boxes = []

        for roi in self.roi_boxes:
            x1, y1, x2, y2 = roi
            roi_gray = gray[y1:y2, x1:x2]

            for tpl0 in self.templates:
                h0, w0 = tpl0.shape
                for s in self.scales:
                    W, H = int(w0 * s), int(h0 * s)
                    if W < 5 or H < 5:
                        continue
                    tpl = cv2.resize(tpl0, (W, H))
                    result = cv2.matchTemplate(roi_gray, tpl, cv2.TM_CCOEFF_NORMED)
                    ys, xs = np.where(result >= self.match_threshold)

                    for y, x in zip(ys, xs):
                        global_x1 = x + x1
                        global_y1 = y + y1
                        global_x2 = global_x1 + W
                        global_y2 = global_y1 + H
                        all_boxes.append([global_x1, global_y1, global_x2, global_y2, float(result[y, x])])

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        top4 = sorted(all_boxes, key=lambda b: -b[4])[:4]

        results = []
        for i, (x1, y1, x2, y2, score) in enumerate(top4):
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            depth = depth_frame.get_distance(cx, cy)
            if depth == 0 or depth > 1.0:
                continue
            X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], depth)
            results.append({'index': i+1, 'X': X, 'Y': Y, 'Z': Z, 'depth': depth, 'bbox': (x1, y1, x2, y2)})

            # 畫在影像上
            cv2.rectangle(color_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(color_frame, (cx, cy), 4, (0, 0, 255), -1)
            label = f"{depth:.3f}m"
            cv2.putText(color_frame, label, (cx + 5, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return results
