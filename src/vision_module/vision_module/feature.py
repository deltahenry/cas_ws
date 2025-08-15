import cv2
import os
import re
import pyrealsense2 as rs
import numpy as np

# ================== 可調參數 ==================
# 相機鎖定（建議 True：避免畫面忽暗忽亮）
LOCK_CAMERA_PARAMS = True
EXPOSURE_US = 1200.0   # 曝光(微秒) → 暗就加大(800~4000)，太大會拖影/亮斑
GAIN       = 32.0      # 增益       → 暗可提高(16~96)，太高會噪點
WHITE_BAL  = 4600.0    # 白平衡(K)  → 室內 4000~5000 常見

# 影像前處理（抗光照）
GAMMA = 1.4            # Gamma 校正；>1 提亮暗部（1.2~1.8 常用）
CLAHE_CLIP = 2.0       # CLAHE 對比限制(1.5~3.0)
CLAHE_TILE = (8, 8)    # CLAHE 區塊大小
BLUR_KSIZE = 5         # 去噪高斯核(奇數 3/5/7)

USE_BLACKHAT = True    # 黑帽增強暗細線（L 型黑線更明顯）
BLACKHAT_K   = 9       # 黑帽核大小(奇數 7/9/11)

# Auto-Canny（依影像中位數計算門檻）
SIGMA = 0.33           # 邊緣量的控制；越大抓到的邊越多(0.25~0.45)

# 邊緣細線加粗（避免小螺絲邊緣斷裂）
USE_DILATE   = True
DILATE_KSIZE = 3       # 3/5；過大會太粗糙
DILATE_ITER  = 1
# ===========================================================

# ===== 固定儲存類型與資料夾 =====
mode = 'feature_shape'
template_dir = os.path.join(os.path.dirname(__file__), f'template_{mode}')
os.makedirs(template_dir, exist_ok=True)

def next_index(dir_path, prefix):
    """回傳 prefix_#.png 的下一個可用編號"""
    pat = re.compile(rf'^{re.escape(prefix)}_(\d+)\.png$')
    max_idx = -1
    for f in os.listdir(dir_path):
        m = pat.match(f)
        if m:
            max_idx = max(max_idx, int(m.group(1)))
    return max_idx + 1

template_count = next_index(template_dir, mode)

# ===== RealSense 初始化（RGB）=====
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)

def lock_rgb_settings():
    """關自動曝光/白平衡並設手動值（RGB Camera 上）"""
    try:
        dev = profile.get_device()
        for s in dev.query_sensors():
            if 'RGB Camera' in s.get_info(rs.camera_info.name):
                if s.supports(rs.option.enable_auto_exposure):
                    s.set_option(rs.option.enable_auto_exposure, 0)
                if s.supports(rs.option.exposure):
                    s.set_option(rs.option.exposure, float(EXPOSURE_US))
                if s.supports(rs.option.gain):
                    s.set_option(rs.option.gain, float(GAIN))
                if s.supports(rs.option.enable_auto_white_balance):
                    s.set_option(rs.option.enable_auto_white_balance, 0)
                if s.supports(rs.option.white_balance):
                    s.set_option(rs.option.white_balance, float(WHITE_BAL))
                print(f"🔒 lock camera: exp={EXPOSURE_US}us, gain={GAIN}, wb={WHITE_BAL}K")
                break
    except Exception as e:
        print(f"⚠️ 無法鎖定相機參數：{e}")

if LOCK_CAMERA_PARAMS:
    lock_rgb_settings()

# ========== 影像前處理（抗光照） ==========
clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_TILE)

def gamma_correct(gray, g):
    """快速 Gamma LUT"""
    g = max(0.4, min(2.5, float(g)))
    table = ((np.arange(256)/255.0) ** (1.0/g) * 255.0).astype(np.uint8)
    return cv2.LUT(gray, table)

def preprocess_to_edges(bgr):
    """
    抗光照 + Auto-Canny：
      BGR → 灰階 → Gamma → CLAHE → GaussianBlur
      → (選) BlackHat 增強暗細線 → Auto-Canny → (選) 膨脹加粗
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = gamma_correct(gray, GAMMA)
    gray = clahe.apply(gray)
    blur = cv2.GaussianBlur(gray, (BLUR_KSIZE, BLUR_KSIZE), 0)

    if USE_BLACKHAT:
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (BLACKHAT_K, BLACKHAT_K))
        bh = cv2.morphologyEx(blur, cv2.MORPH_BLACKHAT, k)
        blur = cv2.add(blur, bh)

    v = np.median(blur)
    lower = int(max(0, (1.0 - SIGMA) * v))
    upper = int(min(255, (1.0 + SIGMA) * v))
    edges = cv2.Canny(blur, lower, upper, L2gradient=True, apertureSize=3)

    if USE_DILATE and DILATE_KSIZE > 0 and DILATE_ITER > 0:
        dk = cv2.getStructuringElement(cv2.MORPH_RECT, (DILATE_KSIZE, DILATE_KSIZE))
        edges = cv2.dilate(edges, dk, iterations=DILATE_ITER)
    return edges

# ===== ROI 滑鼠互動 =====
roi = None
drag = False
ix, iy = -1, -1
def mouse_callback(event, x, y, flags, param):
    global ix, iy, roi, drag
    if event == cv2.EVENT_LBUTTONDOWN:
        ix, iy = x, y; drag = True
    elif event == cv2.EVENT_MOUSEMOVE and drag:
        roi = (ix, iy, x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        roi = (ix, iy, x, y); drag = False

cv2.namedWindow("Canny View", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Canny View", mouse_callback)
print("🟦 在 Canny 視圖拖曳框選 L 型黑線；S 儲存、ESC 離開。")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        bgr = np.asanyarray(color_frame.get_data())

        # —— 核心更動：用抗光照前處理 + Auto-Canny 取代固定 Canny(50,150)
        edges = preprocess_to_edges(bgr)

        display = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if roi:
            x1, y1, x2, y2 = roi
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 255), 2)

        cv2.putText(display, "Drag ROI | S: save | ESC: quit",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        cv2.imshow("Canny View", display)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            break
        elif key == ord('s') and roi:
            h_img, w_img = edges.shape[:2]
            x1, y1, x2, y2 = roi
            # 夾住邊界並排序
            x1, x2 = sorted([max(0, min(w_img-1, x1)), max(0, min(w_img-1, x2))])
            y1, y2 = sorted([max(0, min(h_img-1, y1)), max(0, min(h_img-1, y2))])
            if x2 - x1 < 5 or y2 - y1 < 5:
                print("⚠️ ROI 太小，未儲存。"); continue

            cropped = edges[y1:y2, x1:x2]
            save_path = os.path.join(template_dir, f"{mode}_{template_count}.png")
            if cv2.imwrite(save_path, cropped):
                print(f"✅ 模板儲存成功：{save_path}")
                template_count += 1
                roi = None

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
