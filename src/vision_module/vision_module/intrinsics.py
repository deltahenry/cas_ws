import pyrealsense2 as rs

# 建立 pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)

# 取得 color stream 的內參
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

print("Width:", intr.width)
print("Height:", intr.height)
print("Fx:", intr.fx)
print("Fy:", intr.fy)
print("Ppx:", intr.ppx)
print("Ppy:", intr.ppy)
print("Distortion model:", intr.model)
print("Distortion coeffs:", intr.coeffs)

pipeline.stop()
