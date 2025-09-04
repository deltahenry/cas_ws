import time

class FPSCounter:
    def __init__(self):
        self.frame_count = 0
        self.start_time = time.time()
        
    def update(self):
        self.frame_count += 1
        if self.frame_count % 100 == 0:  # Print every 100 frames
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed
            print(f"FPS: {fps:.1f}")


