import time
import mss
from PIL import Image

def capture_gif(output_path='demo.gif', duration=10, fps=10):
    frames = []
    interval = 1.0 / fps
    print(f"Recording for {duration} seconds at {fps} fps...")
    
    with mss.mss() as sct:
        monitor = sct.monitors[1]  # primary monitor
        
        start_time = time.time()
        while time.time() - start_time < duration:
            loop_start = time.time()
            
            # Capture screen
            sct_img = sct.grab(monitor)
            img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")
            frames.append(img.resize((800, 600)))  # resize to keep gif small
            
            # Sleep to maintain fps
            elapsed = time.time() - loop_start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    print(f"Captured {len(frames)} frames. Saving as GIF...")
    if frames:
        frames[0].save(
            output_path,
            save_all=True,
            append_images=frames[1:],
            optimize=True,
            duration=int(1000/fps),
            loop=0
        )
        print(f"Saved to {output_path}")

if __name__ == '__main__':
    capture_gif('/home/vansh/.gemini/antigravity-ide/brain/329e6daa-9c10-4968-a126-9fea9f6eb764/waferflow_demo.gif', duration=10, fps=5)
