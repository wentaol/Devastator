import cv2
import os
import time

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=59,
    flip_method=2,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def show_camera(outdir = "data"):
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    pipe = gstreamer_pipeline(framerate=120, flip_method=2)
    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        i = 0
        while True:
            ret_val, img = cap.read()
            cv2.imwrite(os.path.join(outdir, f"cap_{i}.jpg"), img)
            i += 1
            time.sleep(0.25)
        cap.release()
    else:
        print("Unable to open camera")


if __name__ == "__main__":
    show_camera()
