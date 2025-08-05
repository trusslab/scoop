import av
from PIL import Image
import sys
import numpy as np
import cv2 as cv

DISPLAY_WINDOW_WIDTH = 1280
DISPLAY_WINDOW_HEIGHT = 720

def convert_from_cv2_to_image(img: np.ndarray) -> Image:
    # return Image.fromarray(img)
    return Image.fromarray(cv.cvtColor(img, cv.COLOR_BGR2RGB))


def convert_from_image_to_cv2(img: Image) -> np.ndarray:
    # return np.asarray(img)
    return cv.cvtColor(np.array(img), cv.COLOR_RGB2BGR)

def main():
    # Argument check
    if (len(sys.argv) != 3):
        print("Usage: python3 analyzer.py [RGB_VIDEO_FILE] [DEPTH_VIDEO_FILE]");
        return
    
    # print("Number of cuda-enabled devices: ", cv.cuda.getCudaEnabledDeviceCount())

    # Open RGB video file
    containerRGB = av.open(sys.argv[1])
    streamRGB = containerRGB.streams.video[0]
    fpsRGB = streamRGB.average_rate
    frameInterval = int(1000 / fpsRGB)
    
    # Open depth video file
    depthFile = open(sys.argv[2], 'rb')
    if not depthFile:
        print("Error: Cannot open depth video file")
        return
    
    # Determine platform ID, resolution, and fps
    platformId = int.from_bytes(depthFile.read(1), byteorder='little')
    depthWidth = int.from_bytes(depthFile.read(8), byteorder='little')
    depthHeight = int.from_bytes(depthFile.read(8), byteorder='little')
    depthFPS = int.from_bytes(depthFile.read(8), byteorder='little')
    print("Platform ID: ", platformId, " Resolution: ", depthWidth, "x", depthHeight, " FPS:", depthFPS)

    # Display RGB video
    for packet in containerRGB.demux(streamRGB):
        for frame in packet.decode():
            frame = frame.to_image()
            frameResized = convert_from_image_to_cv2(frame.resize((DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT), Image.BICUBIC))
            cv.imshow('RGB', frameResized)
            if cv.waitKey(frameInterval) & 0xFF == ord('q'):
                break
            
    while True:
        frameSmall = cv.resize(frame, (DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT))
    
    # Release resources
    containerRGB.close()
    depthFile.close()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
