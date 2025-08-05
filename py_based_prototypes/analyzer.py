import numpy as np
import cv2 as cv
import sys
from skimage.metrics import structural_similarity, mean_squared_error
import open3d as o3d

CAMERA_PARAMETER_INT_SIZE = 4   # Size of integer for reading parameters from depth data in bytes

DISPLAY_WINDOW_WIDTH = 320
DISPLAY_WINDOW_HEIGHT = 180

SURFACE_PIXEL_DIFF_THRESHOLD = 3   # In mm

def exploreSurface(depthFrame, depthWidth, depthHeight, x, y, previousPtDistance, currentSurfacePts, visitedPts=[]):
    if (x, y) in visitedPts:
        return
    visitedPts.append((x, y))
    # print("Exploring: ", x, y)
    if x < 0 or x >= depthWidth or y < 0 or y >= depthHeight:
        return
    currentCheckingPixelVal = depthFrame[y * depthWidth + x]
    if currentCheckingPixelVal == 0 or \
        (currentCheckingPixelVal <= previousPtDistance + SURFACE_PIXEL_DIFF_THRESHOLD and \
            currentCheckingPixelVal >= previousPtDistance - SURFACE_PIXEL_DIFF_THRESHOLD):
        currentSurfacePts.append((x, y))
        exploreSurface(depthFrame, depthWidth, depthHeight, x + 1, y, currentCheckingPixelVal, currentSurfacePts, visitedPts)
        exploreSurface(depthFrame, depthWidth, depthHeight, x, y + 1, currentCheckingPixelVal, currentSurfacePts, visitedPts)

def main():
    # Argument check
    if (len(sys.argv) != 4):
        print("Usage: python3 analyzer.py [RGB_VIDEO_FILE] [DEPTH_VIDEO_FILE] [ML_DEPTH_VIDEO_FILE]");
        return
    
    # print("Number of cuda-enabled devices: ", cv.cuda.getCudaEnabledDeviceCount())

    # Open RGB video file
    capRGB = cv.VideoCapture(sys.argv[1])
    fpsRGB = capRGB.get(cv.CAP_PROP_FPS)
    frameInterval = int(1000 / fpsRGB)
    if not capRGB.isOpened():
        print("Error: Cannot open RGB video file")
        return
             
    # Open depth video file
    depthFile = open(sys.argv[2], 'rb')
    if not depthFile:
        print("Error: Cannot open depth video file")
        return
    
    # Open ML depth video file
    capMlDepth = cv.VideoCapture(sys.argv[3])
    if not capMlDepth.isOpened():
        print("Error: Cannot open ML depth video file")
        return
    
    # Determine platform ID, resolution, and fps
    platformByteOrder = "little"
    platformPixelDataType = np.float16
    platformId = int.from_bytes(depthFile.read(1), byteorder=platformByteOrder)
    isReadingDepthCameraIntrinsicsDynamically = True
    # 'big' for android, 'little' for iOS
    if platformId == 0:
        platformByteOrder = "big"
        platformPixelDataType = np.uint16
        isReadingDepthCameraIntrinsicsDynamically = False
    depthWidth = int.from_bytes(depthFile.read(CAMERA_PARAMETER_INT_SIZE), byteorder=platformByteOrder)
    depthHeight = int.from_bytes(depthFile.read(CAMERA_PARAMETER_INT_SIZE), byteorder=platformByteOrder)
    depthPixelSize = int.from_bytes(depthFile.read(CAMERA_PARAMETER_INT_SIZE), byteorder=platformByteOrder)
    sizeOfDepthFrame = depthWidth * depthHeight * depthPixelSize
    depthFPS = int.from_bytes(depthFile.read(CAMERA_PARAMETER_INT_SIZE), byteorder=platformByteOrder)
    isDynamicFPS = False
    if depthFPS == 0:
        isDynamicFPS = True
    print("Platform ID: ", platformId, " Resolution: ", depthWidth, "x", depthHeight, " FPS:", depthFPS, " Dynamic FPS: ", isDynamicFPS)

    frameNum = 0

    # Display depth video
    while True:
        # Read frame time stamp if dynamic FPS is enabled
        if isDynamicFPS:
            dynamicFrameInterval = int.from_bytes(depthFile.read(CAMERA_PARAMETER_INT_SIZE), byteorder=platformByteOrder)
            # print("Dynamic frame interval: ", dynamicFrameInterval)
            if cv.waitKey(dynamicFrameInterval) & 0xFF == ord('q'):
                break

        # Read camera intrinsics if it is configured to be read dynamically
        fx = 5.318673e2
        fy = 5.318673e2
        cx = 3.1593222e2
        cy = 2.3225371e2
        if isReadingDepthCameraIntrinsicsDynamically:
            bytes4CameraIntrinsics = depthFile.read(CAMERA_PARAMETER_INT_SIZE * 4)
            cameraIntrinsicsArray = np.frombuffer(bytes4CameraIntrinsics, dtype=np.float32)
            fx = cameraIntrinsicsArray[0]
            fy = cameraIntrinsicsArray[1]
            cx = cameraIntrinsicsArray[2]
            cy = cameraIntrinsicsArray[3]
            print("Camera intrinsics: ", fx, fy, cx, cy)
        
        # Read a depth frame
        depthFrame = depthFile.read(sizeOfDepthFrame)
        if not depthFrame:
            print("End of depth video file")
            break

        # Read a ML depth frame
        ret, mlDepthFrame = capMlDepth.read()
        if not ret:
            print("End of ML depth video file")
            break

        # Read a RGB frame
        ret, rgbFrame = capRGB.read()
        if not ret:
            print("End of RGB video file")
            break

        # Invalidate the first 3 bits of each pixel in depth frame if the platform is Android
        if platformId == 0:
            tempDepthFrame = bytearray()
            counter4Skipping = 0
            for byte in depthFrame:
                if (counter4Skipping + 1) == depthPixelSize:
                    tempDepthFrame.append(byte & 0x1F)
                    # tempDepthFrame.append(byte & 0xE0)
                else:
                    tempDepthFrame.append(byte)
                    # tempDepthFrame.append(byte & 0x00)
                counter4Skipping = (counter4Skipping + 1) % depthPixelSize
            depthFrame = tempDepthFrame
        
        # Convert depth frame to numpy array
        depthFrame = np.frombuffer(depthFrame, dtype=platformPixelDataType)
        depthFrame = depthFrame.reshape((depthHeight, depthWidth))

        # Build point cloud with depth frame
        fx = 320 * 0.6
        fy = 180 * 0.6
        x, y = np.meshgrid(np.arange(depthWidth), np.arange(depthHeight))
        x = (x - depthWidth / 2) / fx
        y = (y - depthHeight / 2) / fy
        # x = (x - cx) / fx
        # y = (y - cy) / fy
        z = depthFrame
        points = np.stack((np.multiply(x, z), np.multiply(y, z), z), axis=-1).reshape(-1, 3)

        # Display point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.visualization.draw_geometries([pcd])

        # Convert depth frame to millimeters (as uint16) if the platform is iOS
        if platformId == 1:
            depthFrame = depthFrame * 1000
            depthFrame = depthFrame.astype(np.uint16)

        # Mark plane surfaces in depth frame
        # currentSurfacePts = []
        # exploreSurface(depthFrame, depthWidth, depthHeight, 0, 0, depthFrame[0], currentSurfacePts, [])
        # x,y,w,h = cv.boundingRect(np.array(currentSurfacePts))
        # # hull = cv.convexHull(np.array(currentSurfacePts))
        # # print("First surface is", np.array(currentSurfacePts))
        # # print("Hull is", hull)
        # print("Depth frame pixel 0, 1, 2, 3, 4:", depthFrame[0], depthFrame[1], depthFrame[2], depthFrame[3], depthFrame[4])

        # Visualize depth frame
        depthFrame = cv.normalize(depthFrame, None, 0, 255, cv.NORM_MINMAX, cv.CV_8U)

        # Resize depth frame
        depthFrameResized = cv.resize(depthFrame, (DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT))

        # Resize ML depth frame
        mlDepthFrameResized = cv.resize(mlDepthFrame, (DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT))
        mlDepthFrameResized = cv.cvtColor(mlDepthFrameResized, cv.COLOR_BGR2GRAY)

        # Resize RGB frame
        rgbFrameResized = cv.resize(rgbFrame, (DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT))

        # Store depth and rgb frames
        # cv.imwrite("output/depth_" + str(frameNum) + ".png", depthFrameResized)
        # cv.imwrite("output/rgb_" + str(frameNum) + ".png", rgbFrame)
        # frameNum += 1
        # print("Frame number: ", frameNum)

        # cv.rectangle(depthFrameResized,(x,y),(x+w,y+h),(0,255,0),2)

        # Get relative depth frame
        # relativeDepthFrame = mlDepthFrameResized - depthFrameResized
        # error, diff = mse(depthFrameResized, mlDepthFrameResized)
        # print("MSE: ", error)

        # depthFrameResized = cv.bilateralFilter(depthFrameResized,15,75,75)

        # ret, thresh = cv.threshold(depthFrameResized,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)

        # blur = cv.GaussianBlur(depthFrameResized, (5, 5), 0)
        # # blur = cv.medianBlur(depthFrameResized, 5)
        # _, thresh = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        # contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        # print("first contour: ", contours[0])
        # hull = cv.convexHull(contours[0])
        # print("Hull: ", hull)
        # cv.drawContours(depthFrameResized, hull, -1, (0, 255, 0), 3)

        # edges = cv.Canny(depthFrameResized, 0, 255)
        # cv.imshow('Edges', edges)
        # mlEdges = cv.Canny(mlDepthFrameResized, 0, 255)
        # cv.imshow('ML Edges', mlEdges)

        # mse = mean_squared_error(depthFrameResized, mlDepthFrameResized)
        # print("MSE: ", mse)

        # ssim = structural_similarity(depthFrameResized, mlDepthFrameResized)
        # print("SSIM: ", ssim)

        # Display
        cv.imshow('Depth', depthFrameResized)
        cv.imshow('ML Depth', mlDepthFrameResized)
        # cv.imshow('Relative Depth', relativeDepthFrame)
        # cv.imshow('Diff', diff)
        if cv.waitKey(0) & 0xFF == ord('q'):
            break
        # if not isDynamicFPS and cv.waitKey(frameInterval) & 0xFF == ord('q'):
        #     break
    
    # Display RGB video
    # while True:
    #     ret, frame = capRGB.read()
    #     if not ret:
    #         break
    #     frameResized = cv.resize(frame, (DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT))
    #     cv.imshow('RGB', frameResized)
    #     if cv.waitKey(frameInterval) & 0xFF == ord('q'):
    #         break
    
    # Release resources
    capRGB.release()
    cv.destroyAllWindows()
    depthFile.close()

if __name__ == "__main__":
    main()
