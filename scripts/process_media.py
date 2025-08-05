from PIL import Image
import depth_pro
import os
import sys
import cv2
from pathlib import Path
import tqdm
import torch
import time

# FOCAL_LENGTH_PX = 

targeted_resolutions = [(320, 180), (768, 432), (364, 205)] # ios_video, ios_photo, android

OVERWRITE = False # If True, overwrite existing depth maps

def get_torch_device() -> torch.device:
    """Get the Torch device."""
    device = torch.device("cpu")
    if torch.cuda.is_available():
        device = torch.device("cuda:0")
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    print("Device: ", device)
    return device

def main():

    if (len(sys.argv) != 2):
        print("Usage: python process_media.py <path>")
        sys.exit(1)

    # Read input video path and output path from command line arguments.
    input_path = sys.argv[1]

    # Load model and preprocessing transform
    model, transform = depth_pro.create_model_and_transforms(
        device=get_torch_device(),
        precision=torch.half)
    model.eval()

    # Get all media files in the input path (photos and videos) recursively.
    input_path = Path(input_path)
    photo_files = list(input_path.glob("**/*.jpg"))
    video_files = list(input_path.glob("**/*.mp4"))

    # Intialize two progress bards(one for overall and one for each media file).
    media_progress = tqdm.tqdm(total=100, desc="Media Progress")
    overall_progress = tqdm.tqdm(total=(len(photo_files) + len(video_files)), desc="Overall Progress")

    # Time taken to process each photo
    ios_time_taken = []
    android_time_taken = []

    # Process each photo file.
    for photo_file in photo_files:
        # Check if the photo already has a depth map in its photo directory.
        photo_file_path_str = str(photo_file)
        depth_map_file_path = photo_file_path_str[:photo_file_path_str.rfind('.')] + "_depth_metric.ml_depth_pro.mldep"
        if not OVERWRITE and os.path.exists(depth_map_file_path):
            overall_progress.update(1)
            continue

        # Intialize media_progress bar.
        media_progress.reset(total=1)

        # Determine if the photo is from iOS or Android (by checking if the file name contains "ios" or "android")
        is_ios = "ios" in str(photo_file)

        # Start of the timer.
        start_time = time.time()

        # Read photo file.
        photo = Image.open(photo_file_path_str)

        # Resize the photo to the required size (width: 1536) while maintaining the aspect ratio.
        width, height = photo.size
        new_width = 1536
        new_height = int(height * new_width / width)
        photo = photo.resize((new_width, new_height))

        # Run inference on the photo.
        photo = transform(photo)
        prediction = model.infer(photo)
        depth = prediction["depth"].detach().cpu().numpy().squeeze() # Depth in [m].
        focallength_px = prediction["focallength_px"]  # Focal length in pixels.

        # Resize depth map accordingly
        if is_ios:
            depth = cv2.resize(depth, targeted_resolutions[1], interpolation=cv2.INTER_NEAREST)
        else:
            depth = cv2.resize(depth, targeted_resolutions[2], interpolation=cv2.INTER_NEAREST)

        # Convert to float16
        depth = depth.astype('float16')

        # Write depth map to file.
        depth_map_file = open(depth_map_file_path, "wb")
        depth_map_file.write(depth.tobytes())
        depth_map_file.close()

        # End of the timer.
        end_time = time.time()

        # Record the time taken to process the photo.
        time_taken = end_time - start_time
        if is_ios:
            ios_time_taken.append(time_taken)
        else:
            android_time_taken.append(time_taken)

        media_progress.update(1)
        overall_progress.update(1)

    # Print the average time taken to process each photo and standard deviation.
    ios_time_taken_avg = sum(ios_time_taken) / len(ios_time_taken)
    android_time_taken_avg = sum(android_time_taken) / len(android_time_taken)
    ios_time_taken_std = (sum([(time - ios_time_taken_avg) ** 2 for time in ios_time_taken]) / len(ios_time_taken)) ** 0.5
    android_time_taken_std = (sum([(time - android_time_taken_avg) ** 2 for time in android_time_taken]) / len(android_time_taken)) ** 0.5
    print("Average time taken to process each iOS photo: ", ios_time_taken_avg)
    print("Standard deviation of time taken to process each iOS photo: ", ios_time_taken_std)
    print("Average time taken to process each Android photo: ", android_time_taken_avg)
    print("Standard deviation of time taken to process each Android photo: ", android_time_taken_std)

    # Process each video file.
    for video_file in video_files:
        # Check if the video already has a depth map.
        video_file_path_str = str(video_file)
        depth_map_file_path = video_file_path_str[:video_file_path_str.rfind('.')] + "_depth_metric.ml_depth_pro.mldep"
        if not OVERWRITE and os.path.exists(depth_map_file_path):
            overall_progress.update(1)
            continue

        # Read video file.
        video = cv2.VideoCapture(video_file_path_str)
        if not video.isOpened():
            print("Error opening video file: ", video_file)
            continue

        # Get the frame count and intialize media_progress bar.
        frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        media_progress.reset(total=frame_count)

        # Determine if the photo is from iOS or Android (by checking if the file name contains "ios" or "android")
        is_ios = "ios" in str(video_file)

        video_file_name = video_file_path_str[video_file_path_str.rfind('/') + 1:]
        depth_map_file = open(depth_map_file_path, "wb")

        # Read each frame from the video file.
        while True:
            ret, frame = video.read()
            if not ret:
                break

            # Convert the frame to numpy ndarray.
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to the required size (width: 1536) while maintaining the aspect ratio.
            height, width = frame.shape[:2]
            new_width = 1536
            new_height = int(height * new_width / width)
            frame = cv2.resize(frame, (new_width, new_height))

            # Resize the frame to 320*180.
            # frame = cv2.resize(frame, (320, 180), interpolation=cv2.INTER_NEAREST)

            # Run inference on the frame.
            frame = transform(frame)
            prediction = model.infer(frame)
            depth = prediction["depth"].detach().cpu().numpy().squeeze() # Depth in [m].
            focallength_px = prediction["focallength_px"]  # Focal length in pixels.

            # Resize depth map accordingly
            if is_ios:
                depth = cv2.resize(depth, targeted_resolutions[0], interpolation=cv2.INTER_NEAREST)
            else:
                depth = cv2.resize(depth, targeted_resolutions[2], interpolation=cv2.INTER_NEAREST)

            # Convert to float16
            depth = depth.astype('float16')

            # Write depth map to file.
            depth_map_file.write(depth.tobytes())

            media_progress.update(1)

        depth_map_file.close()
        video.release()
        
        overall_progress.update(1)
            


if __name__ == "__main__":
    main()