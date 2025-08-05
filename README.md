# Scoop: Mitigation of Recapture Attacks on Provenance-Based Media Authentication

:paperclip: [Scoop Paper]() <br/>
:camera: [Scoop iOS Camera](https://github.com/trusslab/scoop_camera_ios) <br/>
:camera: [Scoop Android Camera](https://github.com/trusslab/scoop_camera_android) <br/>
:computer: [Scoop Viewer](https://github.com/trusslab/scoop) <br/>
:floppy_disk: [Scoop Dataset](https://doi.org/10.5281/zenodo.15611905)

## Overview

This is a part of the Scoop project, which can be used to analyze if any misleading recapture is present in the media. 
It can be used as a strong tool to assist users in identifying the potential existence of recapture attacks on media files.

## Requirements
To run the analyzer, you need to have the following dependencies installed:
- [libpcl](https://pointclouds.org/) (Point Cloud Library)
- [OpenCV](https://opencv.org/) (for image processing)
- [Boost](https://www.boost.org/) (for various utilities)
- [Eigen](https://eigen.tuxfamily.org/) (for linear algebra operations)
- [CMake](https://cmake.org/) (for building the project)
- [Python 3](https://www.python.org/) (for running the evaluation script)

## Environment Setup
We provide a script to quickly set up the environment for running the analyzer. 
A virtual environment is recommended to avoid conflicts with other projects.
You can run the following command to set up the environment:

```bash
bash scripts/install_required_libraries.sh
```

This has been tested on Ubuntu 24.04 LTS, but it should work on other Linux distributions (e.g., Fedora, Arch Linux) and MacOS (with Homebrew) as well.

### Manual Environment Setup
If you prefer to set up the environment manually, please follow the instructions below:
1. **OpenCV**: Install OpenCV to the `lib/opencv/build` folder under the project root directory. 
2. **PCL**: Install the Point Cloud Library (PCL) to the `lib/pcl/install` folder under the project root directory.

Alternatively, you may install the required libraries to wherever you prefer, but you will need to modify the `CMakeLists.txt` file under the project root directory accordingly.

## Building the Analyzer
To build the analyzer, you need to have CMake installed.
You can build the analyzer by running the following commands in the project root directory:

```bash
cmake .
make -j$(nproc)
```
This will create an executable file named `Analyzer` in the project root directory.

### Analyzer Configuration
The analyzer can be configured before compilation by modifying the `CompilationFlags.hpp` file in the `include` directory.
You can enable or disable various features of the analyzer by changing the flags in this file.

## Usage

There are two ways to run the analyzer:

```bash
./Analyzer [is_video] [rgb_input_path] [ground_truth_depth_input_path] [perceived_depth_input_path] [config_path]
```

or 

```bash
./Analyzer [is_video] [file_abbreviation] [config_path]
```

The first method requires all the full paths, where second method requires only the file abbreviation. 

## Usage with Scoop Dataset
To run the analyzer with the Scoop dataset, you need to first download the dataset from the [Scoop Dataset](). 
Assuming you have downloaded the dataset and extracted it to path `$SCOOP_DATASET_PATH`, you then need to generate the perceived depth data. 

There are many depth estimation models available, but we recommend using the [ml-depth-pro](https://github.com/apple/ml-depth-pro) model, which is a high-quality depth estimation model provided by Apple. 
We provide a script to generate the perceived depth data using the ml-depth-pro model.
To use it, please make sure you have Conda installed.
You can run the following commands to generate the perceived depth data for all the data points in the dataset:

```bash
cd scripts

git clone https://github.com/apple/ml-depth-pro.git
cd ml-depth-pro
conda create -n depth-pro -y python=3.9
conda activate depth-pro
pip install -e .

cp ../process_media.py .
python3 process_media.py $SCOOP_DATASET_PATH
```

### Example Usage

Below are two examples of how to run the analyzer with one of the sample data point provided in this repository. The first example uses the first method with full paths, while the second example uses the second method with file abbreviation.

```bash
./Analyzer 0 ./sample_data/018_complex_many_items_dark/ios_photo_rgb.jpg ./sample_data/018_complex_many_items_dark/ios_photo_depth.idep ./sample_data/018_complex_many_items_dark/ios_photo_rgb_depth_metric.ml_depth_pro.mldep ./sample_data/018_complex_many_items_dark/config.ini
```

```bash
./Analyzer 0 ./sample_data/018_complex_many_items_dark/ios_photo 1 ./sample_data/018_complex_many_items_dark/config.ini
```

### Quick Evaluation on Sample Data
To quickly evaluate the analyzer on the sample data provided in this repository, you can run the following command:

```bash
python3 scripts/eval.py sample_data/ 0000 9999
```

This will run the analyzer on all the sample data points in the `sample_data` directory, from `0000` to `9999` (though there are only 8 sets of data provided (with 4 unique data points)), and print the results.

## Depth Data Formats

Scoop consists of two types of depth data formats:
1. **Ground Truth Depth**: This is the depth data that is captured by a depth camera, such as the iPhone's LiDAR sensor. It provides accurate depth information of the scene.
2. **Perceived Depth**: This is the depth data that is estimated from a single RGB image using a depth estimation model. It provides an approximation of the depth information.

### Ground Truth Depth Format
The ground truth depth data in Scoop is stored in either `.idep` or `.adep` format. The `.idep` format is used for depth data captured by the iPhone's LiDAR sensor, while the `.adep` format is used for depth data captured by Android depth cameras. 

The depth data files contain metadata followed by the actual depth data for each pixel in the image or video frame.
The depth data is stored as a sequence of bytes, where each pixel's depth value is represented by a fixed number of bytes. The format includes metadata at the beginning, followed by the actual depth values for each pixel in the image or video frame.

Please note that Android depth data is encoded in big-endian format, while iOS depth data is in little-endian format. 
Please note that iOS depth data includes the depth camera intrinsics, while Android depth data does not include this information.
Please note that Android depth data is stored in a 16-bit integer format (in millimeters), while iOS depth data is stored in a 16-bit floating-point format (in meters).

For detailed information on how to interpret the depth data, please refer to Scoop viewer's implementation.

See below for a byte-level description of the depth data format:

```
- `uint8_t platform`: 0 for Android (Galaxy S20 Plus 5G), 1 for iOS (iPhone 14 Pro)
- `uint32_t width`: Width of the depth image
- `uint32_t height`: Height of the depth image
- `uint32_t pixel_size`: Size of each pixel in bytes
- `uint32_t fps`: Frames per second (for videos)
Start of actual depth data (loop for each frame):
Start of iOS only depth camera intrinsics (if applicable):
- `float fx`: Focal length in x-axis
- `float fy`: Focal length in y-axis
- `float cx`: Optical center x-coordinate
- `float cy`: Optical center y-coordinate
End of iOS only depth camera intrinsics
Start of actual depth data for each pixel (loop for each pixel):
- `uint16_t depth_value`: Depth value for each pixel (in millimeters for Android, in meters for iOS)
End of actual depth data for each pixel
End of actual depth data
```

### Perceived Depth Format
The perceived depth data in Scoop is stored in `.mldep` format. This format is used for depth data estimated from a single RGB image using a depth estimation model.
The perceived depth data files contain only the depth values for each pixel in the image or video frame, without any metadata.
The depth data is stored as a sequence of bytes, where each pixel's depth value is represented by 2 bytes (16-bit float; in meters).

See below for a byte-level description of the perceived depth data format:

```
Start of perceived depth data (loop for each frame):
Start of perceived depth data for each pixel (loop for each pixel):
- `float16 depth_value`: Depth value for each pixel (in meters)
End of perceived depth data for each pixel
End of perceived depth data
```

## Miscellaneous

Some sample configurations for running the analyzer are provided in the `sample_configs` directory. 
As mentioned in our paper, ideally a vision model should be used to determine what parameters to use for each component of the analyzer. 
However, for the sake of simplicity, we have provided some sample configurations that can be used to run the analyzer on various media files.

Additionally, some early prototype implementations of the analyzer are provided in the `py_based_prototypes` directory. 
They are not fully functional and are provided for reference only.

## License

