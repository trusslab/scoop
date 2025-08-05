#!/bin/sh
set -e

# Install or uninstall required libraries for the Surface Depth Analyzer project
# Required libraries: OpenCV (with contrib), PCL, Boost
# This script will download and compile the libraries if they are not already installed.

# Set the directory for installation
INSTALL_DIR=$(dirname "$(readlink -f "$0")")/../libs

# Set library versions
OPENCV_VERSION="master"
OPENCV_CONTRIB_VERSION="master"
PCL_VERSION="pcl-1.15.0"
BOOST_VERSION="master"

# Number of CPU cores for parallel compilation (always reserve at least 1 core for system tasks)
CPU_CORES=$(nproc || sysctl -n hw.ncpu)
if [ "$CPU_CORES" -gt 1 ]; then
    CPU_CORES=$((CPU_CORES - 1))
else
    CPU_CORES=1
fi
echo "Will use $CPU_CORES cores for compilation, leaving 1 core for system tasks."

# If the OS is ubuntu, check if it is 20.04 or later (libvtk7-dev is available in 20.04 and libvtk9-dev is available in later versions)
UBUNTU_VTK_PKG="libvtk7-dev"
UBUNTU_QT5_PKGS="qt5-default libqt5opengl5-dev"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" = "ubuntu" ]; then
        UBUNTU_VERSION=$(echo "$VERSION_ID" | cut -d '.' -f 1)
        if [ "$UBUNTU_VERSION" -ge 20 ]; then
            UBUNTU_VTK_PKG="libvtk9-dev"
            UBUNTU_QT5_PKGS="qtbase5-dev libqt5opengl5-dev"
        fi
    fi
fi

# Check if the user wants to install or uninstall libraries
if [ "$1" = "install" ]; then
    echo "Installing required libraries..."
    mkdir -p "$INSTALL_DIR"

    # Install dependencies
    echo "Installing dependencies..."
    # Check the system package manager and install necessary packages (Debian-based, Arch-based, Fedora-based, MacOS)
    if command -v apt >/dev/null; then
        sudo apt update
        sudo apt install -y build-essential cmake git libboost-all-dev libeigen3-dev libtbb-dev libpng-dev libjpeg-dev libtiff-dev $UBUNTU_VTK_PKG $UBUNTU_QT5_PKGS libqhull-dev libtiff-dev
    elif command -v pacman >/dev/null; then
        sudo pacman -Syu --noconfirm base-devel cmake git boost eigen tbb libpng libjpeg-turbo libtiff vtk qt5-base gambas3-gb-qt5-opengl qhull
    elif command -v dnf >/dev/null; then
        sudo dnf groupinstall -y "Development Tools"
        sudo dnf install -y cmake git boost-devel eigen3-devel tbb-devel libpng-devel libjpeg-turbo-devel libtiff-devel vtk-devel qt5-qtbase-devel qhull-devel libtiff-devel
    elif command -v brew >/dev/null; then
        brew update
        brew install cmake git boost eigen tbb libpng libjpeg-turbo libtiff vtk qt@5 qhull flann libomp
    else
        echo "Unsupported package manager. Please install the required dependencies manually."
        exit 1
    fi
    echo "Dependencies installed."

    # Install OpenCV with contrib modules
    if [ ! -d "$INSTALL_DIR/opencv" ]; then
        echo "Installing OpenCV..."
        git clone https://github.com/opencv/opencv.git "$INSTALL_DIR/opencv"
        git clone https://github.com/opencv/opencv_contrib.git "$INSTALL_DIR/opencv_contrib"
    fi
    cd "$INSTALL_DIR/opencv" || exit
    mkdir -p build && cd build
    cmake -DOPENCV_EXTRA_MODULES_PATH="$INSTALL_DIR/opencv_contrib/modules" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR/opencv/install" ..
    cmake --build . -- -j"$CPU_CORES"

    # Install PCL (Point Cloud Library)
    if [ ! -d "$INSTALL_DIR/pcl" ]; then
        echo "Installing PCL..."
        git clone https://github.com/PointCloudLibrary/pcl.git "$INSTALL_DIR/pcl"
    fi
    cd "$INSTALL_DIR/pcl" || exit
    git checkout "$PCL_VERSION"
    git submodule update --init --recursive
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR/pcl/install" ..
    make -j"$CPU_CORES"
    make install

    # # Install Boost
    # if [ ! -d "$INSTALL_DIR/boost" ]; then
    #     echo "Installing Boost..."
    #     git clone https://github.com/boostorg/boost.git "$INSTALL_DIR/boost"
    # fi
    # cd "$INSTALL_DIR/boost" || exit
    # mkdir -p build && cd build
    # cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR/boost/install" ..
    # make -j$(nproc)

elif [ "$1" = "uninstall" ]; then
    echo "Removing required libraries..."
    if [ -d "$INSTALL_DIR" ]; then
        rm -rf "$INSTALL_DIR"
    else
        echo "No libraries to remove."
    fi
else
    echo "Usage: $0 {install|uninstall}"
    exit 1
fi

