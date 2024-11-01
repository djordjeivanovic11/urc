This folder contains all the files used for a live test of generating a set of
ArUco markers and detecting them with a live camera video feed.

This folder follows along with the youtube video at:
https://www.youtube.com/watch?v=YOpJrB6bQxo


# Live ArUco Tag Detection with a Video Capture Device for URC Missions

This project utilizes ArUco and it's built-in functions and dictionaries alongside OpenCV to detect ArUco AR tags from a live video feed. The camera will continuously take frames from the chosen video capture device, shift them to grayscale images, and use ArUco's marker detection function to search the image for AR tags and ID them. This code is for use in the **University Rover Challenge (URC)**

## Table of Contents
1. [Installation](#installation)
2. [Configuration](#configuration)
3. [Usage](#usage)
4. [Description](#description)


## Installation
1. **Clone this repository:**
   ```bash
   git clone <>
   cd <repository_name>
   ```

2. **Install the required Python packages:**
   ```bash
   pip install opencv-python opencv-contrib-python numpy
   ```

## Configuration
1. **Specify video capture device**
  If the video capture device used for tag detection is not the system default, the device index must be edited in the code.
  ```bash
  cap = cv2.VideoCapture(#index)
  ```
2. **Specify the ArUco tags to be detected**
  The URC challenge will be utilizing the 4X4_50 tag library from ArUco for tags. If you use a different library, this must be changed in the code.
  ```bash
  marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
  ```

## Usage
1. **Run the program:**
  Ensure you are in the correct directory
  ```bash
  python detect.py
  ```

2. **The program will:**
   - Open the specified video capture device.
   - Continuously capture frames, shift them to grayscale, and search the frame images for ArUco tags
   - Determines the location of the tag(s) by finding the corners of each tag
   - For each ArUco tag detected, outline the tag and label it with an ID
   - Display each frame back to the user akin to a video

3. **Exit the program** manually by pressing 'q'.

##