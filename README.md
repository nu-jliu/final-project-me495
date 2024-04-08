# ME495 Robotics Embedded Systems Final Project- Polyglotbot
**Group 3**

[Prtfolio Post](https://nu-jliu.github.io/ployglotbot/)

Authors: Megan Black, Henry Buron, Damien Koh, Allen Liu, and Kassidy Shedd

## Description
This package is for the final project from ME495 at Northwestern University.  The goal of the final project is to read in written words/phrases on a whiteboard, translate to another language, and write the translation of the whiteboard with a Franka.  This package also picks up and returns the marker to the holder, speaks the translated word, and allows the user to choose to input a phrase using a microphone.

[IMG.mp4](https://github.com/ME495-EmbeddedSystems/final-project-dkoh555/assets/116540591/02c130ab-0821-4031-aa09-3b525369f90f)


## Quickstart Guide
### Import Mover API
Import the `franka_mover` API packages via
```
cd ${ROS_WS}
vcs import --recursive --input \
    https://raw.githubusercontent.com/ME495-EmbeddedSystems/final-project-dkoh555/main/mover.repos?token=GHSAT0AAAAAACLGI3BYI3DDKDTCRUFGQ2LCZLO4AOA src
```

### Download packages
1. Install `PaddlePaddle` by running 
```
python -m pip install paddlepaddle -i https://pypi.tuna.tsinghua.edu.cn/simple
```
2. Install by `paddleocr` running
```
pip install "paddleocr>=2.0.1"
```
3. Install various language packages by testing `PaddlePaddle` on a test image while in the github directory
```
paddleocr --image_dir read_whiteboard/read_whiteboard/testing_image.png --lang=en
```
Available languages are shown here. 
https://github.com/PaddlePaddle/PaddleOCR/blob/release/2.7/doc/doc_en/multi_languages_en.md#language_abbreviations 

4. Install YOLO by running
```
pip install ultralytics
```
5. Install Google Translate library, run
```
pip install --upgrade googletrans==4.0.0rc1
```
6. Install the `google_speech` api via
```
pip install google-speech
```
7. Install the `pyttsx3` via
```
pip install pyttsx3
```
8. Install the `SoX` via
```
sudo apt-get install sox libsox-fmt-mp3
```
9. Install `sounddevice` via pip:
```bash
pip install sounddevice
```
10. Install `SpeechRecognition` via pip:
```bash
pip install SpeechRecognition
```
11. Install `portaudio19-dev` using apt:
```bash
sudo apt install portaudio19-dev
```
12. Finally, install `pyaudio` via pip:
```bash
pip install pyaudio
```
13. Other packages include: Realsense, and apriltags_ros


## Usage Instructions
1. Make sure the appropriate external hardware are connected to the computer.  Hardware includes a microphone, speaker, and 2 Intel Realsense cameras.
2. Use `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true` to start the Franka simulation in rviz or `ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot` to start the Franka in lab.
3. Launch the project by running
```
ros2 launch polyglotbot polyglotbot.xml
```
4. If desired, run
```
ros2 service call /toggle_mode polyglotbot_interfaces/srv/TranslateString "{input: 'en'}"
```
to switch to the microphone mode.  The language code can be changed to most languages, but will always translate to english.
5. To return the marker at the end of a demo, use 
```
ros2 service call /put_back std_srvs/srv/Trigger
```


## Launchfiles
The `polyglotbot.launch.xml` launchfile in the `polyglotbot` packages launches the `speaker`, `translator_node`, and `polyglotbot`, nodes, and includes launch files for the other packages.  To account for using multiple cameras, the `get_apriltags` and `computer_vision` nodes are assigned different namespaces and camera serial numbers.

The `read_whiteboard.launch.xml` launchfile launches the Realsense camera node with the `rs_launch.py` launchfile and the `computer_vision` node.

The `create_waypoint.launch.xml` launchfile launches the `create_waypoint` node.

The `get_apriltags.launch.xml` launchfile launches the Realsense camera node with the `rs_launch.py` launchfile, `image_proc.launch.py`, rviz2, ros2 `apriltag_node` node, and the `get_apriltags` node.

The `writer.launch.xml` launchfile launches move_it with `moveit.launch.py`, the `vec_parser` node, and the `writer` node.



## System Architecture
### polyglotbot Package
This package contains the main node `polyglotbot`.  This node includes the overall system state machine, which was carefully designed to be robust and prevent failure.

Upon startup, the robot will pick up a marker from the holder at a specified location, then returns to a ready location and enters the `WAITING` state.

In the `WAITING` state, the robot waits until a person is detected at the whiteboard or the `toggle_mode` service is called.  

Once the person leaves the whiteboard, the `computer_vision` node from the `read_whiteboard` package determines the target language and word/phrase to be translated.

Next, the word/phrase is translated into the desired language using the `translator_node` node from the `translation_pkg` package.

The robot speaks the translated word/phrase using the `speaker` node from the `speak_out` package.

The translated word/phrase is seperated into individual characters and creates waypoints for the robot to draw each character using the `string2waypoints_node` node from the `string2waypoints` package.

The robot uses the waypoints to approach the board, draw each letter, lifting the marker when necessary, and return to a ready position using the `vec_parser` and `write` nodes from the `write_letters` package.

If the microphone is used, the robot listens for a spoken word/phrase using the `listen_speech` node from the `speech` package and continues on to translation and the rest of the system pipeline.

When the `toggle_mode` service is called, the robot puts the marker back and enters a `DONE` state to end a demo.

Apriltags are used to localize the whiteboard using the `get_apriltags` node from the `apriltags` package.

### polyglotbot_interfaces Package
This package contains the custom interface types for our project.

Message Types: `AprilCoords` and `CharacterPath`

Service Types: `GetCharacters`, `GrabPen`, `Path`, `SpeakText`, `StringToWaypoint`, `TranslateString`, and `Write`

### read_whiteboard Package
This package handles monitoring the whiteboard for human detection and optical character recognition (OCR).
The `computer_vision` node uses YOLO to detect people at the whiteboard and publishes the average number of detected people over 2 seconds.
Additionally, the `computer_vision` node uses PaddlePaddle OCR to detect written words/phrases.

### translation_pkg Package
This package translates a provided word/phrase into the desired language using the Google Translate library in the `translation_node` node.

### string2waypoints Package
This package converts the string characters into waypoints.
The `string2waypoint_node` uses TextPath from matplotlib to create waypoints for each character.  Additionally, waypoints are added for when the pen should be lifted off the whiteboard.

### write_letter Package
This packages uses our custom ros2 move_it API, created for HW3 (https://github.com/ME495-EmbeddedSystems/homework3group-nu-jliu/tree/main), to move the Franka robot arm.

The `vec_parser` node modifies the waypoints to consider the location from the board, as determined using Apriltags.
The `write` node interfaces with our `motion_plan_pkg` move_it API to command the robot arm through waypoints using the Cartesian path planner.

### apriltags Package
This package interfaces with Apriltags.

The `get_apriltags` node localizes the Apriltags and publishes their coordinates.

### speak_out Package
This package turns the translated word/phrase into audio played through a speaker using the `speaker` node.

### speech Package
This package allows the user to speak the desired word/phrase into a microphone.

The `listen_speech` node interfaces with the microphone to listen for speech in an inputted language.



## Configuration Instructions

### Services
The `toggle_mode` service toggles the state of the `polyglotbot` node to allow for listening to speech from user.  This service is of type `polyglotbot_interfaces/srv/TranslateString` and accepts the spoken launguage as the input string.

The `start_translating` service can be called to start the translation process to test part of the system pipeline.  This service is of type `std_srvs/srv/Empty`.


## Realsense Camera Servial No.

 - **MSR-RS-6**: Device Serial No: `938422073373`
 - **EE-Cam**: Device Serial No: `233522076213`

