# ME495 Embedded Systems Final Project - Read Whiteboard
Author: Megan Black

## Description
This package is for the final project from ME495 at Northwestern University.  The goal of the final project is to read in written words/phrases on a whiteboard, translate to another language, and write the translation of the whiteboard with a Franka.  

This package handles reading the word/phrase on the whiteboard and determining when a person is in the frame.



## Usage Instructions
1. Install `PaddlePaddle` by running 
```
python -m pip install paddlepaddle -i https://pypi.tuna.tsinghua.edu.cn/simple
```
2. Install by `paddleocr` running
```
pip install "paddleocr>=2.0.1"
```
3. Install various language packages by testing `PaddlePaddle` on a test image
```
paddleocr --image_dir read_whiteboard/read_whiteboard/testing_image.png --lang=en
```
Available languages are shown here. 
https://github.com/PaddlePaddle/PaddleOCR/blob/release/2.7/doc/doc_en/multi_languages_en.md#language_abbreviations 

4. Install YOLO by running
```
pip install ultralytics
```




## Configuration Instructions

### Publishers
This node publishes to the `person_detect` topic.  This topic is of type `std_msgs/msg/Float32` and represents the average number of people detected in the frame over the last 2 seconds.

### Subscribers
This node subscribes to the `camera/color/image_raw` topic.  This topic is of type `sensor_msgs/msg/Image` and is the image from the camera.

### Services
The `get_characters` service can be called to read the whiteboard and determine the characters written on the board.  The `get_characters` service is of type `polyglotbot_interfaces/srv/GetCharacters` and accepts an `std_msgs/msg/Empty` request.
The `get_characters` service returns a list of strings.  The first index is the desired language and the second index is the written word/phrase.
