# ME495 Embedded Systems Final Project - Read Whiteboard
Author: Megan Black

## Description
This package is for the final project from ME495 at Northwestern University.  The goal of the final project is to read in written words/phrases on a whiteboard, translate to another language, and write the translation of the whiteboard with a Franka.  This package...



## Usage Instructions
1. Install `PaddlePaddle` by running 
```
python -m pip install paddlepaddle-gpu -i https://pypi.tuna.tsinghua.edu.cn/simple
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




## Configuration Instructions

### Parameters
TBD

### Publishers
TBD

### Services
TBD
