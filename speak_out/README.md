# Speak
**Author**: Allen Liu<br/>
This package speak out the translated text
## Setup Task
Install the `google_speech` api via
```
pip install google-speech
```
Install the `pyttsx3` via
```
pip install pyttsx3
```
Install the `SoX` via
```
sudo apt-get install sox libsox-fmt-mp3
```
## How to run
Run the node via 
```
ros2 run speak_out speak
```
## Nodes
 - `speaker`: Speaks out the requested text in target language.
## Services
 - `speak`: Speak the requested text.
 ```
 ros2 service call speak polyglotbot_interfaces/srv/SpeakText "{text: 'Hello'}"
 ```