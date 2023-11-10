# Translation Package

## Package Setup
1. Install Google Translate library, run `pip install --upgrade googletrans==4.0.0rc1`

## Additional Commands
1. Input a raw string to be translated, run `ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: '你好我的名字 是Damien'}"` or `ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: 'हैलो मेरा नाम हैDamien'}"`
2. Change the target language whilst node is running `ros2 service call /target_language polyglotbot_interfaces/srv/TranslateString "{input: 'es'}"`