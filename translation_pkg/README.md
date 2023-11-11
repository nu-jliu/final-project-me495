# Translation Package

## Package Setup
1. Install Google Translate library, run
    ```
    pip install --upgrade googletrans==4.0.0rc1
    ```

## Quickstart
1. Run the translator node with
```
ros2 run translation_pkg translator_node
```
2. Run the listener node in another terminal with
```
ros2 run translation_pkg listener_node
```
3. Provide a string via srv to begin translating
```
ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: '안녕 내 이름은Damien'}"
```

## Additional Commands
1. Input another raw string to be translated, run `ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: '你好我的名字是Damien'}"` or `ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: 'हैलो मेरा नाम हैDamien'}"`
2. Change the target language whilst node is running `ros2 service call /target_language polyglotbot_interfaces/srv/TranslateString "{input: 'es'}"`