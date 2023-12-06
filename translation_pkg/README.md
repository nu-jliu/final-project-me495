# Translation Package
**Author**: Damien Koh<br/>
This package utilizes a version of the Google Translate API to translate a source string into a specified target language.

## Package Setup
1. To install the Google Translate library, run:
    ```
    pip install --upgrade googletrans==4.0.0rc1
    ```
    Please note that the Google Translate is largely no longer supported, therefore only this specific version works.

## Quickstart
1. Run the translator node with:
    ```
    ros2 run translation_pkg translator_node
    ```
2. Run the listener node in another terminal with:
    ```
    ros2 run translation_pkg listener_node
    ```
3. Provide a source string via service call to begin translating:
    ```
    ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: '안녕 내 이름은Damien'}"
    ```
4. Change the target language to translate source string into whilst the translator node is running (must specify a supported language code):
    ```
    ros2 service call /target_language polyglotbot_interfaces/srv/TranslateString "{input: 'es'}"
    ```

## Supported Languages
The following link contains a list of all the supported languages and their respective language codes:
https://py-googletrans.readthedocs.io/en/latest/

## Nodes
1. `translator_node`: The node responsible for translating a given string into a target language, and then publishing that translated string.
2. `listener_node`: A node for listening to the translator_node to monitor its target language and resulting translated string.

## Additional Commands
1. Input source strings of different languages to be translated, run:
    ```
    `ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: '你好我的名字是Bob'}"`
    ```
    Or:
    ```
    ros2 service call /input_msg polyglotbot_interfaces/srv/TranslateString "{input: 'हैलो मेरा नाम हैBob'}"
    ```