# https://www.reddit.com/r/esp32/comments/1ananz5/comment/l00vpbo/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
# You have to patch .platformio/platforms/espressif32/boards/esp32-c6-devkitc-1.json and add
# build."variant": "esp32c6" and "frameworks": add "arduino"

pio run
cp esp32-c6-devkitc-1.json ~/.platformio/platforms/espressif32/boards/esp32-c6-devkitc-1.json
