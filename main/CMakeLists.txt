idf_component_register(
    SRCS "main.c"
    SRCS "joystick_buttons.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES    "esp32-button" "driver"

    )
