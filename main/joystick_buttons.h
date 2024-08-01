#ifndef __JOYSTICK_BUTTONS_H__
#define __JOYSTICK_BUTTONS_H__

#ifdef __cplusplus
extern "C" {
#endif

// J1 connector
#define BUTTON_1      GPIO_NUM_10
#define BUTTON_2      GPIO_NUM_11
#define BUTTON_3      GPIO_NUM_12
#define BUTTON_4      GPIO_NUM_13
#define BUTTON_5      GPIO_NUM_14
#define BUTTON_6      GPIO_NUM_15
#define BUTTON_7      GPIO_NUM_16
#define BUTTON_8      GPIO_NUM_17
#define BUTTON_9      GPIO_NUM_18
#define BUTTON_10     GPIO_NUM_35
#define BUTTON_11     GPIO_NUM_36
#define BUTTON_12     GPIO_NUM_37
#define BUTTON_13     GPIO_NUM_39
#define BUTTON_14     GPIO_NUM_40
#define BUTTON_15     GPIO_NUM_41
#define BUTTON_16     GPIO_NUM_42
#define BUTTON_17     GPIO_NUM_45
#define BUTTON_18     GPIO_NUM_46
#define BUTTON_19     GPIO_NUM_47
#define BUTTON_20     GPIO_NUM_48




QueueHandle_t * joystick_buttons_init(void);

typedef struct {
    uint8_t pin;
} button_t;

typedef struct {
    uint16_t state;
} joystick_buttons_event_t;

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_BUTTONS_H__ */