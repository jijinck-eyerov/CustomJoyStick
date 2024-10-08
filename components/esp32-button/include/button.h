#define PIN_BIT(x) (1ULL<<x)

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)

typedef struct {
	uint8_t pin;
    uint8_t event;
} button_event_t;

QueueHandle_t * button_Init(unsigned long long pin_select);
