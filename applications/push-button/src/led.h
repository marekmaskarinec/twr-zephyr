#ifndef LED_H_
#define LED_H_

int led_set(int value);
int led_blink(k_timeout_t delay);
int led_init(void);

#endif /* LED_H_ */