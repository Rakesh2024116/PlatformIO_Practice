#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define LED_PIN 25
#define ADC_PIN 26

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    while (1) {
        uint16_t raw = adc_read();
        const float conversion_factor = 3.3f / (1 << 12);
        float voltage = raw * conversion_factor;
        
        printf("Raw value: %d, voltage: %f V\n", raw, voltage);
        
        if (voltage > 1.65f) {
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }
        
        sleep_ms(500);
    }

    return 0;
}
