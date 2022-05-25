#include <pico/bootrom.h>
#include <pico/stdlib.h>



int main(){
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
	for (unsigned int i=0;i<10;i++){
		gpio_put(PICO_DEFAULT_LED_PIN,1);
		sleep_ms(200);
		gpio_put(PICO_DEFAULT_LED_PIN,0);
		sleep_ms(200);
	}
	reset_usb_boot(0,0);
}
