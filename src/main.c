#include <stdio.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>



#define ULTRASONIC_TRIGGER_PIN 16
#define ULTRASONIC_ECHO_PIN 0
#define ULTRASONIC_MAX_DISTANCE 450
#define ULTRASONIC_SOUND_SPEED_FACTOR 0.0343f
#define ULTRASONIC_MAX_WAIT_TICKS 550000
#define ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT 5



static uint32_t _ultrasonic_smooth[1<<ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT];
static uint32_t _ultrasonic_smooth_sum;
static uint32_t _ultrasonic_smooth_index;



static uint32_t _get_distance(void){
	sleep_us(2);
	gpio_put(ULTRASONIC_TRIGGER_PIN,1);
	sleep_us(5);
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	uint32_t ticks=ULTRASONIC_MAX_WAIT_TICKS;
	while (!gpio_get(ULTRASONIC_ECHO_PIN)){
		ticks--;
		if (!ticks){
			goto _return;
		}
	}
	uint32_t time=time_us_32();
	while (gpio_get(ULTRASONIC_ECHO_PIN)){
		ticks--;
		if (!ticks){
			goto _return;
		}
	}
	time=time_us_32()-time;
	_ultrasonic_smooth_sum+=time-_ultrasonic_smooth[_ultrasonic_smooth_index];
	_ultrasonic_smooth[_ultrasonic_smooth_index]=time;
	_ultrasonic_smooth_index=(_ultrasonic_smooth_index+1)&((1<<ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT)-1);
_return:
	return _ultrasonic_smooth_sum>>ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT;
}



int main(){
	for (uint32_t i=0;i<(1<<ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT);i++){
		_ultrasonic_smooth[i]=0;
	}
	_ultrasonic_smooth_sum=0;
	_ultrasonic_smooth_index=0;
	stdio_init_all();
	stdio_usb_init();
	gpio_init(ULTRASONIC_TRIGGER_PIN);
	gpio_init(ULTRASONIC_ECHO_PIN);
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(ULTRASONIC_TRIGGER_PIN,GPIO_OUT);
	gpio_set_dir(ULTRASONIC_ECHO_PIN,GPIO_IN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	while (getchar_timeout_us(1)==PICO_ERROR_TIMEOUT){
		if (stdio_usb_connected()){
			gpio_put(PICO_DEFAULT_LED_PIN,0);
		}
		else{
			gpio_put(PICO_DEFAULT_LED_PIN,1);
		}
		uint32_t dist=_get_distance();
		printf("Distance: %.2f\n",dist*ULTRASONIC_SOUND_SPEED_FACTOR/2);
		sleep_ms(16);
	}
	reset_usb_boot(0,0);
	return 0;
}
