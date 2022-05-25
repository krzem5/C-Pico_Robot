#include <stdio.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>
#include <hardware/clocks.h>



#define ULTRASONIC_TRIGGER_PIN 16
#define ULTRASONIC_PIN_COUNT 2
#define ULTRASONIC_MAX_DISTANCE 75
#define ULTRASONIC_MAX_DISTANCE_TIME ((uint32_t)(ULTRASONIC_MAX_DISTANCE/ULTRASONIC_SOUND_SPEED_FACTOR*2))
#define ULTRASONIC_SOUND_SPEED_FACTOR 0.0343f



static uint32_t _ultrasonic_values[ULTRASONIC_PIN_COUNT];



static void _get_distance(void){
	gpio_put(ULTRASONIC_TRIGGER_PIN,1);
	uint32_t end=time_us_32()+5;
	uint32_t mask=(1<<ULTRASONIC_PIN_COUNT)-1;
	uint32_t last=0;
	uint32_t start_time[ULTRASONIC_PIN_COUNT];
	for (unsigned int i=0;i<ULTRASONIC_PIN_COUNT;i++){
		_ultrasonic_values[i]=ULTRASONIC_MAX_DISTANCE_TIME;
	}
	while (time_us_32()<end);
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	end=time_us_32()+ULTRASONIC_MAX_DISTANCE_TIME*3;
	do{
		uint32_t time=time_us_32();
		if (time>=end){
			return;
		}
		uint32_t state=gpio_get_all()&mask;
		uint32_t change=state^last;
		if (change){
			last=state;
			do{
				uint32_t i=__builtin_ffs(change)-1;
				uint32_t bit=1<<i;
				change&=~bit;
				if (state&bit){
					start_time[i]=time;
				}
				else{
					time-=start_time[i];
					_ultrasonic_values[i]=(time>ULTRASONIC_MAX_DISTANCE_TIME?ULTRASONIC_MAX_DISTANCE_TIME:time);
					mask&=~bit;
				}
			} while (change);
		}
	} while (mask);
}



int main(){
	stdio_init_all();
	stdio_usb_init();
	gpio_init(ULTRASONIC_TRIGGER_PIN);
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(ULTRASONIC_TRIGGER_PIN,GPIO_OUT);
	gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
	for (uint32_t i=0;i<ULTRASONIC_PIN_COUNT;i++){
		gpio_init(i);
		gpio_set_dir(i,GPIO_IN);
	}
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	while (getchar_timeout_us(1)==PICO_ERROR_TIMEOUT){
		if (stdio_usb_connected()){
			gpio_put(PICO_DEFAULT_LED_PIN,0);
		}
		else{
			gpio_put(PICO_DEFAULT_LED_PIN,1);
		}
		uint32_t start=time_us_32();
		_get_distance();
		uint32_t end=time_us_32();
		printf("Distance: %.2f, %.2f -> %u\n",_ultrasonic_values[0]*ULTRASONIC_SOUND_SPEED_FACTOR/2,_ultrasonic_values[1]*ULTRASONIC_SOUND_SPEED_FACTOR/2,(end-start)/1000);
	}
	reset_usb_boot(0,0);
	return 0;
}
