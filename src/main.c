#include <stdio.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>



#define ULTRASONIC_TRIGGER_PIN 16
#define ULTRASONIC_PIN_COUNT_SHIFT 1
#define ULTRASONIC_MAX_DISTANCE 75
#define ULTRASONIC_MAX_DISTANCE_TIME ((uint32_t)(ULTRASONIC_MAX_DISTANCE/ULTRASONIC_SOUND_SPEED_FACTOR*2))
#define ULTRASONIC_SOUND_SPEED_FACTOR 0.0343f
#define ULTRASONIC_MAX_WAIT_TICKS 80000
#define ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT 5



static uint32_t _ultrasonic_smooth[1<<(ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT+ULTRASONIC_PIN_COUNT_SHIFT)];
static uint32_t _ultrasonic_smooth_sum[1<<ULTRASONIC_PIN_COUNT_SHIFT];
static uint32_t _ultrasonic_smooth_index[1<<ULTRASONIC_PIN_COUNT_SHIFT];



static void _get_distance(void){
	uint32_t mask=(1<<(1<<ULTRASONIC_PIN_COUNT_SHIFT))-1;
	uint32_t ticks=ULTRASONIC_MAX_WAIT_TICKS;
	uint32_t start_time[1<<ULTRASONIC_PIN_COUNT_SHIFT];
	for (unsigned int i=0;i<(1<<ULTRASONIC_PIN_COUNT_SHIFT);i++){
		start_time[i]=0;
	}
	sleep_us(2);
	gpio_put(ULTRASONIC_TRIGGER_PIN,1);
	sleep_us(5);
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	uint32_t last=0;
	while (mask){
		uint32_t state=gpio_get_all()&mask;
		uint32_t change=state^last;
		if (change){
			last=state;
			uint32_t time=time_us_32();
			while (change){
				uint32_t i=__builtin_ffs(change)-1;
				uint32_t bit=1<<i;
				change&=change-1;
				if (state&bit){
					start_time[i]=time;
				}
				else{
					start_time[i]=(time-start_time[i])|0x80000000;
					mask&=~bit;
				}
			}
		}
		ticks--;
		if (!ticks){
			break;
		}
	}
	for (unsigned int i=0;i<(1<<ULTRASONIC_PIN_COUNT_SHIFT);i++){
		uint32_t time;
		if (start_time[i]>>31){
			time=start_time[i]&0x7fffffff;
			if (time>ULTRASONIC_MAX_DISTANCE_TIME){
				time=ULTRASONIC_MAX_DISTANCE_TIME;
			}
		}
		else{
			time=ULTRASONIC_MAX_DISTANCE_TIME;
		}
		uint32_t* ptr=_ultrasonic_smooth+(i<<ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT)+_ultrasonic_smooth_index[i];
		_ultrasonic_smooth_sum[i]+=time-(*ptr);
		*ptr=time;
		_ultrasonic_smooth_index[i]=(_ultrasonic_smooth_index[i]+1)&((1<<ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT)-1);
	}
}



int main(){
	for (uint32_t i=0;i<(1<<(ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT+ULTRASONIC_PIN_COUNT_SHIFT));i++){
		_ultrasonic_smooth[i]=ULTRASONIC_MAX_DISTANCE_TIME;
	}
	stdio_init_all();
	stdio_usb_init();
	gpio_init(ULTRASONIC_TRIGGER_PIN);
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(ULTRASONIC_TRIGGER_PIN,GPIO_OUT);
	gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
	for (uint32_t i=0;i<(1<<ULTRASONIC_PIN_COUNT_SHIFT);i++){
		_ultrasonic_smooth_sum[i]=ULTRASONIC_MAX_DISTANCE_TIME<<ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT;
		_ultrasonic_smooth_index[i]=0;
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
		printf("Distance: %.2f, %.2f -> %u\n",(_ultrasonic_smooth_sum[0]>>ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT)*ULTRASONIC_SOUND_SPEED_FACTOR/2,(_ultrasonic_smooth_sum[1]>>ULTRASONIC_SMOOTH_ARRAY_LENGTH_SHIFT)*ULTRASONIC_SOUND_SPEED_FACTOR/2,(end-start)/1000);
		sleep_ms(1);
	}
	reset_usb_boot(0,0);
	return 0;
}
