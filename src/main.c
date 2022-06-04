#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>



#define ULTRASONIC_TRIGGER_PIN 10
#define ULTRASONIC_PIN_TRIGGER_PULSE_US 5
#define ULTRASONIC_PIN_COUNT 8
#define ULTRASONIC_PIN_OFFSET 2
#define ULTRASONIC_MAX_DISTANCE 75
#define ULTRASONIC_MAX_DISTANCE_TIME ULTRASONIC_DISTANCE_TO_TIME(ULTRASONIC_MAX_DISTANCE)
#define ULTRASONIC_SOUND_SPEED_FACTOR 0.0343f
#define ULTRASONIC_DISTANCE_TO_TIME(x) ((uint32_t)((x)/ULTRASONIC_SOUND_SPEED_FACTOR*2))
#define ULTRASONIC_TRANSITION_VALUES(old,new) ((uint32_t)((old)*0.75f+(new)*0.25f))

#define ADXL345_BAUDRATE 400000
#define ADXL345_REGISTER_DEVICE_ID 0x00
#define ADXL345_REGISTER_OFFSETS 0x1e
#define ADXL345_REGISTER_POWER_CONTROL 0x2d
#define ADXL345_REGISTER_DATA 0x32
#define ADXL345_ENABLE_MEASUREMENT 0b00001000
#define ADXL345_I2C_BLOCK i2c0
#define ADXL345_I2C_SDA_PIN 20
#define ADXL345_I2C_SCL_PIN 21
#define ADXL343_I2C_ADDRESS 0x53
#define ADXL345_DEVICE_ID 0xe5
#define ADXL345_ACCELERATION_FACTOR (9.80665f/256)

#define MOTOR_PWM_FREQUENCY 60
#define MOTOR_PWM_WRAP 4096
#define MOTOR_PWM_1A 16
#define MOTOR_PWM_1B 17
#define MOTOR_PWM_SLICE_1 0
#define MOTOR_PWM_2A 18
#define MOTOR_PWM_2B 19
#define MOTOR_PWM_SLICE_2 1

#define ROBOT_RESET_BUTTON_PIN 22
#define ROBOT_WALL_MAX_DISTANCE 20



typedef struct _SENSOR_DATA{
	uint32_t ultrasonic[ULTRASONIC_PIN_COUNT];
	int16_t accelerometer[2];
} sensor_data_t;



static volatile sensor_data_t _sensors[2];
static volatile _Bool _sensor_offset=0;
static int16_t _acceleration_offsets[2];



static inline void _update_sensors(void){
	uint8_t i2c_data[6]={ADXL345_REGISTER_DATA};
	i2c_write_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,i2c_data,1,0);
	gpio_put(ULTRASONIC_TRIGGER_PIN,1);
	uint32_t end=time_us_32()+ULTRASONIC_PIN_TRIGGER_PULSE_US;
	uint32_t mask=((1<<ULTRASONIC_PIN_COUNT)-1)<<ULTRASONIC_PIN_OFFSET;
	uint32_t last=0;
	uint32_t start_time[ULTRASONIC_PIN_COUNT];
	_Bool idx=!_sensor_offset;
	for (unsigned int i=0;i<ULTRASONIC_PIN_COUNT;i++){
		_sensors[idx].ultrasonic[i]=ULTRASONIC_TRANSITION_VALUES(_sensors[!idx].ultrasonic[i],ULTRASONIC_MAX_DISTANCE_TIME);
	}
	while (time_us_32()<end);
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	end=time_us_32()+ULTRASONIC_MAX_DISTANCE_TIME*4;
	do{
		uint32_t time=time_us_32();
		if (time>=end){
			break;
		}
		uint32_t state=gpio_get_all()&mask;
		uint32_t change=state^last;
		if (!change){
			continue;
		}
		last=state;
		do{
			uint32_t i=__builtin_ctz(change);
			uint32_t bit=1<<i;
			change&=~bit;
			i-=ULTRASONIC_PIN_OFFSET;
			if (state&bit){
				start_time[i]=time;
			}
			else{
				time-=start_time[i];
				_sensors[idx].ultrasonic[i]=ULTRASONIC_TRANSITION_VALUES(_sensors[!idx].ultrasonic[i],(time>ULTRASONIC_MAX_DISTANCE_TIME?ULTRASONIC_MAX_DISTANCE_TIME:time));
				mask&=~bit;
			}
		} while (change);
	} while (mask);
	i2c_read_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,i2c_data,6,0);
	_sensors[idx].accelerometer[0]=((int16_t*)i2c_data)[0]+_acceleration_offsets[0];
	_sensors[idx].accelerometer[1]=((int16_t*)i2c_data)[2]+_acceleration_offsets[1];
	_sensor_offset=idx;
}



static inline void _drive_motors(int32_t left,int32_t right){
	pwm_set_both_levels(MOTOR_PWM_SLICE_1,(left>0?left:0),(left<0?-left:0));
	pwm_set_both_levels(MOTOR_PWM_SLICE_2,(right>0?right:0),(right<0?-right:0));
}



static inline void _init_led(void){
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN,1);
	sleep_ms(200);
	gpio_put(PICO_DEFAULT_LED_PIN,0);
	sleep_ms(200);
	gpio_put(PICO_DEFAULT_LED_PIN,1);
	sleep_ms(200);
	gpio_put(PICO_DEFAULT_LED_PIN,0);
	sleep_ms(200);
}



static inline void _init_reset_pin(void){
	gpio_init(ROBOT_RESET_BUTTON_PIN);
	gpio_set_dir(ROBOT_RESET_BUTTON_PIN,GPIO_IN);
}



static inline void _init_ultrasonic(void){
	gpio_init(ULTRASONIC_TRIGGER_PIN);
	gpio_set_dir(ULTRASONIC_TRIGGER_PIN,GPIO_OUT);
	gpio_put(ULTRASONIC_TRIGGER_PIN,0);
	for (uint32_t i=0;i<ULTRASONIC_PIN_COUNT;i++){
		gpio_init(i);
		gpio_set_dir(i,GPIO_IN);
		_sensors[0].ultrasonic[i]=ULTRASONIC_MAX_DISTANCE_TIME;
		_sensors[1].ultrasonic[i]=ULTRASONIC_MAX_DISTANCE_TIME;
	}
}



static inline uint8_t _init_accelerometer(void){
	i2c_init(ADXL345_I2C_BLOCK,ADXL345_BAUDRATE);
	gpio_set_function(ADXL345_I2C_SDA_PIN,GPIO_FUNC_I2C);
	gpio_set_function(ADXL345_I2C_SCL_PIN,GPIO_FUNC_I2C);
	gpio_set_pulls(ADXL345_I2C_SDA_PIN,1,0);
	gpio_set_pulls(ADXL345_I2C_SCL_PIN,1,0);
	uint8_t data[6]={ADXL345_REGISTER_DEVICE_ID};
	i2c_write_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,data,1,0);
	i2c_read_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,data,1,0);
	if (data[0]!=ADXL345_DEVICE_ID){
		return 0;
	}
	data[0]=ADXL345_REGISTER_POWER_CONTROL;
	data[1]=ADXL345_ENABLE_MEASUREMENT;
	i2c_write_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,data,2,0);
	data[0]=ADXL345_REGISTER_OFFSETS;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	i2c_write_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,data,4,0);
	data[0]=ADXL345_REGISTER_DATA;
	i2c_write_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,data,1,0);
	i2c_read_blocking(ADXL345_I2C_BLOCK,ADXL343_I2C_ADDRESS,data,6,0);
	_acceleration_offsets[0]=-((int16_t*)data)[0];
	_acceleration_offsets[1]=-((int16_t*)data)[2];
	return 1;
}



static inline void _init_motors(void){
	gpio_init(MOTOR_PWM_1A);
	gpio_init(MOTOR_PWM_1B);
	gpio_init(MOTOR_PWM_2A);
	gpio_init(MOTOR_PWM_2B);
	gpio_set_function(MOTOR_PWM_1A,GPIO_FUNC_PWM);
	gpio_set_function(MOTOR_PWM_1B,GPIO_FUNC_PWM);
	gpio_set_function(MOTOR_PWM_2A,GPIO_FUNC_PWM);
	gpio_set_function(MOTOR_PWM_2B,GPIO_FUNC_PWM);
	pwm_config pwm_cfg=pwm_get_default_config();
	pwm_config_set_clkdiv(&pwm_cfg,clock_get_hz(clk_sys)/((float)(MOTOR_PWM_FREQUENCY*MOTOR_PWM_WRAP)));
	pwm_config_set_wrap(&pwm_cfg,MOTOR_PWM_WRAP);
	pwm_init(MOTOR_PWM_SLICE_1,&pwm_cfg,true);
	pwm_init(MOTOR_PWM_SLICE_2,&pwm_cfg,true);
	pwm_set_both_levels(MOTOR_PWM_SLICE_1,0,0);
	pwm_set_both_levels(MOTOR_PWM_SLICE_2,0,0);
	pwm_set_enabled(MOTOR_PWM_SLICE_1,1);
	pwm_set_enabled(MOTOR_PWM_SLICE_2,1);
	_drive_motors(0,0);
}



static void _thread(void){
	_drive_motors(MOTOR_PWM_WRAP*3/4,MOTOR_PWM_WRAP*3/4);
	while (1){
		if (_sensors[_sensor_offset].ultrasonic[4]<ULTRASONIC_DISTANCE_TO_TIME(ROBOT_WALL_MAX_DISTANCE)||_sensors[_sensor_offset].ultrasonic[5]<ULTRASONIC_DISTANCE_TO_TIME(ROBOT_WALL_MAX_DISTANCE)){
			gpio_put(PICO_DEFAULT_LED_PIN,1);
			_drive_motors(-MOTOR_PWM_WRAP/2,MOTOR_PWM_WRAP/2);
			uint32_t end=time_us_32()+500;
			while (_sensors[_sensor_offset].ultrasonic[4]<ULTRASONIC_DISTANCE_TO_TIME(ROBOT_WALL_MAX_DISTANCE)||_sensors[_sensor_offset].ultrasonic[5]<ULTRASONIC_DISTANCE_TO_TIME(ROBOT_WALL_MAX_DISTANCE)||time_us_32()<end);
			gpio_put(PICO_DEFAULT_LED_PIN,0);
			_drive_motors(MOTOR_PWM_WRAP*3/4,MOTOR_PWM_WRAP*3/4);
		}
	}
}



int main(){
	_init_led();
	_init_reset_pin();
	_init_ultrasonic();
	if (_init_accelerometer()){
		_init_motors();
		multicore_launch_core1(_thread);
		while (!gpio_get(ROBOT_RESET_BUTTON_PIN)){
			_update_sensors();
		}
		multicore_reset_core1();
	}
	_drive_motors(0,0);
	for (unsigned int i=0;i<10;i++){
		gpio_put(PICO_DEFAULT_LED_PIN,i&1);
		sleep_ms(50);
	}
	reset_usb_boot(0,0);
}
