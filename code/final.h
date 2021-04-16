#ifndef final_H_
#define final_H_
#include "Arduino.h"

#include "Timer1.h"
#include "Timer2.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define FORWARD 0x09
#define BACKWARD 0x06
#define LEFT_U 0x0A
#define RIGHT_U 0x05
#define LEFT 0x08
#define RIGHT 0x01
#define STOP 0x00

#define Encoder(x) (x*14.6)//1cm이동시 14.6의 엔코더 펄스 발생

#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
}
#endif

void ADC_Compare(void);
unsigned char SensorD_read(void);
void SensorA_read(void);
void DAC_CH_Write(unsigned int ch, unsigned int da);
void DAC_setting(unsigned int data);
void infrared_init();
void Motor_mode(int da);
void Motor_Control(char da, unsigned int OC_value);
void Timer1_ISR();
void Timer2_ISR();
void serialEvent(int a);
void angle();
void motor_stop();
void Encoder_count_L();
void Encoder_count_R();
void x_cm(int x_data);
#endif
