#include "final3.h"

MPU6050 mpu;

char buf[10];
int  Move_flag=0, Move_flag1=0, dir=0, input_flag=0, Timer_counter=0, over_flag=0;
signed int Angle_value=0, Angle_cur=0, Angle_standard=0, Angle_pre=0;
signed int data=0, Angle_offset=2;
int Motor[6] = {22,23,24,25,4,5};
int read_counter = 1;

int BUZZER=10;
unsigned char Buzzer = 0;
unsigned char Command = 0;

int mine_b[5]={0xE7, 0x81, 0x00, 0x81, 0xE7};
int compare_b[5]={0,0,0,0,0};

int pattern_ch=3;

//제어 변수
int count=0;
int check = 1;

//점멸 변수
int Front_LED = 10;
int Back_LED = 9;
int LED_state = 0;

//자이로센서 변수 선언
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
float standard;
int rotate_direction = 0;

//적외선센서 변수 선언
int S_DIN = 42, S_SCLK = 43, S_SYNCN = 44,IN_SEN_EN = 26;
int SensorA[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int SensorD[8] = {30,31,32,33,34,35,36,37};
int direction_data1=0;
unsigned int Buff_A[8] = {0,0,0,0,0,0,0,0};
unsigned int ADC_MAX[8] = {0,0,0,0,0,0,0,0};
unsigned int ADC_MIN[8] = {1023,1023,1023,1023,1023,1023,1023,1023};
unsigned int ADC_MEAN[8] = {0,0,0,0,0,0,0,0};
unsigned char Sensor_data = 0;

//거리이동을 위한 변수 선언
int ENCODER_CNT_L=0, ENCODER_CNT_R=0, ENCODER_value=0, ENCODER_data=0;
int  x_dir=0, x_tmp=0, i=0, x_Move_flag=0;
int move_check;

int initch=0;

void setup(){
	int i;

	pinMode(IN_SEN_EN,OUTPUT);
	pinMode(S_DIN,OUTPUT);
	pinMode(S_SCLK,OUTPUT);
	pinMode(S_SYNCN,OUTPUT);
	digitalWrite(S_SCLK,LOW);
	digitalWrite(S_SYNCN,HIGH);
	digitalWrite(IN_SEN_EN,HIGH);

	for(i=0;i<6;i++){
		pinMode(Motor[i],OUTPUT);
		digitalWrite(Motor[i],LOW);
	}

	attachInterrupt(6, Encoder_count_L, RISING);
	attachInterrupt(7, Encoder_count_R, RISING);

	pinMode(10, OUTPUT);
	pinMode(9, OUTPUT);
	digitalWrite(10, LOW);
	digitalWrite(9, LOW);

	for(i=0;i<8;i++)
	{
		pinMode(SensorD[i],INPUT);
	}
	Serial.begin(115200);
	DAC_setting(0x9000);
	for(i=0;i<8;i++)
	{
		DAC_CH_Write(i,255);
	}

	Wire.begin();

	Serial.println(F("Initializing I2C devices.."));
	mpu.initialize();
	Serial.println(F("Testing device connections.."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful"):F("MPU6050 connection failed"));
	Serial.println(F("Initializing DMP.."));
	Serial.write(0x0D);
	devStatus=mpu.dmpInitialize();

	if(devStatus==0){
		Serial.println(F("Enabling DMP.."));
		mpu.setDMPEnabled(true);
		packetSize=mpu.dmpGetFIFOPacketSize();
	}
	else{
		Serial.print("DMP Initialization failed(code");
		Serial.print(devStatus);
		Serial.println(")");
	}
	Serial.write(0x0D);
	Motor_Control('A', 100);

	//Timer1::set(1000000, Timer1_ISR);

	//Timer1::start();

	pinMode(Front_LED,OUTPUT);
	pinMode(Back_LED,OUTPUT);

	//Timer2::set(25000, Timer2_ISR);
	//Timer2::start();
	infrared_init();
}



void loop()
{
	int Speed_data_R=0, Speed_data_L=0, direction_data = 0;

	if(read_counter){
		Motor_Control('A',100);
		Sensor_data = SensorD_read();
	}

	switch(Sensor_data)
	{
		case 0xFF:
		case 0xE7: //11100111
		case 0xEF: //11101111
		case 0xF7: //11110111
		case 0xC7: //11000111
		case 0xE3: //11100011
		case 0xC3: //11000011
			direction_data = FORWARD;
			Speed_data_L = 35;
			Speed_data_R = 35;
			break;

		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
		case 0x08:
		case 0x09:
		case 0x0A:
		case 0x0B:
			Move_flag = 1;
			direction_data = STOP;
			Speed_data_L = 0;
			Speed_data_R = 0;
			break;

		default:
			Serial.println("Find pattern, comparing");
			direction_data=FORWARD;
			Speed_data_L = 35;
			Speed_data_R = 35;
			compare();
			break;
	}

	if(direction_data1 != direction_data)
	{
		direction_data1 = direction_data;
		Motor_Control('L',Speed_data_L);
		Motor_Control('R',Speed_data_R);
		Motor_mode(direction_data);
	}
	delay(5);


	if(Move_flag&&(rotate_direction%2==0)){
		check=1;
		Serial.println("Get in");
		Motor_mode(STOP);
		delay(1000);
		read_counter = 0;
		standard=0;

		while(check){
			while(1) {
				mpu.resetFIFO();
				mpuIntStatus = mpu.getIntStatus();
				fifoCount = mpu.getFIFOCount();
				if(mpuIntStatus & 0x02) {
					while(fifoCount < packetSize)	fifoCount = mpu.getFIFOCount();
					mpu.getFIFOBytes(fifoBuffer, packetSize);
					fifoCount -= packetSize;
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
					break;
				}
			delay(1000);
			}

			if(standard==0.0){
				standard = (180-(ypr[0] * 180/M_PI));
				Serial.print(" standard : ");
				Serial.println(standard);
				Motor_Control('L',150);
				Motor_Control('R',150);
				Motor_mode(RIGHT_U);
			}

			if(standard<270 && (180-(ypr[0]*180/M_PI)-5)<=(standard+90) && (180-(ypr[0]*180/M_PI)+5)>=(standard+90)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}

			if(standard>270 && (180-(ypr[0]*180/M_PI)-5)<=(standard-270) && (180-(ypr[0]*180/M_PI)+5)>=(standard-270)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}
		}
		delay(1000);

		x_cm(25);
		delay(1000);

		check = 1;
		standard = 0;
		while(check){
			while(1) {
				mpu.resetFIFO();
				mpuIntStatus = mpu.getIntStatus();
				fifoCount = mpu.getFIFOCount();
				if(mpuIntStatus & 0x02) {
					while(fifoCount < packetSize)	fifoCount = mpu.getFIFOCount();
					mpu.getFIFOBytes(fifoBuffer, packetSize);
					fifoCount -= packetSize;
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
					break;
				}
				delay(1000);
			}


			if(standard==0.0){
				standard = (180-(ypr[0] * 180/M_PI));
				Serial.print(" standard : ");
				Serial.println(standard);
				Motor_Control('L',150);
				Motor_Control('R',150);
				Motor_mode(RIGHT_U);
			}

			if(standard<270 && (180-(ypr[0]*180/M_PI)-5)<=(standard+90) && (180-(ypr[0]*180/M_PI)+5)>=(standard+90)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}

			if(standard>270 && (180-(ypr[0]*180/M_PI)-5)<=(standard-270) && (180-(ypr[0]*180/M_PI)+5)>=(standard-270)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}
		}
		delay(1000);

		x_cm(-25);
		rotate_direction++;
		//infrared_init();
		read_counter = 1;
		Move_flag=0;
	}

	else if(Move_flag&&(rotate_direction%2==1)){
		check=1;
		Serial.println("Get in 2");
		Motor_mode(STOP);
		delay(1000);
		read_counter = 0;
		standard = 0;

		while(check){
			while(1) {
				mpu.resetFIFO();
				mpuIntStatus = mpu.getIntStatus();
				fifoCount = mpu.getFIFOCount();
				if(mpuIntStatus & 0x02) {
					while(fifoCount < packetSize)	fifoCount = mpu.getFIFOCount();
					mpu.getFIFOBytes(fifoBuffer, packetSize);
					fifoCount -= packetSize;
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
					break;
				}
				delay(1000);
			}

			if(standard==0.0){
				standard = (180-(ypr[0] * 180/M_PI));
				Serial.println(standard);
				Motor_Control('L',150);
				Motor_Control('R',150);
				Motor_mode(LEFT_U);
			}

			if(standard<90 && (180-(ypr[0]*180/M_PI)-5)<=(standard+270) && (180-(ypr[0]*180/M_PI)+5)>=(standard+270)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}
			if(standard>90 && (180-(ypr[0]*180/M_PI)-5)<=(standard-90) && (180-(ypr[0]*180/M_PI)+5)>=(standard-90)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}
		}
		delay(1000);

		x_cm(25);
		delay(1000);

		check = 1;
		standard = 0;
		while(check){
			while(1) {
				mpu.resetFIFO();
				mpuIntStatus = mpu.getIntStatus();
				fifoCount = mpu.getFIFOCount();
				if(mpuIntStatus & 0x02) {
					while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
						 mpu.getFIFOBytes(fifoBuffer, packetSize);
						 fifoCount -= packetSize;
						 mpu.dmpGetQuaternion(&q, fifoBuffer);
						 mpu.dmpGetGravity(&gravity, &q);
						 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
						 break;
				}
				delay(1000);
			}

			if(standard==0.0){
				standard = (180-(ypr[0] * 180/M_PI));
				Serial.println(standard);
				Motor_Control('L',150);
				Motor_Control('R',150);
				Motor_mode(LEFT_U);
			}
			if(standard<90 && (180-(ypr[0]*180/M_PI)-5)<=(standard+270) && (180-(ypr[0]*180/M_PI)+5)>=(standard+270)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}
			if(standard>90 && (180-(ypr[0]*180/M_PI)-5)<=(standard-90) && (180-(ypr[0]*180/M_PI)+5)>=(standard-90)){
				Serial.println(standard);
				Serial.println(180-(ypr[0]*180/M_PI));
				Motor_mode(STOP);
				Motor_Control('L',0);
				Motor_Control('R',0);
				check=0;
			}
		}
		delay(1000);

		x_cm(-25);
		rotate_direction ++;
		//infrared_init();
		read_counter = 1;
		Move_flag=0;
	}


}

void ADC_Compare(void)
{
	int z;
	for(z=0;z<8;z++)
	{
		if(ADC_MAX[z] < Buff_A[z])

			ADC_MAX[z] = Buff_A[z];

		if(ADC_MIN[z] > Buff_A[z])

			ADC_MIN[z] = Buff_A[z];
	}
}

unsigned char SensorD_read(void)
{
	unsigned char data=0,z;
	for(z=0;z<8;z++)
	{
		data >>= 1;
		if(digitalRead(SensorD[z]))
			data |= 0x80;
	}
	return data;
}


void SensorA_read(void)
{
	int z;
	for(z=0;z<8;z++)
		Buff_A[z] = analogRead(SensorA[z]);
}


void Timer2_ISR()
{
	count++;
	if(count>=10000)
	{
		for(int i=0;i<6;i++){
			pinMode(Motor[i],OUTPUT);
			digitalWrite(Motor[i],LOW);
		}
	}
}

void DAC_CH_Write(unsigned int ch, unsigned int da)
{
	unsigned int data = ((ch<<12)&0x7000) | ((da<<4) & 0x0FF0);
	DAC_setting(data);
}

void DAC_setting(unsigned int data)
{
	int z;
	digitalWrite(S_SCLK,HIGH);
	delayMicroseconds(1);
	digitalWrite(S_SCLK,LOW);
	delayMicroseconds(1);
	digitalWrite(S_SYNCN,LOW);
	delayMicroseconds(1);
	for(z=16;z>0;z--)
	{
		digitalWrite(S_DIN,(data>>(z-1))&0x1);
		digitalWrite(S_SCLK,HIGH);
		delayMicroseconds(1);
		digitalWrite(S_SCLK,LOW);
		delayMicroseconds(1);
	}
	digitalWrite(S_SYNCN,HIGH);
}

void infrared_init()
{
	int z,error = 0;
	Serial.println(" infrared init Start");
	Motor_Control('A',160);
	while(1)
	{
		Motor_mode(LEFT_U);
		for(z=0;z<50;z++)
		{
			SensorA_read();
			ADC_Compare();
			Serial.print(" Infrared Read : ");
			Serial.println(z);
			delay(8);
		}
		Motor_mode(RIGHT_U);
		for(z=0;z<100;z++)
		{
			SensorA_read();
			ADC_Compare();
			Serial.print(" Infrared Read : ");
			Serial.println(z);
			delay(8);
		}
		Motor_mode(LEFT_U);
		for(z=0;z<62;z++)
		{
			delay(8);
		}
		Motor_mode(LEFT_U);
		Motor_mode(STOP);
		Serial.println("\n\r ADC MAX");

		for(z=0;z<8;z++)
		{
			Serial.print(" ");
			Serial.print(ADC_MAX[z]);
		}

		Serial.println("\n\r ADC MIN");

		for(z=0;z<8;z++)
		{
			Serial.print(" ");
			Serial.print(ADC_MIN[z]);
		}

		delay(1000);

		for(z=0;z<8;z++)
		{
			if((ADC_MAX[z]-ADC_MIN[z])<200)
			error++;
		}

		if(error == 0)
		{
			Serial.println("\n\r infrared init END");
			break;
		}

		else
		{
			error = 0;
			Serial.println("\n\r infrared init Restart");
			for(z=0;z<8;z++)
			{
				ADC_MAX[z] = 0;
				ADC_MIN[z] = 1023;
			}
		}
	}


	Serial.println("DAC Setting Start");
	for(z=0;z<8;z++)
	{
		ADC_MEAN[z] = (ADC_MAX[z]+ADC_MIN[z])/2;
		DAC_CH_Write(z,ADC_MEAN[z]/4);
	}
	Serial.println("DAC_Setting");

	for(z=0;z<8;z++)
	{
		Serial.print(" ");
		Serial.print(ADC_MEAN[z]);
	}

	Serial.println("\n\r DAC Setting End");
}

void Encoder_count_L(){
	ENCODER_CNT_L++;
}

void Encoder_count_R(){
	ENCODER_CNT_R++;
}

void Motor_mode(int da)
{
	int z;
	for(z=0;z<4;z++)
		digitalWrite(Motor[z],(da>>z) & 0x01);
}

void Motor_Control(char da, unsigned int OC_value)
{
	switch(da)
	{
		case 'L':
			analogWrite(Motor[4],OC_value);
			break;
		case 'R':
			analogWrite(Motor[5],OC_value);
			break;
		case 'A':
			analogWrite(Motor[4],OC_value);
			analogWrite(Motor[5],OC_value);
			break;
	}
}

void motor_stop(){
	for(int i=0;i<6;i++){
		pinMode(Motor[i],OUTPUT);
		digitalWrite(Motor[i],LOW);
	}
}

void x_cm(int x_data){
	Serial.println(" x_cm");
	x_Move_flag =1;
	Motor_Control('A',100);
	if(	x_Move_flag){
		if(x_data<0){
			x_dir=BACKWARD;
			x_data=-x_data;
		}
		else{
			x_dir=FORWARD;
		}
		x_tmp=x_data;
		ENCODER_value=Encoder(x_tmp);
		ENCODER_CNT_R=0;
		ENCODER_CNT_L=0;
		ENCODER_data=0;

		if(x_dir!=STOP){
			Motor_mode(x_dir);
			while(ENCODER_value>ENCODER_data){
				delay(1);
				ENCODER_data=(ENCODER_CNT_R+ENCODER_CNT_L)/2;
			}
			Motor_mode(STOP);
		}
		x_Move_flag=0;
		delay(1000);
	}
}


void s_pattern(){
	int i;
	Serial.println("Searching Pattern");
	for(i=0; i<5; i++){
		Serial.print("Sensor data : ");
		Serial.println(Sensor_data);
		Serial.println("Move 2cm");
		Sensor_data = SensorD_read();
		compare_b[i]=Sensor_data;
		delay(1000);
		x_cm(2);
	}
	return;
}

void compare(){
	int i, c_count=0;
	Serial.println("Comapring Pattern");
	s_pattern();

	for(i=0; i<5; i++){
		if(mine_b[i]==compare_b[i])//패턴을 비교
			c_count++;
	}

	Serial.print("c_count : ");
	Serial.println(c_count);

	if(c_count==5){//패턴이 5줄 모두 일치
		Serial.println("mine finded");
		Buzzer=1;
		Motor_mode(STOP);
		digitalWrite(BUZZER, Buzzer);
		delay(3000);
		Serial.println("BUZZER for 3 sec");
		Motor_mode(FORWARD);
		delay(1000);
		Serial.println("Restart search");
		return;
		//버저 3초 울림
	}

	else{//패턴이 일치하지 않으면
		Serial.println("no mine! restart search");
		Motor_mode(FORWARD);
		return;
	}
}
