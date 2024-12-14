#include "definitions.h"


int pwm_lower_limit = 10;    //full range 0 - 255
int pwm_upper_limit = 63;  //full range

Encoder encoder_sx = Encoder(ENCODER_SX_A, ENCODER_SX_B);
Encoder	encoder_dx = Encoder(ENCODER_DX_A, ENCODER_DX_B);

Motor motor_sx = Motor(encoder_sx, MOTOR_SX_FORWARD, MOTOR_SX_BACKWARD, MOTOR_SX_PWM, pwm_lower_limit, pwm_upper_limit);
Motor motor_dx = Motor(encoder_dx, MOTOR_DX_FORWARD, MOTOR_DX_BACKWARD, MOTOR_DX_PWM, pwm_lower_limit, pwm_upper_limit);

// MPU6050_QMC5883L imu;
// MPU6050 imu;  // 0x68
// QMC5883L imu;  //
BMI160 imu;  // 0x68

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
HardwareSerial LidarSerial(UART_NUM_0); // TX 17, RX 16
LDS_RPLIDAR_A1 lidar;


void init_i2c() {
	Serial.println("INIT I2C");

	Wire.begin(I2C_SDA, I2C_SCL);
#ifdef WIRE_HAS_TIMEOUT
	Wire.setWireTimeout(30000);
	Serial.println("Timeout");
#endif
}

void init_imu() {
	Serial.println("\t - INIT IMU");
	int err = 1;
	int old_error = 0;
	while (err != 0) {
		err = imu.init(calib, IMU_ADDR);

		if (err != old_error) {
			old_error = err;
			Serial.print("Error initializing IMU: ");
			Serial.println(err);
		}

		if (err != 0)
			delay(500);
	}
}

void init_serial_lidar() {
	Serial.println("Init LidarSerial");
	LidarSerial.begin(115200, SERIAL_8N1); // Use default GPIO TX 17, RX 16
	// Assign TX, RX pins
	// LidarSerial.begin(baud_rate, SERIAL_8N1, rxPin, txPin);
	// Details https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h
	// Tutorial https://www.youtube.com/watch?v=eUPAoP7xC7A
	while (LidarSerial.read() >= 0);
}

int lidar_serial_read_callback() {
	return LidarSerial.read();
//  int ch = LidarSerial.read();
//  if (ch != -1)
//    Serial.println(ch);
//  return ch;
}

size_t lidar_serial_write_callback(const uint8_t* buffer, size_t length) {
	return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
	static int i = 0;

	if ((i++ % 20 == 0) || scan_completed) {
		Serial.print(i);
		Serial.print(' ');
		Serial.print(distance_mm);
		Serial.print(' ');
		Serial.print(angle_deg);
		if (scan_completed)
			Serial.println('*');
		else
			Serial.println();
	}
}

void lidar_info_callback(LDS::info_t code, String info) {
	Serial.print("LiDAR info ");
	Serial.print(lidar.infoCodeToString(code));
	Serial.print(": ");
	Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
	Serial.print("LiDAR error ");
	Serial.print(lidar.resultCodeToString(code));
	Serial.print(": ");
	Serial.println(aux_info);
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
	int pin = LDS_MOTOR_PWM_PIN;

	if (value <= -3) { //LDS::DIR_INPUT) {
		// Configure pin direction
		if ((int) value == LDS::DIR_OUTPUT_PWM) {
			//ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ, LDS_MOTOR_PWM_BITS);
			//ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
		} else
			pinMode(pin, ((int) value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
		return;
	}

	if ((int) value < LDS::VALUE_PWM) // set constant output
		digitalWrite(pin, ((int) value == LDS::VALUE_HIGH) ? HIGH : LOW);
	else { // set PWM duty cycle
		int pwm_value = ((1 << LDS_MOTOR_PWM_BITS) - 1) * value;
		ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
	}
}

void lidar_packet_callback(uint8_t* packet, uint16_t length, bool scan_completed) {
	return;
}

void init_lidar() {
	lidar.setScanPointCallback(lidar_scan_point_callback);
	lidar.setPacketCallback(lidar_packet_callback);
	lidar.setSerialWriteCallback(lidar_serial_write_callback);
	lidar.setSerialReadCallback(lidar_serial_read_callback);
	lidar.setMotorPinCallback(lidar_motor_pin_callback);
	lidar.init();

	LDS::result_t result = lidar.start();
	Serial.print("LiDAR start() result: ");
	Serial.println(lidar.resultCodeToString(result));

	if (result < 0)
		Serial.println("Is the LiDAR connected to ESP32?");
}

void init_motors() {
	Serial.println("XXXXXXXXXXXXXXXXXXXXX");
	Serial.println(encoder_dx.get_interrupts());

	motor_sx.init(KP, KI, KD);
	motor_dx.init(KP, KI, KD);
}