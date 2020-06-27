#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <IRremote.h>

unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;

//--- Simple Moving Average Globals ------------------------------------------*/
const int samples = 15;
int a_x_readings[samples];
int a_y_readings[samples];
int a_z_readings[samples];
long int a_read_index = 0;
long int a_read_total[3] = {0, 0, 0};
long int a_read_ave[3] = {0, 0, 0};

/*--- IMU Globals ------------------------------------------------------------*/
float rad_to_degrees = 57.29577951f;
float degrees_to_rad = 0.017453293f;
double lsb_coefficient = (1.0f / 32.8f); // see datasheet
float roll, pitch, yaw;
long g_drift[3];
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float correction_gain = 0.2f;

/*--- DEBUGGING --------------------------------------------------------------*/
// This method prints the time taken from the beginning of the scan to the time this method is envocked. In order not to kill performance, this is only printed every idk 100000 microseconds.
void debug_loopTime(){
  if (elapsed_time - last_time_print > 100000){
    Serial.print(micros() - elapsed_time);
    Serial.print("\n");
    last_time_print = micros();
  }
}

/*--- SETUP MPU --------------------------------------------------------------*/
void setup_mpu(){
  // Activate the MPU-6050
  // 0x68 = Registry address of mpu6050
  // 0x6B = Send starting register
  // 0x00 = Tell the MPU not to be asleep
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Configure the accelerometer (+/-8g)
  // 0x68 = Registry address of mpu6050
  // 0x1C = Registry address of accelerometer
  // 0x10 = Full scale range of accelerometer (data sheet)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // Configure the gyro (500dps full scale
  // 0x68 = Registry address of mpu6050
  // 0x1B = Registry address of gyroscope
  //        0x08 = 500 degree / sec Range of the gyro in degree/sec (data sheet)
  //        0x10 = 1000 degree / sec range
  // 0x12 = 2000 degree / sec range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();
}

/*--- READ MPU  --------------------------------------------------------------*/
void read_mpu(int ** sensor_output_array){

  int array_size = 10;
  *sensor_output_array = (int*) malloc(sizeof(int) * array_size);
  /* Access the accellerometer register and requst
  14 bits. Assign each high and low bit to a variable. */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while(Wire.available() < 14){}; // Wait for all of the bits to be recieved:
  // Assign values to each element of the array:
  (*sensor_output_array)[0] = Wire.read()<<8|Wire.read(); // a_x
  (*sensor_output_array)[1] = Wire.read()<<8|Wire.read(); // a_y
  (*sensor_output_array)[2] = Wire.read()<<8|Wire.read(); // a_z
  (*sensor_output_array)[3] = Wire.read()<<8|Wire.read(); // temp
  (*sensor_output_array)[4] = Wire.read()<<8|Wire.read(); // g_x
  (*sensor_output_array)[5] = Wire.read()<<8|Wire.read(); // g_y
  (*sensor_output_array)[6] = Wire.read()<<8|Wire.read(); // g_z
}

/*--- DATA PROCESSING --------------------------------------------------------*/
// Simple moving average filter. This method smoothes out the noisey accelerometer data using a simple moving average filter. It isn't too expensive. Be careful when setting the number of samples: Too many samples will lead to a large time-delay, too few samples will lead to a negligible smoothing effect.
void accel_data_processing(int * sensor_data[]){  //Simple moving average filter
  a_read_total[0] -= a_x_readings[a_read_index];
  a_read_total[1] -= a_y_readings[a_read_index];
  a_read_total[2] -= a_z_readings[a_read_index];
  a_x_readings[a_read_index] = (*sensor_data)[0];
  a_y_readings[a_read_index] = (*sensor_data)[1];
  a_z_readings[a_read_index] = (*sensor_data)[2];
  a_read_total[0] += a_x_readings[a_read_index];
  a_read_total[1] += a_y_readings[a_read_index];
  a_read_total[2] += a_z_readings[a_read_index];
  a_read_index += 1;
  if (a_read_index >= samples){
    a_read_index = 0;
  }
  a_read_ave[0] = a_read_total[0] / samples;
  a_read_ave[1] = a_read_total[1] / samples;
  a_read_ave[2] = a_read_total[2] / samples;
}

// Remove the average gyroscope drift / offset (recorded in the calibration method) from the gyroscope data that is recorded during each scan.
void gyro_data_processing(int * sensor_data[]){
  (*sensor_data)[4] -= g_drift[0];
  (*sensor_data)[5] -= g_drift[1];
  (*sensor_data)[6] -= g_drift[2];
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
// A cheap way to find the inverse squareroot of a number.
float invSqrt( float number ){
    union {
        float f;
        uint32_t i;
    } conv;

    float x2;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    conv.f  = number;
    conv.i  = 0x5f3759df - ( conv.i >> 1 );
    conv.f  = conv.f * ( threehalfs - ( x2 * conv.f * conv.f ) );
    return conv.f;
}

// Calculate attitude during runtime.
void calculate_attitude(int sensor_data[]){
  /*--- Madgwick Filter ------------------------------------------------------*/
  // Heavily based off of the Arduino Madgwick AHRS library by Paul Stoffregen
  float recip_norm;

  float ax = sensor_data[0];
  float ay = sensor_data[1];
  float az = sensor_data[2];
  float gx = sensor_data[4] * (lsb_coefficient) * (1.09) * degrees_to_rad;
  float gy = sensor_data[5] * (lsb_coefficient) * (1.09) * degrees_to_rad;
  float gz = sensor_data[6] * (lsb_coefficient) * (1.09) * degrees_to_rad;

  float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
  float qDot2 = 0.5f * (q0*gx + q2*gz - q3*gy);
  float qDot3 = 0.5f * (q0*gy - q1*gz + q3*gx);
  float qDot4 = 0.5f * (q0*gz + q1*gy - q2*gx);

  /* If accelerometer values are valid (ie don't lead to NAN) then correct gyro
  values */
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

  	// Normalise accelerometer measurement
  	recip_norm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recip_norm;
  	ay *= recip_norm;
  	az *= recip_norm;

  	float _2q0 = 2.0f * q0;
		float _2q1 = 2.0f * q1;
  	float _2q2 = 2.0f * q2;
  	float _2q3 = 2.0f * q3;
		float _4q0 = 4.0f * q0;
  	float _4q1 = 4.0f * q1;
  	float _4q2 = 4.0f * q2;
  	float _8q1 = 8.0f * q1;
  	float _8q2 = 8.0f * q2;
  	float q0q0 = q0 * q0;
  	float q1q1 = q1 * q1;
  	float q2q2 = q2 * q2;
  	float q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  	float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  	float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
  	recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
  	s0 *= recip_norm;
  	s1 *= recip_norm;
  	s2 *= recip_norm;
  	s3 *= recip_norm;

  	// Apply feedback step
  	qDot1 -= correction_gain * s0;
  	qDot2 -= correction_gain * s1;
  	qDot3 -= correction_gain * s2;
		qDot4 -= correction_gain * s3;
	}

  q0 += qDot1 * sample_time;
  q1 += qDot2 * sample_time;
  q2 += qDot3 * sample_time;
  q3 += qDot4 * sample_time;

  recip_norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recip_norm;
  q1 *= recip_norm;
  q2 *= recip_norm;
  q3 *= recip_norm;

  roll = atan2f(2*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * rad_to_degrees - 1.0f;
  pitch = asinf(2.0f * (q0*q2 - q1*q3)) * rad_to_degrees + 3.0f;
  yaw = atan2f(2*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * rad_to_degrees;
}

/*--- CALIBRATE IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* THE IMU MUST NOT BE MOVED DURING STARTUP */

  /*--- Simple Moving ave Setup ---*/
  for (int i = 0; i < samples; i++){
    a_x_readings[i] = 0;
    a_y_readings[i] = 0;
    a_z_readings[i] = 0;
  }

  /*--- Calibrate gyroscope data and initial attitude: ---*/
  int cal_count = 750;
  Serial.print("\nCalibrating \n");
  for (int i = 0; i < cal_count; i ++){
    sample_time = (micros() - elapsed_time) / 1000000.0f;
    elapsed_time = micros();

    // Print the loading bar blips n times
    if(i % 50 == 0) { Serial.print("-"); }

    // Collect data from MPU
    int * data_xyzt;
    read_mpu(&data_xyzt);

    g_drift[0] += data_xyzt[4];
    g_drift[1] += data_xyzt[5];
    g_drift[2] += data_xyzt[6];

    accel_data_processing(&data_xyzt);

    free(data_xyzt); // Clear dynamic memory allocation

    delay(3);
  }
  // Find the averages drift / offset of the raw gyroscope data:
  g_drift[0] /= cal_count;
  g_drift[1] /= cal_count;
  g_drift[2] /= cal_count;
}

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  Serial.begin(2000000);
  Wire.begin();

  // Calibrate imu
  setup_mpu();
  calibrate_imu();
}

/*--- MAIN -------------------------------------------------------------------*/
void loop(){
  sample_time = (micros() - elapsed_time) / 1000000.0f;
  elapsed_time = micros();

  //IMU
  int * data_xyzt;
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);
  debug_loopTime();
  calculate_attitude(data_xyzt);
  debug_loopTime();
  //debug_loopTime();
  free(data_xyzt);  // Clear allocated memory for data array.

  // REFRESH RATE
  while (micros() - elapsed_time < 5500);
  // if (micros() - elapsed_time > 5500){  //Freeze if the loop takes too long
  //   while(true);
  // }
}
