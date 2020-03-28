#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <IRremote.h>

#define ACTIVATED HIGH
unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;
/*--- Propeller Servos -------------------------------------------------------*/
Servo right_prop;
Servo left_prop;
double throttle = 1100;
int button_state = 0;
int previous_time_pressed;
bool start_motors = false;

//--- Simple Moving Average Globals ------------------------------------------*/
const int samples = 5;
int a_x_readings[samples];
int a_y_readings[samples];
int a_z_readings[samples];
long int a_read_index = 0;
long int a_read_total[3] = {0, 0, 0};
long int a_read_ave[3] = {0, 0, 0};

/*--- Time Control -----------------------------------------------------------*/
int refresh_rate = 250;
float dt = 1 / refresh_rate;
const float loop_micros = (dt) * 1000000;

/*--- IMU Globals ------------------------------------------------------------*/
float rad_to_degrees = 57.29577951f;
float degrees_to_rad = 0.017453293f;
double lsb_coefficient = (1.0f / 32.8f);
float roll, pitch, yaw;
long g_cal[3];
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float correction_gain = 0.2f;

/*--- PID Globals ------------------------------------------------------------*/
float pid, pwm_right, pwm_left, error, previous_error, previous_roll;
float pid_p = 0, pid_i = 0, pid_d = 0;
float k_p = 1.0f; //3.5
float k_i = 0.0f; //
float k_d = 0.2f;  //0.85
// float k_p = 1.4;
// float k_i = 1.82;
// float k_d = 1.04;
float desired_angle = 0.0;

/*--- REMOTE CONTROL ---------------------------------------------------------*/
IRrecv irrecv(12);    // IR reciver digital input to pin 12.
decode_results results;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;

/*--- DEBUGGING --------------------------------------------------------------*/
void debugging(){
  int mode = 1;

  if (elapsed_time - last_time_print > 20000){
    if(mode == 1){
        Serial.print("Roll: ");
      Serial.print(roll);

      Serial.print(" - Pitch: ");
      Serial.print(pitch);

      Serial.print(" - pwm left: ");
      Serial.print(pwm_left);

      Serial.print(" - pwm right: ");
      Serial.print(pwm_right);

      Serial.print(" - PID: ");
      Serial.print(pid);

      Serial.print(" - Run Motors?: ");
      Serial.print(start_motors);

      Serial.print(" - k_p: ");
      Serial.print(k_p);
      Serial.print(" - k_i: ");
      Serial.print(k_i);
      Serial.print(" - k_d: ");
      Serial.print(k_d);

      Serial.print("\n");
    }
    if(mode == 2){
    //  Serial.print("");
    //  Serial.print();
      // Serial.print(" aPitch: ");
      // Serial.print(a_pitch);
      // Serial.print("gPitch: ");
      Serial.print(pitch);
      Serial.print(" ");
      // Serial.print(" - aRoll: ");
      // Serial.print(a_roll);
      // Serial.print(" - gRoll: ");
      Serial.print(roll);
      // Serial.print(" ");
      // Serial.print(90);
      // Serial.print(" ");
      // Serial.print(-90);
      Serial.print("\n");
    }
    if(mode == 3){
      // Serial.print(" gRoll: ");
      // Serial.print(roll);
      // Serial.print(" - gPitch: ");
      Serial.print(pitch);
      // Serial.print(" ");
      // Serial.print(90);
      // Serial.print(" ");
      // Serial.print(-90);
      // Serial.print(" - k_p: ");
      // Serial.print(k_p);
      // Serial.print(" - k_i: ");
      // Serial.print(k_i);
      // Serial.print(" - k_d: ");
      // Serial.print(k_d);
      Serial.print("\n");
    }
    last_time_print = micros();
  }
}

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

void gyro_data_processing(int * sensor_data[]){
  (*sensor_data)[4] -= g_cal[0];
  (*sensor_data)[5] -= g_cal[1];
  (*sensor_data)[6] -= g_cal[2];
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
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
  values (sort of like a complimentary filter): */
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
  //yaw = atan2f(2*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * rad_to_degrees;
}

/*--- CALIBRATE IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* THE IMU MUST NOT BE MOVED DURING SETUP */
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
    int * data_xyzt; read_mpu(&data_xyzt);
    g_cal[0] += data_xyzt[4];
    g_cal[1] += data_xyzt[5];
    g_cal[2] += data_xyzt[6];
    accel_data_processing(&data_xyzt);
    calculate_attitude(data_xyzt);
    free(data_xyzt); // Clear dynamic memory allocation
    delay(3);
  }
  // Find the average value of the data that was recorded above:
  g_cal[0] /= cal_count;
  g_cal[1] /= cal_count;
  g_cal[2] /= cal_count;
}

/*--- FLIGHT CONTROLLER ------------------------------------------------------*/
void flight_controller(){
  error = desired_angle - roll;

  // PROPORTIONAL COMPONENET
  pid_p = k_p * error;

  // INTEGRAL COMPONENT
  int k_i_thresh = 8;
  if (error < k_i_thresh && error > -k_i_thresh) {
    pid_i = pid_i * (k_i * error);
  }
  if (error > k_i_thresh && error < -k_i_thresh){
    pid_i = 0;
  }

  /* DERIVATIVE COMPONENT*/
  // Derivitive of the process variable (roll), NOT THE ERROR
  // Taking derivative of the error results in "Derivative Kick".
  // https://www.youtube.com/watch?v=KErYuh4VDtI
  pid_d = (-1.0f) * k_d * ((roll - previous_roll) / sample_time);
  // pid_d = k_d * ((error - previous_error) / sample_time);
  /* Sum the the components to find the total pid value. */
  pid = pid_p + pid_i + pid_d;

  /* Clamp the maximum & minimum pid values*/
  if (pid < -1000){
    pid = -1000;
  }
  if (pid > 1000){
    pid = 1000;
  }

  /* Calculate PWM width. */
  pwm_right = throttle + pid;
  pwm_left = throttle - pid;

  /* clamp the PWM values. */
  //----------Right---------//
  if (pwm_right < 1000){
    pwm_right = 1000;
  }
  if (pwm_right > 2000){
    pwm_right = 2000;
  }
    //----------Left---------//
  if (pwm_left < 1000){
    pwm_left = 1000;
  }
  if (pwm_left > 2000){
    pwm_left = 2000;
  }

  if (start_motors == true){
    right_prop.writeMicroseconds(pwm_right);
    left_prop.writeMicroseconds(pwm_left);
  } else{
    right_prop.writeMicroseconds(1000);
    left_prop.writeMicroseconds(1000);
  }

  previous_error = error;
  previous_roll = roll;
}

void motors_on_off(){
  button_state = digitalRead(13);
  long int elapsed_time = millis();
  if (button_state == ACTIVATED && start_motors == false && ((elapsed_time - previous_time_pressed) > 700)){
    start_motors = true;
    previous_time_pressed = millis();
  }
  else if(button_state == ACTIVATED && start_motors == true && ((elapsed_time - previous_time_pressed) > 700)){
    start_motors = false;
    previous_time_pressed = millis();
  }
}

void IR_remoteControl(){
  /*--- Store IR reciever remote value ---*/
  if(irrecv.decode(&results)){
    if(results.value != 4294967295){
      //Serial.println(results.value, HEX);

      /*--- Change PID gain values ---*/
      switch(results.value){

        case 1320906895:    // Power Button:
          if (start_motors){
            start_motors = false;
          } else{
            start_motors = true;
          }
          break;

        case 1320929335:    // Button 1
          k_p += 0.05;
          break;

        case 1320880375:    // Button 2
          k_d += 0.05;
          break;

        case 1320913015:    // Button 3
          k_i += 0.02;
          break;

        case 1320939535:    // Button 4
          k_p -= 0.02;
          break;

        case 1320890575:    // Button 5
          k_d -= 0.02;
          break;

        case 1320923215:    // Button 6
          k_i -= 0.02;
          break;

        case 1320887005:    // Up Button
          throttle += 50;
          break;

        case 1320925255:
          throttle -= 50;   // Down Button
          break;

        default:
          break;
      }
    }
    irrecv.resume();
  }
}

void change_setpoint(){

  if (receiver_input_channel_1 != 0){
    desired_angle = map(receiver_input_channel_1, 1000, 2000, 20, -20);
  }
}

void setup_interrupts(){
  // put your setup code here, to run once:
  PCICR |= (1 << PCIE0);  // Set OCIE0 to enable PCMSK0 to scan
  PCMSK0 |= (1 << PCINT0);  // set digital input 8 to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);  // etc
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
}

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  pinMode(7, INPUT);
  setup_interrupts();
  Serial.begin(2000000);
  Wire.begin();
  irrecv.enableIRIn();
  // Motors
  right_prop.attach(5);
  left_prop.attach(3);
  right_prop.writeMicroseconds(1000);
  left_prop.writeMicroseconds(1000);
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
  change_setpoint();
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);
  calculate_attitude(data_xyzt);
  free(data_xyzt);  // Clear allocated memory for data array.
  // FLIGHT CONTROLLER
  flight_controller();
  // DEBUGGING
  debugging();
  //CALIBRATION CONTROLS
  IR_remoteControl();
  //debug_loopTime();
  // REFRESH RATE
  while (micros() - elapsed_time < 5500);
  // if (micros() - elapsed_time > 5500){  //Freeze if the loop takes too long
  //   while(true);
  // }
}

ISR(PCINT0_vect){
  /*----- CHANNEL 1 -----*/
  if(last_channel_1 == 0 && PINB & B00000001){
    last_channel_1 = 1;
    timer_1 = micros();
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){
    last_channel_1 = 0;
    receiver_input_channel_1 = micros() - timer_1;
  }
}
