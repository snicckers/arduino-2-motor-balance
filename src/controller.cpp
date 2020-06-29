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
const int sma_samples = 15;
int a_x_readings[sma_samples];
int a_y_readings[sma_samples];
int a_z_readings[sma_samples];
long int a_read_index = 0;
long int a_read_total[3] = {0, 0, 0};
long int a_read_ave[3] = {0, 0, 0};

/*--- IMU Globals ------------------------------------------------------------*/
float rad_to_degrees = 57.29577951f;
float degrees_to_rad = 0.017453293f;
double lsb_coefficient = (1.0f / 32.8f); // see datasheet
float roll, pitch, yaw;
long g_drift[3];
float q_0 = 1.0f;
float q_1 = 0.0f;
float q_2 = 0.0f;
float q_3 = 0.0f;
float correction_gain = 0.2f;

/*--- PID Globals ------------------------------------------------------------*/
float pid, pwm_right, pwm_left, error, previous_error, previous_roll;
float pid_p = 0, pid_i = 0, pid_d = 0;
float k_p = 3.78f; //1.4
float k_i = 0.05f; //1.82
float k_d = 0.85f;  //1.04
float desired_angle = 0.0;

/*--- Remote Control Globals -------------------------------------------------*/
IRrecv irrecv(12);    // IR reciver digital input to pin 12.
decode_results results;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;

/*--- Debugging --------------------------------------------------------------*/
void debugging(){
  int mode = 1;

  // Serial Print has a significant impact on performance. Only use it once every n scans.
  if (elapsed_time - last_time_print > 20000){
    if(mode == 1){  // Used to test controller tuning
      Serial.print("Roll: ");
      Serial.print(roll);

      Serial.print(" - Pitch: ");
      Serial.print(pitch);

      Serial.print(" - Pitch: ");
      Serial.print(yaw);

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

    if(mode == 2){  //  Used to test imu
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

    if(mode == 3){  // Used to test imu
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

    if (elapsed_time - last_time_print > 20000){
      if(mode == 4){
        Serial.print("Roll: ");
        Serial.print(roll);

        Serial.print(" - Pitch: ");
        Serial.print(pitch);

        Serial.print(" - Pitch: ");
        Serial.print(yaw);

        Serial.print("\n");
      }

    last_time_print = micros();
    }

    if (elapsed_time - last_time_print > 20000){
      last_time_print = micros();
    }
  }
}

// I don't have an oscilloscope so here you go:
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
// Simple moving average filter. This method smoothes out noisey accelerometer data. It isn't too expensive. Be careful when setting the number of sma_samples: A large sma_samples will lead to a large time-delay, too few sma_samples will lead to a negligible smoothing effect.
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
  if (a_read_index >= sma_samples){
    a_read_index = 0;
  }
  a_read_ave[0] = a_read_total[0] / sma_samples;
  a_read_ave[1] = a_read_total[1] / sma_samples;
  a_read_ave[2] = a_read_total[2] / sma_samples;
}

// Remove the average gyroscope drift / offset (recorded in the calibration method) from the gyroscope data that is recorded during each scan.
void gyro_data_processing(int * sensor_data[]){
  (*sensor_data)[4] -= g_drift[0];
  (*sensor_data)[5] -= g_drift[1];
  (*sensor_data)[6] -= g_drift[2];
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
// Cheapest/fastest inverse square root I could find (99.94% accurate to 1 / sqrt(x))
// Source: http://www.dbfinteractive.com/forum/index.php?topic=6269.0
float invSqrt( float x ){
    float xhalf = 0.5f*x;
    union {
        float x;
        int i;
    } u;
    u.x = x;
    u.i = 0x5f375a86 - (u.i >> 1);
    /* The next line can be repeated any number of times to increase accuracy */
    u.x = u.x * (1.5f - xhalf * u.x * u.x);
    return u.x;
}

// Calculate attitude during runtime.
void calculate_attitude(int sensor_data[]){
  /*--- Madgwick Filter ------------------------------------------------------*/
  float normalize;

  //Import and normalize accelerometer data
  float a_x = sensor_data[0];
  float a_y = sensor_data[1];
  float a_z = sensor_data[2];
  normalize = invSqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  a_x *= normalize; a_y *= normalize; a_z *= normalize;

  // 1.09 = fudge factor. g_x in radians / sec
  float g_x = sensor_data[4] * (lsb_coefficient) * (1.0) * degrees_to_rad;
  float g_y = sensor_data[5] * (lsb_coefficient) * (1.0) * degrees_to_rad;
  float g_z = sensor_data[6] * (lsb_coefficient) * (1.0) * degrees_to_rad;

  // q_dot = 0.5 angular velocity rotation maxtrix * q.
  // Reference: A New Quaternion-Based Kalman Filter for Real-Time Attitude Estimation Using the Two-Step Geometrically-Intuitive Correction Algorithm. Equation 32 in section 2.3.1

  float qDot_0 = 0.5f*(-q_1*g_x - q_2*g_y - q_3*g_z);
  float qDot_1 = 0.5f*(q_0*g_x + q_2*g_z - q_3*g_y);
  float qDot_2 = 0.5f*(q_0*g_y + q_2*g_x - q_1*g_z);
  float qDot_3 = 0.5f*(q_0*g_z + q_1*g_y - q_2*g_x);

  /* References:
      1. https://nitinjsanket.github.io/tutorials/attitudeest/madgwick - (primary)
      2. Estimation of IMU and MARG orientation using a gradient descent algorithm (Sebastian O.H. Madgwick, Andrew J.L. Harrison, Ravi Vaidyanathan) - (supplementary) */

  // Setup for gradient descent algorithm: precalculate any values that occur more than once. Doing this saves the processer 30 multiplication operations.
  float q2_0 = q_0 * q_0; //a2
  float q2_1 = q_1 * q_1; //b2
  float q2_2 = q_2 * q_2; //c2
  float q2_3 = q_3 * q_3; //d2

  float _4q_0 = 4.0f * q_0; //4a
  float _4q_1 = 4.0f * q_1; //4b
  float _4q_2 = 4.0f * q_2; //4c
  float _4q_3 = 4.0f * q_3; //4d

  float _2q_0 = 2.0f * q_0; //2a
  float _2q_1 = 2.0f * q_1; //2b
  float _2q_2 = 2.0f * q_2; //2c
  float _2q_3 = 2.0f * q_3; //2d

  float _8q_1 = 8.0f * q_1; //8b
  float _8q_2 = 8.0f * q_2; //8c

  // Gradient Descent algorithm
  float delF_0 = _4q_0 * q2_2 + _4q_0 * q2_1 + _2q_2 * a_x - _2q_1 * a_y;
  float delF_1 = _8q_1*q2_1 + _4q_1*q2_3 + _4q_1*q2_0 - _4q_1 + _8q_1*q2_2 - _2q_3*a_x - _2q_0*a_y + _4q_1*a_z;
  float delF_2 = _8q_2*q2_2 - _4q_2 + _4q_2*q2_3 + _4q_2*q2_0 + _8q_2*q2_1 + _2q_0*a_x - _2q_3*a_y + _4q_2*a_z;
  float delF_3 = _4q_3*q2_2 + _4q_3*q2_1 - _2q_1*a_x - _2q_2*a_y;

  // Change correction_gain for more or less influence of accelerometer on gyro rates.
  qDot_0 -= correction_gain * delF_0;
  qDot_1 -= correction_gain * delF_1;
  qDot_2 -= correction_gain * delF_2;
  qDot_3 -= correction_gain * delF_3;
  q_0 += qDot_0 * sample_time;
  q_1 += qDot_1 * sample_time;
  q_2 += qDot_2 * sample_time;
  q_3 += qDot_3 * sample_time;

  normalize = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 *= normalize; q_1 *= normalize; q_2 *= normalize; q_3 *= normalize;

  roll = atan2f(2*(q_0*q_1 + q_2*q_3), 1.0f - 2.0f*(q_1*q_1 + q_2*q_2)) * rad_to_degrees + 0.0f;
  pitch = asinf(2.0f * (q_0*q_2 - q_1*q_3)) * rad_to_degrees + 2.0f;
  yaw = atan2f(2*(q_0*q_3 + q_1*q_2), 1.0f - 2.0f*(q_2*q_2 + q_3*q_3)) * rad_to_degrees;
}

/*--- CALIBRATE IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* KEEP IMU STATIONARY DURING STARTUP */

  /*--- Simple Moving Average Setup ---*/
  for (int i = 0; i < sma_samples; i++){
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

    free(data_xyzt); //Clear memory

    delay(3);
  }
  // Average drift / offset of the raw gyroscope data:
  g_drift[0] /= cal_count;
  g_drift[1] /= cal_count;
  g_drift[2] /= cal_count;
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
  // Take the derivative of the process variable (ROLL) instead of the error
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

  /* clamp PWM values. */
  //----------Right---------//
  if (pwm_right < 1015){
    pwm_right = 1015;
  }
  if (pwm_right > 1985){
    pwm_right = 1985;
  }
  //----------Left---------//
  if (pwm_left < 1015){
    pwm_left = 1015;
  }
  if (pwm_left > 1985){
    pwm_left = 1985;
  }

  if (start_motors == true){
    right_prop.writeMicroseconds(pwm_right);
    left_prop.writeMicroseconds(pwm_left);
  } else{
    right_prop.writeMicroseconds(1010);
    left_prop.writeMicroseconds(1010);
  }

  previous_error = error;
  previous_roll = roll;
}

void change_setpoint(){
  if (receiver_input_channel_1 != 0){
    desired_angle = map(receiver_input_channel_1, 1000, 2000, 35, -25);
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

void setup_interrupts(){
  // put your setup code here, to run once:
  PCICR |= (1 << PCIE0);    // Set OCIE0 to enable PCMSK0 to scan
  PCMSK0 |= (1 << PCINT0);  // set digital input 8 to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);  // input 9
  PCMSK0 |= (1 << PCINT2);  // input 10
  PCMSK0 |= (1 << PCINT3);  // input 11
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
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);
  calculate_attitude(data_xyzt);
  //debug_loopTime();
  free(data_xyzt);  // Clear allocated memory for data array.

  // FLIGHT CONTROLLER
  change_setpoint();
  flight_controller();

  // DEBUGGING
  //debugging();
  debug_loopTime();

  //CALIBRATION CONTROLS
  IR_remoteControl();

  // REFRESH RATE
  while (micros() - elapsed_time < 5500);
  // if (micros() - elapsed_time > 5500){  //Freeze if the loop takes too long
  //   while(true);
  // }
}

// Physical interrupts - whenever a signal from the radio reciever is detected, execture the following method:
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
