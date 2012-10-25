#include <stdlib.h>
//#include <string.h>
#include <LSM303.h>
#include <L3G4200D.h>
#include <Event.h>
#include <Timer.h>
#include <PID_v1.h>
#include <TinyGPS.h>
#include <Servo.h>
//#include <GSM_Shield.h> // MAY NOT HAVE TO INCLUDE
#include <Wire.h>



#define ascent_mode 1
#define freefall_mode 2
#define space_mode 3
#define ground_mode 4

#define descent_trigger_velocity 45


//Set error indicator LEDs
#define gps_no_fix_indicator 26
#define gsm_no_reception_indicator 27   //SET ALL AS OUTPUTS!
#define gsm_radio_not_ready_indicator 28
#define datalogger_not_ready_indicator 30
#define system_ready_indicator 32

//Set running LEDs
#define led_1 19
#define led_2 20
#define led_3 21
#define led_4 22  //SET ALL AS OUTPUTS!!!

//Define pin names
#define deployment_charge 4
//#define actuator 5
#define buzzer 6
#define heater 7
#define ambient_light_pin 10

  //Orientation
    double pitchAngle = 0;
    double yawAngle = 0;
    double rollAngle = 0;
    
        
  //PID
    double roll_setpoint, pitch_setpoint, yaw_setpoint, roll_output, pitch_output, yaw_output;
    double aggKp=4, aggKi=0.2, aggKd=1; //Aggressie tuning parameters
    double consKp=1, consKi=0.05, consKd=0.25; //Conservative tuning parameters

//Create necessary objects (GPS,GSM, Servo)
Servo actuator;
Servo pitch;
Servo yaw1;
Servo yaw2;
Servo roll;
TinyGPS gps;
//GSM gsm;
Timer gps_timer;
Timer gsm_timer;
Timer pid_timer;
Timer transmit_timer;
Timer led_timer;
PID rollPID(&rollAngle, &roll_output, &roll_setpoint, consKp, consKi, consKd, DIRECT);
PID pitchPID(&pitchAngle, &pitch_output, &pitch_setpoint, consKp, consKi, consKd, DIRECT);
PID yawPID(&yawAngle, &yaw_output, &yaw_setpoint, consKp, consKi, consKd, DIRECT);
L3G4200D gyro;
LSM303 compass;

//Declare global variables

    int current_mode;
    int ascent_first_run = 0;
    char number[]="+13103827718";  //Destination GSM number
    int signal_strength;
    

    
  //GPS Data
    int year;
    byte month, day, hour, minutes, second, hundredths;
    long lat, lon;
    unsigned long fix_age, time, date, speed, course, chars = 0;
    unsigned short sentences=0, failed_checksum=0;
    float flat, flon;
    float falt = gps.f_altitude(); // +/- altitude in meters
    float fc = gps.f_course(); // course in degrees
    float fk = gps.f_speed_knots(); // speed in knots
    float fmph = gps.f_speed_mph(); // speed in miles/hr
    float fmps = gps.f_speed_mps(); // speed in m/sec
    float fkmph = gps.f_speed_kmph(); // speed in km/hr
    float fdist;
    char cardinal;
     float HOME_LAT = 0, HOME_LON = 0;
    

    
  //TWI Addresses
    int tmp102Address_internal = 0x49;
    int tmp102Address_external = 0x48;

//  //GSM Radio
//    char number[]="+13103827718";  //Destination number
//    char text[]="hello world";  //SMS to send
//    byte type_sms=SMS_UNREAD;      //Type of SMS
//    byte del_sms=0;                //0: No deleting sms - 1: Deleting SMS
//    char sms_rx[122]; //Received text SMS
//    //int inByte=0;    //Number of byte received on serial port
//    char number_incoming[20];
//    int call;
//    int error;
//    int signal_strength;
    
  //Environmental Variables
    int pressure;
    int humidity;
    float i_temp_f;
    int uv_level;
    float e_temp_f;
    int ambient_light;
    int carbon_monoxide;
    boolean smoke = 0;
    
  //IMU
    int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
    #define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
    #define ToRad(x) ((x)*0.01745329252)  // *pi/180
    #define ToDeg(x) ((x)*57.2957795131)  // *180/pi
    
    // L3G4200D gyro: 2000 dps full scale
    // 70 mdps/digit; 1 dps = 0.07
    #define Gyro_Gain_X 0.07 //X axis Gyro gain
    #define Gyro_Gain_Y 0.07 //Y axis Gyro gain
    #define Gyro_Gain_Z 0.07 //Z axis Gyro gain
    #define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
    #define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
    #define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second
    
    // LSM303 magnetometer calibration constants; use the Calibrate example from
    // the Pololu LSM303 library to find the right values for your board
    #define M_X_MIN -796
    #define M_Y_MIN -457
    #define M_Z_MIN -424
    #define M_X_MAX 197
    #define M_Y_MAX 535
    #define M_Z_MAX 397
    
    #define Kp_ROLLPITCH 0.02
    #define Ki_ROLLPITCH 0.00002
    #define Kp_YAW 1.2
    #define Ki_YAW 0.00002
    
    /*For debugging purposes*/
    //OUTPUTMODE=1 will print the corrected data, 
    //OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
    #define OUTPUTMODE 1   
    #define PRINT_ANALOGS 0 //Will print the analog raw data
    #define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
    #define STATUS_LED 13 
    
    float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
    long integrate_timer=0;   //general purpuse timer
    long timer_old;
    long timer24=0; //Second timer used to print values 
    int AN[6]; //array that stores the gyro and accelerometer data
    int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
    int gyro_x;
    int gyro_y;
    int gyro_z;
    int accel_x;
    int accel_y;
    int accel_z;
    int magnetom_x;
    int magnetom_y;
    int magnetom_z;
    float c_magnetom_x;
    float c_magnetom_y;
    float c_magnetom_z;
    float MAG_Heading;    
    float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
    float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
    float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
    float Omega_P[3]= {0,0,0};//Omega Proportional correction
    float Omega_I[3]= {0,0,0};//Omega Integrator
    float Omega[3]= {0,0,0};
    float errorRollPitch[3]= {0,0,0}; 
    float errorYaw[3]= {0,0,0}; 
    unsigned int counter=0;
    byte gyro_sat=0;
    
    float DCM_Matrix[3][3]= {
      {
        1,0,0  }
      ,{
        0,1,0  }
      ,{
        0,0,1  }
    }; 
    float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
    float Temporary_Matrix[3][3]={
      {
        0,0,0  }
      ,{
        0,0,0  }
      ,{
        0,0,0  }
    };





void setup()
{ 
//    //Initialize data I/O pins
//  for(int pin = 19; pin < 23; pin++){
//    pinMode(pin,OUTPUT); // Set running lights to output
//    digitalWrite(pin,LOW);
//  }
//  
//  for(int pin = 26; pin < 33; pin++){
//    pinMode(pin,OUTPUT); // Set error indicators to output Mode
//    digitalWrite(pin,LOW);
//  }
  
  for(int initPin = 4; initPin < 10 !=8; initPin++){
    pinMode(initPin, OUTPUT); // Initializes deployment charge, actuator, buzzer, heater switch,
    
  }
  
  //Activate all indicators
  digitalWrite(gps_no_fix_indicator,HIGH);
  digitalWrite(gsm_no_reception_indicator,HIGH);
  digitalWrite(gsm_radio_not_ready_indicator,HIGH);
  digitalWrite(datalogger_not_ready_indicator,HIGH);
  
  //Initialize Serial Ports
  Serial.begin(9600); //GSM Radio
  Serial1.begin(9600); // GPS
  //Serial2.begin(115200); // Currently unused
  Serial3.begin(9600); // OpenLog (datalogger)
  
  //Initialize TWI 
  Wire.begin();
  
  Serial.print("Serials ok");
  
  //Initialize OpenLog
  openLogInit();
  while(1)
  {
    //Temporary pause to debug openlog setup
  }
  //Serial.print("SP's I'd");
  gpsInit(); // Initialize GPS unit.
  gsmInit(); // Initialize GSM Radio.
  recordEvent("COM devices initialized.");
  
  
  recordEvent("Serial ports initialized.");
  
  //Initialize IMU
  imuInit();
  recordEvent("IMU Initialized.");

  
  digitalWrite(deployment_charge,LOW);
  pinMode(8, INPUT); // Smoke Detector
  recordEvent("Sensor ports initialized.");
  
  //Initialize Servo Objects
  actuator.attach(5);// Check to see which pin the actuator is currently plugged into
  pitch.attach(12);
  yaw1.attach(14);
  yaw2.attach(15);
  roll.attach(20);
  recordEvent("Servos initialized.");
  
 // attachInterrupt(1, smokeDetect, RISING); // Attach smoke detector interrupt to interrupt 1, NOT PIN 1!!
  recordEvent("Interrupts set.");
 
  
  pid_timer.every(100,stabilize);
  gsm_timer.every(1000 *60 * 5,transmit);
  led_timer.every(500,ledToggle);
  
  Serial.print("done");
  digitalWrite(system_ready_indicator,HIGH); //Activate system ready LED
  current_mode = ascent_mode; //Begin in ascent mode
  
}

void loop()
{
  switch(current_mode){
    case ascent_mode:
      ascent();
      break;
    case freefall_mode:
       freefall();
       break;
    case space_mode:
        space();
        break;
    case ground_mode:
        ground();
        break;
  }
}


void ascent()
{
  /*ASCENT ROUTINE
      Transmit GPS coordinates
      Gather environmental data
      Check altitude, enter space mode when appropriate
      Check for freefall
      
  
  */
  if(ascent_first_run == 0){
    recordEvent("Liftoff \n");
    recordEvent("Ascent control sequence activated.");
    ascent_first_run = 1;
  }
    
  gsm_timer.update();
  gsmCheck(); // check for incoming commands
  gpsRefresh();
  senseEnvironment();
  
  if(falt >= 70000)
  {
    current_mode = space_mode;
  }
  
  

  
  
  //Check for freefall
  if((fmph > descent_trigger_velocity) && (falt > 10000) && (falt < 70000)){
    current_mode = freefall_mode;
    //String current_alt = String(falt);
    
    char cutdown_msg[50];
    sprintf(cutdown_msg, "Premature cutdown at &f feet.",falt); 
    recordEvent(cutdown_msg);// FIX ME! recordEvent will not take objects of type String
  }
    
  
}

void freefall()
{
  /* FREEFALL ROUTINE
        Transmit freefall notification
        Release balloon tether
        Activate stability mode
        Activate running lights
        Activate buzzer
        
        Transmit GPS coordinates
        Gather environmental data
        Check for altitude < 10000ft
        Check for landing
        
  
  */
  
  recordEvent("Freefall control sequence activated.");
  pid_timer.update();
  led_timer.update();
  tone(buzzer,9000);
  gsmCheck(); // check for incoming commands
  gpsRefresh();
  senseEnvironment();
  
  if(falt <= 10000){
    deploy();
  }
  
  if(falt <=100 && fmph <=10)
  {
    current_mode = ground_mode;
  }
  
}

void space()
{
  /* IN-SPACE ROUTINE
        Operate Pan/Tilt cam
        Transmit GPS Coordinates
        Gather Environmental data
        Check for freefall
  
  */
  recordEvent("In-space control sequence activated.\n");
  gsmCheck(); // check for incoming commands
  
  
  if(falt >= 100000){
    recordEvent("100000 ft altitude ceiling surpassed.");
    //Send alert
  }
  
  if(fmph >= descent_trigger_velocity){
    current_mode = freefall_mode;
    }
}

void ground()
{
  
  /* ON-GROUND ROUTINE  
        Transmit landing notification
        Transmit GPS Coordinates
        
   */

   char land_msg[50];
   sprintf(land_msg,"Payload landed successfully at %f,%f",flat,flon);
   recordEvent(land_msg);
   recordEvent("On-ground control sequence activated.");
   gsmCheck(); // check for incoming commands
   gsm_timer.update();
   
  
}









//Barometric pressure sensor measurement
void getPressure(){
  int pressure = analogRead(4);
  char pres[5];
  sprintf(pres,"%d",pressure);
  appendFile( pres, "PRESSURE.txt",1,1);
  //TEMP
}

void getImage(){
  //TEMP
}

//Gather and store environmental data
void senseEnvironment(){
  getITemp();
  getETemp();
  gps.get_position(&lat, &lon, &fix_age); // retrieves +/- lat/long in 100000ths of a degree
  gps.get_datetime(&date, &time, &fix_age); // time in hhmmsscc, date in ddmmyy
  speed = gps.speed(); // returns speed in 100ths of a knot
  getHumidity();
  getPressure();
  //getUV(); //UV Photodiode not flying on initial flight
  getImage(); // likely will not be utilized. TEMPORARY.  
}

////Part of the parachute deployment routine.
//void setFallStatus(){
//  fallStatus = 1;
//  recordEvent("Zero-G detected. Checking freefall status...")
//}
//
////Part of the parachute deployment routine.
//void freeFallCheck(){
//  if(fallStatus == 1 && fmph > 35){
//    tetherRelease();
//    recordEvent("Tether released. Entering freefall.");
//  }
//}

//Releases the teather attaching the payload to the balloon
void tetherRelease(){
  actuator.write(135);
  //delay(300);
  recordEvent("Lift tether released.");
  //String releaseMessage = String("Lift tether released. Payload descent initiated.");
 // char releaseMessage[];
  //sprintf(releaseMessage,"Lift tether released. Payload descent initiated.");
  gsm_send_message("Lift tether released. Payload descent initiated.");
}


//Humidity sensor
int getHumidity(){
  humidity = analogRead(2);
  char hum[5];
  sprintf(hum,"%d",humidity);
//  Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%rh");
//  Serial1.print("Humidity: "); Serial1.print(humidity); Serial1.println("%rh");
  appendFile( hum, "HUMIDITY.txt",1,1);
  return humidity;
}

//Ambient Light Sensor
int getAmbientLight(){
  //DEFINE AMBIENT LIGHT PIN
   ambient_light = analogRead(ambient_light_pin);
   char amb[5];
   sprintf(amb,"%d",ambient_light);

   appendFile(amb,"AMBILGHT.txt",1,1);
   return ambient_light;
}

//UV Photodiode - NOT INCLUDED IN FIRST FLIGHT
//int getUV(){
//   uv_level = analogRead(5);
//  appendFile(uvLevel, "UV Photodiode.txt",1);
//  return uv_level;
//}

//Smoke detector
void smokeDetect(){
  smoke = 1;
  i_temp_f = getITemp();  
  char smoke_msg[50];
  sprintf(smoke_msg,"Possible fire onboard. Internal temperature is: %f",i_temp_f);
  gsm_send_message(smoke_msg);
//  Serial.print("Possible fire onboard. Internal temperature is: "); //FIX ME - NEEDS TO BE SENT OVER THE GSM MESSAGE USING THE TEXT FUNCTION
//  Serial.println(i_temp_f);
}



//Internal Temperature
float getITemp(){
 Wire.requestFrom(tmp102Address_internal,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 
  i_temp_f = (1.8 * TemperatureSum*0.0625) + 32;
  char temps[8];
  sprintf(temps,"%f",i_temp_f);
 appendFile(temps, "INTETEMP.txt\n",1,1);
 
 
 if(i_temp_f < 45){
   digitalWrite(7, HIGH); // Switch heater on.
   recordEvent("Heater switched on.");
 }
 else{
   digitalWrite(7, LOW);
 }
 
 return i_temp_f;
}

//External Temperature
float getETemp(){
 Wire.requestFrom(tmp102Address_external,2); 
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 
  e_temp_f = (1.8 * TemperatureSum*0.0625) + 32;
  char temps[8];
  sprintf(temps,"%f",e_temp_f);
 appendFile(temps, "EXTETEMP.txt\n",1,1);
 
 return e_temp_f;
}




//STABILITY SYSTEM FUNCTIONS

// Stability control code. NEEDS TO BE TESTED. LIKELY NOT FUNCTIONAL, PID returns error, appropriate servo angle needs to be calculated.
void stabilize(){
  updateOrientation(); 
  double roll_error = abs(roll_setpoint - rollAngle);
  if(roll_error < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    rollPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {  //we're far from setpoint, use aggressive tuning parameters
     rollPID.SetTunings(aggKp, aggKi, aggKd);
  }
  rollPID.Compute();
  roll.write(roll_output);
  
  
  double pitch_error = abs(pitch_setpoint - pitchAngle);
  if(pitch_error<10)
  {  //we're close to setpoint, use conservative tuning parameters
    pitchPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     pitchPID.SetTunings(aggKp, aggKi, aggKd);
  }
  pitchPID.Compute();
  pitch.write(pitch_output);
  
  
  double yaw_error = abs(yaw_setpoint - yawAngle);
  if(yaw_error<10)
  {  //we're close to setpoint, use conservative tuning parameters
    yawPID.SetTunings(consKp, consKi, consKd);
  }
  else
  { //we're far from setpoint, use aggressive tuning parameters
     yawPID.SetTunings(aggKp, aggKi, aggKd);
  }
  yawPID.Compute();
  yaw1.write(yaw_output);
  yaw2.write(yaw_output);
  
//  rollRate = (analogRead(A0) - 507);
//    Serial.println(data);
//    if(abs(data) < 10){
//      roll.write(0);
//    }
//    if(data > 100 || data < -100 && data != 0){
//    val = map(data, -540, 540, 0, 179);
//    roll.write(val);
//    delay(15);
//    }
}








void updateOrientation(){
  
  timer_old = integrate_timer;
    integrate_timer=millis();
    if (integrate_timer>timer_old)
      G_Dt = (integrate_timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
   
  Read_Gyro();
  Read_Accel();
  Read_Compass();
  Compass_Heading();
  // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();

}

//Descent Running Lights

void ledToggle(){
  //NOT FINISHED!
  static int state = 0;
  
  for(int i = led_1;i<(led_4 + 1);i++){
    digitalWrite(i,state);
  }
  state = !state;
}

void deploy(){
  digitalWrite(deployment_charge, HIGH);
  recordEvent("Parachute deployed.");
}

/* UNCOMMENT ME
boolean SerialReadUntil(String key, int timeout) {
  // Key = aab
  // String = aaab
  
//  switch(port){
//    case 0:
//      device = Serial;
//      break;
//      
//    case 1:
//      device = Serial1;
//      break;
//    
//    case 2:
//      device = Serial2;
//      break;
//    
//    case 3:
//      device = Serial3;
//      break;
    

  unsigned long start = millis();

  int pos = 0;

  while(start + timeout > millis()) {
    while (start + timeout > millis() && Serial->available()) {
      char a = Serial->read();
      //      Serial.print(a);
      if (a == key[pos]) {
        pos++;
      } else {
        pos = 0;
      }
      if (pos == key.length()) {
        return true;
      }
    }
  }
  return false;
}
}
*/
