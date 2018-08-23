/*************************************************************************************************
 *  Angle (in °) & Travel (in mm) automatic calculator for RC model using arduinon & an MPU 6050 
 *  
 *  Single control surface version using :. 
 *    1 I2c MPU 6050 accelero & gyro
 *    1 pc Arduino Pro
 *    1 pc I2c Oled 0.96" SSD1306
 *    3 pc standards push button
 *    3D printed parts
 *
 *  Libraries used 
 *    - I2Cdev & MPU6050 by Jeff Rowberg
 *    - Adafruit_SSD1306
 *    - math for trigonometric functions
 *    - Wire for I2c
 *  
 *  Bibliography :
 *    - https://delta-iot.com/la-theorie-du-filtre-complementaire/
 *    - https://www.firediy.fr/article/calibrer-le-capteur-mpu6050-avec-un-arduino-drone-ch-5
 *    - https://github.com/ZINKTiti/Angle_and_Throw_Meter by Thierry de Faudoas
 *    
 *  For all files and explainations please visit https://github.com/adesandr/MAD_Single    
 *  
 *  alain.desandre@wanadoo.fr - V1.0 - 23/08/2018
 ************************************************************************************************/

/*--- I2Cdev, MPU6050, and Adafruit_SSD1306 must be installed as libraries ---*/
#include "Arduino.h"                      // BAsic Arduino definition
#include "Wire.h"                         // I2C library
#include "I2Cdev.h"                       // MPU6050 library by Jeff Rowberg, see https://github.com/jrowberg
#include "MPU6050.h"
#include "Adafruit_GFX.h"                 // Adafruit SSD1306 libray
#include "Adafruit_SSD1306.h"
#include "math.h"                         // atan2(), sin(), PI, ...

/*--- Globals define                                                      ---*/
#define DELAY_LEDON 250                   // Flashy led state ON by step of 4 ms
#define PB_MINUS 5                        // Input push button definition - Minus decrease de chord
#define PB_INIT 6                         // INIT start calibration
#define PB_PLUS 7                         // PLUS increase the chord
#define DELAY_SPLASH_SCREEN  2000         // Push button scrutation period in ms.
#define DELAY_DEBOUNCE 50                 // Debounce delay for Push Button
#define DELAY_DISPLAY 250                 // Display period
#define DELAY_MEASURE 10                  // Measure period
#define MAD_DELAY_WELCOME 3000            // Welcome Message delay
#define DELAY_START_INIT 2000             // delay to start calibration when PB_INIT was pressed

/*--- Globals variable setup                                              ---*/
#define OLED_RESET 4                      // OLED constructor - OLED used the 0x3C I2c Adress
Adafruit_SSD1306 display(OLED_RESET);

MPU6050 accelgyro;                        // MPU6050 constructor - MPU6050 used 0x68 I2c Adress, A0 low, it mean not connected.

int chordControlSurface = 50;             // Chord in mm. 50 mm by default.

byte ledPin = LED_BUILTIN;
byte varCompteur = 0;                     // For ISR Timer2 management

long t1 = 0;                              // Timer for Display management
long t2 = 0;                              // Timer for measure

long lastDebounceTimePB_MINUS = 0;        // Push button management
long lastDebounceTimePB_INIT = 0;
long lastDebounceTimePB_PLUS = 0;
boolean lastStatePB_MINUS = HIGH;
boolean lastStatePB_INIT = HIGH;
boolean lastStatePB_PLUS = HIGH;
boolean triggerPB_MINUS = 0;
boolean triggerPB_INIT = 0;
boolean triggerPB_PLUS = 0;
boolean buttonStatePB_MINUS = HIGH;
boolean buttonStatePB_INIT = HIGH;
boolean buttonStatePB_PLUS = HIGH;
boolean readingPB_MINUS = HIGH;
boolean readingPB_INIT = HIGH;
boolean readingPB_PLUS = HIGH;

boolean bInit = 0;                        // Calibration requested
boolean bInvert = 0;                      // Toggle Display Mode

int16_t ax, ay, az;                       // raw measure
int16_t gx, gy, gz;
uint8_t Accel_range;
uint8_t Gyro_range;
float travel = 0.0;
float angle=0.0;

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

void setup() {

  // open serial monitor for debug
  Serial.begin(115200);

  /*--- Push-button initialisation. All inputs are configured in PULLUP Mode ---*/
  pinMode(PB_PLUS, INPUT_PULLUP);
  pinMode(PB_INIT, INPUT_PULLUP);
  pinMode(PB_MINUS, INPUT_PULLUP);

  /*--- OLED init and splash screen ---*/
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.display();                          // splash screen display. See Adafruit_SSD1306.cpp, for the memory buffer init.
  delay(DELAY_SPLASH_SCREEN);

  // Welcome message
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(16,0);
  display.println(F("MAD V1.0"));
  display.setCursor(8,28);
  display.println(F("ANEG 2018"));
  display.setCursor(0,56);
  display.setTextSize(1);
  display.print(F(__DATE__));display.print(F(" "));display.print(F(__TIME__));
  display.display(); 
  delay(MAD_DELAY_WELCOME);
  
  /*--- MPU6050 initialization      ---*/ 
  Serial.println(F("Initialisation MPU6050...")); 
  accelgyro.initialize(); 

  // verify connection 
  Serial.println(F("Test de la conection du dispositif ...")); 
  Serial.println(accelgyro.testConnection() ? F("MPU6050 connection reussie") : F("MPU6050 connection echec")); 
  delay(1000); 
  
  /*--- Led intialization and ISR on TIMER2 is set ---*/
  pinMode(ledPin, OUTPUT); //led
  digitalWrite(ledPin, HIGH);
  cli();                                          // General interrupt disable
  bitClear(TCCR2A, WGM20);                        // WGM20 = 0
  bitClear(TCCR2A, WGM21);                        // WGM21 = 0
  TCCR2B = 0b00000110;                            // Clock/256, it mean 16 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001;                            // Enable local interrupt by TOIE2
  sei();                                          // General interrupt enable

}/* End setup() */

/*******************************************************
 * ISR interrupt on TIMER 2
 * 
 * flash the LEDBUILT_IN periodically
 *******************************************************/
ISR(TIMER2_OVF_vect){
  TCNT2 = 256 - 250;                // 250 x 16 micro-s = 4 ms
  if(varCompteur++ > DELAY_LEDON)           // 250 * 4 ms = 1s for 1/2 period
  {
    varCompteur = 0;
    digitalWrite(ledPin, !digitalRead(ledPin));
  }

} /* end ISR */

/*******************************************************
 * void Init(void)
 * 
 * MPU6050 Initialization
 *******************************************************/
void Init()
{

  /*--- Display message ---*/
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(F("Calibration Start..."));
  display.display();

  /*--- reset offsets   ---*/
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  /*--- Step 1 : first reading ---*/
  display.println(F("\nFirst reading..."));
  display.display();
  meansensors();
  delay(1000);

  /*--- Step 2 : Compute offsets ---*/
  display.println(F("\nCompute offsets..."));
  display.print(F("\n"));
  display.display();
  calibration();
  delay(1000);

  /*--- New reads and offsets display ---*/
  meansensors();
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("FINISHED!"));
  display.setCursor(0,8);
  display.print(F("R.w.off:"));
  display.setCursor(80,8); 
  display.print(F("Y.off:"));
  display.setCursor(0,16);   
  display.println(mean_ax);
  display.println(mean_ay);
  display.println(mean_az);
  display.println(mean_gx);
  display.println(mean_gy);
  display.println(mean_gz);
  display.setCursor(80,16); 
  display.print(ax_offset);
  display.setCursor(80,24); 
  display.print(ay_offset);
  display.setCursor(80,32); 
  display.print(az_offset);
  display.setCursor(80,40); 
  display.print(gx_offset);
  display.setCursor(80,48); 
  display.print(gy_offset);
  display.setCursor(80,56); 
  display.println(gz_offset);
  display.display();

  delay(10000);

  /*--- Set offsets with the compute values ---*/
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);
  
} /* End Init() */

/*******************************************************
 * void meansensors(void)
 * 
 * average sensors reading
 *******************************************************/
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
} /* End meansensors() */

/*******************************************************
 * void calibration(void)
 * 
 * MPU6050 calibration
 *******************************************************/
void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    display.print(F("."));
    display.display();

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
} /* End calibration */

/*******************************************************
 * void managePB(void)
 * 
 * Push button management
 *******************************************************/
void managePB()
{

  /*--- Debounce PB_MINUS and trigger if pressed ---*/
  readingPB_MINUS = digitalRead(PB_MINUS);
  triggerPB_MINUS = 0;    
  if(readingPB_MINUS != lastStatePB_MINUS)
    lastDebounceTimePB_MINUS = millis();
    
  if((millis() - lastDebounceTimePB_MINUS) > DELAY_DEBOUNCE)
    {
      if (readingPB_MINUS != buttonStatePB_MINUS)
        {
          buttonStatePB_MINUS = readingPB_MINUS;
          if(buttonStatePB_MINUS == LOW)
            triggerPB_MINUS = 1;
        }
    }

  lastStatePB_MINUS = readingPB_MINUS;  

  /*--- Debounce PB_INIT and trigger if pressed ---*/
  readingPB_INIT = digitalRead(PB_INIT);
  triggerPB_INIT = 0;
  if(readingPB_INIT != lastStatePB_INIT)
    lastDebounceTimePB_INIT = millis();
    
  if((millis() - lastDebounceTimePB_INIT) > DELAY_DEBOUNCE)
    {
      if (readingPB_INIT != buttonStatePB_INIT)
        {
          buttonStatePB_INIT = readingPB_INIT;
          if(buttonStatePB_INIT == LOW)
            triggerPB_INIT = 1;
        }
    }
   
  lastStatePB_INIT = readingPB_INIT;

  /*--- If PB_INIT pressed during more than DELAY_START_INIT -> set bInit to start calibration ---*/
  if((millis() - lastDebounceTimePB_INIT > DELAY_START_INIT) && (lastStatePB_INIT == LOW))
     bInit = 1;
  
  /*--- Debounce PB_PLUS and trigger if pressed ---*/
  readingPB_PLUS = digitalRead(PB_PLUS);
  triggerPB_PLUS = 0;    
  if(readingPB_PLUS != lastStatePB_PLUS)
    lastDebounceTimePB_PLUS = millis();
    
  if((millis()- lastDebounceTimePB_PLUS) > DELAY_DEBOUNCE)
    {
      if (readingPB_PLUS != buttonStatePB_PLUS)
        {
          buttonStatePB_PLUS = readingPB_PLUS;
          if(buttonStatePB_PLUS == LOW)
            triggerPB_PLUS = 1;
        }
    }
    
  lastStatePB_PLUS = readingPB_PLUS;

  /*--- Decrease chord if PB_MINUS is pressed ---*/
  if(triggerPB_MINUS)
  {
    /*--- we check that chordControlSurface stay greater than 0 ---*/
    if(chordControlSurface > 2)
      chordControlSurface--;
  }

  /*--- PB_INIT ---*/
  if(triggerPB_INIT)
  {
    bInvert = !bInvert;
  }

  /*--- Increase chord if PB_PLUS is pressed ---*/
  if(triggerPB_PLUS)
  {
    chordControlSurface++;

  }
  
} /* End managePB() */

/*******************************************************
 * void manageMeasure()
 * 
 * MPU6050 measurement
 *******************************************************/
void manageMeasure(){

  if(millis() - t2 > DELAY_MEASURE)
  {

    t2 = millis();
    
    /*--- Compute Y angle in degre. Complementary filter is used to combine accelero and gyro datas       ---*/
    /*--- see  http://www.pieter-jan.com/node/11 for more information regarding the complementary filter  ---*/
    /*--- or https://delta-iot.com/la-theorie-du-filtre-complementaire/ (in french)                       ---*/
    /*--- Basically complementary filter avoid used of kallman filter, quiet difficult to implement in    ---*/
    /*--- arduino platform. Gyro are used for fast motion as accelero are used for slow motion.           ---*/                                 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angle=0.98*(angle+float(gy)*0.01/131) + 0.02*atan2((double)ax,(double)az)*180/PI;

    /*--- Compute Control surface travel using : 2* sin(angle/2)* chord. Angle for sinus function needs  ---*/
    /*--- to be converted in radian (angleDegre = angleRadian *(2*PI)/360)                             ---*/ 
    travel = chordControlSurface * sin((angle*(2.0*PI)/360.0)/2.0) * 2.0;

  }
  
} /* End manageMeasure */

/*******************************************************
 * void manageDisplay()
 * 
 * OLED Management
 *******************************************************/
void manageDisplay(){

  int angle_aff = abs(angle * 10);
  int travel_aff = abs(travel * 10);

  if((millis() - t1) > DELAY_DISPLAY)
  {
    
  t1 = millis();

  display.clearDisplay();  

  if(bInit)
    {
    Init();
    bInit = 0;
    }
  
  if(bInvert)
    {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print(F("Chord: ")); display.print(chordControlSurface);display.print(F(" mm"));
    display.setCursor(0,16);
    display.print(F("Travel: ")); display.print(travel_aff/10);display.print(",");display.print(travel_aff%10);display.println(F(" mm"));
    display.setCursor(0,32);
    display.print(F("Angle: "));
    display.setTextSize(2);
    display.setCursor(26,45);
    display.print(angle_aff/10);display.print(",");display.print(angle_aff%10); display.print(F(" deg"));
    display.setTextColor(BLACK, WHITE);
    display.setCursor(112,0);
    display.print(F("A"));  
    }
  else
    {
    display.setTextSize(1);
    display.setTextColor(WHITE);    
    display.setCursor(0,0);
    display.print(F("Chord: ")); display.print(chordControlSurface);display.print(F(" mm"));
    display.setCursor(0,16);
    display.print(F("Angle: "));
    display.print(angle_aff/10);display.print(",");display.print(angle_aff%10); display.print(F(" deg"));
    display.setCursor(0,32);
    display.print(F("Travel:"));
    display.setTextSize(2);
    display.setCursor(26,45);
    display.print(travel_aff/10);display.print(",");display.print(travel_aff%10);display.println(F(" mm"));
    display.setTextColor(BLACK, WHITE);
    display.setCursor(112,0);
    display.print(F("T"));  
    }

  display.display();  
 
 }
   
} /* End manageDisplay */
 
/*******************************************************
 * void loop()
 * 
 * Main loop conrrol
 *******************************************************/
void loop() {

  /*--- Measure management ---*/
  manageMeasure();

  /*--- Push button management ---*/
  managePB();

  /*--- Display management     ---*/
  manageDisplay();

} /* End loop() */
