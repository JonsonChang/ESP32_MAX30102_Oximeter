/* jonson
  #  simple SpO2 plotter for MH-ET LIVE MAX30102 breakout board and ESP32 devkit-C
  Using Sparkfun MAX3010X library
  https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library

  ESP32_MAX30102_simple-SpO2_plotter.ino
  by coniferconifer Copyright 2020
  LICENSED under Apache License 2.0

  Version 1.0

  Shows SpO2 and the user's heart beat on Arduino's serial plotter.
  No display hardware is required.
  This program should not be used for medical purposes.
  I wrote this to learn how SpO2 can be measured and pay tributes for the inventors.

  Pulse oximetry was developed in 1972, by Takuo Aoyagi and Michio Kishi,
  bioengineers, at Nihon Kohden in Japan.
  https://ethw.org/Takuo_Aoyagi

  Since MH-ET LIVE MAX30102 breakout board seems outputting IR and RED swapped.
  red = particleSensor.getFIFOIR();
  ir = particleSensor.getFIFORed();
  is used in my code. If you have Sparkfun's MAX30105 breakout board , try to 
  use #define MAX30105

  ## Tips:
  SpO2 is calculated as R=((square root means or Red/Red average )/((square root means of IR)/IR average))
  SpO2 = -23.3 * (R - 0.4) + 100;
  // taken from a graph in https://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
  // https://ww1.microchip.com/downloads/en/Appnotes/00001525B.pdf
  ## Instructions:
  0) Install Sparkfun's MAX3010X library
  1) Load code onto ESP32 with MH-ET LIVE MAX30102 board
  2) Put MAX30102 board in plastic bag and insulates from your finger.
     and attach sensor to your finger tip
  3) Run this program by pressing reset botton on ESP32
  4) Wait for 3 seconds and Open Arduino IDE Tools->'Serial Plotter'
     Make sure the drop down is set to 115200 baud
  5) Search the best position and presure for the sensor by watching
     the blips on Arduino's serial plotter
     I recommend to place LED under the backside of nail and wrap you
     finger and the sensor by rubber band softly.
  6) Checkout the SpO2 and blips by seeing serial Plotter
     100%,95%,90%,85% SpO2 lines are always drawn on the plotter

  ## Hardware Connections (Breakoutboard to ESP32 Arduino):
  -VIN = 3.3V
  -GND = GND
  -SDA = 21 (or SDA)
  -SCL = 22 (or SCL)
  -INT = Not connected

  this script also works on Arduino nao
  ## Hardware Connections (Breakoutboard to Arduino nano): experimental
  -VIN = 3.3V 
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  
  ## Trouble Shooting:
  Make sure to solder jumper on 3V3 side.
  if you forget this, I2C does not work and can not find MAX30102.
  says "MAX30102 was not found. Please check wiring/power."


*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//===============================================

#include "MAX30105.h" //sparkfun MAX3010X library
MAX30105 particleSensor;

// ===============hart rate variable =================================
const byte RATE_SIZE = 20; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

float beatData[9];
bool checkForBeat(float data){
  beatData[0] = beatData[1];
  beatData[1] = beatData[2];
  beatData[2] = beatData[3];
  beatData[3] = beatData[4];
  beatData[4] = beatData[5];
  beatData[5] = beatData[6];
  beatData[6] = beatData[7];
  beatData[7] = beatData[8];
  beatData[8] = data;

  if((beatData[4] > beatData[0]) &&  (beatData[4] >= beatData[5]) &&  
     (beatData[4] > beatData[1]) &&  (beatData[4] > beatData[6]) &&  
     (beatData[4] > beatData[2]) &&  (beatData[4] > beatData[7]) &&   
     (beatData[4] >= beatData[3]) &&  (beatData[4] > beatData[8])
     
     )
  {
    return true;
  }
  else{return false;}

}

//#define MAX30105 //if you have Sparkfun's MAX30105 breakout board , try #define MAX30105


void LCD_spO2_display(double espO2, int hart_rate)
{
  display.clearDisplay();
  display.setTextSize(3);             
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0,0);
  display.print(espO2);
  display.println(" %");
  display.print(hart_rate);
  display.println(" BPM");
  display.display();
  //delay(2000);
}

void LCD_no_finger(){
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);             
  display.setCursor(0,0);
  display.println("Finger");
  display.println("Please");
  display.display();
}


void LCD_setup()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
    {
      ; // Don't proceed, loop forever
    }
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(200); // Pause for 2 seconds
  LCD_no_finger();
}

#define USEFIFO
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");
  LCD_setup();

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}
double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//calculate SpO2 by this sampling interval

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0




void loop()
{

  uint32_t ir, red, green;
  double fred, fir;
  double SpO2 = 0; // raw SpO2 before low pass filtered

#ifdef USEFIFO
  particleSensor.check(); // Check the sensor, read up to 3 samples

  while (particleSensor.available())
  { // do we have new data
#ifdef MAX30105
    red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();   // Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR(); // why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); // why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate); // average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate);    // average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered);        // square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);             // square sum of alternate component of IR level
    if ((i % SAMPLING) == 0)
    { // slow down graph plotting speed for arduino Serial plotter by thin out
      if (millis() > TIMETOBOOT)
      {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        // trancation for Serial plotter's autoscaling
        if (ir_forGraph > 100.0)
          ir_forGraph = 100.0;
        if (ir_forGraph < 80.0)
          ir_forGraph = 80.0;
        if (red_forGraph > 100.0)
          red_forGraph = 100.0;
        if (red_forGraph < 80.0)
          red_forGraph = 80.0;
        //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
        if (ir < FINGER_ON){
          ESpO2 = MINIMUM_SPO2;    // indicator for finger detached
          LCD_no_finger();
          beatAvg = 70;
        }else{
          if (checkForBeat(ir) == true)                        //If a heart beat is detected
          {
            long delta = millis() - lastBeat; // Measure duration between two beats
            lastBeat = millis();

            beatsPerMinute = 60 / (delta / 1000.0); // Calculating the BPM

            if (beatsPerMinute < 255 && beatsPerMinute > 20) // To calculate the average we strore some values (4) then do some math to calculate the average
            {
              rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
              rateSpot %= RATE_SIZE;                    // Wrap variable

              // Take average of readings
              beatAvg = 0;
              for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
              beatAvg /= RATE_SIZE;
              
            }
          }
        }
        Serial.print(ir_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(",");
        Serial.print(red_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(",");
        Serial.print(ESpO2); // low pass filtered SpO2
        Serial.print(",");
        Serial.print(beatAvg); // low pass filtered beatAvg , hart rate
        Serial.print(",");
        Serial.print(85.0); // reference SpO2 line
        Serial.print(",");
        Serial.print(90.0); // warning SpO2 line
        Serial.print(",");
        Serial.print(95.0); // safe SpO2 line
        Serial.print(",");
        Serial.println(100.0); // max SpO2 line
      }
    }
    if ((i % Num) == 0)
    {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100;               // http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0;
      sumirrms = 0.0;
      i = 0;
      if (ir < FINGER_ON)
      {
        LCD_no_finger();
      }
      else
      {
        LCD_spO2_display(ESpO2, beatAvg);
      }
      break;
    }
    particleSensor.nextSample(); // We're finished with this sample so move to next sample
    // Serial.println(SpO2);
  }

#endif
}
