
# Simple SpO2 plotter for MH-ET LIVE MAX30102

## 所需材料

1. 血氧模組 - Max30102 (請確認背面的電壓跳線有焊上 3V3 )
2. OLED 小螢幕 SSD1306 I2C ，畫面大小128x64 (請確認I2C address)
3. ESP32 開發板
4. 其他線材


Using Sparkfun MAX3010X library
  https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library

本專案修改自：https://github.com/coniferconifer/ESP32_MAX30102_simple-SpO2_plotter


## Tips:
- Used argorithm

  DC component of IR and RED data (average) is calculated by digital low pass filter and subtracted from raw data to get AC components.
  Then, square root means of AC component of RED and IR are calculated for every 100 samples.


  SpO2 is calculated as 
```  
  R=((square root means of RED/ RED average )/((square root means of IR)/IR average)) 
  
  SpO2 = -23.3 * (R - 0.4) + 100
```
  The last formula is read from the graph in https://ww1.microchip.com/downloads/en/Appnotes/00001525B.pdf
  or https://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf

  


- when IR signal is smaller than 30000 (#define FINGER_ON 30000), then SpO2 becomes 80 to indicate your finger is not on the sensor.

- Since MH-ET LIVE MAX30102 breakout board seems outputting IR and RED swapped when Sparkfun's library is used.
```C
red = particleSensor.getFIFOIR();
ir = particleSensor.getFIFORed();
```
  is used in my code. If you have Sparkfun's MAX30105 breakout board , use #define MAX30105



## Instructions:

  0) Install Sparkfun's MAX3010X library
  1) Load the code onto ESP32 with MH-ET LIVE MAX30102 board
  2) Put MAX30102 board in a plastic bag , insulating from your finger
     and attach the sensor to your finger tip
  3) Run this program by pressing reset botton on ESP32 devkitC
  4) Wait for 3 seconds and Open Arduino IDE Tools->'Serial Plotter'
     Make sure the drop down is set to 115200 baud
  5) Search the best position and pressure for the sensor by watching
     the blips on Arduino's serial plotter.
     I recommend to place LED under the backside of nail , wrap you
     finger and the sensor by rubber band softly.
  6) Checkout the SpO2 and blips by seeing serial plotter.
     100%,95%,90%,85% SpO2 lines are always drawn on the plotter

## Hardware Connections (Breakout board to ESP32 Arduino):
```
  -VIN = 3.3V
  -GND = GND
  -SDA = 21 (or SDA)
  -SCL = 22 (or SCL)
  -INT = Not connected
```

## Hardware Connections (Breakout board to Arduino nano): 
  this script also works on Arduino nao.(experimental)
```
  -VIN = 3.3V 
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
```
## Trouble Shooting:
  Make sure to solder jumper on 3V3 side. 
  If you forget this, I2C does not work and can not find MAX30102, 
  says
```
 "MAX30102 was not found. Please check wiring/power/solder jumper."
```


### LICENSED under Apache License 2.0

## References
- #316 Pulse Oximeter test, function, and usage
  https://www.youtube.com/watch?v=fsJjHEnlQkU
- BLE Oximeter Hack with ESP32 for COVID-19 Projects
  https://www.youtube.com/watch?v=FIVIPHrAuAI
- ATTiny85 Pulse Oximeter with Photoplethysmogram (PPG) display
  https://github.com/jeffmer/tinyPulsePPG
- MAX30102 datasheet
  https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf
- Recommended Configurations and Operating Profiles
  for MAX30101/MAX30102 EV Kits
  https://pdfserv.maximintegrated.com/en/an/AN6409.pdf
- Pulse Oximeter Design Using Microchip's Analog Devices and dsPIC Digital Signal Controllers(DSCs)
  https://ww1.microchip.com/downloads/en/Appnotes/00001525B.pdf

## Pics

![](./pics/(1).jpg)
![](./pics/(2).jpg)
![](./pics/(3).jpg)
![](./pics/(4).jpg)
![](./pics/(6).jpg)
![](./pics/(7).jpg)
![](./pics/(8).jpg)
![](./pics/(9).jpg)
![](./pics/(10).jpg)
