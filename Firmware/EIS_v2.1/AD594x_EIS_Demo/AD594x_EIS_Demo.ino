/*
    AUTHORS: Kevin Alessandro Bautista, Shannon Riegle
    EMAIL: kbautis@purdue.edu, sdriegle@iu.edu

    DISCLAIMER: 
    Linnes Lab code, firmware, and software is released under the MIT License
    (http://opensource.org/licenses/MIT).
    
    The MIT License (MIT)
    
    Copyright (c) 2024 Linnes Lab, Purdue University, West Lafayette, IN, USA
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is furnished to do
    so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "HELPStat.h"
#include <vector>
#include <string>

/* Reference constants for gain values - taken from AD5941.h library by Analog Devices */
//#define HSTIARTIA_200               0     /**< HSTIA Internal RTIA resistor 200  */
//#define HSTIARTIA_1K                1     /**< HSTIA Internal RTIA resistor 1K   */
//#define HSTIARTIA_5K                2     /**< HSTIA Internal RTIA resistor 5K   */
//#define HSTIARTIA_10K               3     /**< HSTIA Internal RTIA resistor 10K  */
//#define HSTIARTIA_20K               4     /**< HSTIA Internal RTIA resistor 20K  */
//#define HSTIARTIA_40K               5     /**< HSTIA Internal RTIA resistor 40K  */
//#define HSTIARTIA_80K               6     /**< HSTIA Internal RTIA resistor 80K  */
//#define HSTIARTIA_160K              7     /**< HSTIA Internal RTIA resistor 160K */
//#define HSTIARTIA_OPEN              8     /**< Open internal resistor */

//#define EXCITBUFGAIN_2              0   /**< Excitation buffer gain is x2 */
//#define EXCITBUFGAIN_0P25           1   /**< Excitation buffer gain is x1/4 */

//#define HSDACGAIN_1                 0   /**< Gain is x1 */
//#define HSDACGAIN_0P2               1   /**< Gain is x1/5 */

/* Using I2C pins as GPIOs for buttons - Optional */
#define BUTTON 7
#define LEDPIN 6

calHSTIA test[] = {          
  {0.1,         HSTIARTIA_160K},     // Very low freq - electrode polarization ~50-100kΩ
  {0.5,         HSTIARTIA_80K},      // Low freq - still dominated by electrode effects  
  {1,           HSTIARTIA_40K},      // Transition from electrode to solution effects
  {5,           HSTIARTIA_20K},      // Mid-low frequencies ~10-20kΩ expected
  {10,          HSTIARTIA_10K},      // Transition region ~5-10kΩ
  {50,          HSTIARTIA_5K},       // Solution resistance becoming dominant ~2-5kΩ
  {100,         HSTIARTIA_1K},       // Solution resistance ~500-2kΩ
  {500,         HSTIARTIA_1K},       // Pure solution resistance ~300-1kΩ
  {1000,        HSTIARTIA_1K},       // Stable solution resistance ~200-800Ω
  {5000,        HSTIARTIA_1K},       // **CHANGED** from 200Ω - better SNR at high freq
  {10000,       HSTIARTIA_1K},       // **CHANGED** from 200Ω - reduce high-freq noise  
  {50000,       HSTIARTIA_1K}        // **CHANGED** from 200Ω - remove 100kHz point
};
 
/* Variables for function inputs */
int gainSize = (int)sizeof(test) / sizeof(test[0]); // Now 8 entries instead of 6

uint32_t numCycles = 10; 
uint32_t delaySecs = 2;  
uint32_t numPoints = 10; // 10 points per decade for good quality, reasonable time (~71 total points, ~45 minutes) 

float startFreq = 0.1; //0.15Hz
float endFreq = 100000; //200kHz
float biasVolt = 0.0; 
float zeroVolt = 0.0; 
float rcalVal = 9930; // Use the measured resistance of the chosen calibration resistor

int extGain = 1; 
int dacGain = 1; 

float rct_estimate = 127000;
float rs_estimate = 150;

HELPStat demo;

/* Initializing Analog Pins and Potentiostat pins */
String folderName = "folder-name-here"; 
String fileName = "file-name-here"; 

void setup() {

  delay(1000);
  Serial.begin(115200);
  
  demo.BLE_setup();
  /* Optional inputs to establish pins for button and LEDs*/
  pinMode(LEDPIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(LEDPIN, HIGH); // Initial LED State
  
  /* 
   *  Initializing Analog Pins and Potentiostat pins - need this function 
   *  regardless of running Analog Devices or our EIS code.
  */
  demo.AD5940Start();

  // Configuration is complete
  Serial.println("Pins configured! Press button to begin.");

   // put your main code here, to run repeatedly: 
  demo.BLE_settings();
  delay(10); // Additional Debounce delay - 10 ms 

  // if(buttonStatus == LOW)
  // {
  digitalWrite(LEDPIN, LOW);
  Serial.println("Button pressed - starting measurements!");

  demo.print_settings();

  // blinkLED(3, 0);
    
  /* Main Testing Code - also used for current draw as a standard sweep measurement */
  demo.AD5940_TDD(startFreq, endFreq, numPoints, biasVolt, zeroVolt, rcalVal, test, gainSize, extGain, dacGain); 
  demo.runSweep();
  //demo.calculateResistors();
  //demo.BLE_transmitResults();
  //demo.saveDataEIS(); 


    /* Current Draw Code - (no sweep but set for measurement)*/
  // demo.AD5940_TDD(startFreq, endFreq, 6, 0.0, 0.0, 9870, test, gainSize, 1, 1);
  // demo.configureFrequency(endFreq);

    /* After Impedance Measurement - drive LED High and get ready to restart measurement */
  delay(500);
  blinkLED(0, 1); 
  Serial.println("Ready to go again!");
  // }
}

void loop() {
  // put your main code here, to run repeatedly: 
  demo.BLE_settings();
  delay(10); // Additional Debounce delay - 10 ms 

  // if(buttonStatus == LOW)
  // {
  digitalWrite(LEDPIN, LOW);
  Serial.println("Button pressed - starting measurements!");
  // blinkLED(3, 0);
    
  /* Main Testing Code - also used for current draw as a standard sweep measurement */
  demo.AD5940_TDD(startFreq, endFreq, numPoints, biasVolt, zeroVolt, rcalVal, test, gainSize, extGain, dacGain); 
  demo.print_settings();
  demo.runSweep();
  //demo.calculateResistors();
  //demo.BLE_transmitResults();
  //demo.saveDataEIS(); 


    /* Current Draw Code - (no sweep but set for measurement)*/
  // demo.AD5940_TDD(startFreq, endFreq, 6, 0.0, 0.0, 9870, test, gainSize, 1, 1);
  // demo.configureFrequency(endFreq);

    /* After Impedance Measurement - drive LED High and get ready to restart measurement */
  delay(500);
  blinkLED(0, 1); 
  Serial.println("Ready to go again!");
  // }
}

void blinkLED(int cycles, bool state) {
  if(state) 
  {
    digitalWrite(LEDPIN, HIGH);
    delay(10);
  }
  else
  {
    for(int i = 0; i < cycles; i++)
    {
      digitalWrite(LEDPIN, HIGH); 
      delay(1000); 
      digitalWrite(LEDPIN, LOW);
      delay(1000);
    }
  }
}
