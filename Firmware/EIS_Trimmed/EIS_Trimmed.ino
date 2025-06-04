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
    copies or substantial portions of the Software.AD594x_EIS_Demo/EIS_Trimmed.ino
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include <HelpStat.h>
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

// Initial Parameters 

#define BUTTON 7
#define LEDPIN 6

calHSTIA test[] = {          
  {0.51,  HSTIARTIA_40K},       
  {1.5,         HSTIARTIA_10K},
  {20,      HSTIARTIA_5K},
  {150,       HSTIARTIA_5K},
  {400,         HSTIARTIA_1K},
  {100000,     HSTIARTIA_200}  
}; 

/* Variables for function inputs */

int gainSize = (int)sizeof(test) / sizeof(test[0]);

uint32_t numCycles = 0; 
uint32_t delaySecs = 0; 
uint32_t numPoints = 6; 

float startFreq = 100000; 
float endFreq = 1; 
float biasVolt = 0.0; 
float zeroVolt = 0.0; 
float rcalVal = 10000; // Use the measured resistance of the chosen calibration resistor

int extGain = 1; 
int dacGain = 1; 

HELPStat demo; 

String folderName = "Results"; 
String fileName = "EIS_data_1";

void setup(){ 

    Serial.begin(115200);

    demo.BLE_setup();   

    pinMode(LEDPIN, OUTPUT); 
    pinMode(BUTTON, INPUT_PULLUP); 
    digitalWrite(LEDPIN, HIGH); 

    demo.AD5940Start(); // Initialize the AD5940

    Serial.println("Pins configured! Press button to begin"); 

}

void loop(){ 
    demo.BLE_settings(); 
    delay(10); 

    digitalWrite(LEDPIN, LOW); 
    delay(1000); 
    Serial.println("Button pressed - starting measurements!"); 

    demo.print_settings(); 

    // Main Testing Code:
    demo.AD5940_TDD(test, gainSize); 
    demo.runSweep(); 
    std::vector<float> resistors = demo.calculateResistors();
    //demo.BLE_transmitResults(); 
    demo.saveDataEIS(folderName, fileName); 

    delay(500); 
    blinkLED(0, 1);
    Serial.println("Ready to go again!"); 
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