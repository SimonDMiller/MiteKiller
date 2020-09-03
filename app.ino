#include <OneButton.h>
#include <PID_v1.h>
#include <OneWire.h>              // Dependency Dallas Temperature
#include <DallasTemperature.h>
#include <TimeLib.h>



// Global variables live here
#define numSamples 10              // Samples of temps to average (define for array sizing)
#define THERMISTORPIN A0          // ADC Pin for Pad Thermocouple
#define DS18B20PIN 12             // IO for OneWire Probe
#define BUTTONPIN 3  
#define RELAYONE 6                
#define RELAYTWO 5
#define LED_RED 17
#define LED_GRN 18
#define LED_BLUE 19

double sensorPadValue = 0;       // Sensor in thermal pad itself - PTC100 NTC3950
double sensorProbeValue;         // Thermocouple at top of hive - DS18B20   
double outputPWM = 0;            // PWM duty cycle 0-100%
double outputSetpoint = 106;     // Final Setpoint (106F)
bool outputStatus = false;       // Options True = Running & False = Cancel
int coeff_P = 2;                 // Gain coefficient, default is ??
int coeff_I = 0;                 // Integral coefficient, default is ??
int coeff_D = 0;                 // Derivative coefficient, default is ??
double heatWindowSize = 1000;          // Window for 'PWM' relay control in msec
unsigned long windowStartTime;      // Temp value to start Window
unsigned long timeAtHeat = 0;
unsigned long startTime = 0;

long seriesResistor = 100000;       // Value of series resistor for pad thermocouple PTC 3950
long padResistance = 100000;        // Value of pad thermocouple resistance
int tempNominal = 25;               // Assumed Room Temp for calibration
int BCOEFFICIENT = 3950;          // Beta coefficient for pad thermocouple, default 3950
int padSamples[numSamples];       // Sample array Pad Thermocouple

OneWire oneWire(DS18B20PIN);          // Create OneWire
DallasTemperature sensors(&oneWire);  // Pass to Dallas Temp Module
PID myPiD(&sensorProbeValue, &outputPWM, &outputSetpoint, 2, 300, 300, P_ON_M, DIRECT); // Initialize with defaults
OneButton startStop(BUTTONPIN); // Create button, active LOW, pullup off

void setup()    
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(RELAYONE,OUTPUT);       //Set Relay Pin to output and off 
  digitalWrite(RELAYONE,HIGH);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GRN, HIGH);    // Common Anode, Active Low LED
  pinMode(LED_GRN, OUTPUT);
  digitalWrite(LED_RED, HIGH);    // Common Anode, Active Low LED
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);   // Common Anode, Active Low LED
  digitalWrite(LED_RED, LOW);

  setTime(0,0,0,1,1,2020);       //Set time to midnight Jan 1st. Used for ramp and interval only
	sensors.begin();
  myPiD.SetOutputLimits(0, heatWindowSize);
  //startStop.attachClick(click);
  startStop.attachPress(click);
  windowStartTime = millis();
}
void loop()
{
  startStop.tick();
  sensors.requestTemperatures();      // Onewire Sensor requires processing time - call before ADC, read after. 
  
  // *****************************************
  // This section grabs and processes the temp from the internal sensor in the 
  // heating pad unit. Currently a PTC100 with Beta 3950. It's used to be sure 
  // the bees don't get burned by heating too fast
  //******************************************

  // Grab 5 samples 
  uint8_t i;
  float average;
  for (i=0; i< numSamples; i++) {
   padSamples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< numSamples; i++) {
     average += padSamples[i];
  }
  average /= numSamples;      // process average to get from counts to resistance
  average = 1023 / average - 1;
  average = seriesResistor / average;
  
  // Convert resistance value from previous step to temp in C
  double steinhart;
  steinhart = average / padResistance;        // (R/Ro)
  steinhart = log(steinhart);                 // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                  // 1/B * ln(R/Ro)
  steinhart += 1.0 / (tempNominal + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                // Invert
  steinhart -= 273.15;                        // convert to C
  sensorPadValue = (steinhart)*9/5 + 32;      // convert to F 

  if(sensorPadValue > 150){
    digitalWrite(RELAYONE, HIGH);
  }

// Get Probe temps from One Wire Digital Temperature Sensor - DS18B20
// Requires OneWire library 
// Sensor requested above, should be ready to read now

double sensorProbeRAW = sensors.getTempCByIndex(0);
sensorProbeValue = ((sensorProbeRAW * 9.0) / 5.0 + 32);
startStop.tick();

if (outputStatus){
  bool atTemp = evaluateTemp(sensorProbeValue);
  selectLED(atTemp);
  accumulateTime(atTemp);
  cycleHeat();
  }
  else{
  digitalWrite(LED_GRN, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE,LOW);
  outputPWM = 0;
  }
double outputPWMScaled = outputPWM/10;
sendToPC(&sensorProbeValue, &sensorPadValue, &outputPWMScaled, &timeAtHeat);
}

///////////////////////////END LOOP//////////////////////////////

bool evaluateTemp(double tmp)
{
  if(106 >= tmp && tmp >= 104){
    return true;
  } 
  else {
    return false;
  } 
}

void accumulateTime(bool atTemp)
{
  if (atTemp){
    //  FIGURE OUT HOW TO DO TIMER HERE
  }
  else{
    //  FIGURE OUT HOW TO DO TIMER HERE
  }

}

void sendToPC(double* data1, double* data2, double* data3, unsigned long* data4)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte buf[16] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                byteData4[0], byteData4[1], byteData4[2], byteData4[3]};
  Serial.write(buf, 16);
}

void selectLED(bool atTemp)
{
    digitalWrite(LED_BLUE, HIGH);             // System is heating, kill blue LED

    if(atTemp){             // In correct range - green LED should be lit
      digitalWrite(LED_GRN, LOW);
      digitalWrite(LED_RED, HIGH);
    }      
    else{
      digitalWrite(LED_RED, LOW);             // Out of range (high or low) - red LED should be lit
      digitalWrite(LED_GRN, HIGH);
    }
}

void cycleHeat()
  {
    myPiD.Compute();

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > heatWindowSize)
  { //time to shift the Relay Window
    windowStartTime += heatWindowSize;
  }
  if (outputPWM > now - windowStartTime) 
    digitalWrite(RELAYONE, LOW);
  else 
    digitalWrite(RELAYONE, HIGH);
    

}
void click()
{
 if(outputStatus)                       // System is running
 {
   outputStatus = false;                // Toggle status to OFF
   myPiD.SetMode(MANUAL);               // Toggle PID Controller to OFF
   outputPWM = 0;                       // FORCE output to 0
   digitalWrite(RELAYONE, HIGH);        // FORCE relay off for safety

 } 
 else{
   outputStatus = true;                 // System is not running
   myPiD.SetMode(AUTOMATIC);            // Turn on PID to begin heating
   
 }
}

    
