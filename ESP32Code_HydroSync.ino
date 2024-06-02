boolean debugMode = false;  //debug mode on/off

#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <WiFi.h>               //for esp32
#include <WiFiClient.h>         //for esp32
#include <LiquidCrystal_I2C.h>  //https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

// ----------------Blynk Data Setup ----------------
// v0: ph
// v1: flow rate
// v2: turbidity
// v3: water level
// v4: total volume
// v5: Temperature
// v6: isAutoSwitch
// v7: TotalVolumeResetPin
// v8: Waflow calibration value
// v9: Solenoid Valve State
// ----------------Blynk Data Setup ----------------

/*--------PINS - ESP32--------*/
int ONE_WIRE_BUS = 19;      //D4  TEMP
int solenoidPin = 18;       //RX  SOL
int PH_SENSOR = 34;         //D5  PH SEN (ANALOG)
int TURBIDITY_SENSOR = 35;  //D3  TURB (ANALOG)
int WATERFLOW_SENSOR = 5;   //D0  WATER FLOW
int levelLowPin = 16;       //D7   WATER LEVEL LOW
int levelHighPin = 17;      //D8  WATER LEVEL HIGH

//________________ TEMPERATURE____________
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float Celsius = 0;
//________________ TEMPERATURE END_________

//---------PH START---------
float calibration_value = 21.34;
float ph_act;
float phvolt;
//---------PH END---------

//---------WaterFlow Start---------
int X;
int Y;
float TIME = 0;
float FREQUENCY = 0;
float WATER = 0;
float TOTAL = 0;
float LPM = 0;
unsigned long previousMillis = 0;
const long interval = 1000;  // 1000 milliseconds = 1 second
//---------WaterFlow END---------

//---------Turbidity Start---------
float turbvolt;
float ntu;
//---------Turbidity End---------

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "YourTemplateID"
#define BLYNK_TEMPLATE_NAME "HydroSync"
#define BLYNK_AUTH_TOKEN "YourAuthToken"

#include <BlynkSimpleEsp32.h>  //esp32

char ssid[] = "YourSSID";
char pass[] = "YourPassword";

int sensLow = 0;
int sensHigh = 0;

// for autoSwitching Solenoid Valve
int autoSwitch;
int isValveOpen;
int waterLevel;
int solenoidState;

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

//NEW FLOW SENSOR
long currentMillis = 0;
float calibrationFactor = 4.3;
volatile byte pulseCount = 0;
byte pulse1Sec = 0;
float flowRate = 0.0;
float flowLiters;
float totalLiters;

// Variables for scrolling text
String scrollText;
int scrollIndex = 0;
unsigned long lastScrollTime = 0;
const long scrollInterval = 300;  // 300 milliseconds = 0.3 seconds

void setup() {
  // Debug console
  Serial.begin(9600);
  pinMode(WATERFLOW_SENSOR, INPUT_PULLUP);  //Water Flow
  pinMode(solenoidPin, OUTPUT);             // activate solenoid
  pinMode(levelHighPin, INPUT_PULLUP);      //enable +bias resistors on pin
  pinMode(levelLowPin, INPUT_PULLUP);       //enable +bias resistors on pin

  digitalWrite(solenoidPin, LOW);  //turn off solenoid by default
  lcd.init();
  // lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("   Loading...");
  lcd.setCursor(0, 1);
  lcd.print("  Please Wait");

  while (!Blynk.connected()) {
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    delay(1000);
  }

  //FLOW SENSOR
  attachInterrupt(digitalPinToInterrupt(WATERFLOW_SENSOR), pulseCounter, FALLING);
  sensors.begin();  //temp sensor
  Wire.begin();
  delay(10);
}


void loop() {
  lcd.setCursor(0, 0);
  lcd.print("   HydroSync    ");

  while(!Blynk.connected()){
    digitalWrite(solenoidPin, LOW);
    lcd.print("Reconnecting... ");
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  }
  

  sensLow = digitalRead(levelLowPin);
  sensHigh = digitalRead(levelHighPin);

  waterLevel = getLevel();
  float loop_PH = readPH();
  float loop_Turbidity = readTurbidity();
  float loop_Temp = readTemperature();

  // Prepare the scrolling text
  scrollText = "               Flowrate: " + String(flowRate, 2) + "L/min   Total: " + String(totalLiters, 2) + "L   pH: " + String(loop_PH, 2) + "  Turbidity: " + String(loop_Turbidity, 2) + "ntu    Temp: " + String(loop_Temp, 2) + " deg. C               ";


  // Handle scrolling text
  if (millis() - lastScrollTime >= scrollInterval) {
    lcd.setCursor(0, 1);
    lcd.print(scrollText.substring(scrollIndex, scrollIndex + 16));
    scrollIndex++;
    if (scrollIndex + 16 > scrollText.length()) {
      scrollIndex = 0;
    }
    lastScrollTime = millis();
  }


  // Check if a second has elapsed
  if (millis() - previousMillis >= interval) {
    pulse1Sec = pulseCount;
    pulseCount = 0;

    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    flowLiters = (flowRate / 60);

    totalLiters += flowLiters;

    if (debugMode) {
      Serial.print("FlowRate: ");
      Serial.print(float(flowRate));
      Serial.print("L/min");
      Serial.print("\t");
      Serial.print("T. Liters: ");
      Serial.print(totalLiters);
      Serial.print("L");
      Serial.print("\t Cal. Factor: ");
      Serial.print(calibrationFactor);
      Serial.print("\t pH: ");
      Serial.print(loop_PH);
      Serial.print("\t pH Voltage: ");
      Serial.print(phvolt);
      Serial.print("\t Turbidity: ");
      Serial.print(loop_Turbidity);
      Serial.print(" tubvolt: ");
      Serial.print(turbvolt);
      Serial.print("\t Temp: ");
      Serial.print(loop_Temp);
    }
    Blynk.virtualWrite(V1, flowRate);
    Blynk.virtualWrite(V4, totalLiters);
    Blynk.virtualWrite(V0, loop_PH);
    Blynk.virtualWrite(V2, loop_Turbidity);
    Blynk.virtualWrite(V3, waterLevel);
    Blynk.virtualWrite(V5, loop_Temp);

    valveStateFunction();
  }
  Blynk.run();
}

float readTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float readPH() {
  int buffer_arr[10];
  unsigned long int avgval = 0;
  unsigned long int avg;
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(PH_SENSOR);
  }
  for (int i = 0; i < 8; i++) avgval += buffer_arr[i];
  avg = (avgval / 8);
  phvolt = (float)avg * 3.3 / 4095 + 0.07;
  ph_act = -5.70 * phvolt + calibration_value;
  return ph_act;
}

float readTurbidity() {
  int buffer_arr[10];
  unsigned long int avgval = 0;
  unsigned long int avg;
  turbvolt = 0;
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(TURBIDITY_SENSOR);
  }
  for (int i = 0; i < 8; i++) avgval += buffer_arr[i];
  avg = (avgval / 8);
  turbvolt = (float)avg * 3.06 / 4095 + 0.07;
  unsigned long mm = mapfloat(avg, 0, 4095, 3000, 0);
  if (mm < 150) {
    ntu = 0;
  } else {
    ntu = mm;
  }

  if (mm < 0) {
    // ntu = 0;
  }


  return ntu;
}

int getLevel() {
  if (sensLow == LOW && sensHigh == LOW) {
    return 2;
  } else if (sensLow == LOW && sensHigh == HIGH) {
    return 1;
  }
  return 0;
}

void valveStateFunction() {
  if (debugMode) {
    Serial.print("\t WaterLevel: ");
    Serial.print(waterLevel);
    Serial.print(", AutoSwitch: ");
    Serial.print(autoSwitch);
    Serial.print(", Solenoid:");
    Serial.println(digitalRead(solenoidPin));
  }
  if (autoSwitch == 1) {
    if (waterLevel == 0) {
      digitalWrite(solenoidPin, HIGH);
    } else if (waterLevel == 1) {
      digitalWrite(solenoidPin, HIGH);
    } else if (waterLevel >= 2) {
      digitalWrite(solenoidPin, LOW);
    }
  } else {
    if (isValveOpen == 1) {
      if (waterLevel >= 2) {
        digitalWrite(solenoidPin, LOW);
      } else {
        digitalWrite(solenoidPin, HIGH);
      }
    } else {
      digitalWrite(solenoidPin, LOW);
    }
  }
}
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

BLYNK_WRITE(V6) {
  if (debugMode) Serial.println("BLYNK_WRITE(V6) - called");
  int isSetToAutoSwitch = param.asInt();
  if (isSetToAutoSwitch == 1) {
    autoSwitch = 1;
    if (debugMode) Serial.println("autoSwitch True");
  } else {
    autoSwitch = 0;
    if (debugMode) Serial.println("autoSwitch False");
  }
}

// Trigger for resetting the Total Value
BLYNK_WRITE(V7) {
  if (debugMode) Serial.println("BLYNK_WRITE(V7) - called");
  int totalVolumeResetPin = param.asInt();
  if (totalVolumeResetPin == 1) {
    if (TOTAL != 0) {
      TOTAL = 0;
    }
  }
}

BLYNK_WRITE(V8) {
  if (debugMode) Serial.println("BLYNK_WRITE(V8) - called");
  float newCalibrationFactor = param.asFloat();
  if (debugMode) Serial.print("new Calibration Value: ");
  if (debugMode) Serial.println(newCalibrationFactor);
  if (calibrationFactor != newCalibrationFactor) {
    calibrationFactor = newCalibrationFactor;
  }
}

BLYNK_WRITE(V9) {
  if (debugMode) Serial.println("BLYNK_WRITE(V9) - called");
  solenoidState = param.asInt();
  if (solenoidState == 1) {
    isValveOpen = 1;
  } else {
    isValveOpen = 0;
  }
}

BLYNK_CONNECTED() {
  Blynk.syncAll();
}

ICACHE_RAM_ATTR void pulseCounter() {
  pulseCount++;
}