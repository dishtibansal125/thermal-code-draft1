# thermal-code-draft1
# not tested yet
# Thermaldraft.ino
Test 
#include &lt;Arduino.h&gt;
#include &lt;Wire.h&gt;
#include &lt;SPI.h&gt;
#include &lt;Adafruit_MAX31865.h&gt;
#include &lt;SparkFun_TMP117.h&gt;
// Global Variable and Constant Definitions
// Pin Definitions
// Heaters (Coliao 12V 12W Kapton strips via IRFZ44N MOSFETs)
const int heaterBattery_pin = 3; // H1 - Battery pack, 2 strips, PWM
const int heaterLeftMotor_pin = 5; // H2 - Left motor, 1 strip, PWM
const int heaterRightMotor_pin = 6; // H3 - Right motor, 1 strip, PWM
const int heaterElecBay_pin = 1; // H4 - Electronics bay, 1 strip, ON/OFF
// Sensors - Analog (NTC Thermistors)
const int ntc_pin_battery = A0; // S1 - Battery pack
const int ntc_pin_driver = A1; // S4 - L298 motor driver
// Sensors - SPI (PT100 RTD via MAX31865)
const int rtd_pin_cs_left = A2; // S2 - Left motor PT100 chip select
const int rtd_pin_cs_right = A3; // S3 - Right motor PT100 chip select
// Sensors - I2C (TMP117, Adafruit 4821)
// S5 - Electronics bay, SDA=A4, SCL=A5
//
// Temperature thresholds (C)
const float temp_heaterOn = 15.0; // Heater kicks on below this
const float temp_heaterOff = 28.0; // Heater shuts off above this
const float temp_criticalLow = 10.0; // Critical cold, max heater power
const float temp_criticalHigh = 40.0; // Critical hot, force heaters off
const float temp_sensorFault = -999.0; // Returned if sensor fails
// Heater PWM levels (0-255 = 0-12W per strip)
const int pwm_batteryNormal = 159; // H1 - ~15W total (2 strips at ~7.5W each)
const int pwm_batteryMax = 255; // H1 - 24W total (critical cold)
const int pwm_motorNormal = 212; // H2/H3 - ~10W
const int pwm_motorMax = 255; // H2/H3 - 12W (critical cold)
const int pwm_off = 0;
// NTC thermistor constants (AOHIEDI 10k NTC, Beta equation)
const float ntc_nominalR = 10000.0; // 10k ohm at 25C
const float ntc_nominalTemp = 25.0; // Reference temperature (C)
const float ntc_beta = 3950.0; // Beta coefficient
const float ntc_seriesR = 10000.0; // Series resistor in voltage divider
const int adc_max = 1023; // 10-bit ADC max
//
// Sensor objects
Adafruit_MAX31865 rtd_leftMotor = Adafruit_MAX31865(rtd_pin_cs_left); // S2 - Left motor PT100
Adafruit_MAX31865 rtd_rightMotor = Adafruit_MAX31865(rtd_pin_cs_right); // S3 - Right motor PT100
TMP117 tmp117_elecBay; // S5 - Electronics bay
// Temperature readings
float temp_battery = 0.0; // S1 - Battery pack
float temp_leftMotor = 0.0; // S2 - Left motor

float temp_rightMotor = 0.0; // S3 - Right motor
float temp_driver = 0.0; // S4 - L298 motor driver
float temp_elecBay = 0.0; // S5 - Electronics bay
// Reads NTC thermistor pin and returns temperature in Celsius
float readNTC(int pin) {
int raw = analogRead(pin);
if (raw == 0 || raw == adc_max) return temp_sensorFault; // Open or short circuit
float resistance = ntc_seriesR * ((float)adc_max / raw - 1.0);
float steinhart = log(resistance / ntc_nominalR) / ntc_beta;
steinhart += 1.0 / (ntc_nominalTemp + 273.15);
return (1.0 / steinhart) - 273.15;
}
// Reads all 5 sensors and updates temp variables
void readAllSensors() {
temp_battery = readNTC(ntc_pin_battery); // S1 - Battery NTC
temp_leftMotor = rtd_leftMotor.temperature(100, 430); // S2 - Left motor PT100
if (rtd_leftMotor.readFault()) {
rtd_leftMotor.clearFault();
temp_leftMotor = temp_sensorFault;
}
temp_rightMotor = rtd_rightMotor.temperature(100, 430); // S3 - Right motor PT100
if (rtd_rightMotor.readFault()) {
rtd_rightMotor.clearFault();
temp_rightMotor = temp_sensorFault;
}
temp_driver = readNTC(ntc_pin_driver); // S4 - Driver NTC
if (tmp117_elecBay.dataReady()) // S5 - Electronics bay TMP117
temp_elecBay = tmp117_elecBay.readTempC();
}
// H1 - Battery pack heater, 2 strips on pin 3, reads S1
void controlHeater_Battery() {
if (temp_battery == temp_sensorFault) {
analogWrite(heaterBattery_pin, pwm_batteryNormal); // Fault fallback
return;
}
if (temp_battery &lt;= temp_criticalLow) {
analogWrite(heaterBattery_pin, pwm_batteryMax); // Max power
} else if (temp_battery &lt; temp_heaterOn) {
analogWrite(heaterBattery_pin, pwm_batteryNormal); // Normal heating
} else if (temp_battery &gt;= temp_heaterOff || temp_battery &gt;= temp_criticalHigh) {
analogWrite(heaterBattery_pin, pwm_off); // Too warm
}
}
// H2 - Left motor heater, 1 strip on pin 5, reads S2
void controlHeater_LeftMotor() {
if (temp_leftMotor == temp_sensorFault) {
analogWrite(heaterLeftMotor_pin, pwm_motorNormal / 2); // Fault fallback
return;

}
if (temp_leftMotor &lt;= temp_criticalLow) {
analogWrite(heaterLeftMotor_pin, pwm_motorMax); // Max power
} else if (temp_leftMotor &lt; temp_heaterOn) {
analogWrite(heaterLeftMotor_pin, pwm_motorNormal); // Normal heating
} else if (temp_leftMotor &gt;= temp_heaterOff || temp_leftMotor &gt;= temp_criticalHigh) {
analogWrite(heaterLeftMotor_pin, pwm_off); // Too warm
}
}
// H3 - Right motor heater, 1 strip on pin 6, reads S3
void controlHeater_RightMotor() {
if (temp_rightMotor == temp_sensorFault) {
analogWrite(heaterRightMotor_pin, pwm_motorNormal / 2); // Fault fallback
return;
}
if (temp_rightMotor &lt;= temp_criticalLow) {
analogWrite(heaterRightMotor_pin, pwm_motorMax); // Max power
} else if (temp_rightMotor &lt; temp_heaterOn) {
analogWrite(heaterRightMotor_pin, pwm_motorNormal); // Normal heating
} else if (temp_rightMotor &gt;= temp_heaterOff || temp_rightMotor &gt;= temp_criticalHigh) {
analogWrite(heaterRightMotor_pin, pwm_off); // Too warm
}
}
// H4 - Electronics bay heater, 1 strip on pin 1, reads S5
void controlHeater_ElecBay() {
if (temp_elecBay &lt;= temp_heaterOn) {
digitalWrite(heaterElecBay_pin, HIGH); // Heater on
} else if (temp_elecBay &gt;= temp_heaterOff || temp_elecBay &gt;= temp_criticalHigh) {
digitalWrite(heaterElecBay_pin, LOW); // Heater off
}
}
// Prints all sensor readings to serial monitor
void printThermalStatus() {
Serial.println(&quot;===== THERMAL STATUS =====&quot;);
Serial.print(&quot;S1 Battery: &quot;); Serial.print(temp_battery); Serial.println(&quot; C&quot;);
Serial.print(&quot;S2 Left Motor: &quot;); Serial.print(temp_leftMotor); Serial.println(&quot; C&quot;);
Serial.print(&quot;S3 Right Motor: &quot;); Serial.print(temp_rightMotor); Serial.println(&quot; C&quot;);
Serial.print(&quot;S4 Driver: &quot;); Serial.print(temp_driver); Serial.println(&quot; C&quot;);
Serial.print(&quot;S5 Elec Bay: &quot;); Serial.print(temp_elecBay); Serial.println(&quot; C&quot;);
Serial.println(&quot;==========================&quot;);
}
// Initializes sensors and heater pins, sets all heaters to off
void setup() {
Serial.begin(9600); // Initialize serial communication
Wire.begin(); // Start I2C for TMP117
tmp117_elecBay.begin();
rtd_leftMotor.begin(MAX31865_3WIRE); // Start MAX31865 for left motor PT100
rtd_rightMotor.begin(MAX31865_3WIRE); // Start MAX31865 for right motor PT100
// Set heater pins as outputs, all off at start

pinMode(heaterBattery_pin, OUTPUT); analogWrite(heaterBattery_pin, pwm_off);
pinMode(heaterLeftMotor_pin, OUTPUT); analogWrite(heaterLeftMotor_pin, pwm_off);
pinMode(heaterRightMotor_pin, OUTPUT); analogWrite(heaterRightMotor_pin, pwm_off);
pinMode(heaterElecBay_pin, OUTPUT); digitalWrite(heaterElecBay_pin, LOW);
}
//
void loop() {
readAllSensors();
printThermalStatus();
controlHeater_Battery(); // H1 - Battery pack
controlHeater_LeftMotor(); // H2 - Left motor
controlHeater_RightMotor(); // H3 - Right motor
controlHeater_ElecBay(); // H4 - Electronics bay
delay(2000); // Check thermal state every 2 seconds
}
