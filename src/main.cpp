#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
HardwareSerial SerialGPS(1); // Use Serial1 for ESP32

// Define the SoftwareSerial object for SIM808 communication
SoftwareSerial SIM808_SERIAL(22, 23); // RX pin: D22, TX pin: D23

const char* ssid = "Krish"; // Change this to your WiFi SSID
const char* password = "8atix9ia"; // Change this to your WiFi password

// Replace "YOUR_THINGSPEAK_API_KEY" with your actual ThingSpeak API key
const char* thingSpeakApiKey = "3FNKIUCYX6RAKO7Y";
const int channel_ID = 2473236;

float Latitude, Longitude, xAccel, yAccel, zAccel;
int alcoholValue;

WiFiClient client;
// Define Analog Input Pins
#define LED 2
#define ALCOHOL_SENSOR_PIN 35 // Assuming alcohol sensor is connected to analog pin D35 - A0
#define X_AXIS_PIN 32 // ADC1_4 (Interrupt-capable pin)
#define Y_AXIS_PIN 33 // ADC1_5 (Interrupt-capable pin)
#define Z_AXIS_PIN 34 // ADC1_6 (Interrupt-capable pin)

// Define Threshold for Alcohol Sensor 
#define ALCOHOL_THRESHOLD 1020 // Adjust the threshold value accordingly

// Define Calibration Values (adjust as needed)
#define X_ZERO_G_VOLTAGE 1.65 // ADXL335's zero-g voltage for X-axis (should be around 1.65V)
#define Y_ZERO_G_VOLTAGE 1.65 // ADXL335's zero-g voltage for Y-axis (should be around 1.65V)
#define Z_ZERO_G_VOLTAGE 1.65 // ADXL335's zero-g voltage for Z-axis (should be around 1.65V)
#define SENSITIVITY 0.3       // ADXL335's sensitivity (should be around 0.3V/g)

// Define Thresholds for Accident Detection (adjust as needed)
#define ACCIDENT_THRESHOLD_X 1.5  // Threshold for X-axis acceleration (in g)
#define ACCIDENT_THRESHOLD_Y 1.5  // Threshold for Y-axis acceleration (in g)
#define ACCIDENT_THRESHOLD_Z 1.5  // Threshold for Z-axis acceleration (in g)
#define ACCIDENT_THRESHOLD_XN -1.5 // Threshold for X-axis acceleration (in g)
#define ACCIDENT_THRESHOLD_YN -1.5 // Threshold for Y-axis acceleration (in g)
#define ACCIDENT_THRESHOLD_ZN -1.5 // Threshold for Z-axis acceleration (in g)

// Define AT Command Strings
String AT = "AT\r\n";
String AT_CMGF = "AT+CMGF=1\r\n"; // Set SMS mode to text
String AT_CMGS = "AT+CMGS=\"7284890760\"\r\n"; // Replace PHONE_NUMBER with the destination phone number
String SMS_TEXT = "Accident Detected at here - ";

unsigned long previousMillis = 0; // Variable to store the last time data was sent to ThingSpeak
const long interval = 10000; // Interval to send data to ThingSpeak (in milliseconds)

volatile bool accidentDetected = false; // Flag to indicate accident detection

// Send AT command to SIM808
void sendATCommand(String cmd) {
  SIM808_SERIAL.println(cmd);
  delay(500);
  while (SIM808_SERIAL.available()) {
    Serial.write(SIM808_SERIAL.read());
  }
}

// Send SMS function with latitude, longitude, and Google Maps link
void sendSMS(String text, float lat, float lon) {
  String message = text + " Latitude: " + String(lat, 6) + " Longitude: " + String(lon, 6);
  message += " Google Maps link: https://www.google.com/maps?q=" + String(lat, 6) + "," + String(lon, 6);

  sendATCommand(AT_CMGF);
  delay(500);
  sendATCommand(AT_CMGS);
  delay(500);
  SIM808_SERIAL.print(message);
  SIM808_SERIAL.write(0x1A);
  delay(5000);
}


float readAccel(int pin) {
  int rawValue = analogRead(pin);
  return ((rawValue * 3.3 / 4096) - 1.65) / 0.3; // Adjust zero-g voltage and sensitivity as needed
}

void sendToThingSpeak(){
  // Send GPS latitude and longitude to ThingSpeak
  ThingSpeak.setField(1, Latitude);
  ThingSpeak.setField(2, Longitude);

  // Send alcohol sensor value to ThingSpeak
  ThingSpeak.setField(3, alcoholValue);

  // Send Accelerometer!
  ThingSpeak.setField(4, xAccel);
  ThingSpeak.setField(5, yAccel);
  ThingSpeak.setField(6, zAccel);

  // Write the fields to the ThingSpeak channel
  ThingSpeak.writeFields(channel_ID,thingSpeakApiKey);
  Serial.println("Data sent to ThingSpeak.");

  if (alcoholValue > ALCOHOL_THRESHOLD) {
    digitalWrite(LED,HIGH);   // Turn the LED on if alcohol is detected
    delay(3000);              // Wait for a short duration
    digitalWrite(LED,LOW);    // Turn the LED off after the
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); //For MQ3 sensor
  Serial.begin(921600);
  //GPS module NEO-6M
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // RX2 pin 16, TX2 pin 17
  delay(100);

  // Start serial communication with SIM808
  SIM808_SERIAL.begin(9600); // 

  // Ensure SIM card is ready
  sendATCommand(AT);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize ThingSpeak
  ThingSpeak.begin(client);// Initialize ThingSpeak

  // Set up ADC pins
  pinMode(X_AXIS_PIN, INPUT);
  pinMode(Y_AXIS_PIN, INPUT);
  pinMode(Z_AXIS_PIN, INPUT);

  Serial.println("Setup complete.");
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  if ((currentMillis - previousMillis) >= interval) { // Check if it's time to send data to ThingSpeak
    sendToThingSpeak(); // Call function to send data to ThingSpeak
    previousMillis = currentMillis; // Update the previous time
  }

  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        Latitude = gps.location.lat();
        Longitude = gps.location.lng();
      }
    }
  }

  // Read alcohol sensor value
  alcoholValue = analogRead(ALCOHOL_SENSOR_PIN);

  // Read accelerometer data
  xAccel = readAccel(X_AXIS_PIN);
  yAccel = readAccel(Y_AXIS_PIN);
  zAccel = readAccel(Z_AXIS_PIN);

  //Check for accident detection based on accelerometer thresholds
  if (xAccel > ACCIDENT_THRESHOLD_X || yAccel > ACCIDENT_THRESHOLD_Y  || zAccel > ACCIDENT_THRESHOLD_Z || xAccel < ACCIDENT_THRESHOLD_XN || yAccel < ACCIDENT_THRESHOLD_YN || zAccel < ACCIDENT_THRESHOLD_ZN) {
    accidentDetected = true;
    //GSM code part--- Come out of the loop upon detection! (Not part of thingSpeak)
    //Send SMS when accident detected
    sendSMS(SMS_TEXT, Latitude, Longitude); // Include latitude and longitude
    while(true) { }
  } 
  else {
    accidentDetected = false;
  }

  delay(100); // Adjust delay as needed
}
