#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>


#include <HardwareSerial.h>

// MPU6050 instance
Adafruit_MPU6050 mpu;

 // GPS instance
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

// Buzzer pin configuration (GPIO 4)
const int buzzerPin = 4;

// Threshold for triggering the buzzer
const double BUZZER_THRESHOLD = 100.0;

 // GPS data
double latitude = 0.0;
double longitude = 0.0;

// Wi-Fi credentials
const char* ssid = "LeonLee"; // Replace with your Wi-Fi SSID
const char* password = "001101001101";  // Replace with your Wi-Fi Password

// Web server URL (replace with your web server URL)
const char* serverName = "http://172.20.10.11:5278/api/FallDetection"; // 确保你使用正确的IP和端口

void setup() {
  // Initialize serial communication for debugging
  Serial.println("Start...");
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 34, 12); // GPS TX=34, RX=12

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

 // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
 
  // Configure the MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
 
  // Initialize buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off initially

  delay(100);
}

void loop() {
    
 // Get new sensor events from the MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
 
  // Calculate the magnitude of acceleration
  float accelerationMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                     a.acceleration.y * a.acceleration.y +
                                     a.acceleration.z * a.acceleration.z);
     Serial.println("Current Acceleration Magnitude: "+String(accelerationMagnitude));
  // Check if the acceleration magnitude exceeds the threshold
  if (accelerationMagnitude > BUZZER_THRESHOLD) {
    // Activate the buzzer
    // digitalWrite(buzzerPin, HIGH);
 
    // Keep the buzzer on for a short time (e.g., 1 second)
    delay(1000);
 
    // Turn off the buzzer after the delay
    digitalWrite(buzzerPin, LOW);
 
    // // Send the GPS location if available
    // sendGPSLocationToServer();
    getGPSLocation();
  }
 
  // Continuously read and process GPS data
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
 
  delay(100); // Adjust delay for responsiveness
}
// Function to send GPS latitude and longitude to the web server
void getGPSLocation() {
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
 
    // Print the GPS location to the Serial Monitor
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
  } 
  // else {
  //   Serial.println("Unable to get GPS Location")
  // }
}


