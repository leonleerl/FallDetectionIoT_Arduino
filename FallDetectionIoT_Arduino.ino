#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// MPU6050 instance
Adafruit_MPU6050 mpu;

 // GPS instance
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

// Buzzer pin configuration (GPIO 4)
const int buzzerPin = 4;

// Threshold for triggering the buzzer
const double BUZZER_THRESHOLD = 50.0;

 // GPS data
double latitude = 0.0;
double longitude = 0.0;

// Wi-Fi credentials
const char* ssid = "LeonLee"; // Replace with your Wi-Fi SSID
const char* password = "001101001101";  // Replace with your Wi-Fi Password

// Web server URL (replace with your web server URL)
const char* serverName = "http://172.20.10.5:5278/api/FallDetection"; 

void setup() {
  // Initialize serial communication for debugging
  Serial.println("Start...");
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 34, 12); // GPS TX=34, RX=12

  connectWIFI();

  InitializeMPU6050();

  // fetchData();

  delay(100);
}

// Function to fetch data from the WebAPI and display it
void fetchData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Specify the server URL
    http.begin(serverName);

    // Send the GET request
    int httpResponseCode = http.GET();

    // Check if the GET request was successful
    if (httpResponseCode > 0) {
      String response = http.getString();  // Get the response payload
      Serial.println("HTTP Response Code: " + String(httpResponseCode));
      Serial.println("Response:");
      Serial.println(response);

      // Parse JSON data
      StaticJsonDocument<2048> doc;
      DeserializationError error = deserializeJson(doc, response);

      // Check if parsing was successful
      if (error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.f_str());
        return;
      }

      // Iterate over the JSON array
      for (JsonObject obj : doc.as<JsonArray>()) {
        Serial.println("----- Fall Detection Record -----");
        Serial.print("ID: ");
        Serial.println(obj["id"].as<String>());
        Serial.print("Name: ");
        Serial.println(obj["name"].as<String>());
        Serial.print("Fall Date: ");
        Serial.println(obj["fallDate"].as<String>());
        Serial.print("Longitude: ");
        Serial.println(obj["longitude"].as<String>());
        Serial.print("Latitude: ");
        Serial.println(obj["latitude"].as<String>());
        Serial.print("Acceleration X: ");
        Serial.println(obj["accelX"].as<String>());
        Serial.print("Acceleration Y: ");
        Serial.println(obj["accelY"].as<String>());
        Serial.print("Acceleration Z: ");
        Serial.println(obj["accelZ"].as<String>());
        Serial.println("--------------------------------");
      }

    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
    }

    // End the HTTP connection
    http.end();
  } else {
    Serial.println("Wi-Fi is not connected.");
  }
}

void loop() {
    // Get new sensor events from the MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the magnitude of acceleration
    float accelerationMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                       a.acceleration.y * a.acceleration.y +
                                       a.acceleration.z * a.acceleration.z);

    // Check if the acceleration magnitude exceeds the threshold
    if (accelerationMagnitude > BUZZER_THRESHOLD) {
        Serial.println("Current Acceleration Magnitude: " + String(accelerationMagnitude));

        // Activate the buzzer
        digitalWrite(buzzerPin, HIGH);
        
        // Keep the buzzer on for a short time (e.g., 1 second)
        delay(1000);
        
        // Turn off the buzzer after the delay
        digitalWrite(buzzerPin, LOW);

        // Get the GPS location if available
        getGPSLocation();

        // Prepare JSON payload
        if (WiFi.status() == WL_CONNECTED) {
            HTTPClient http;
            http.begin(serverName); // Specify the server URL

            http.addHeader("Content-Type", "application/json"); // Set content type header

            StaticJsonDocument<200> jsonDoc;
            
            // Add relevant data to JSON payload without nested objects
            jsonDoc["Name"] = "Fall Event";  // Replace with actual event name if available
            jsonDoc["FallDate"] = "2024-10-04T12:34:56";  // Use real-time in ISO 8601 format if available
            jsonDoc["Latitude"] = String(latitude, 6);  // Get GPS latitude
            jsonDoc["Longitude"] = String(longitude, 6);  // Get GPS longitude
            jsonDoc["accelX"] = String(a.acceleration.x, 2); // X-axis acceleration
            jsonDoc["accelY"] = String(a.acceleration.y, 2); // Y-axis acceleration
            jsonDoc["accelZ"] = String(a.acceleration.z, 2); // Z-axis acceleration
            jsonDoc["IsChecked"] = false;  // Boolean indicating the status
            
            // Serialize JSON document to string
            String requestBody;
            serializeJson(jsonDoc, requestBody);

            // Send POST request
            int httpResponseCode = http.POST(requestBody);

            // Check the response
            if (httpResponseCode > 0) {
                String response = http.getString();  // Get the response payload
                Serial.println("HTTP Response Code: " + String(httpResponseCode));
                Serial.println("Response: " + response);
            } else {
                Serial.println("Error on sending POST: " + String(httpResponseCode));
            }

            // Serialize JSON document to string for debugging
            String tempPrint;
            serializeJson(jsonDoc, tempPrint);
            Serial.println("Request Body:");
            Serial.println(tempPrint);

            http.end(); // End HTTP connection
        } else {
            Serial.println("Wi-Fi not connected");
        }

        delay(1000); // Adjust delay for responsiveness and avoid rapid requests
    }
}


void connectWIFI(){
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
}

void InitializeMPU6050(){
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
}

// Function to send GPS latitude and longitude to the web server
void getGPSLocation() {
   Serial.println("Inside getGPSLocation function");
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




