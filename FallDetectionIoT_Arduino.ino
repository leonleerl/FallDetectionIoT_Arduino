#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <axp20x.h>
 

 AXP20X_Class axp;


 // GPS instance
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

 // GPS data
double latitude = 0.0;
double longitude = 0.0;

// Wi-Fi credentials
const char* ssid = "LeonLee";           // 将此处替换为你的WiFi热点名称
const char* password = "001101001101"; // 将此处替换为你的WiFi密码

// 定义WebAPI的URL
const char* serverName = "http://172.20.10.11:5278/api/FallDetection"; // 确保你使用正确的IP和端口

void setup() {
  // WIFI Connection
  // 启动串口监视器，方便查看调试信息
  Serial.println("Start...");
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 34, 12); // GPS TX=34, RX=12

  // 开始连接WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  // 检查WiFi是否成功连接
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // 打印成功连接后的信息
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  // 打印设备的IP地址

 delay(1000);
}

void loop() {
    
  getGPSLocation();
  // Continuously read and process GPS data
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
 
  delay(5000); 
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
}


