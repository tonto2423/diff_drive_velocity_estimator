#include <WiFi.h>

const char* ssid = "ESP32_AP";
const char* password = "Your_AP_PASSWORD";

WiFiServer server(80);

// HTMLを文字列として埋め込む
String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Motor Control</title>
</head>
<body>
    <h2>ESP32 Motor Control</h2>
    <p>Click a button to control the motor:</p>
    <button onclick="location.href='/turnOn'">Turn ON</button>
    <button onclick="location.href='/turnOff'">Turn OFF</button>
</body>
</html>
)";

void setup() {
  Serial.begin(115200);

  // Set up the ESP32 as an access point
  WiFi.softAP(ssid, password);
  server.begin();

  Serial.println("Server started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  pinMode(22, OUTPUT);
}

void loop() {
  WiFiClient client = server.available();
//  Serial.println("来てる？");
  if (client) {
  	Serial.println("来たぜ卍");
    String request = client.readStringUntil('\r');
    client.flush();

    if (request.indexOf("/turnOn") != -1) {
      // Handle motor turn ON here
      Serial.println("Motor ON");
		digitalWrite(22, HIGH);
    }
    if (request.indexOf("/turnOff") != -1) {
      // Handle motor turn OFF here
      Serial.println("Motor OFF");
		digitalWrite(22, LOW);
    }

    // Send the response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    client.println(html);
    client.println();

    delay(1);
  }
}
