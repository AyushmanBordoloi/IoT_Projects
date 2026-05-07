#include <WiFi.h>
#include <WebServer.h>

const char* ssid     = "NoName";
const char* password = "NoNamelol";

WebServer server(80);

void handleRoot() {
  String html = R"(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32 Dashboard</title>
      <meta name='viewport' content='width=device-width, initial-scale=1'>
      <style>
        body { font-family: Arial; text-align: center; padding: 40px; background: #1a1a2e; color: white; }
        h1 { color: #00d4ff; }
        .card { background: #16213e; padding: 20px; border-radius: 12px; margin: 20px auto; max-width: 300px; }
      </style>
    </head>
    <body>
      <h1>ESP32 Dashboard</h1>
      <div class='card'><p>✅ Connected via Home WiFi!</p></div>
    </body>
    </html>
  )";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("Open this in your browser: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
}