#include <WiFi.h>

const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

WiFiServer server(80);
String header;

String output1State = "off";
String output2State = "off";
String output3State = "off";
String output4State = "off";
String output5State = "off";
String output6State = "off";

// Safe GPIOs
const int output1 = 14;
const int output2 = 27;
const int output3 = 26;
const int output4 = 25;
const int output5 = 33;
const int output6 = 32;

void setup() {
  Serial.begin(115200);
  Serial.println("Boot successful");

  pinMode(output1, OUTPUT);
  pinMode(output2, OUTPUT);
  pinMode(output3, OUTPUT);
  pinMode(output4, OUTPUT);
  pinMode(output5, OUTPUT);
  pinMode(output6, OUTPUT);

  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);
  digitalWrite(output3, LOW);
  digitalWrite(output4, LOW);
  digitalWrite(output5, LOW);
  digitalWrite(output6, LOW);

  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        header += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if (header.indexOf("GET /1/on") >= 0) {
              Serial.println("GPIO 1 on");
              output1State = "on";
              digitalWrite(output1, HIGH);
            } else if (header.indexOf("GET /1/off") >= 0) {
              Serial.println("GPIO 1 off");
              output1State = "off";
              digitalWrite(output1, LOW);
            }

            if (header.indexOf("GET /2/on") >= 0) {
              Serial.println("GPIO 2 on");
              output2State = "on";
              digitalWrite(output2, HIGH);
            } else if (header.indexOf("GET /2/off") >= 0) {
              Serial.println("GPIO 2 off");
              output2State = "off";
              digitalWrite(output2, LOW);
            }

            if (header.indexOf("GET /3/on") >= 0) {
              Serial.println("GPIO 3 on");
              output3State = "on";
              digitalWrite(output3, HIGH);
            } else if (header.indexOf("GET /3/off") >= 0) {
              Serial.println("GPIO 3 off");
              output3State = "off";
              digitalWrite(output3, LOW);
            }

            if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");
              output4State = "on";
              digitalWrite(output4, HIGH);
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");
              output4State = "off";
              digitalWrite(output4, LOW);
            }

            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("GPIO 5 on");
              output5State = "on";
              digitalWrite(output5, HIGH);
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("GPIO 5 off");
              output5State = "off";
              digitalWrite(output5, LOW);
            }

            if (header.indexOf("GET /6/on") >= 0) {
              Serial.println("GPIO 6 on");
              output6State = "on";
              digitalWrite(output6, HIGH);
            } else if (header.indexOf("GET /6/off") >= 0) {
              Serial.println("GPIO 6 off");
              output6State = "off";
              digitalWrite(output6, LOW);
            }

            client.println("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>");
            client.println("<style>html{font-family:Helvetica;text-align:center;}.button{padding:16px 40px;font-size:30px;}");
            client.println(".button2{background-color:#555;}</style></head><body><h1>ESP32 Web Server</h1>");

            auto sendButton = [&](int pin, String state) {
              client.println("<p>GPIO " + String(pin) + " - State " + state + "</p>");
              if (state == "off")
                client.println("<p><a href='/" + String(pin) + "/on'><button class='button'>ON</button></a></p>");
              else
                client.println("<p><a href='/" + String(pin) + "/off'><button class='button button2'>OFF</button></a></p>");
            };

            sendButton(1, output1State);
            sendButton(2, output2State);
            sendButton(3, output3State);
            sendButton(4, output4State);
            sendButton(5, output5State);
            sendButton(6, output6State);

            client.println("</body></html>");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    header = "";
    client.stop();
    Serial.println("Client disconnected.");
  }
}
