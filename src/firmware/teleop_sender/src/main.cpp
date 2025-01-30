#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define PIN_GND 33
#define PIN_VCC 32
#define PIN_DX 35
#define PIN_DY 34
#define PIN_BTN 39

#define THRESHOLD 5

void readMacAddress()
{
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK)
  {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  }
  else
  {
    Serial.println("Failed to read MAC address");
  }
}

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x08, 0xb6, 0x1f, 0xef, 0xa5, 0xd0};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  int x;
  int y;
  int btn;
} struct_message;

float offset_x, offset_y;
float threshold = 0.01;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  readMacAddress();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(PIN_GND, OUTPUT);
  pinMode(PIN_VCC, OUTPUT);
  pinMode(PIN_DX, INPUT);
  pinMode(PIN_DY, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

  digitalWrite(PIN_GND, 0);
  digitalWrite(PIN_VCC, 1);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  int samples = 20;
  for (int cnt = 0; cnt < samples; cnt++)
  {
    offset_x += analogRead(PIN_DX);
    offset_y += analogRead(PIN_DY);
  }
  offset_x /= samples;
  offset_y /= samples;
}

long millisLast = 0;
void loop()
{
  if (millis() - millisLast > 50)
  {
    millisLast = millis();
    // Set values to send
    int samples = 10;
    for (int cnt = 0; cnt < samples; cnt++)
    {
      myData.x += (analogRead(PIN_DX) - offset_x);
      myData.y += (analogRead(PIN_DY) - offset_y);
    }
    myData.x /= samples;
    myData.y /= samples;

    // myData.x=map(myData.x,X_MIN, X_MAX, -100,100);
    // myData.y=map(myData.y,Y_MIN, Y_MAX, -100,100);

    myData.btn = !digitalRead(PIN_BTN);

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  }
}
