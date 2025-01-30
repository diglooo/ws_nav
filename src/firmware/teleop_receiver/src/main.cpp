#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

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

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int x;
  int y;
  int btn;
} struct_message;

// Create a struct_message called myData
struct_message myData;

#define THRESHOLD 5
int over = 0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  if (abs(myData.x) > THRESHOLD || abs(myData.y) > THRESHOLD)
  {
    over = 1;
    Serial.print("START,");
    Serial.print(myData.x);
    Serial.print(",");
    Serial.println(myData.y);
  }
  else
  {
    if (over)
    {
      over = 0;
      Serial.print("START,0,0");
    }
  }
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  readMacAddress();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{
  delay(20);
}