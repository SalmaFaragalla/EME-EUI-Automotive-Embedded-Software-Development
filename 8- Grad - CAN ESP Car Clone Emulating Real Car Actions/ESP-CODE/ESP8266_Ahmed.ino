

#include <ESP8266WiFi.h>
#include <espnow.h>

#define ERROR_PIN 2


// REPLACE WITH RECEIVER MAC Addresses
uint8_t broadcastAddress [] = {0x48, 0x55, 0x19, 0x15, 0x2B, 0x92};
uint8_t broadcastAddress2 [] = {0xC8, 0xC9, 0xA3, 0x56, 0x3C, 0x42};

// Insert your SSID
constexpr char WIFI_SSID[] = "ahmed";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i=0; i<n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}



uint8_t   error_flag;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  uint8_t a;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0){
  }
  else{
    error_flag=1;
    
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
   int32_t channel = getWiFiChannel(WIFI_SSID);

  //WiFi.printDiag(Serial); /* Uncomment to verify channel number before  */
  wifi_promiscuous_enable(1);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(0);
  //WiFi.printDiag(Serial); /* Uncomment to verify channel change after */
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    digitalWrite(ERROR_PIN,HIGH);
    delay(1000);
    digitalWrite(ERROR_PIN,LOW);
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress2, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() {
     if (Serial.available() > 0) {
    // read the incoming byte:
    myData.a = Serial.read();
    // Send message via ESP-NOW
    esp_now_send(0, (uint8_t *) &myData, sizeof(myData));     /*    0:Broadcast    */
     }
     if(1==error_flag)
     {
      digitalWrite(ERROR_PIN,HIGH);
    delay(500);
    digitalWrite(ERROR_PIN,LOW);
    error_flag=0;
     }
  }
