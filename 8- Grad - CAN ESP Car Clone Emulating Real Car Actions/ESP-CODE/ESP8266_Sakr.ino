/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-esp-now-wi-fi-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <espnow.h>
#include <ESP8266WiFi.h>
#include "ESPAsyncWebServer.h"
#include "ESPAsyncTCP.h"
#include <Arduino_JSON.h>


/*    Data Mapping Start    */

#define Throttle_0  16
#define Throttle_20 17
#define Throttle_40 18
#define Throttle_60 19
#define Throttle_80 20

#define Transmission_N  112
#define Transmission_D  113
#define Transmission_R  119
#define Transmission_P  127

#define Brake_Light_Off 128
#define Brake_Light_On  129

#define R_indicator_Off 144
#define R_indicator_On  145

#define L_indicator_Off 160
#define L_indicator_On  161

#define Hazard_indicator_Off  176
#define Hazard_indicator_On   177

#define Front_Lights_Off  192
#define Front_Lights_On   193

#define Reverse_Lights_Off  208
#define Reverse_Lights_On   209

#define Steering_ST     224
#define Steering_R      226
#define Steering_SR     227
#define Steering_L      232
#define Steering_SL     236

/*    Data Mapping End    */


// Replace with your network credentials (STATION)
const char* ssid = "ahmed";
const char* password = "ahmed271";

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  unsigned char UART_Data;
} struct_message;

struct_message incomingReadings;

JSONVar board;

AsyncWebServer server(80);
AsyncEventSource events("/events");

// callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac_addr, uint8_t *incomingData, uint8_t len) { 
  // Copies the sender mac address to a string
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.write(incomingReadings.UART_Data);
  switch(incomingReadings.UART_Data)
  {
    case Throttle_0:
    board["str_Data"] = String("Throttle 0%");
    break;
    
    case Throttle_20:
    board["str_Data"] = String("Throttle 20%");
    break;
    
    case Throttle_40:
    board["str_Data"] = String("Throttle 40%");
    break;

    case Throttle_60:
    board["str_Data"] = String("Throttle 60%");
    break;

    case Throttle_80:
    board["str_Data"] = String("Throttle 80%");
    break;

    case Transmission_N:
    board["str_Data"] = String("Transmission N");
    break;
    
    case Transmission_D:
    board["str_Data"] = String("Transmission D");
    break;
    
    case Transmission_R:
    board["str_Data"] = String("Transmission R");
    break;

    case Transmission_P:
    board["str_Data"] = String("Transmission P");
    break;

    case 128:
    board["str_Data"] = String("Brakes Off");
    break;
    
    case 129:
    board["str_Data"] = String("Brakes On");
    break;

    case 144:
    board["str_Data"] = String("Right Ind. Off");
    break;

    case 145:
    board["str_Data"] = String("Right Ind. On");
    break;

    case 160:
    board["str_Data"] = String("Left Ind. Off");
    break;

    case 161:
    board["str_Data"] = String("Left Ind. On");
    break;

    case 176:
    board["str_Data"] = String("Hazard Ind. Off");
    break;

    case 177:
    board["str_Data"] = String("Hazard Ind. On");
    break;

    case 192:
    board["str_Data"] = String("Front Lights Off");
    break;

    case 193:
    board["str_Data"] = String("Front Lights On");
    break;

    case 208:
    board["str_Data"] = String("Reverse Lights Off");
    break;

    case 209:
    board["str_Data"] = String("Reverse Lights On");
    break;

    case 224:
    board["str_Data"] = String("Steering Middle");
    break;

    case 226:
    board["str_Data"] = String("Steering Right");
    break;

    case 227:
    board["str_Data"] = String("Steering Sharp Right");
    break;

    case 232:
    board["str_Data"] = String("Steering Left");
    break;

    case 236:
    board["str_Data"] = String("Steering Sharp Left");
    break;
    
    default:
    break;
  }
  
  String jsonString = JSON.stringify(board);
  
  events.send(jsonString.c_str(), "new_readings", millis());
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP-NOW DASHBOARD</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h1 {  font-size: 2rem;}
    body {  margin: 0;}
    .topnav { overflow: hidden; background-color: #2f4468; color: white; font-size: 1.7rem; }
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); }
    .reading { font-size: 2.8rem; }
    .timestamp { color: #bebebe; font-size: 1rem; }
    .card-title{ font-size: 1.2rem; font-weight : bold; }
    .card.temperature { color: #B10F2E; }
    .card.humidity { color: #50B8B4; }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>ESP-NOW DASHBOARD</h1>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card temperature">
        <p class="card-title"><i class="fas fa-car"></i> RC Car #1 - Data</p><p><span class="reading"><span id="t1"></span> </span></p><p class="timestamp">Last Reading: <span id="rt6"></span></p>
      </div>
    </div>
  </div>
<script>

function getDateTime() {
  var currentdate = new Date();
  var datetime = currentdate.getDate() + "/"
  + (currentdate.getMonth()+1) + "/"
  + currentdate.getFullYear() + " at "
  + currentdate.getHours() + ":"
  + currentdate.getMinutes() + ":"
  + currentdate.getSeconds();
  return datetime;
}
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);
 
 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);
 
 source.addEventListener('new_readings', function(e) {
  console.log("new_readings", e.data);
  var obj = JSON.parse(e.data);
  document.getElementById("t1").innerHTML = obj.str_Data;
  document.getElementById("rt6").innerHTML = getDateTime();
  //document.getElementById("rh"+obj.id).innerHTML = getDateTime();
 }, false);
}
</script>
</body>
</html>)rawliteral";

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  
  // Set device as a Wi-Fi Station
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
   
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();
}
 
void loop() {
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 5000;
  if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
    events.send("ping",NULL,millis());
    lastEventTime = millis();
  }
  wdt_disable();
}
