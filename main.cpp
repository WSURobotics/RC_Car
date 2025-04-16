// ============================================================================================================================================
//
// This is the preliminary code created for the RC car built by Sam, Evan, and Kenan during the Spring 2025 Hardware Hackathon. 
// There are three sections - all compiled into this one file. Comment out the two sections you do not intend to upload (or create three individual files).
//
// Section 1 - Begin Remote/Sender Code:
//    This is what the ESP32 remote should run. It uses ESP_NOW to communicate one-way to the ESP32 on the car itself. The broadcastAddress is the 
//    MAC address of the receiving ESP32 (see Section 3), the defined values are the pin numbers for the jumper wires (may change depending on how you wire).
//    The struct message sends the Left and Right joystick values. We use analogRead() to read the incoming joystick values, remap them between 0 to 100,
//    and send the value to the receiving ESP32 on the Car. 
// Section 2 - Receiver Code:
//    This is what the ESP32 on the car should run. It uses the receiving end of ESP_NOW for one-way communication. The defined values are similarly the
//    pin numbers for jumper wires (battery, motor controller). We then map the incoming speed values to PWM values (0 - 255) and use the driveMotor
//    function to write the values to the motor controller (which then goes straight to the wheels).
// Section 3 - Read ESP32 MAC Address:
//    This is a brief section that will collect and print an ESP32's MAC address. This is necessary for gathering the correct MAC address of
//    the receiver (ESP32 on the car).
//
// We predict most troubles will likely be due to wiring/incorrect pins. Double check the ESP32 pinout diagrams (for IO, Ground, Incoming power pins)
// and the X versus Y values for the analog joystick read (This implementation only uses 'either-or' for tank controls). This code works with the pins provided,
// so double check pins and wiring.
//
// Code written:        3/28/2025
// Description written: 4/15/2025
//
// ============================================================================================================================================





// ============================================================================================================================================
//                                                                BEGIN REMOTE/SENDER CODE
// ============================================================================================================================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xc8, 0xf0, 0x9e, 0x33, 0xc8, 0x0c}; // Be sure this is correct (MAC address of receiving ESP32)

#define JOYSTICK1X 33
#define JOYSTICK1Y 35

#define JOYSTICK2X 32
#define JOYSTICK2Y 34

typedef struct struct_message { // MUST MATCH RECEIVER STRUCT
  uint16_t a;
  uint16_t b;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// When data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Set device as WiFi Station

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  uint16_t J1x = analogRead(JOYSTICK1X);
  uint16_t J1y = analogRead(JOYSTICK1Y);
  uint16_t J2x = analogRead(JOYSTICK2X);
  uint16_t J2y = analogRead(JOYSTICK2Y);

  J1x = map(J1x, 0, 4095, 0, 100);
  J1y = map(J1y, 0, 4095, 0, 100);
  J2x = map(J2x, 0, 4095, 0, 100);
  J2y = map(J2y, 0, 4095, 0, 100);

  // Note that these may need to switch between J1y/J1x and J2y/J2x, depending on the orientation of the joystick. 
  myData.a = J1y; // J1x or J1y
  myData.b = J2y; // J2x or J2y
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  delay(100);
}


// ============================================================================================================================================
//                                                                END REMOTE/SENDER CODE
// ============================================================================================================================================
// ============================================================================================================================================
//                                                                BEGIN ROBOT/RECEIVER CODE
// ============================================================================================================================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Motor 1 (Left)
#define MOTOR1_PIN1 32
#define MOTOR1_PIN2 33
#define ENABLE1_PIN 25
#define PWM_CHANNEL_1 0

// Motor 2 (Right)
#define MOTOR2_PIN1 26
#define MOTOR2_PIN2 27
#define ENABLE2_PIN 14
#define PWM_CHANNEL_2 1

const int freq = 30000;
const int resolution = 8;

typedef struct struct_message { // MUST MATCH SENDER STRUCT
    uint16_t a;
    uint16_t b;
} struct_message;

// remap w/ deadzone
int mapSpeed(int value) {
  if (value >= 48) {
      return map(value, 48, 100, 0, 255);  // Forward (third balue (0) was 50)
  } else if (value <= 45) {
      return map(value, 45, 0, 0, -255);  // Reverse (second 0 was -50)
  }
  return 0; // Dead zone
}

void driveMotor(int speed, int pin1, int pin2, int enablePin, int pwmChannel) {
  if (speed > 0) {  // Forward
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {  // Reverse
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      ledcWrite(pwmChannel, abs(speed));  // Ensure PWM is always positive
  } else {  // Stop
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      ledcWrite(pwmChannel, 0);
  }
}

struct_message myData;

// When data received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("a: ");
  Serial.print(myData.a);
  Serial.print("\tb: ");
  Serial.println(myData.b);

  int speed1 = mapSpeed(myData.a); // Multiply by -1 if polarity/direction needes to change for motor1
  int speed2 = mapSpeed(myData.b); // Multiply by -1 if polarity/direction needes to change for motor2

  Serial.print("Motor 1 Speed: ");
  Serial.print(speed1);
  Serial.print("\tMotor 2 Speed: ");
  Serial.println(speed2);
  Serial.println();
  Serial.println();
    
  driveMotor(speed1, MOTOR1_PIN1, MOTOR1_PIN2, ENABLE1_PIN, PWM_CHANNEL_1);
  driveMotor(speed2, MOTOR2_PIN1, MOTOR2_PIN2, ENABLE2_PIN, PWM_CHANNEL_2);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Set device as WiFi Station

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Rregister for recv CB to get recv packet info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // motor pins
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(ENABLE1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(ENABLE2_PIN, OUTPUT);

  // pwm channels
  ledcSetup(PWM_CHANNEL_1, freq, resolution);
  ledcSetup(PWM_CHANNEL_2, freq, resolution);
  ledcAttachPin(ENABLE1_PIN, PWM_CHANNEL_1);
  ledcAttachPin(ENABLE2_PIN, PWM_CHANNEL_2);
}
 
void loop() {}

// ============================================================================================================================================
//                                                                BEGIN ROBOT/RECEIVER CODE
// ============================================================================================================================================
// ============================================================================================================================================
//                                                                BEGIN MAC ADDRESS CODE
// ============================================================================================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup(){
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin();

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
}
 
void loop(){
  delay(1000);
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
  Serial.println();
}

// ============================================================================================================================================
//                                                                END MAC ADDRESS CODE
// ============================================================================================================================================