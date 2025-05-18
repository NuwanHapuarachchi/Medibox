#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHTesp.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>  // Added for secure MQTT connection
#include <time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define BUZZER 17
#define LED_1 5
#define LED_2 16
#define PB_CANCEL 18
#define PB_OK 32
#define PB_UP 33
#define PB_DOWN 12
#define DHT 14
#define NTP_SERVER "pool.ntp.org"

// LDR Pin definition
#define LDR_PIN 34  // LDR sensor pin
#define servoPin 27  // Servo motor pin

// NTP Server settings
int UTC_OFFSET = 0;
int UTC_OFFSET_DST = 0;

// Object creation
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHTesp dhtSensor;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);

// MQTT client setup
WiFiClientSecure espClient;  // Changed to secure client
PubSubClient client(espClient);
const char* mqtt_server = "df790ce588574283a4471d41351c34cc.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;  // Alternative MQTT port
const char* mqtt_username = "hivemq.webclient.1747304453555";
const char* mqtt_password = "Ih;y5Z2pDmG!1z>f9K@Y";

// Increase PubSubClient buffer size to handle larger messages
#define MQTT_MAX_PACKET_SIZE 1024

// Servo motor setup
Servo servo;

// Global variables
int days = 0;
int hours = 0;
int minutes = 0;
int seconds = 0;

// Alarms
bool alarm_enable = true;
int n_alarms = 2;
int alarm_hours[] = {0, 1};
int alarm_minutes[] = {1, 10};
bool alarm_triggered[] = {false, false};

// Notes for the buzzer
int n_notes = 8;
int C = 262;
int D = 294;
int E = 330;
int F = 349;
int G = 392;
int A = 440;
int B = 494;
int C_H = 523;

int notes[] = {C, D, E, F, G, A, B, C_H};

// States of Menu
int current_mode = 0;
String modes[] = {
    "1 - Set Time Zone",
    "2 - Set Alarm 1",
    "3 - Set Alarm 2",
    "4 - Active Alarms",
    "5 - Delete an Alarm"};
int max_modes = 5;

// DHT sensor thresholds
int max_temp = 32;
int min_temp = 24;
int max_humidity = 80;
int min_humidity = 65;

unsigned long timeNow = 0;
unsigned long timeLast = 0;
int prev_theta = 0;

// Light intensity monitoring variables
unsigned long lastSampleTime = 0;
unsigned long lastSendTime = 0;
int sampleInterval = 5000;    // Default 5 seconds (in milliseconds)
int sendInterval = 120000;    // Default 2 minutes (in milliseconds)
int sampleCount = 0;
float lightSum = 0;
float lastLightAvg = 0;

// Servo control parameters
int t_offset = 30;
float gamma_i = 0.75;

// Character arrays for MQTT publishing
char tempArr[6];
char humArr[6];
char ldrArr[6];

// Add this variable to your global variables section
float Tmed = 30.0; // Default ideal storage temperature (°C)

// Angle variable for the servo - only auto mode is used
int servo_angle = 90;    // Default servo position to 90

// Function prototypes to fix "was not declared in this scope" errors
void connectToBroker(); // Ensure prototype is declared
void setupMQTT();       // Add prototype for setupMQTT
void set_time_zone();
void set_alarm(int alarm);
void view_active_alarms();
void delete_alarm();
void ring_alarm();
void go_to_menu();
void run_mode(int mode);
void update_time_with_check_alarm();
void check_temp();
void lightIntensity();
float readLightIntensity(int pin);
float calculateTheta(float thetaOffset, float I, float ts, float tu, float T, float Tmed, float Lambda);
void servoAngle(float lightIntensity);
void print_line(String text, int column, int row, int text_size);
void beep();

// Function to print text on OLED display
void print_line(String text, int column, int row, int text_size) {
  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);
  display.display();
}

// Function to play a beep sound
void beep() {
  tone(BUZZER, notes[3]);
  delay(10);
  noTone(BUZZER);
  delay(10);
}

// Function to read light intensity from LDR and normalize to 0-1 range
float readLightIntensity(int pin) {
  int rawValue = analogRead(pin);
  
  // Check for invalid readings
  if (rawValue < 0) {
    Serial.printf("Error reading from pin %d\n", pin);
    return 0.5; // Return mid-range as fallback
  }
  
  // Map analog reading (0-4095 for ESP32's 12-bit ADC) to 0-1 range
  float normalizedValue = 1.0 - (rawValue / 4095.0);
  return normalizedValue;
}

// Function to control servo based on automatic temperature control only
void servoAngle(float lightIntensity) {
  // Get the current temperature reading for automatic mode
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  float currentTemp = data.temperature;
  int theta = 0; // Final angle to set
  
  // Always in automatic mode
  Serial.println("Auto mode only - calculating servo position");
  
  // Automatic mode - calculate angle based on the formula:
  // θ = θoffset + (180 - θoffset) × I × γ × (T/Tmed)
  float tempRatio = currentTemp / Tmed;
  
  // Calculate the angle using the formula
  theta = t_offset + (180 - t_offset) * lightIntensity * gamma_i * tempRatio;
  
  // Create debug message for formula calculation
  char debugMsg[100];
  sprintf(debugMsg, "θ = %d + (180-%d) × %.2f × %.2f × (%.1f/%.1f) = %d", 
          t_offset, t_offset, lightIntensity, gamma_i, currentTemp, Tmed, theta);
  client.publish("Medibox_Calculation_Debug_Nuwan", debugMsg, true);
  
  // Detailed debug of the calculation
  Serial.print("Auto mode calculation: t_offset=");
  Serial.print(t_offset);
  Serial.print(", light=");
  Serial.print(lightIntensity);
  Serial.print(", gamma=");
  Serial.print(gamma_i);
  Serial.print(", tempRatio=");
  Serial.print(tempRatio);
  Serial.print(", raw angle=");
  Serial.println(theta);
  
  // Constrain servo angle within physical limits (0-180 degrees)
  if (theta < 0) {
    Serial.println("Warning: Calculated angle was negative, clamping to 0");
    theta = 0;
  }
  if (theta > 180) {
    Serial.println("Warning: Calculated angle exceeds 180, clamping to 180");
    theta = 180;
  }
  
  Serial.print("Servo in auto mode, Temp: ");
  Serial.print(currentTemp);
  Serial.print("°C, Light: ");
  Serial.print(lightIntensity);
  Serial.print(", Final Angle: ");
  Serial.println(theta);
  
  // Update status
  client.publish("Medibox_Servo_Mode_Status_Nuwan", "Automatic temperature-based control", true);
  
  // Store the calculated angle
  servo_angle = theta;
  
  // Apply the calculated angle
  servo.write(theta);
  prev_theta = theta;
  
  // Only publish light intensity if it's changed enough to be meaningful
  static float lastPublishedLight = -1;
  if (abs(lastPublishedLight - lightIntensity) > 0.01) { // Only publish if changed by more than 1%
    lastPublishedLight = lightIntensity;
    char lightStr[8];
    sprintf(lightStr, "%.2f", lightIntensity);
    client.publish("medibox/light/average", lightStr);
  }
  
  // Only publish angle if it has changed
  static int lastPublishedAngle = -1;
  if (lastPublishedAngle != theta) {
    lastPublishedAngle = theta;
    char angleStr[5];
    sprintf(angleStr, "%d", theta);
    client.publish("Medibox_Window_Angle_Nuwan", angleStr);
  }
}

// Function to process light intensity readings
void lightIntensity() {
  // Read current light intensity
  float light = readLightIntensity(LDR_PIN);

  // Call servo function (now manual-only)
  servoAngle(light);

  // Format and publish light sensor readings
  char lightStr[8];
  sprintf(lightStr, "%.2f", light);
  
  // Publish to multiple topics for maximum compatibility with dashboard
  client.publish("Medibox_Light_Nuwan", lightStr);
  client.publish("medibox/light/instant", lightStr);
  client.publish("medibox/light/average", lightStr); // Direct to chart
  
  // Save this for the main loop to use
  String(light, 2).toCharArray(ldrArr, 6);
  
  // Debug output to serial
  Serial.print("Light reading: ");
  Serial.println(light, 2);
}

// MQTT message callback function
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0'; // Ensure payload is null-terminated
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(message);
  
  // Check which topic received the message
  if (String(topic) == "medibox/settings/sampleInterval") {
    int newInterval = message.toInt() * 1000; // Convert seconds to milliseconds
    if (newInterval >= 1000 && newInterval <= 60000) { // Between 1 second and 1 minute
      sampleInterval = newInterval;
      Serial.print("Sample interval set to: ");
      Serial.print(sampleInterval / 1000);
      Serial.println(" seconds");
      
      // Reset sampling to start fresh with new interval
      lightSum = 0;
      sampleCount = 0;
      lastSampleTime = millis();
      
      // Publish confirmation of the new setting
      char confirmMsg[10];
      sprintf(confirmMsg, "%d", sampleInterval / 1000);
      client.publish("medibox/settings/sampleInterval/confirm", confirmMsg, true);
    } else {
      Serial.println("Invalid sample interval. Must be between 1 and 60 seconds.");
    }
  } 
  else if (String(topic) == "medibox/settings/sendInterval") {
    int newInterval = message.toInt() * 1000; // Convert seconds to milliseconds
    if (newInterval >= 30000 && newInterval <= 300000) { // Between 30 seconds and 5 minutes
      sendInterval = newInterval;
      Serial.print("Send interval set to: ");
      Serial.print(sendInterval / 1000);
      Serial.println(" seconds");
      Serial.print("Expected samples per period: ~");
      Serial.println(sendInterval / sampleInterval);
      
      // Reset averaging to start fresh with new interval
      lightSum = 0;
      sampleCount = 0;
      lastSendTime = millis();
      
      // Publish confirmation of the new setting
      char confirmMsg[10];
      sprintf(confirmMsg, "%d", sendInterval / 1000);
      client.publish("medibox/settings/sendInterval/confirm", confirmMsg, true);
    } else {
      Serial.println("Invalid send interval. Must be between 30 and 300 seconds.");
    }
  }  else if (String(topic) == "Medibox_Servo_Angle_Nuwan") {
    t_offset = message.toInt();
    Serial.print("Offset angle (θoffset) set to: ");
    Serial.println(t_offset);
    
    // Update servo position immediately
    float light = readLightIntensity(LDR_PIN);
    servoAngle(light);
    
    // Remove echo to avoid feedback loop
    // client.publish("Medibox_Servo_Angle_Nuwan", message.c_str(), true);
  }
  else if (String(topic) == "Medibox_Servo_CF_Nuwan") {
    gamma_i = message.toFloat();
    Serial.print("Control factor (γ) set to: ");
    Serial.println(gamma_i);
    
    // Update servo position immediately
    float light = readLightIntensity(LDR_PIN);
    servoAngle(light);
    
    // Remove echo to avoid feedback loop
    // client.publish("Medibox_Servo_CF_Nuwan", message.c_str(), true);
  }
  else if (String(topic) == "Medibox_Ideal_Temp_Nuwan") {
    Tmed = message.toFloat();
    Serial.print("Ideal temperature (Tmed) set to: ");
    Serial.println(Tmed);
    
    // Update servo position immediately
    float light = readLightIntensity(LDR_PIN);
    servoAngle(light);
    
    // Remove echo to avoid feedback loop
    // client.publish("Medibox_Ideal_Temp_Nuwan", message.c_str(), true);
  }// Handle legacy servo mode command (now deprecated as we only use auto mode)
  else if (String(topic) == "Medibox_Servo_Mode_Nuwan") {
    // We now only support automatic mode, so inform user via status message
    client.publish("Medibox_Servo_Mode_Status_Nuwan", "Automatic temperature-based control only", true);
    Serial.println("Received servo mode change request - system only supports automatic mode");
    
    // Force an immediate update of servo angle based on auto mode calculation
    float light = readLightIntensity(LDR_PIN);
    servoAngle(light);
  }
  // Handle direct servo angle adjustments (now handled through other parameters)
  else if (String(topic) == "Medibox_Manual_Servo_Nuwan") {
    // This is now deprecated, but we'll respond to let the user know
    Serial.println("Manual servo control is deprecated - adjust through t_offset, gamma_i, or Tmed parameters");
    client.publish("Medibox_Servo_Mode_Status_Nuwan", "Use control parameters to adjust automatic behavior", true);
  }
}

// Setup MQTT connection
void setupMQTT() {
  // Set MQTT server and port
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Set max packet size to handle larger messages
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  
  // Connect to MQTT broker
  connectToBroker();
}

// Enhance the connectToBroker function to publish initial values for servo controls
void connectToBroker() {
  int retryCount = 0;
  while (!client.connected() && retryCount < 5) {
    Serial.println("Attempting MQTT connection to HiveMQ Cloud...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Set up Last Will and Testament for connection monitoring
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password, "MediBox_Status_Nuwan", 1, true, "offline")) {
      Serial.println("Connected to MQTT broker successfully!");
      
      // Publish online status immediately upon connection
      client.publish("MediBox_Status_Nuwan", "online", true);
      
      // Subscribe to relevant topics
      client.subscribe("medibox/settings/sampleInterval", 1);
      client.subscribe("medibox/settings/sendInterval", 1);
      client.subscribe("Medibox_Servo_Angle_Nuwan", 1);
      client.subscribe("Medibox_Servo_CF_Nuwan", 1);
      client.subscribe("Medibox_Ideal_Temp_Nuwan", 1);
      client.subscribe("Medibox_Servo_Mode_Nuwan", 1);
      client.subscribe("Medibox_Manual_Servo_Nuwan", 1);
      
      // Wait a moment to ensure subscription is complete
      delay(100);
      
      // Critical: Publish all current values to Node-RED topics
      Serial.println("Publishing initial values to dashboard");
      
      // Status message - always in automatic mode now
      client.publish("Medibox_Servo_Mode_Nuwan", "auto", true);
      client.publish("Medibox_Servo_Mode_Status_Nuwan", "Automatic temperature-based control", true);
      
      // θoffset - minimum angle
      char offsetStr[10];
      sprintf(offsetStr, "%d", t_offset);
      client.publish("Medibox_Servo_Angle_Nuwan", offsetStr, true);
      
      // γ - controlling factor
      char cfStr[10];
      sprintf(cfStr, "%.2f", gamma_i);
      client.publish("Medibox_Servo_CF_Nuwan", cfStr, true);
      
      // Tmed - ideal temperature
      char tmedStr[10];
      sprintf(tmedStr, "%.1f", Tmed);
      client.publish("Medibox_Ideal_Temp_Nuwan", tmedStr, true);
      
      // Current servo angle (display-only value)
      char windowAngleStr[10];
      sprintf(windowAngleStr, "%d", servo_angle);
      client.publish("Medibox_Window_Angle_Nuwan", windowAngleStr, true);
      
      // Publish sample interval and send interval
      char sampleIntervalStr[10];
      sprintf(sampleIntervalStr, "%d", sampleInterval / 1000); // Convert ms to seconds
      client.publish("medibox/settings/sampleInterval", sampleIntervalStr, true);
      
      char sendIntervalStr[10];
      sprintf(sendIntervalStr, "%d", sendInterval / 1000); // Convert ms to seconds
      client.publish("medibox/settings/sendInterval", sendIntervalStr, true);
      
      // Calculate the correct starting angle using auto mode formula
      TempAndHumidity data = dhtSensor.getTempAndHumidity();
      float lightIntensity = readLightIntensity(LDR_PIN);
      float tempRatio = data.temperature / Tmed;
      int calculated_angle = t_offset + (180 - t_offset) * lightIntensity * gamma_i * tempRatio;
      
      // Constrain within limits
      if (calculated_angle < 0) calculated_angle = 0;
      if (calculated_angle > 180) calculated_angle = 180;
      
      // Set position
      servo.write(calculated_angle);
      servo_angle = calculated_angle;
      prev_theta = calculated_angle;
      
      // Update window angle display
      char angleStr[5];
      sprintf(angleStr, "%d", calculated_angle);
      client.publish("Medibox_Window_Angle_Nuwan", angleStr, true);
      
      Serial.print("Starting with calculated servo angle: ");
      Serial.println(calculated_angle);
      
      Serial.println("Initial values published to dashboard");
    } else {
      retryCount++;
      Serial.print("Failed to connect to MQTT broker - ");
      int state = client.state();
      switch (state) {
        case -4: Serial.println("Connection timeout"); break;
        case -3: Serial.println("Connection lost"); break;
        case -2: Serial.println("Connection failed"); break;
        case -1: Serial.println("Disconnected"); break;
        case 1: Serial.println("Bad protocol"); break;
        case 2: Serial.println("Bad client ID"); break;
        case 3: Serial.println("Unavailable"); break;
        case 4: Serial.println("Bad credentials"); break;
        case 5: Serial.println("Unauthorized"); break;
        default: Serial.println(state);
      }
      delay(2000);
    }
  }
  
  if (!client.connected()) {
    Serial.println("Could not connect to MQTT after multiple attempts. Restarting device...");
    delay(3000);
    ESP.restart();
  }
}

// Play alarm melody function
void play_alarm_melody() {
  // Play the scale up and down a few times
  for(int repeat = 0; repeat < 3; repeat++) {
    // Play ascending scale
    for(int i = 0; i < n_notes; i++) {
      tone(BUZZER, notes[i], 100);
      delay(130);
      noTone(BUZZER);
    }
    
    // Play descending scale
    for(int i = n_notes-1; i >= 0; i--) {
      tone(BUZZER, notes[i], 100);
      delay(130);
      noTone(BUZZER);
    }
    
    delay(200); // Brief pause between repetitions
  }
}

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);  pinMode(PB_CANCEL, INPUT);
  pinMode(PB_OK, INPUT);
  pinMode(PB_UP, INPUT);
  pinMode(PB_DOWN, INPUT);
  pinMode(LDR_PIN, INPUT);  // Set up LDR pin

  // Set up DHT sensor
  dhtSensor.setup(DHT, DHTesp::DHT22);

  // Set analog read resolution to 12-bit (ESP32 native)
  analogReadResolution(12);    // Ensure default values are set
  t_offset = 30;         // Default 30 degrees
  gamma_i = 0.75;        // Default 0.75
  Tmed = 30.0;           // Default 30°C
  servo_angle = 90;      // Default servo position to 90°
  
  // Initialize servo
  servo.attach(servoPin);
  servo.write(servo_angle);  // Set initial position to 90°
  prev_theta = servo_angle;

  Serial.begin(115200);
  Serial.println("MediBox is Initializing!");
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  
  display.display();
  display.clearDisplay();
  print_line("Welcome", 10, 5, 2);
  print_line("to", 10, 25, 2);
  print_line("MediBox", 10, 45, 2);

  delay(2000);
  display.clearDisplay();
  // Connect to WiFi
  WiFi.begin("Wokwi-GUEST", "", 6);
  int wifiCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    wifiCounter++;
    display.clearDisplay();
    print_line("Connecting to WiFi", 2, 0, 0);
    Serial.print(".");
    // If can't connect after 20 attempts (10 seconds), restart
    if (wifiCounter > 20) {
      Serial.println("WiFi connection failed, restarting...");
      ESP.restart();
    }
  }
  display.clearDisplay();
  print_line("Connected to WiFi", 2, 0, 0);
  Serial.print("Connected to WiFi with IP: ");
  Serial.println(WiFi.localIP());
  // Configure secure client for MQTT TLS connection
  espClient.setInsecure();  // Skip certificate validation
  
  // Optionally add a timeout for the SSL handshake
  espClient.setTimeout(15); // 15 seconds timeout
  
  // Start NTP client
  timeClient.begin();
  timeClient.update(); // Force an immediate update
  Serial.println("NTP client started");
  
  // Setup MQTT with enhanced logging
  Serial.println("Setting up MQTT connection to HiveMQ...");
  setupMQTT();
  Serial.println("MQTT setup completed");

  // Welcome sound
  for (int i = 0; i < 8; i++) {
    tone(BUZZER, notes[i]);
    delay(60);
  }
  noTone(BUZZER);
  
  delay(1000);
}

void print_time_now(void) {
  // Get epoch time and convert it to struct tm
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime(&epochTime);

  int day = ptm->tm_mday;
  int month = ptm->tm_mon + 1;    // tm_mon is 0-indexed
  int year = ptm->tm_year + 1900; // tm_year counts from 1900

  display.clearDisplay();
  display.setTextSize(1); // Set smaller font for date and time visibility
  display.setTextColor(SSD1306_WHITE);

  // Print Date (Day/Month/Year)
  display.setCursor(35, 42);
  display.print(day);
  display.print("/");
  display.print(month);
  display.print("/");
  display.print(year);

  display.setTextSize(3); // Set larger font for time

  // Print Time (Hour:Minute)
  display.setCursor(15, 15);
  if (hours < 10)
    display.print("0"); // Leading zero for hours
  display.print(hours);
  display.print(":");
  if (minutes < 10)
    display.print("0"); // Leading zero for minutes
  display.print(minutes);

  display.display();
}

void update_time() {
  timeClient.update(); // Ensure NTP time updates

  hours = timeClient.getHours();
  minutes = timeClient.getMinutes();
  seconds = timeClient.getSeconds();
}

// Function - Ringing the alarm
void ring_alarm() {
  display.clearDisplay();
  print_line("Medicine Time!", 10, 10, 2);
  digitalWrite(LED_1, HIGH);

  bool break_happened = false;
  bool snooze = false;

  while (digitalRead(PB_CANCEL) == HIGH && break_happened == false) {
    play_alarm_melody();

    if (digitalRead(PB_CANCEL) == LOW) { // Stop Alarm
      delay(200);
      break_happened = true;
      break;
    }
    else if (digitalRead(PB_OK) == LOW) { // Snooze for 5 minutes
      delay(200);
      snooze = true;
      break_happened = true;
      break;
    }
  }

  digitalWrite(LED_1, LOW);
  display.clearDisplay();

  if (snooze) {
    int snooze_time = 5; // Snooze for 5 minutes
    minutes += snooze_time;
    if (minutes >= 60) {
      minutes -= 60;
      hours = (hours + 1) % 24;
    }
    print_line("Snoozed for 5 mins", 10, 20, 1);
    delay(2000);
  }
}

void update_time_with_check_alarm(void) {
  update_time();
  print_time_now();
  if (alarm_enable == true) {
    for (int i = 0; i < n_alarms; i++) {
      if (alarm_triggered[i] == false && alarm_hours[i] == hours && alarm_minutes[i] == minutes) {
        ring_alarm();
        alarm_triggered[i] = true;
      }
    }
  }
}

int wait_for_button_press(unsigned long timeout_ms = 5000) {
  unsigned long start_time = millis();

  while (millis() - start_time < timeout_ms) {
    if (digitalRead(PB_UP) == LOW) {
      beep();
      delay(50);
      while (digitalRead(PB_UP) == LOW);
      return PB_UP;
    }
    else if (digitalRead(PB_DOWN) == LOW) {
      beep();
      delay(50);
      while (digitalRead(PB_DOWN) == LOW);
      return PB_DOWN;
    }
    else if (digitalRead(PB_OK) == LOW) {
      beep();
      delay(50);
      while (digitalRead(PB_OK) == LOW);
      return PB_OK;
    }
    else if (digitalRead(PB_CANCEL) == LOW) {
      beep();
      delay(50);
      while (digitalRead(PB_CANCEL) == LOW);
      return PB_CANCEL;
    }
  }

  return -1; // No button press detected within timeout
}

void check_temp() {
  // Read temperature and humidity data from the DHT sensor
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  bool Condition_Perfect = true;

  // Format values for MQTT
  String(data.temperature, 2).toCharArray(tempArr, 6);
  String(data.humidity, 2).toCharArray(humArr, 6);

  // Check if the temperature exceeds the maximum threshold
  if (data.temperature > max_temp) {
    Condition_Perfect = false;
    digitalWrite(LED_2, HIGH);
    print_line("Temp is too High!", 1, 0, 0);
  }
  // Check if the temperature is below the minimum threshold
  else if (data.temperature < min_temp) {
    Condition_Perfect = false;
    digitalWrite(LED_2, HIGH);
    print_line("Temp is too Low!", 1, 0, 0);
  }

  // Check if the humidity exceeds the maximum threshold
  if (data.humidity > max_humidity) {
    Condition_Perfect = false;
    digitalWrite(LED_2, HIGH);
    print_line("Humidity is too High!", 1, 50, 0);
  }
  // Check if the humidity is below the minimum threshold
  else if (data.humidity < min_humidity) {
    Condition_Perfect = false;
    digitalWrite(LED_2, HIGH);
    print_line("Humidity is too Low", 1, 50, 0);
  }

  // If all conditions are perfect, turn off the LED
  if (Condition_Perfect) {
    digitalWrite(LED_2, LOW);
  }
}

// Function to execute the selected menu mode
void run_mode(int mode) {
  if (mode == 0) {
    // Mode 0: Set the time zone
    set_time_zone();
  }
  else if (mode == 1 || mode == 2) {
    // Mode 1 or 2: Set Alarm 1 or Alarm 2
    set_alarm(mode - 1);
  }
  else if (mode == 3) {
    // Mode 3: View active alarms
    view_active_alarms();
  }
  else if (mode == 4) {
    // Mode 4: Delete an alarm
    delete_alarm();
  }
}

void go_to_menu(void) {
  while (true) {
    display.clearDisplay();
    for (int i = 0; i < max_modes; i++) {
      if (i == current_mode) {
        // Highlight selected mode
        display.fillRect(0, i * 12, SCREEN_WIDTH, 12, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Inverted text color
      } else {
        display.setTextColor(SSD1306_WHITE);
      }

      display.setCursor(5, i * 12);
      display.setTextSize(1);
      display.println(modes[i]);
    }

    display.display();

    int pressed = wait_for_button_press();

    if (pressed == PB_UP) {
      current_mode = (current_mode - 1 + max_modes) % max_modes;
    }
    else if (pressed == PB_DOWN) {
      current_mode = (current_mode + 1) % max_modes;
    }
    else if (pressed == PB_OK) {
      run_mode(current_mode);
    }
    else if (pressed == PB_CANCEL) {
      break;
    }
  }
}

// Function implementations that were referenced earlier
void set_time_zone() {
  int temp_hour = UTC_OFFSET / 3600;
  int temp_minute = (UTC_OFFSET % 3600) / 60;

  while (true) {
    display.clearDisplay();
    print_line("Enter Hour: " + String(temp_hour), 0, 0, 1);

    int pressed = wait_for_button_press();

    if (pressed == PB_UP) {
      delay(200);
      temp_hour++;
      if (temp_hour > 12)
        temp_hour = -12;
    }
    else if (pressed == PB_DOWN) {
      delay(200);
      temp_hour--;
      if (temp_hour < -12)
        temp_hour = 12;
    }
    else if (pressed == PB_OK) {
      delay(200);
      break;
    }
    else if (pressed == PB_CANCEL) {
      delay(200);
      return;
    }
  }

  while (true) {
    display.clearDisplay();
    print_line("Enter Minute: " + String(temp_minute), 0, 0, 1);

    int pressed = wait_for_button_press();

    if (pressed == PB_UP) {
      delay(200);
      temp_minute += 30;
      if (temp_minute >= 60)
        temp_minute = 0;
    }
    else if (pressed == PB_DOWN) {
      delay(200);
      temp_minute -= 30;
      if (temp_minute < 0)
        temp_minute = 30;
    }
    else if (pressed == PB_OK) {
      delay(200);
      break;
    }
    else if (pressed == PB_CANCEL) {
      delay(200);
      return;
    }
  }

  UTC_OFFSET = (temp_hour * 3600) + (temp_minute * 60);
  timeClient.setTimeOffset(UTC_OFFSET);

  display.clearDisplay();
  print_line("Time Zone Updated!", 10, 20, 1);
  delay(1000);
}

void set_alarm(int alarm) {
  bool alarm_set = false;

  int temp_hour = alarm_hours[alarm];

  while (true) {
    display.clearDisplay();
    print_line("Enter hour: " + String(temp_hour), 0, 0, 1);

    int pressed = wait_for_button_press();

    if (pressed == PB_UP) {
      delay(200);
      temp_hour = (temp_hour + 1) % 24;
    }
    else if (pressed == PB_DOWN) {
      delay(200);
      temp_hour = (temp_hour == 0) ? 23 : temp_hour - 1;
    }
    else if (pressed == PB_OK) {
      delay(200);
      alarm_hours[alarm] = temp_hour;
      break;
    }
    
    else if (pressed == PB_CANCEL) {
      delay(200);
      return;
    }
  }

  int temp_minute = (alarm_minutes[alarm] / 10) * 10;

  while (true) {
    display.clearDisplay();
    print_line("Enter Minute: " + String(temp_minute), 0, 0, 1);

    int pressed = wait_for_button_press();

    if (pressed == PB_UP) {
      delay(200);
      temp_minute = (temp_minute + 10) % 60;
    }
    else if (pressed == PB_DOWN) {
      delay(200);
      temp_minute = (temp_minute == 0) ? 50 : temp_minute - 10;
    }
    else if (pressed == PB_OK) {
      delay(200);
      alarm_minutes[alarm] = temp_minute;
      alarm_set = true;
      break;
    }
    else if (pressed == PB_CANCEL) {
      delay(200);
      return;
    }
  }

  if (alarm_set) {
    if (alarm >= n_alarms) {
      n_alarms++;
    }

    alarm_enable = true;
    alarm_triggered[alarm] = false;

    display.clearDisplay();
    print_line("Alarm " + String(alarm + 1) + " is set", 10, 20, 1);
    delay(2000);
  }
}

void delete_alarm() {
  if (n_alarms == 0) {
    display.clearDisplay();
    print_line("No alarms to delete", 10, 20, 1);
    delay(2000);
    return;
  }

  int selected_alarm = 0;
  while (true) {
    display.clearDisplay();
    print_line("Delete Alarm: " + String(selected_alarm + 1), 0, 0, 1);
    display.display();

    int pressed = wait_for_button_press();
    if (pressed == PB_UP) {
      selected_alarm = (selected_alarm - 1 + n_alarms) % n_alarms;
    }
    else if (pressed == PB_DOWN) {
      selected_alarm = (selected_alarm + 1) % n_alarms;
    }
    else if (pressed == PB_OK) {
      for (int i = selected_alarm; i < n_alarms - 1; i++) {
        alarm_hours[i] = alarm_hours[i + 1];
        alarm_minutes[i] = alarm_minutes[i + 1];
        alarm_triggered[i] = alarm_triggered[i + 1];
      }
      n_alarms--;
      display.clearDisplay();
      print_line("Alarm Deleted", 10, 20, 1);
      delay(2000);
      break;
    }
    else if (pressed == PB_CANCEL) {
      break;
    }
  }
}

void view_active_alarms() {
  display.clearDisplay();

  if (n_alarms == 0) {
    print_line("No active alarms", 10, 20, 1);
  }
  else {
    for (int i = 0; i < n_alarms; i++) {
      print_line("Alarm " + String(i + 1) + ": " +
                 String(alarm_hours[i]) + ":" +
                 String(alarm_minutes[i]),
             0, i * 10, 1);
    }
  }

  display.display();
  delay(2000);
}

// Global variables for network monitoring
unsigned long lastNetworkCheckTime = 0;
const int networkCheckInterval = 60000; // Check network status every minute

// Function to monitor network connectivity
void checkNetworkStatus() {
  unsigned long currentTime = millis();
  
  // Check network status periodically
  if (currentTime - lastNetworkCheckTime >= networkCheckInterval) {
    lastNetworkCheckTime = currentTime;
    
    // Check WiFi status
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost! Attempting to reconnect...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin("Wokwi-GUEST", "", 6);
      
      // Wait for reconnection with timeout
      int wifiReconnectAttempts = 0;
      while (WiFi.status() != WL_CONNECTED && wifiReconnectAttempts < 10) {
        delay(500);
        Serial.print(".");
        wifiReconnectAttempts++;
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi reconnected successfully!");
        Serial.print("New IP address: ");
        Serial.println(WiFi.localIP());
        
        // Reconnect to MQTT after WiFi reconnection
        if (!client.connected()) {
          connectToBroker();
        }
      } else {
        Serial.println("Failed to reconnect to WiFi. Restarting device...");
        ESP.restart();
      }
    }
    
    // Ping the MQTT broker to verify connection is alive
    if (client.connected()) {
      if (!client.publish("MediBox_Heartbeat_Nuwan", "ping")) {
        Serial.println("MQTT heartbeat failed, attempting to reconnect...");
        client.disconnect();
        connectToBroker();
      }
    }
  }
}

void loop() {
  // Handle MQTT connection
  if (!client.connected()) {
    Serial.println("MQTT connection lost. Reconnecting...");
    connectToBroker();
  }
  
  // Process incoming MQTT messages - make sure this is called frequently
  client.loop();
  
  // Update time and check alarms
  update_time_with_check_alarm();
  
  // Check temperature and humidity
  check_temp();
  
  // Process immediate light intensity data for servo control
  lightIntensity();
  
  // Get current time for various timing operations
  unsigned long currentTime = millis();
  
  // Send current servo status periodically - less frequently to avoid network congestion
  static unsigned long lastServoStatusTime = 0;
  if (currentTime - lastServoStatusTime >= 300000) { // Changed to every 5 minutes (300,000ms)
    lastServoStatusTime = currentTime;
    
    // Debug message
    Serial.println("Publishing device status to MQTT");
    
    // Publish a simple device status
    client.publish("MediBox_Status_Nuwan", "online", true);
    
    // Publish current window angle for display
    char angleStr[6];
    sprintf(angleStr, "%d", prev_theta);
    client.publish("Medibox_Window_Angle_Nuwan", angleStr, true);
  }
  
  // Check if we can successfully publish data
  bool publishSuccess = true;
  // Send sensor data over MQTT
  publishSuccess &= client.publish("MediBox_TEMP_Nuwan", tempArr, true); // Retained
  publishSuccess &= client.publish("MediBox_HUM_Nuwan", humArr, true);   // Retained
  publishSuccess &= client.publish("Medibox_Light_Nuwan", ldrArr, true); // Retained
  
  if (!publishSuccess) {
    Serial.println("Failed to publish one or more messages");
  }
  
  // Light intensity monitoring with 5-second interval sampling and 2-minute averaging
  // Sample light intensity at the specified interval (default: every 5 seconds)
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    
    float lightIntensity = readLightIntensity(LDR_PIN);
    
    // Add to running sum for average calculation
    lightSum += lightIntensity;
    sampleCount++;
    
    // Display current light intensity on OLED
    char lightStr[10];
    sprintf(lightStr, "%.2f", lightIntensity);

    
    // Publish instant light reading for real-time display
    client.publish("medibox/light/instant", lightStr, false);
    
    Serial.print("LDR sample #");
    Serial.print(sampleCount);
    Serial.print(": ");
    Serial.println(lightIntensity, 3);
  }
  
  // Send average light intensity at the specified interval (default: every 2 minutes)
  if (currentTime - lastSendTime >= sendInterval && sampleCount > 0) {
    lastSendTime = currentTime;
    
    // Calculate average
    lastLightAvg = lightSum / sampleCount;
    
    // Prepare and send MQTT message with averaged light value
    char msg[10];
    sprintf(msg, "%.2f", lastLightAvg);
    
    // Publish to both topics for compatibility
    client.publish("medibox/light/average", msg, true); // Retained message
    client.publish("MediBox_LDR_Avg_Nuwan", msg, true); // Retained message with consistent naming
    
    Serial.print("Sending 2-minute light average: ");
    Serial.print(lastLightAvg, 3);
    Serial.print(" (from ");
    Serial.print(sampleCount);
    Serial.println(" samples)");
    
    // Reset for next averaging period
    lightSum = 0;
    sampleCount = 0;
  }
  
  // Check network status
  checkNetworkStatus();
  
  if (digitalRead(PB_OK) == LOW) {
    beep();
    delay(200);
    go_to_menu();
  }
  
  // Process incoming MQTT messages again to ensure quick response
  client.loop();
  
  delay(5000); 
}
