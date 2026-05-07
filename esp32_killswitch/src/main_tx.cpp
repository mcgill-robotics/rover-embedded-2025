#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_gap_bt_api.h"

#include "hardware_pins.h"

#if RADIO_MODE == RADIO_MODE_TX
// INSTRUCTIONS
//  1. Set RADIO_MODE to 0 for TX, 1 for RX and upload to ESP32.
//  2. Power on slave (RX) ESP32.
//  3. Power on master (TX) ESP32.
//  4. Wait for master to connect to slave.
//  5. Press button on TX ESP32 to send "ON" to RX ESP32.

// BLUETOOTH ------------------------------------------------------------------------------
// BT: Bluetooth availabilty check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
// BT: Serial Bluetooth availabilty check
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial ESP_BT; // Object for Bluetooth
const char *server_name = "ESP32_RX";
const int maxRetries = 100000; // Maximum number of connection attempts
// const int retryDelay = 500;    // Delay between retries in milliseconds

int microsBetweenReadings = 1000;
int lastReadTime = 0;

// Function prototype for non-blocking tone
void update_tone();
void bt_send_safe(String Str);

// Variables for non-blocking tone management
unsigned long previous_millis_tone = 0; // stores the last time the tone was updated
// const long tone_interval = 500;    // interval at which to play tone (milliseconds)
const long interval_bt = 500;
volatile bool tone_state = false;     // state of the tone (on/off)
unsigned long previous_millis_bt = 0; // stores the last time the tone was updated

volatile bool rover_stopped = false;

// sends "ON" so RX will stop rover
// done so function sits in ram instead of flash so its called faster
void tx_send_stop_rover()
{
  // ESP_BT.println("OFF"); // Send "ON" via Bluetooth
  bt_send_safe("OFF");
  rover_stopped = true;
}

// Buzzes the buzzer for a short period of time
void update_tone(int tone_interval)
{
  unsigned long current_millis = millis();

  // Check if it's time to update the tone state
  if (current_millis - previous_millis_tone >= tone_interval)
  {

    if (tone_state)
    {
      tone(BUZZER, 100); // Start playing the tone
    }
    else
    {
      noTone(BUZZER); // Stop playing the tone
    }

    // Toggle the tone state for the next interval
    tone_state = !tone_state;
    // Save the last time you updated the tone
    previous_millis_tone = current_millis;
  }
}

// Buzzes the buzzer for a short period of time
void bt_send_safe(String Str)
{
  unsigned long current_millis = millis();

  // Check if it's time to update the tone state
  if (current_millis - previous_millis_bt >= interval_bt)
  {
    ESP_BT.println(Str);
    previous_millis_bt = current_millis;
  }
}

void tx_setup()
{
  // PIN SETUP
  // Button is pulled up, so when it's pressed, it will read LOW
  pinMode(BUTTON_PIN, INPUT); // setting up the pins
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // TODO shouldn't this be FALLING? check if it works
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), tx_send_stop_rover, RISING); // interrupt so we don't miss button being pressed, set to rising edge

  Serial.begin(115200);

  // Initialize Bluetooth with name ESP32_TX, true sets it as master
  ESP_BT.begin("ESP32_TX", true);

  digitalWrite(LED_PIN, LOW);

  int attempts = 0;
  // attempt to connect and stop after failing 100 000 times
  while (!ESP_BT.connect(server_name) && attempts < maxRetries)
  {
    attempts++;
    Serial.println("Attempting to connect to " + String(server_name) + ", attempt " + String(attempts));
    // delay(retryDelay);
  }

  if (ESP_BT.connected())
  {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Connected to " + String(server_name));
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
    Serial.println("Failed to connect to " + String(server_name));
  }

  // Set the initial time for the tone management
  previous_millis_tone = millis();
  previous_millis_bt = millis();
}

void tx_loop()
{
  // delay(10);

  // Reconnect if the connection is lost
  if (ESP_BT.connected())
  {
    digitalWrite(LED_PIN, HIGH);
    // Read the button state. If it's HIGH, send "ON" to the receiver. If it's LOW, send "OFF".
    int button_state = !digitalRead(BUTTON_PIN);
    String button_state_str = button_state == HIGH ? "pressed" : "released";
    Serial.println("Button state: " + button_state_str);
    if (button_state == HIGH)
    {
      // tx_send_stop_rover();
      bt_send_safe("OFF");
      // ESP_BT.println("OFF"); // Send "OFF" via Bluetooth
      update_tone(200);
    }
    else if (button_state == LOW)
    {
      // ESP_BT.println("ON"); // Send "OFF" via Bluetooth
      bt_send_safe("ON");
      update_tone(500);
      // noTone(BUZZER); // Stop playing the tone
    }
    ESP_BT.flush();
  }
  else if (!ESP_BT.connected())
  {
    digitalWrite(LED_PIN, LOW);
    int attempts = 0;
    while (!ESP_BT.connect(server_name) && attempts < 10)
    {
      attempts++;
      Serial.println("Reconnecting to " + String(server_name) + ", attempt " + String(attempts));
      // delay(retryDelay);
    }
  }
}

void setup()
{
#if DEBUG_MODE == 1
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
#endif

  tx_setup();
}

void loop()
{
#if DEBUG_MODE == 1
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
#endif

  tx_loop();
}
#endif // RADIO_MODE == RADIO_MODE_TX