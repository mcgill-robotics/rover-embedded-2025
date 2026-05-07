#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_gap_bt_api.h"
#include "Wire.h"

#include "hardware_pins.h"

#if RADIO_MODE == RADIO_MODE_RX
// INSTRUCTIONS
//  1. Set RADIO_MODE to 0 for TX, 1 for RX and upload to ESP32.
//  2. Power on slave (RX) ESP32.
//  3. Power on master (TX) ESP32.
//  4. Wait for master to connect to slave.
//  5. Press button on TX ESP32 to send "ON" to RX ESP32.

// VARIABLES
volatile bool rover_stopped = false;

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

int microsBetweenReadings = 1000;
int lastReadTime = 0;

// Function prototype for non-blocking tone
void update_tone();

// Variables for non-blocking tone management
unsigned long previous_millis_tone = 0; // stores the last time the tone was updated
const long tone_interval = 500;         // interval at which to play tone (milliseconds)
const long interval_bt = 100;           // interval at which to play tone (milliseconds)
volatile bool tone_state = false;       // state of the tone (on/off)

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

void rx_setup()
{
  // Transistor gate connected to GPIO 14.
  pinMode(TRANSISTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(COMMUNICATION_WITH_TEENSY_PIN, OUTPUT);

  /* I2C code commented
  Wire.begin(RX_I2C_DATA_PIN, RX_I2C_CLOCK_PIN, 0x30); // Initialize I2C with specified SDA and SCL pins
  Wire.setClock(100000); // Set I2C clock speed to 100 kHz  
  */
  
  Serial.begin(115200);     // Start Serial communication for debugging
  ESP_BT.begin("ESP32_RX"); // Start Bluetooth with name ESP32_RX
  Serial.println("Bluetooth Switch is Ready to Pair");

  digitalWrite(LED_PIN, LOW);
  digitalWrite(TRANSISTOR_PIN, LOW);
  digitalWrite(COMMUNICATION_WITH_TEENSY_PIN, LOW);

  lastReadTime = micros();
}

void rx_loop()
{
  const size_t MAX_MSG_LEN = 128; // Set the maximum message length
  char buffer[MAX_MSG_LEN + 1];   // Create a buffer to store incoming data
  Wire.beginTransmission(0x30); // Start I2C transmission to the device with address 0x30
  if (ESP_BT.connected())
  { // check if connection is established with TX
    digitalWrite(LED_PIN, HIGH);
    if (ESP_BT.available())
    { // Check if data is available to read
      // String msg = ESP_BT.readStringUntil('\n');                           // Read the incoming data
      size_t bytesRead = ESP_BT.readBytesUntil('\n', buffer, MAX_MSG_LEN); // Read available data into the buffer
      buffer[bytesRead] = '\0';                                            // Null-terminate the string
      // Process the received message
      String msg(buffer);
      msg.trim(); // Remove any whitespace

      // Flip if necessary.
      if (msg == "ON")
      {
        if (rover_stopped)
        { // if the rover is currently dead, make sound and start rover
          tone(BUZZER, 85);
          delay(500);
          noTone(BUZZER);
          delay(250);

          rover_stopped = false; // toggle kill var to signal the fact that we have started the rover
        }
        digitalWrite(LED_PIN, LOW);
        digitalWrite(TRANSISTOR_PIN, HIGH);
        digitalWrite(COMMUNICATION_WITH_TEENSY_PIN, HIGH); // Teensy should read that high when dead
        
        /* I2C code commented
        Wire.write("ON"); // Send "ON" command to the I2C device
        Wire.endTransmission(); // End I2C transmission
        */
      }
      else if (msg == "OFF")
      {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(TRANSISTOR_PIN, LOW);
        digitalWrite(COMMUNICATION_WITH_TEENSY_PIN, LOW);  // Teensy should read that high when dead
        
        /* I2C code commented
        Wire.write("OFF"); // Send "OFF" command to the I2C device
        Wire.endTransmission(); // End I2C transmission
        */
        
        rover_stopped = true; // toggle kill var to signal the fact that we have killed the rover
                              // rover is currently dead
        tone(BUZZER, 85);
        delay(500);
        noTone(BUZZER);
        delay(250);
      }
    }
    ESP_BT.flush();
  }
  else if (!ESP_BT.connected())
  { // connection was lost/not established, so kill rover
    digitalWrite(LED_PIN, LOW);
    digitalWrite(TRANSISTOR_PIN, LOW);
    Wire.write("OFF"); // Send "OFF" command to the I2C device
    Wire.endTransmission(); // End I2C transmission
  }
  Serial.printf("LED: %d, TRANSISTOR: %d\r\n", digitalRead(LED_PIN), digitalRead(TRANSISTOR_PIN));
}

void setup()
{
#if DEBUG_MODE == 1
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
#endif

  rx_setup();
}

void loop()
{
#if DEBUG_MODE == 1
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
#endif

  rx_loop();
}
#endif // RADIO_MODE == RADIO_MODE_RX
