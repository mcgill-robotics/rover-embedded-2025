#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoSTL.h>
#include <vector> 

// This allows members to define their own setups or add to it if needed
__attribute__((weak)) void initCustomSetup() {
  Serial.begin(9600);
  
}

// Just to avoid errors if initCustomCommands() isn't defined in the module importing this
__attribute__((weak)) void initCustomCommands() {
  
}

// Define the type for callback functions --> pointer type. Allows for flexibility
typedef void (*CommandCallback)(JsonVariant data);


// Associate command names with callback functions
struct CommandHandler {

  // Name of command goes here
  const char* commandName;

  // The actual function we care about
  CommandCallback callback;
};

// Example callback function --> Won't actually be using this, but it will be similar to what an elec member creates in their code
void ledOn(JsonVariant data) {

  int val = data["value"] | 0;  // Takes 0 if "value" doesn't exist

  digitalWrite(LED_BUILTIN, val);
  Serial.println("{\"status\": \"ok\"}");

}

// This will serve as a dynamic container. Allows for us to add more and more commandName/callback pairs
std::vector<CommandHandler> handlers;

void registerCommand(const char* cmdName, CommandCallback callback){

  CommandHandler newHandler = {cmdName, callback};
  handlers.push_back(newHandler);

}



void setup() {

  // This is for running any setup code. For example, pinMode(pin, OUTPUT)
  initCustomSetup();

  // ELEC MEMBERS SHOULD CREATE THEIR OWN initCustomCommands() where they use registerCommand() for every function
  initCustomCommands();

}


void loop() {
  if (Serial.available()) {

    // Read until new message sent
    String json = Serial.readStringUntil('\n');

    // Want to store a json document in memory
    DynamicJsonDocument doc(256);


    // This just checks for any errors if they occur
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
      Serial.println("{\"status\": \"error\", \"msg\": \"bad json\"}");
      return;
    }

    const char* cmd = doc["cmd"];

    if (!cmd) {
      Serial.println("{\"status\": \"error\", \"msg\": \"no command\"}");
      return;
    }

    // Look for a matching handler
    for (int i = 0; i < handlers.size(); i++) {

      // Note: strcmp returns 0 if the two strings ARE equal
      if (strcmp(cmd, handlers[i].commandName) == 0) {

        // Feed the callback function with the JSON we provide from the serial monitor
        handlers[i].callback(doc);
        return;
      }
    }

    // If no handler was found
    Serial.println("{\"status\": \"error\", \"msg\": \"unknown command\"}");
  }
}
