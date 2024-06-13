#include <Arduino.h>
#include "Adafruit_MCP23X17.h"
#include <ESP32Servo.h> // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

//Definitions of the pins on the ESP32 board and what they are connected to in real life.
#define auxServoPin 5
#define aux1ServoPin 18
#define cabLights 32
#define auxLights 33

#define pivot0 15       // Used for controlling pivot movement (e.g., for controlling tilt)
#define pivot1 14       // Used for controlling pivot movement (e.g., for controlling tilt)
#define mainBoom0 9     // Used for controlling the main boom movement
#define mainBoom1 8     // Used for controlling the main boom movement
#define secondBoom0 0   // Used for controlling the second boom movement
#define secondBoom1 1   // Used for controlling the second boom movement
#define boomAttach0 3   // Used for controlling attachment boom movement
#define boomAttach1 2   // Used for controlling attachment boom movement
#define thumb0 11       // Used for controlling the thumb movement
#define thumb1 10       // Used for controlling the thumb movement
#define auxAttach0 12   // Used for controlling auxiliary attachment movement
#define auxAttach1 13   // Used for controlling auxiliary attachment movement

#define leftMotor0 7    // Used for controlling the left motor movement
#define leftMotor1 6    // Used for controlling the left motor movement
#define rightMotor0 4   // Used for controlling the right motor movement
#define rightMotor1 5   // Used for controlling the right motor movement

#define rightTrigger1 32
#define rightTrigger2 128
#define leftTrigger1 16
#define leftTrigger2 64
#define thumbButtonX 1
#define thumbButtonY 2
#define thumbButtonT 8
#define thumbButtonS 4
#define leftAxisPressed 100
#define rightAxisPressed 200
#define dpadUp 1
#define dpadDown 2

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

//Change this if you want to turn debugging mode on where the gamepad values will be output in the serial montior view.
#define debugMode false
//Change this to true if you want to stop the model from moving useful while in debug mode
#define blockControls false

Adafruit_MCP23X17 mcp;
Servo auxServo;
Servo aux1Servo;
int auxServoValue = 90;
int aux1ServoValue = 90;

bool cabLightsOn = false;
bool auxLightsOn = false;

//Main loop, this is called every "tick" so all processing code should be triggered from inside this method.
void processGamepad(ControllerPtr ctl) {
  if (debugMode) {
    dumpGamepad(ctl);
  }

  if (!blockControls) {
    processMovement(ctl->buttons());
    processArmsAndBucketControls(ctl);
    processLights(ctl->buttons());

    //Exec V2 with new grabbing attachment
    processGrabAttachment(ctl->buttons());
    processFrontBucketAttachment(ctl->dpad());
  } 
}

void processMovement(int movementBitmask) {
  //Right Motor
  moveMotor(rightMotor0, rightMotor1, calculateButtonPress(movementBitmask, rightTrigger1, rightTrigger2));
  //Left Motor
  moveMotor(leftMotor0, leftMotor1, calculateButtonPress(movementBitmask, leftTrigger1, leftTrigger2));
}

void processArmsAndBucketControls(ControllerPtr ctl) {
  //swingarm
  moveMotor(pivot0, pivot1, calculateAxis(ctl->axisX()));
  //dipper
  moveMotor(secondBoom1, secondBoom0, calculateAxis(ctl->axisRX()));
  //boom
  moveMotor(mainBoom0, mainBoom1, calculateAxis(ctl->axisRY()));
  //attachment
  moveMotor(boomAttach1, boomAttach0, calculateAxis(ctl->axisY()));
}

void processLights(int bitMask) {
  if (bitMask & leftAxisPressed) { 
    controlLights(bitMask, cabLightsOn, cabLights);
  }

  if (bitMask & rightAxisPressed) { 
    controlLights(bitMask, auxLightsOn, auxLights);
  }
}

//Added with V2 of the Exec
void processGrabAttachment(int movementBitmask) {
  //Rotate Grabber Motor
  moveMotor(thumb0, thumb1, calculateButtonPress(movementBitmask, thumbButtonX, thumbButtonY));
  //Open or close the grabber
  moveServo(calculateButtonPress(movementBitmask, thumbButtonS, thumbButtonT), auxServo, auxServoValue);
}

void processFrontBucketAttachment(int movementBitmask) {
  moveMotor(auxAttach0, auxAttach1, calculateButtonPress(movementBitmask, dpadUp, dpadDown));
}

int calculateAxis(int axisValue) {
  int minimal_control_input_stick = 300;

  int movement = STOP;
  if (abs(axisValue) >= minimal_control_input_stick) {
    if (axisValue > 0) {
      movement++;
    }
    if (axisValue < 0) {
      movement--;
    }
  }

  return movement;
}

int calculateButtonPress(int movementBitMask, int forwards, int backwards){
  int movement = STOP;
  if (movementBitMask & forwards) {
    movement++;
  }
  if (movementBitMask & backwards) {
    movement--;
  }
  return movement;
}

void moveServo(int movement, Servo &servo, int &servoValue) {
  switch (movement) {
    case 1:
        if(servoValue >= 10 && servoValue < 170)
        {
          servoValue = servoValue + 5;
          servo.write(servoValue);
          delay(10);
        }
    break;
    case -1:
        if(servoValue <= 170 && servoValue > 10)
        {
          servoValue = servoValue - 5;
          servo.write(servoValue);
          delay(10);
        }
    break;
  }
}

void controlLights(int bitMask, bool &lightStatus, int lightPin) {
    if (!lightStatus) {
      digitalWrite(lightPin, HIGH);
      lightStatus = true;
    } else {
      digitalWrite(lightPin, LOW);
      lightStatus = false;
    }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity == FORWARD)
  {
    mcp.digitalWrite(motorPin0, HIGH);
    mcp.digitalWrite(motorPin1, LOW);
  }
  else if (velocity == BACKWARD)
  {
    mcp.digitalWrite(motorPin0, LOW);
    mcp.digitalWrite(motorPin1, HIGH);
  }
  else
  {
    mcp.digitalWrite(motorPin0, LOW);
    mcp.digitalWrite(motorPin1, LOW);
  }
}

// Used to bind the pins so we can access in the code.
void pinSetup() {
    //Binding the outputs of the MCP chip.
    mcp.pinMode(leftMotor0, OUTPUT);
    mcp.pinMode(leftMotor1, OUTPUT);
    mcp.pinMode(rightMotor0, OUTPUT);
    mcp.pinMode(rightMotor1, OUTPUT);
    mcp.pinMode(pivot0, OUTPUT);
    mcp.pinMode(pivot1, OUTPUT);
    mcp.pinMode(secondBoom0, OUTPUT);
    mcp.pinMode(secondBoom1, OUTPUT);
    mcp.pinMode(mainBoom0, OUTPUT);
    mcp.pinMode(mainBoom1, OUTPUT);
    mcp.pinMode(boomAttach0, OUTPUT);
    mcp.pinMode(boomAttach1, OUTPUT);
    mcp.pinMode(thumb0, OUTPUT);
    mcp.pinMode(thumb1, OUTPUT);
    mcp.pinMode(auxAttach0, OUTPUT);
    mcp.pinMode(auxAttach1, OUTPUT);

    //Attach the servo pins.
    auxServo.attach(auxServoPin);
    auxServo.write(auxServoValue);

    aux1Servo.attach(aux1ServoPin);
    aux1Servo.write(aux1ServoValue);
}

// -------------------------------------------------------------- Boiler plate code, no need to modify this -------------------------------------------------------------------- //

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    mcp.begin_I2C();
    //   put your setup code here, to run once:
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
    // You could add additional error handling here,
    // such as logging the error or attempting to recover.
    // For example, you might attempt to reset the MCP23X17
    // and retry initialization before giving up completely.
    // Then, you could gracefully exit the program or continue
    // running with limited functionality.  
    
    pinSetup();
  }

  //Used for just debugging input values produced by the gamepad buttons presses.
  void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
      "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
      "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
      ctl->index(),        // Controller Index
      ctl->dpad(),         // D-pad
      ctl->buttons(),      // bitmask of pressed buttons
      ctl->axisX(),        // (-511 - 512) left X Axis
      ctl->axisY(),        // (-511 - 512) left Y axis
      ctl->axisRX(),       // (-511 - 512) right X axis
      ctl->axisRY(),       // (-511 - 512) right Y axis
      ctl->brake(),        // (0 - 1023): brake button
      ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
      ctl->miscButtons(),  // bitmask of pressed "misc" buttons
      ctl->gyroX(),        // Gyro X
      ctl->gyroY(),        // Gyro Y
      ctl->gyroZ(),        // Gyro Z
      ctl->accelX(),       // Accelerometer X
      ctl->accelY(),       // Accelerometer Y
      ctl->accelZ()        // Accelerometer Z
    );
  }

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
    {
        processControllers();
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    else {vTaskDelay(1);}
}

