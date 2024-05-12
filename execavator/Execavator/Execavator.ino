#include <Arduino.h>
#include "Adafruit_MCP23X17.h"
#include <ESP32Servo.h> // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define clawServoPin 5
#define auxServoPin 18
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

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Adafruit_MCP23X17 mcp;
Servo clawServo;
Servo auxServo;
int clawServoValue = 90;
int auxServoValue = 90;

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

int processTriggerServo(int trigger1, int trigger2, int movementBitmask) {
  int movement = STOP;
  if (movementBitmask & trigger1) {
    movement++;
  }
  if (movementBitmask & trigger2) {
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

void processGamepad(ControllerPtr ctl) {
  processMovement(ctl->buttons());
  processThumbControl(ctl->buttons());
  processAuxControl(ctl->dpad());

  //swingarm
  processAxis(ctl->axisX(), pivot0, pivot1);
  //dipper
  processAxis(ctl->axisRX(), secondBoom1, secondBoom0);
  //boom
  processAxis(ctl->axisRY(), mainBoom0, mainBoom1);
  //attachment
  processAxis(ctl->axisY(), boomAttach1, boomAttach0);

  moveServo(processTriggerServo(thumbButtonT, thumbButtonS, ctl->buttons()), clawServo, clawServoValue);
}

void processAuxControl(int movementBitmask) {
  int movement = STOP;

  if (movementBitmask & thumbButtonX) {
    movement++;
  }
  if (movementBitmask & thumbButtonY) {
    movement--;
  }
  moveMotor(auxAttach0, auxAttach1, movement);
}

void processAxis(int axisValue, int motor0, int motor1) {
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
  moveMotor(motor0, motor1, movement);
}

void processThumbControl(int movementBitmask) {
  //Thumb Motor
  int movement = STOP;
  if (movementBitmask & thumbButtonX) {
    movement++;
  }
  if (movementBitmask & thumbButtonY) {
    movement--;
  }
  moveMotor(thumb0, thumb1, movement);
}

void processMovement(int movementBitmask) {
  //Right Motor
  int rightMovement = STOP;
  if (movementBitmask & rightTrigger1) {
    rightMovement++;
  }
  if (movementBitmask & rightTrigger2) {
    rightMovement--;
  }
  moveMotor(rightMotor0, rightMotor1, rightMovement);

  //Left Motor
  int leftMovement = STOP;
  if (movementBitmask & leftTrigger1) {
    leftMovement++;
  }
  if (movementBitmask & leftTrigger2) {
    leftMovement--;
  }
  moveMotor(leftMotor0, leftMotor1, leftMovement);
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

    clawServo.attach(clawServoPin);
    clawServo.write(clawServoValue);
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

