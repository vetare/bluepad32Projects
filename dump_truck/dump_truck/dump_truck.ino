// make sure to upload with ESP32 Dev Module selected as the board under tools>Board>ESP32 Arduino

#include <Arduino.h>
#include <ESP32Servo.h> // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// defines
#define steeringServoPin  23
#define dumpServoPin 22

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define ARMUP 5
#define ARMDOWN 6
#define STOP 0

#define motor0 32
#define motor1 33

#define FORWARD 1
#define BACKWARD -1

#define rightTrigger1 32
#define rightTrigger2 128
#define leftTrigger1 16
#define leftTrigger2 64

// global variables

Servo steeringServo;
Servo dumpServo;

int dumpBedServoValue = 150;
int steeringServoValue = 50;

void processGamepad(ControllerPtr ctl) {
  processMovement(ctl->axisY(), ctl->axisX());

  processdumpServoMovement(processAxisServo(ctl->axisRY()), dumpServo, dumpBedServoValue);
  processSteeringServoMovement(processAxisServo(ctl->axisRX()), steeringServo, steeringServoValue);
}

void processMovement(int axisYValue, int axisXValue) {
  int minimal_control_input_stick = 300;

  int movement = STOP;
  if (abs(axisYValue) >= minimal_control_input_stick) {
    if (axisYValue > 0) {
      movement++;
    }
    if (axisYValue < 0) {
      movement--;
    }
  }
  moveMotor(motor0, motor1, movement);
}

void setUpPinModes()
{
  pinMode(motor0, OUTPUT);
  pinMode(motor1, OUTPUT);
  
  steeringServo.attach(steeringServoPin);
  dumpServo.attach(dumpServoPin);

  //steeringServo.write(bucketServoValue);
  dumpServo.write(dumpBedServoValue);
  steeringServo.write(steeringServoValue);
}

int processAxisServo(int axisValue) {
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

void processSteeringServoMovement(int movement, Servo &servo, int &servoValue){
  switch (movement) {
    case -1:
        if(servoValue >= 5 && servoValue < 130)
        {
          Serial.println(servoValue);
          servoValue = servoValue + 5;
          servo.write(servoValue);
          delay(10);
        }
    break;
    case 1:
        if(servoValue <= 130 && servoValue > 5)
        {
          Serial.println(servoValue);
          servoValue = servoValue - 5;
          servo.write(servoValue);
          delay(10);
        }
    break;
    default :
          delay(10);
          servoValue = 50;
          servo.write(servoValue);
  }
}

void processdumpServoMovement(int movement, Servo &servo, int &servoValue){
  switch (movement) {
    case 1:
        if(servoValue >= 5 && servoValue < 155)
        {
          Serial.println(servoValue);
          servoValue = servoValue + 5;
          servo.write(servoValue);
          delay(10);
        }
    break;
    case -1:
        if(servoValue <= 155 && servoValue > 5)
        {
          Serial.println(servoValue);
          servoValue = servoValue - 5;
          servo.write(servoValue);
          delay(10);
        }
    break;
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity == FORWARD)
  {
    analogWrite(motorPin0,255);
    analogWrite(motorPin1, 0);
  }
  else if (velocity == BACKWARD)
  {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 255);
  }
  else
  {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
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

void setup(void)
{
  Serial.begin(115200);
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

  setUpPinModes();
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
