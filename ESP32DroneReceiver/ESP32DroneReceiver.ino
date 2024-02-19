#include <Bluepad32.h>
#include "sbus.h"

/* Define and setup SBUS Rx and Tx objects
   Inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits.
*/
#define SBUS_RX 16 // ESP32
#define SBUS_TX 17 // ESP32

bfs::SbusTx sbus_tx(&Serial2, SBUS_RX, SBUS_TX, true); // Set up TX SBUS using inverted serial logic to conform to SBUS standard

/* SBUS data */
bfs::SbusData data;

/* Set-up min/max for AXIS controls and BetaFlight min/max inputs */
#define AXIS_MIN -512
#define AXIS_MAX 511

#define BETAFLIGHT_MIN 1000
#define BETAFLIGHT_MAX 2000

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
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

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // DPAD
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-512 - 511) left X Axis
        ctl->axisY(),        // (-512 - 511) left Y axis
        ctl->axisRX(),       // (-512 - 511) right X axis
        ctl->axisRY(),       // (-512 - 511) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmak of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

/*
 * ctl : pointer to Controller/Gamepad
 * mode: Drone Flying Mode 1,2,3,4 (default = 1)
 */
void writeGamepadAxisWithSBUS(ControllerPtr ctl, int droneMode = 1) {

    // Left to right on Gamepad is -512 to 511 so map it to 1000 to 2000 (left should be 1000 and right should be 2000)
    int16_t mappedLeftX = map(ctl->axisX(), AXIS_MIN, AXIS_MAX, BETAFLIGHT_MIN, BETAFLIGHT_MAX);
    int16_t mappedRightX = map(ctl->axisRX(), AXIS_MIN, AXIS_MAX, BETAFLIGHT_MIN, BETAFLIGHT_MAX);
  
    // Up to Down on Gamepad is -512 to 511 so map it to 2000 to 1000 so that 2000 is up and 1000 is down
    // Flip fromLow and fromHigh in map function to easily keep negative as high and positive as low, post mapping
    int16_t mappedLeftY = map(ctl->axisY(), AXIS_MAX, AXIS_MIN, BETAFLIGHT_MIN, BETAFLIGHT_MAX);
    int16_t mappedRightY = map(ctl->axisRY(), AXIS_MAX, AXIS_MIN, BETAFLIGHT_MIN, BETAFLIGHT_MAX);
  
  
    Serial.printf("leftAxisX=%4d, leftAxisXMapped=%4d \n", ctl->axisX(), mappedLeftX);
    Serial.printf("leftAxisY=%4d, leftAxisYMapped=%4d \n", ctl->axisY(), mappedLeftY);
    
    Serial.printf("rightAxisX=%4d, rightAxisXMapped=%4d \n", ctl->axisRX(), mappedRightX);
    Serial.printf("rightAxisY=%4d, rightAxisYMapped=%4d \n", ctl->axisRY(), mappedRightY);

    int16_t aileron = 1500;
    int16_t elevator = 1500;
    int16_t throttle = 1500;
    int16_t rudder = 1500;

    if (droneMode == 1)
    {
      aileron = mappedRightX;
      elevator = mappedLeftY;
      throttle = mappedRightY;
      rudder = mappedLeftX;
    }
    
    // Setup channels 0-3 for sending A E T R
    Serial.printf("AILERON=%4d, ELEVATOR=%4d, THROTTLE=%4d, RUDDER=%4d  \n", aileron, elevator, throttle, rudder);
    
    // Aileron / ROLL
    data.ch[0] = aileron;
    // Elevator / PITCH
    data.ch[1] = elevator;
    // Throttle / THROTTLE
    data.ch[2] = throttle;
    // Rudder / YAW
    data.ch[3] = rudder;

    // See 11 bit channel values as binary before being packed into buffer format
    Serial.println(data.ch[0] & 0x07FF, BIN);
    Serial.println(data.ch[1] & 0x07FF, BIN);
    Serial.println(data.ch[2] & 0x07FF, BIN);
    Serial.println(data.ch[3] & 0x07FF, BIN);

    // Set new data to write using SBUS restrictions so channel values only take 11 bits and are packed across bytes
    // Write onto TX line which was instantiated earlier
    sbus_tx.data(data);
    sbus_tx.Write();
}

void processGamepad(ControllerPtr ctl) {
    // dumpGamepad(ctl); // Use this to see raw values across entire controller
    writeGamepadAxisWithSBUS(ctl);
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    /* Begin the SBUS communication */
    sbus_tx.Begin();
    
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "vritual device", is a mouse
    // By default it is disabled.
    BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr myController = myControllers[i];

        if (myController && myController->isConnected()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.printf("Data not available yet\n");
                continue;
            }
            // See ArduinoController.h for all the available functions.
        }
    }
    
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
    delay(1500);
}
