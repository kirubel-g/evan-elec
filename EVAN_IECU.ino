
// Include the SPI library for SPI communication
#include <SPI.h>
// Include the mcp_can library for CAN bus communication
#include <mcp_can.h>
// Include the PID_v1 library for PID control
#include <PID_v1.h>

// Define constants for the SPI Chip Select (CS) pins used for the CAN controllers
const int spiCSPinFCAN = 35;  // Chip Select pin for the first CAN controller
const int spiCSPinFCAN2 = 53; // Chip Select pin for the second CAN controller
const int spiCSPinBCAN = 37;  // Chip Select pin for the third CAN controller

// Variables to keep track of the last time CAN messages were sent
// This is used to manage timing and ensure that messages are not sent too frequently
unsigned long previousCanSend = 0;   // Variable to store the last send time for the first CAN controller
unsigned long previousCanSend2 = 0;  // Variable to store the last send time for the second CAN controller

// Define pin numbers for various vehicle controls and sensors
int horn = A1;         // Horn control pin
int hazRet = A4;       // Hazard lights return pin
int auxSw1 = A5;       // Auxiliary switch 1
int auxSw2 = A6;       // Auxiliary switch 2
int igSw1 = A2;        // Ignition switch 1
int igSw2 = A3;        // Ignition switch 2
int app1 = A15;        // Accelerator pedal position sensor 1
int app2 = A14;        // Accelerator pedal position sensor 2
int brake = A13;       // Brake sensor
int lock = A10;        // Door lock control
int unlock = A9;       // Door unlock control
int seatBelt = A8;     // Seat belt sensor

// Define pins for vehicle's electrical system components
int r7 = 42;              // Relay or component connected to pin 42
int accPow = 47;          // Accessory power control
int illumi = 49;          // Illumination control (e.g., dashboard lights)
int motPow = 5;           // Motor power control
int contactorPos = 45;    // Positive contactor for the main battery or motor
int contactorNeg = 43;    // Negative contactor for the main battery or motor
int precharge = 4;        // Precharge circuit control
int dashPow = 6;          // Dashboard power control
int revSig = 46;          // Reverse signal control

// Variables for ignition and security logic
int pushCount = 0;      // Counter for push button presses
int keyOn = 0;          // State variable indicating if the key is in the "On" position
int security = 0;       // Security or authentication state
int IECUshutdown = 0;   // Flag indicating if the Intelligent Electronic Control Unit (IECU) should shut down
int genericFlag1 = 0;   // A generic flag for extending logic as needed
int ignite = 0;         // State indicating if ignition has occurred
int holdECUOn = 0;      // Flag to keep the ECU powered on, regardless of other conditions
int igState = 0;        // State of the ignition system
int igSwitch1 = 0;      // State of ignition switch 1
int igSwitch2 = 0;      // State of ignition switch 2
int br = 0;             // Brake state or sensor reading


// Timing variables for button presses and IECU operation
unsigned long pushTimer = 0;  // Timer for tracking duration of button presses
unsigned long IECUtimer = 0;  // Timer for tracking IECU operation time

// Steering control and data
int i = 0;  // Generic counter or index variable
int j = 0;  // Another generic counter or index variable
int counter = 0;  // Counter for looping or iterative operations
unsigned char stmp202[8] = {0x30, 0x03, 0, 0x00, 0x00, 0, 0x19, 0x93}; // Data array for Tachometer CAN message
unsigned char stmp2170[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xFC}; // Data array for Steering Wheel/Vehicle Speed Sensor (VSS) CAN message
unsigned long lastSend = 0;  // Timestamp for last CAN message sent

// Climate control variables
byte acByte = 0;    // Byte to control air conditioning status
byte fanByte = 0;   // Byte to control fan speed
byte tempByte = 0;  // Byte to control temperature setting
int acReq = 0;      // AC request status (on/off)
int heatReq = 0;    // Heater request status (on/off)
int fan = 0;        // Fan speed level

// Lighting control commands
int brakeCommand = 0;    // Command to activate/deactivate brake lights
int hazCommand = 0;      // Command to activate/deactivate hazard lights
int hornCommand = 0;     // Command to activate/deactivate the horn
int leftBlink = 0;       // Command for left blinker
int rightBlink = 0;      // Command for right blinker
int headlampOn = 0;      // Headlamp status (on/off)
int highbeamOn = 0;      // High beam status (on/off)
int parkLampsOn = 0;     // Parking lamps status (on/off)

// Gear and transmission information
int rawGear = 0;    // Raw gear value from sensor or input
int gear = 0;       // Processed or actual gear position

// Motor and drive system variables
int prechrgFinished = 0;  // Status of precharge process (finished/not finished)
int maxDisc = 700;        // Maximum discharge current limit
int maxChrg = 500;        // Maximum charge current limit
int vehicleState = 0;     // Current state of the vehicle (e.g., driving, stopped)
int keyState = 0;         // State of the vehicle key (in ignition, turned, etc.)
int mainRelayCmd = 1;     // Command to control the main relay
int maxDiscRaw = 0;       // Raw data for maximum discharge current
int maxChrgRaw = 0;       // Raw data for maximum charge current
int gearLvrPos = 0;       // Gear lever position
int accelRaw = 0;         // Raw accelerator pedal position
float accelPct = 0;       // Accelerator pedal position as a percentage
float accelPct2 = 0;      // Secondary accelerator pedal position as a percentage
float regenPct = 0;       // Regenerative braking percentage
int workMode = 0;         // Work mode of the vehicle
int motorMode = 1;        // Current mode of the motor
int rollingCounter = 0;   // Counter for rolling code or timing
int acCommand = 1;        // Command for air conditioning system
int motorRotation = 3;    // Direction of motor rotation
int motorDat1 = 0;        // General purpose motor data variable

// Motor speed and vehicle speed sensor (VSS) variables
uint16_t motorSpeed = 0;  // Current speed of the motor
float vss = 0;            // Vehicle speed sensor reading in km/h or mph
uint16_t vssRaw = 0;      // Raw VSS data
int vssDash = 0;          // VSS reading to display on the dashboard

// Precharge and contactor control variables
int preChargeStart = 0;               // Indicator for start of precharge process
unsigned long contactorTimer = 0;     // Timer for contactor operation
unsigned long previousCanMotor1 = 10; // Offset from power steering commands to avoid collision
unsigned long timer = 0;              // General purpose timer for

// Variables for handling acceleration and braking sensor data
int total4 = 0;                            // Sum of readings for the accelerator pedal
const int numReadings4 = 10;               // Number of readings to average for the accelerator pedal
int readings4[numReadings4];               // Array to store readings for the accelerator pedal
int readIndex4 = 0;                        // Current index for inserting the next reading for the accelerator pedal
unsigned long passedtime3 = 3;             // Time since the last reading was taken for fuel calculation
float gasoline = 0;                        // Calculated gasoline consumption based on accelerator pedal position

int total5 = 0;                            // Sum of readings for the brake pedal
const int numReadings5 = 10;               // Number of readings to average for the brake pedal
int readings5[numReadings5];               // Array to store readings for the brake pedal
int readIndex5 = 0;                        // Current index for inserting the next reading for the brake pedal
float gasoline2 = 0;                       // Calculated gasoline consumption based on brake pedal position

int throttleError = 0;                     // Error flag for throttle position
float outputValue = 0;                     // PID controller output value for throttle control
unsigned long previousMillis = 0;          // Previous time in milliseconds used for throttle control timing
float previousValue = 0;                   // Previous throttle position value for calculating rate of change
float decreasePercentage = 0;              // Percentage decrease in throttle position to apply
int lockup = 0;                            // Lockup flag for the throttle system
int motorCounter = 0;                      // Counter for motor control operations

uint16_t brakePos = 0;                     // Position of the brake pedal
int brakePosInt = 0;                       // Integer representation of the brake pedal position
float stopCommand = 0;                     // Command value for stopping the vehicle
int lockout = 0;                           // Lockout flag for braking system
int rearWipe1 = 0;                         // Control flag for rear windshield wiper
int regenLevel = 2;                        // Default regeneration level for coasting
int regenOnSig = 1;                        // Signal for enabling regeneration only when gas pedal is off
int regenChange = 0;                       // Flag for indicating a change in regeneration level
unsigned long regenChangeTimer = 0;        // Timer for tracking regeneration level changes
int disengageRegen = 0;                    // Flag for disengaging regeneration

// Variables for cruise control
double Setpoint, Input, Output;            // PID controller inputs and output for cruise control
double Kp = 40, Ki = 0.8, Kd = 0;          // PID controller constants for cruise control
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // PID controller instance for cruise control
float cruiseTorqueCmd = 0;                 // Torque command for cruise control
int cruise1 = 0;                           // Cruise control flag 1
int cruise2 = 0;                           // Cruise control flag 2
int cruiseSet = 0;                         // Cruise control set flag
int cruiseMainOn = 0;                      // Main cruise control on flag
int cruiseOn = 0;                          // Cruise control on flag
int resOn = 0;                             // Resume cruise control flag
int antiloop1 = 0;                         // Anti-loop flag for cruise control

int genericTimer1 = 0;                     // Generic timer 1 for miscellaneous use
int genericTimer2 = 0;                     // Generic timer 2 for miscellaneous use

// Variables for vehicle security
int tagsecurity = 0;                       // Security flag for tag-based authentication
int auxsecurity = 0;                       // Auxiliary security flag
int authcounter = 0;                       // Counter for authentication attempts
int vehicleOnWithManAuth = 0;              // Flag for manual authentication override

// Variables for Battery Management System (BMS)
int DTC1 = 0;                              // Diagnostic Trouble Code 1
int DTC2 = 0;                              // Diagnostic Trouble Code 2
int DTC3 = 0;                              // Diagnostic Trouble Code 3
int DTC4 = 0;                              // Diagnostic Trouble Code 4
uint16_t DCL = 0;                          // Discharge Current Limit
uint16_t CCL = 0;                          // Charge Current Limit
int SOC = 0;                               // State Of Charge
int highTemp = 0;                          // Highest temperature reading
int lowTemp = 0;                           // Lowest temperature reading
int AAT = 0;                               // Ambient Air Temperature
int averageTemp = 0;                       // Average temperature

// Variables for dashboard and warning indicators
int batteryLight = 0;                      // Battery light indicator flag
int brakeWarning = 0;                      // Brake warning light flag
int absWarning = 0;                        // ABS warning light flag
int pkbWarning = 0;                        // Parking brake warning light flag
int bsmWarn = 0;                           // Blind Spot Monitor warning flag
unsigned long dashTimer1 = 0;              // Dashboard timer for updates
uint16_t invCurr = 0;                      // Inverter current
uint16_t invVol = 0;                       // Inverter voltage
int dashCounter = 0;                       // Dashboard update counter
int invCurrSend = 0;                       // Flag for sending inverter current data
float invCurrfloat = 0;                    // Floating-point representation of inverter current
int seatBeltWarn = 0;                      // Seatbelt warning flag
int previousLeftBlink = 0;                 // Previous state of left blinker
int previousRightBlink = 0;                // Previous state of right blinker
int motTemp = 0;                           // Motor temperature
int invTemp = 0;                           // Inverter temperature
int moduleVoltage = 0;                     // Module voltage
int chargeOn = 0;                          // Charging state flag



MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's
MCP_CAN CAN2(spiCSPinFCAN2);  //start talking on FCAN2 which is motor 2
MCP_CAN CAN3(spiCSPinBCAN);  //start talking on BCAN which is HVAC


void setup() {
  // Initial pin setup for the ECU power control
  pinMode(r7, OUTPUT);             // Set r7 pin as OUTPUT for ECU control
  digitalWrite(r7, LOW);            // Initially set r7 LOW
  digitalWrite(r7, HIGH);           // Then set r7 HIGH to turn on the ECU

  // Setup pin modes for various vehicle components
  pinMode(accPow, OUTPUT);          // Accessory Power
  pinMode(illumi, OUTPUT);          // Illumination (e.g., dashboard lights)
  pinMode(motPow, OUTPUT);          // Motor Power
  pinMode(contactorPos, OUTPUT);    // Positive Contactor for the battery
  pinMode(contactorNeg, OUTPUT);    // Negative Contactor for the battery
  pinMode(precharge, OUTPUT);       // Precharge circuit for electric motor
  pinMode(dashPow, OUTPUT);         // Dashboard Power
  pinMode(revSig, OUTPUT);          // Reverse Signal

  // Initialize all the pins to LOW (turn off the components)
  digitalWrite(accPow, LOW);
  digitalWrite(illumi, LOW);
  digitalWrite(motPow, LOW);
  digitalWrite(contactorPos, LOW);
  digitalWrite(contactorNeg, LOW);
  digitalWrite(precharge, LOW);
  digitalWrite(dashPow, LOW);
  digitalWrite(revSig, LOW);

  // Serial communication setup
  Serial.begin(9600);               // Start serial communication with PC at 9600 baud rate
  Serial3.begin(RDM6300_BAUDRATE);  // Start serial communication with RFID receiver
  Serial2.begin(115200);            // Start serial communication on Serial2 at 115200 baud rate
  rdm6300.begin(&Serial3);          // Initialize RFID reader

  // Initialize CAN buses with specific speeds
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {
    Serial.println("FCAN BUS init Failed");
    delay(100);
  }
  Serial.println("FCAN BUS Shield Init OK!");

  while (CAN_OK != CAN2.begin(CAN_500KBPS, MCP_8MHz)) {
    Serial.println("FCAN2 BUS init Failed");
    delay(100);
  }
  Serial.println("FCAN2 BUS Shield Init OK!");

  while (CAN_OK != CAN3.begin(CAN_125KBPS, MCP_8MHz)) {
    Serial.println("BCAN BUS init Failed");
    delay(100);
  }
  Serial.println("BCAN BUS Shield Init OK!");

  // Set the PID controller to automatic mode
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Read brake sensor value
  br = analogRead(brake);

  // Timing control for power management
  if (millis() - genericTimer1 > 20) {
    power();                         // Call the power management function
    genericTimer1 = millis();        // Reset the timer
  }

  // Timing control for light management
  if (millis() - genericTimer2 > 50) {
    lights();                        // Call the lights management function
    genericTimer2 = millis();        // Reset the timer
  }

  // If the ignition is on, handle steering controls
  if (igState > 0) {
    steering();
  }

  // Climate control handling
  hvac();

  // Dashboard update control
  if (millis() - dashTimer1 > 150) {
    dash();                          // Call the dashboard update function
    dashTimer1 = millis();           // Reset the timer
  }

  // Motor and precharge control handling
  motah();
  preChargeLoop();

  // Security checks
  if (igState == 0) {
    security1();                     // Call the security function if ignition is off
  }

  // CAN message handling
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);       // Read the CAN message
    canId = CAN.getCanId();          // Get the CAN ID
  }

  // Check if CAN message ID is 0x199, which is related to gear position
if (canId == 0x199) {
  rawGear = buf[0]; // Store the first byte of the CAN message as the raw gear value
  gears();          // Call the gears() function to handle the gear change logic
}

// Check if CAN message ID is 0x103, which is related to blinkers, headlamp, and BSM warning
if (canId == 0x103) {
  leftBlink = buf[0];      // Store the first byte as left blinker status
  rightBlink = buf[1];     // Store the second byte as right blinker status
  headlampOn = buf[3];     // Store the fourth byte as headlamp status
  highbeamOn = buf[4];     // Store the fifth byte as high beam status
  parkLampsOn = buf[2];    // Store the third byte as parking lamps status
  bsmWarn = buf[5];        // Store the sixth byte as Blind Spot Monitoring (BSM) warning status
}

// Check if right blinker status has changed
if (rightBlink != previousRightBlink) {
  previousRightBlink = rightBlink; // Update the previous right blinker status
  dash();                          // Call the dash() function to update the dashboard display
}

// Check if left blinker status has changed
if (leftBlink != previousLeftBlink) {
  previousLeftBlink = leftBlink;   // Update the previous left blinker status
  dash();                          // Call the dash() function to update the dashboard display
}

// Check if CAN message ID is 0x105, which is related to motor speed and direction
if (canId == 0x105) {
  motorSpeed = (buf[0] << 8) | buf[1]; // Combine the first two bytes to form the motor speed
  motorSpeed = motorSpeed * 0.25;      // Adjust the motor speed by a scaling factor
  // Calculate vehicle speed (vss) based on motor speed and various ratios and conversions
  vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828);

  motorDat1 = buf[5]; // Store the sixth byte as motor data
  // Decode motor rotation direction from motorDat1
  if ((motorDat1 & 0b10000000)) {    // Check the first bit for rotation direction
    motorRotation = 2;
  }
  if ((motorDat1 & 0b01000000)) {    // Check the second bit for rotation direction
    motorRotation = 1;
  }
  if ((motorDat1 & 0b01000000) && (motorDat1 & 0b10000000)) {  // Check for error in rotation direction
    motorRotation = 3;
  }
  motorCounter++; // Increment the motor counter for each message received
}

// Logic to handle motor speed analysis every 10 messages
if (motorCounter == 10) {
  motorCounter = 0; // Reset the motor counter
  if (vss > 15 && previousValue != 0.0) { // Check if vehicle speed is above 15 and there is a previous value
    // Calculate the percentage decrease in speed
    decreasePercentage = ((previousValue - motorSpeed) / previousValue) * 100;
    // Check if the decrease percentage exceeds 12%, indicating potential slip
    if (decreasePercentage > 12) {
      lockup = 1; // Set lockup flag
    }
  }
  previousValue = motorSpeed; // Update the previous value with current motor speed
}

// Check if CAN message ID is 0xE7, related to brake position
if (canId == 0xE7) {
  brakePos = (buf[0] << 8) | buf[1]; // Combine the first two bytes to form the brake position
  brakePosInt = (brakePos * .01) - 53; // Convert brake position to an integer value
}


  // Handling cruise control commands based on CAN message with ID 0x9D
if (canId == 0x9D) {
    cruise1 = buf[0]; // First byte of message contains cruise control commands

    // Decrement speed set point
    if ((cruise1 & 0b00100000)) {
        if (cruiseSet == 0) { // Check if cruise control is not already set
            cruiseSet = 1;   // Set cruise control
        } else {
            if (antiloop1 == 0) { // Prevent continuous decrement until button is released
                Setpoint--;       // Decrement the set point for cruise control speed
                antiloop1 = 1;    // Lock out further decrements
            }
        }
    } else {
        antiloop1 = 0; // Reset lockout on button release to allow new decrement
    }

    // Disable cruise control
    if ((cruise1 & 0b00000001)) {
        cruiseMainOn = 0; // Turn off the main cruise control flag
    }

    // Increment speed set point
    if ((cruise1 & 0b00010000)) {
        if (resOn != 2) { // Check if resume is not already processed
            resOn = 1;    // Set resume on
        }
    } else {
        if (resOn == 2) { // Reset resume on button release
            resOn = 0;
        }
    }

    // Enable cruise control and set current speed as Setpoint
    cruise2 = buf[1]; // Second byte for additional cruise control commands
    if ((cruise2 & 0b01000000)) {
        cruiseMainOn = 1;       // Activate main cruise control
        Setpoint = vss;         // Use current vehicle speed sensor (VSS) reading as the new set point
    }
}

// Handling regenerative braking level adjustment commands based on CAN message with ID 0x91
if (canId == 0x91) {
    rearWipe1 = buf[2]; // Third byte contains regen control commands

    // Adjust regen level
    if ((rearWipe1 & 0b00000001) && regenChange == 0) {
        if (rearWipe1 & 0b00000100) {
            regenLevel++; // Increase regen level
        } else {
            regenLevel--; // Decrease regen level
        }
        regenLevel = constrain(regenLevel, 0, 30); // Keep regen level within bounds (0-30)
        regenChange = 1;                           // Mark that a change occurred
        regenChangeTimer = millis();               // Start timer to limit rate of change
    }

    // Toggle regen on signal based on command
    if (rearWipe1 & 0b00000100) {
        if (rearWipe1 & 0b00001000) {
            regenOnSig = 1; // Set to no brake regen
        } else {
            regenOnSig = 2; // Set to full regen
        }
    } else {
        regenOnSig = 0; // Turn all regen off
    }

    // Reset regen change flag after 200ms to prevent rapid changes
    if (millis() - regenChangeTimer > 200) {
        regenChange = 0;
    }
}

// Processing BMS data from CAN message with ID 0x6B0
if (canId == 0x6B0) {
    DTC1 = buf[0]; // Diagnostic Trouble Code 1
    DTC2 = buf[1]; // Diagnostic Trouble Code 2
    DCL = (buf[2] << 8) | buf[3]; // Discharge Current Limit
    CCL = (buf[4] << 8) | buf[5]; // Charge Current Limit
    SOC = buf[6]; // State Of Charge, multiply by 0.5 for real value
}

// Additional BMS data from CAN message with ID 0x6B1
if (canId == 0x6B1) {
    DTC3 = buf[4]; // Diagnostic Trouble Code 3
    DTC4 = buf[5]; // Diagnostic Trouble Code 4
    highTemp = buf[0]; // High temperature, subtract 40 for real value
    lowTemp = buf[1]; // Low temperature, subtract 40 for real value
    AAT = buf[2]; // Ambient Air Temperature, subtract 40 for real value
    averageTemp = buf[3]; // Average temperature, subtract 40 for real value
    chargeOn = buf[6]; // Charging state flag
}


 // Handling motor 1 data from CAN message with ID 0x107
if (canId == 0x107) {  
  // Calculate inverter current by combining two bytes and potentially doubling it if assuming 2 motors
  invCurr = (buf[2] << 8) | buf[3];
  // invCurr = invCurr * 2; // Uncomment to double the value if using 2 motors
  // Calculate inverter voltage by combining two bytes
  invVol = (buf[0] << 8) | buf[1];
  // Debug print inverter voltage
  // Serial.println(invVol);
}

// Handling temperature data from CAN message with ID 0x106
if (canId == 0x106) {
  motTemp = buf[0]; // Motor temperature
  invTemp = buf[1]; // Inverter temperature
}

// Handling warning signals from CAN message with ID 0x108
if (canId == 0x108) {
  absWarning = buf[0];   // ABS warning signal
  brakeWarning = buf[1]; // Brake system warning signal
}

// Sending a CAN message on ID 0x102 every 50 milliseconds
if (millis() - previousCanSend > 50) {
  // Prepare the message payload with vehicle control statuses
  unsigned char stmp[8] = {acReq, igState, brakeCommand, hazCommand, gear, hornCommand, fanByte, heatReq};
  // Send the CAN message
  CAN.sendMsgBuf(0x102, 0, 8, stmp);
  // Update the timestamp for the last sent CAN message
  previousCanSend = millis();
}

// Function to determine gear position based on rawGear value
void gears() {
  // Decode the rawGear value to set the gear variable
  if (rawGear == 0xC0) gear = 0; // Park
  if (rawGear == 0x20) gear = 1; // Reverse
  if (rawGear == 0x10) gear = 2; // Neutral
  if (rawGear == 0x08) gear = 3; // Drive

  // If the ignition state is off, force gear to Park
  if (igState <= 1) gear = 0;

  // Set the reverse signal and gear lever position based on the gear
  if (gear == 1) {
    digitalWrite(revSig, HIGH); // Activate reverse signal
    gearLvrPos = 1;             // Set gear lever position for reverse
  } else {
    digitalWrite(revSig, LOW);  // Deactivate reverse signal
    gearLvrPos = 3;             // Set gear lever position for other gears
  }
}

// Function to control vehicle lights based on sensor readings and status variables
void lights() {
  // Control brake light based on brake sensor reading
  if (br < 200) brakeCommand = 1; // Brake applied
  else brakeCommand = 0;          // Brake not applied

  // Control hazard lights based on hazard switch reading
  if (analogRead(hazRet) < 200) hazCommand = 1; // Hazard switch activated
  else hazCommand = 0;                          // Hazard switch not activated

  // Control horn based on horn button reading
  if (analogRead(horn) < 200) hornCommand = 1; // Horn button pressed
  else hornCommand = 0;                        // Horn button not pressed

  // Control illumination (dashboard and possibly other lights) based on park lamps status
  if (parkLampsOn == 1) digitalWrite(illumi, HIGH); // Turn on illumination
  else digitalWrite(illumi, LOW);                  // Turn off illumination
}


void hvac() {
  // Initialize variables to store CAN message details
  unsigned char len = 0; // Length of the CAN message
  unsigned char buf[8]; // Buffer to store CAN message data
  unsigned long canId = 0; // CAN message ID

  // Check if there is a message available on CAN bus 3
  if (CAN_MSGAVAIL == CAN3.checkReceive()) {
    // Read the message into the buffer
    CAN3.readMsgBuf(&len, buf);
    // Get the message ID
    canId = CAN3.getCanId();
    // Debug print the CAN message ID
    // Serial.println(canId);
  }

  // Handle specific HVAC-related CAN message identified by its unique ID
  if (canId == 251231057) {  // Convert hex ID 0xEF97B51 to decimal
    // Extract HVAC control bytes from the CAN message
    acByte = buf[4]; // AC control byte
    fanByte = buf[0]; // Fan speed control byte
    tempByte = buf[1]; // Temperature control byte

    // AC control logic based on the acByte value
    if (acByte & 0b00010000) {
      acReq = 0; // AC request off
    } else {
      acReq = 1; // AC request on
    }

    // Ensure AC is off when fan is turned off
    if (fanByte == 0x0F) {
      acReq = 0; // Override AC request to off
    }

    // Heating control logic based on the tempByte value
    if (tempByte == 0x04) {
      heatReq = 1; // Heating request on
    } else {
      heatReq = 0; // Heating request off
    }
  }
}



void steering() {
  // Steering control logic based on ignition state
  if (igState < 3) { // If engine is not running, turn power steering off
    stmp202[0] = 0x00; // Control byte for steering off
  }

  if (igState == 3) { // If engine is running, turn power steering on
    stmp202[0] = 0x30; // Control byte for steering on
  }

  // Send steering control message periodically
  if ((millis() - lastSend > 20)) { // Every 20 milliseconds
    // Convert vehicle speed sensor (VSS) reading to raw value for CAN message
    vssRaw = static_cast<uint16_t>(vss * 100); // Scale VSS and convert to kph

    // Encode VSS raw value into CAN message bytes assuming big-endian encoding
    stmp202[2] = (vssRaw >> 8) & 0xFF;  // Most significant byte of VSS
    stmp202[3] = vssRaw & 0xFF;         // Least significant byte of VSS

    // Send CAN messages with updated steering and vehicle speed information
    CAN.sendMsgBuf(0x202, 0, 8, stmp202); // Message for steering control
    CAN.sendMsgBuf(0x217, 0, 8, stmp2170); // Additional message, possibly related to steering or speed

    // Update counters for message consistency or diagnostics
    lastSend = millis(); // Update timestamp for last message sent
    // Manipulate diagnostic or counter bytes in the message
    i = stmp2170[6];
    i = i + 1;
    j = stmp2170[7];
    j = j - 1;

    // Reset counters if a specific condition is met (e.g., overflow)
    if (stmp2170[6] == 223) {
      stmp2170[6] = 0xD0;
      stmp2170[7] = 0xFC;
      i = stmp2170[6];
      j = stmp2170[7];
    }

    // Update message with new counter values
    stmp2170[6] = i;
    stmp2170[7] = j;
  }
}





void power() {
  // Read ignition switch states
  igSwitch1 = analogRead(igSw1);
  igSwitch2 = analogRead(igSw2);

  // Logic to turn on the ignition if key is not being used and security check passes
  if ((igSwitch1 < 200 || igSwitch2 < 200) && pushCount == 0 && ignite == 0 && security == 1) {
    // Detect potential ignition switch failure
    if ((igSwitch1 < 200 && igSwitch2 > 200) || (igSwitch1 > 200 && igSwitch2 < 200)) {
      Serial.println("Possible IG switch failure, invalid key switch signal detected");
    }

    // Set ignition on and prevent IECU shutdown
    ignite = 1; // Indicate ignition is on
    IECUshutdown = 0; // Prevent board from shutting off with car on
    pushCount = 1;
    genericFlag1 = 0; // Prevent other modules from sleeping after key cycle
    pushTimer = millis(); // Update timer for push count logic

    // Activate accessory power and ensure ECU is awake
    digitalWrite(accPow, HIGH); // Accessory power on
    digitalWrite(r7, HIGH); // ECU in "awake" mode
    igState = 1; // Update ignition state
    Serial.println("ig1");
  }
}


  // Initial check for manual start condition
if ((igSwitch2 > 200 && igSwitch1 > 200) && pushCount == 1) {
  // Button was not held down, indicating no manual start; prepare for next state
  pushCount = 3;
}

// Check for ignition stage 2 (ig2) with brake condition
if ((igSwitch2 < 200 || igSwitch1 < 200) && pushCount == 3 && br > 300) {
  // Ignition stage 2 activated with brake applied
  ignite = 1; // Ignition on
  // Powering on motor, dashboard, and accessory power
  digitalWrite(motPow, HIGH);
  digitalWrite(dashPow, HIGH);
  digitalWrite(accPow, HIGH);
  pushCount = 5; // Update pushCount to indicate ig2 running
  igState = 2; // Update ignition state to 2
}

// Prepare for shutdown after cranking stops and button is released
if ((igSwitch2 > 200 && igSwitch1 > 200) && pushCount == 5) {
  pushCount = 9; // Update pushCount to indicate preparation for shutdown
  Serial.println("why"); // Debug print
}

// Auto-start or ig2 system state handling
if (((igSwitch2 > 200 && igSwitch1 > 200) && pushCount == 3 && br < 200) ||
    ((igSwitch2 < 200 || igSwitch1 < 200) && pushCount == 9 && br < 200)) {
  // Condition for auto-start on
  ignite = 1; // Ignition on
  // Powering on motor, dashboard, and accessory power
  digitalWrite(motPow, HIGH);
  digitalWrite(dashPow, HIGH);
  digitalWrite(accPow, HIGH);
  pushCount = 6; // Update pushCount to indicate vehicle running
  igState = 3; // Update ignition state to 3 (running)
}

// Prepare for shutdown after vehicle is running and button is released
if ((igSwitch2 > 200 && igSwitch1 > 200) && pushCount == 6) {
  pushCount = 7; // Update pushCount for shutdown preparation
}

// Handling ignition off state
if ((igSwitch2 < 200 || igSwitch1 < 200) && pushCount == 7) {
  pushCount = 8; // Update pushCount to indicate ignition off process
  // Powering off motor, dashboard, and accessory power
  digitalWrite(motPow, LOW);
  digitalWrite(accPow, LOW);
  digitalWrite(dashPow, LOW);
  ignite = 0; // Ignition off
  igState = 0; // Reset ignition state
  // Reset security parameters if vehicle was started manually
  if (vehicleOnWithManAuth == 1) {
    tagsecurity = 0;
    auxsecurity = 0;
    security = 0;
    vehicleOnWithManAuth = 0;
  }
}

// Handling ignition stage 2 off condition
if ((igSwitch2 < 200 || igSwitch1 < 200) && pushCount == 9 && br > 300) {
  pushCount = 8; // Similar shutdown process as above
  // Additional logic identical to the previous block
}

// Reset pushCount after shutdown preparation
if ((igSwitch2 > 200 && igSwitch1 > 200) && pushCount == 8) {
  pushCount = 0; // Reset pushCount, ready for new ignition cycle
}

// Additional logic to handle automatic shutdown of the LCM (Light Control Module) after a timeout
if (ignite == 0) {
  if (genericFlag1 == 0) { // If shutdown process starts
    IECUtimer = millis(); // Start shutdown timer
    genericFlag1 = 1; // Indicate that shutdown timer is active
  }
  // Shutdown LCMs after 5 minutes if vehicle is off and not charging
  if (genericFlag1 == 1 && millis() - IECUtimer > 300000 && holdECUOn == 0 && chargeOn == 0x00) {
    digitalWrite(r7, LOW); // Power off the control module
    IECUshutdown = 1; // Indicate IECU shutdown
  } else {
    digitalWrite(r7, HIGH); // Otherwise, keep the module powered
  }
}
}


void preChargeLoop() {
  // Initiate the precharge process when the engine (or equivalent) is running and precharge hasn't started yet
  if (igState == 3 && preChargeStart == 0) { // Engine running check
    contactorTimer = millis(); // Record the start time of the precharge process
    preChargeStart = 1; // Update the precharge process state to indicate it has started
  }

  // Activate the negative contactor after 500ms of precharge initiation
  if (millis() - contactorTimer > 500 && preChargeStart == 1) {
    digitalWrite(contactorNeg, HIGH); // Turn on the negative contactor
    preChargeStart = 2; // Move to the next state of the precharge process
  }

  // Activate the precharge circuit after 1000ms from the start of the precharge process
  if (millis() - contactorTimer > 1000 && preChargeStart == 2) {
    digitalWrite(precharge, HIGH); // Engage the precharge circuit to start charging the capacitors
    preChargeStart = 3; // Progress to the next precharge state
  }

  // Activate the positive contactor after 4000ms from the start of the precharge process
  if (millis() - contactorTimer > 4000 && preChargeStart == 3) {
    digitalWrite(contactorPos, HIGH); // Turn on the positive contactor
    preChargeStart = 4; // Advance to the final precharge state
  }

  // Complete the precharge process and disengage the precharge circuit after 5000ms
  if (millis() - contactorTimer > 5000 && preChargeStart == 4) {
    digitalWrite(precharge, LOW); // Turn off the precharge circuit
    preChargeStart = 5; // Indicate the precharge process is complete
    prechrgFinished = 1; // Flag that precharging has finished successfully
  }

  // Initiate the discharge process when the engine is not running, precharge is complete or in progress, and the motor is not spinning
  if (igState != 3 && preChargeStart <= 5 && preChargeStart > 0 && motorSpeed == 0) {
    contactorTimer = millis(); // Record the start time of the discharge process
    preChargeStart = 6; // Update the process state for discharging
    prechrgFinished = 0; // Reset the precharge finished flag
  }

  // Begin by turning off the positive contactor and precharge circuit after 4000ms of discharge initiation
  if (millis() - contactorTimer > 4000 && preChargeStart == 6) {
    digitalWrite(contactorPos, LOW); // Turn off the positive contactor
    digitalWrite(precharge, LOW); // Disengage the precharge circuit
    preChargeStart = 7; // Proceed to the next discharge state
  }

  // Turn off the negative contactor after 5500ms from the start of the discharge process
  if (millis() - contactorTimer > 5500 && preChargeStart == 7) {
    digitalWrite(contactorNeg, LOW); // Deactivate the negative contactor
    preChargeStart = 8; // Move to the final discharge state
  }

  // Reset the precharge process state after 6500ms, allowing another precharge cycle to begin
  if (millis() - contactorTimer > 6500 && preChargeStart == 8) {
    preChargeStart = 0; // Reset the precharge process state for future cycles
  }
}


// Function declaration for 'motah'.
void motah() {
  
  // Checks if the vehicle's ignition state is on, precharge is finished, and the contactor timer has exceeded 6 seconds.
  if (igState == 3 && prechrgFinished == 1  && millis() - contactorTimer > 6000) {  
    // Sets the vehicle state to 'ready' and key state to 'on'.
    vehicleState = 1;
    keyState = 2;

    // Checks if the vehicle is in a gear that allows it to move.
    if (gear == 1 || gear == 3) {  
      // Calls the pedals() function to potentially engage throttle, assuming certain conditions (like cruise control being off) are met.
      pedals();
    } else {
      // If the vehicle is not in a gear that allows movement, sets acceleration and regeneration percentages to 0.
      accelPct = 0;
      regenPct = 0;
      accelPct2 = 0;
      regenPct = 0;
    }
  } else {
    // If the initial conditions are not met, reset vehicle state, acceleration, and regeneration percentages to their default values.
    vehicleState = 0;
    accelPct = 0; // If key not on, throttle = 0
    regenPct = 0;
    accelPct2 = 0;
    throttleError = 0; // Reset throttle error
    keyState = 0;
  }

  // Checks if the time since the last CAN message regarding the motor is more than 18 milliseconds and the vehicle state is 'ready'.
  if (millis() - previousCanMotor1 > 18 && vehicleState == 1) {
    // Prepares a CAN message indicating that precharging is finished and the precharge relay is open.
    unsigned char stmp[8] = {0, 0, 0, 0x14, 0, 0, 0, 0}; 
    CAN.sendMsgBuf(0x1A0, 0, 8, stmp);
    CAN2.sendMsgBuf(0x1A0, 0, 8, stmp);

    // Calculates raw values for maximum discharge and charge.
    maxDiscRaw = (maxDisc + 500) * 10;
    maxChrgRaw = (maxChrg + 500) * 10;

    // Prepares a CAN message with maximum discharge and charge values.
    unsigned char stmp1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    stmp1[0] = static_cast<char>((maxDiscRaw >> 8) & 0xFF);
    stmp1[1] = static_cast<char>(maxDiscRaw & 0xFF);
    stmp1[2] = static_cast<char>((maxChrgRaw >> 8) & 0xFF);
    stmp1[3] = static_cast<char>(maxChrgRaw & 0xFF);
    CAN.sendMsgBuf(0x1A1, 0, 8, stmp1);
    CAN2.sendMsgBuf(0x1A1, 0, 8, stmp1);

    // Prepares a CAN message for auxiliary relay commands with gear lever position, vehicle state, and other parameters.
    unsigned char stmp2[8] = {0, 0, 0, 0, 0, 0, 0x10, 0};
    stmp2[4] |= static_cast<char>((gearLvrPos & 0x07) << 4);
    stmp2[4] |= static_cast<char>((vehicleState & 0x01) << 0);
    stmp2[4] |= static_cast<char>((mainRelayCmd & 0x01) << 3);
    stmp2[5] |= static_cast<char>((keyState & 0x03) << 6);
    stmp2[5] |= static_cast<char>((motorMode & 0x03) << 2);
    stmp2[5] |= static_cast<char>((workMode & 0x01) << 1);
    stmp2[5] |= static_cast<char>((acCommand & 0x01) << 0);

    // Calculates raw acceleration value based on pedal percentage and motor mode.
    if (motorMode == 1) {
      accelRaw = round(accelPct / 0.392);
    }
    if (motorMode == 2) {
      accelRaw = round(regenPct / 0.392);
    }
    stmp2[0] = static_cast<char>(accelRaw & 0xFF);

    // Manages a rolling counter for message sequencing and calculates a checksum for message integrity.
    rollingCounter++;
    if (rollingCounter == 16) {
      rollingCounter = 0;
    }
    stmp2[6] |= static_cast<char>((rollingCounter & 0x0F) << 0);
    char sum = stmp2[0] + stmp2[1] + stmp2[2] + stmp2[3] + stmp2[4] + stmp2[5] + stmp2[6];
    char checksum = sum ^ 0xFF;
    stmp2[7] = checksum;

    // Sends the prepared CAN messages to both CAN networks.
    CAN.sendMsgBuf(0x101, 0, 8, stmp2);
    CAN2.sendMsgBuf(0x101, 0, 8, stmp2);

    // Updates the time of the last CAN message regarding the motor.
    previousCanMotor1 = millis();
  }
}


void pedals() {
  //.println(analogRead(app1));  //215 - 975
  //Serial.println(analogRead(app2));  //100-485

  if (millis() - passedtime3 > 12 && throttleError == 0) {  //accel pedal delay (12 default)
    gasoline = map(analogRead(app1), 260, 950, 0, 100);
    gasoline = constrain(gasoline, 0, 100);

    gasoline2 = map(analogRead(app2), 135, 475, 0, 100);
    gasoline2 = constrain(gasoline2, 0, 100);



    total4 = total4 - readings4[readIndex4];
    // read from the sensor:
    readings4[readIndex4] = gasoline;
    // add the reading to the total:
    total4 = total4 + readings4[readIndex4];
    // advance to the next position in the array:
    readIndex4 = readIndex4 + 1;

    // if we're at the end of the array...
    if (readIndex4 >= numReadings4) {
      // ...wrap around to the beginning:
      readIndex4 = 0;
    }

    // calculate the average:
    accelPct = total4 / numReadings4;
    //Serial.println(averagefuel);
    // send it to the computer as ASCII digits



    //APP2
    total5 = total5 - readings5[readIndex5];
    // read from the sensor:
    readings5[readIndex5] = gasoline2;
    // add the reading to the total:
    total5 = total5 + readings5[readIndex5];
    // advance to the next position in the array:
    readIndex5 = readIndex5 + 1;

    // if we're at the end of the array...
    if (readIndex5 >= numReadings5) {
      // ...wrap around to the beginning:
      readIndex5 = 0;
    }

    // calculate the average:
    accelPct2 = total5 / numReadings4;


    //ERRORS
    if (abs(accelPct2 - accelPct) > 10) {
      Serial.println("throttle diff error");
    }

    if (abs(accelPct2 - accelPct) > 15) {
      // accelPct = 0; //disable Throttle
      //  accelPct2 = 0;
      //  throttleError = 1;  //TESTING ONLY
    }

    //Serial.println(accelPct);
    accelPct = constrain(accelPct, 0, 95);

    passedtime3 = millis();
    // Serial.println(abs(accelPct2 - accelPct));

    if (accelPct > 25) {
      if (outputValue < 25) {
        outputValue = 25;
        //Serial.println("going");
      }
      //Serial.println(outputValue);
      //Serial.println(accelPct);
      if (outputValue < accelPct) {
        //Serial.println("going");
        outputValue += 0.75; //ammount of jump per loop
        //Serial.println(outputValue);

        if (outputValue > accelPct) {
          outputValue = accelPct;
          //  Serial.println("going2");
        }

      } else if (outputValue > accelPct) {

        outputValue = accelPct;
        //  Serial.println("going3");

      }


      accelPct = outputValue;
    } else {
      outputValue = accelPct;
    }

    accelPct = outputValue;

  }
  accelPct = constrain(accelPct, 0, 95);


  //REGEN
  if (accelPct == 0 && regenOnSig >= 1) {  //OFF throttle regen going Forward/Reverse when requested by driver

    if (lockout == 0) {
      motorMode = 2;
    }

    if (motorSpeed > 110 && ((motorRotation == 1 && gear == 3) || (motorRotation == 2 && gear == 1)) && disengageRegen == 0) {  //if we are above 100rpms and motor is rotating same direction as requested gear, then do regen
      regenPct = regenLevel;  //goes to a percent regen when off gas
    } else if (motorSpeed > 35 && ((motorRotation == 1 && gear == 3) || (motorRotation == 2 && gear == 1)) && disengageRegen == 0) {
      //regenPct = round(regenLevel * 0.5); //low regen to stop
      regenPct = round(regenLevel * ((motorSpeed - 35) / 75.0f)); //low regen at stop
      regenPct = constrain(regenPct, 0, 25); //saftey command incase fault
    } else {
      regenPct = 0;
      disengageRegen = 1;
    }

  } else {
    motorMode = 1;  //this forces system to read accel pedal if on gas
    regenPct = 0;  //sets regen to 0 if on pedal
    // disengageRegen = 0;  //go back to regen after we have once again hit the gas
  }

  if (br > 400) {
    disengageRegen = 0; //go back to regen after we have once take foot off of brake
    lockup = 0;
  }

  //Serial.println(brakePosInt);
  if (brakePosInt > 10 && br < 300 && regenOnSig == 2  && lockup == 0) {  //progressive regen with brake pedal, will cause motor not to function if EBB goes offline during brake push, ADD something here to also use brake sw
    //Serial.println("regen active");
    stopCommand = map(brakePosInt, 20, 70, 5, 25); //20 mm to 150 mm of travel goes to 5-25 pct regen command
    stopCommand = constrain(stopCommand, 0, 25);
    //Serial.println("error");

    if (motorSpeed > 110 && ((motorRotation == 1 && gear == 3) || (motorRotation == 2 && gear == 1)) && disengageRegen == 0) {  //if we are above 100rpms and motor is rotating same direction as requested gear, then do regen
      regenPct = stopCommand;  //overwrite off throttle regen command if brake pedal pressed
    } else if (motorSpeed > 35 && ((motorRotation == 1 && gear == 3) || (motorRotation == 2 && gear == 1)) && disengageRegen == 0) {
      //regenPct = round(stopCommand * 0.5); //low regen at stop
      regenPct = round(stopCommand * ((motorSpeed - 35) / 75.0f)); //low regen at stop
      regenPct = constrain(regenPct, 0, 45); //saftey command incase fault

      //disengageRegen = 1; //coming to a stop disengage regen and prevent reactivation
    } else {
      regenPct = 0;
      disengageRegen = 1; //coming to a stop disengage regen and prevent reactivation
    }

    if (motorMode == 1) {  //if we are on the gas and on the brake...
      lockout = 1;  //prevent off throttle regen from trying to kick in because we overwrote accelpct to be 0
      accelPct = 0;
      accelPct2 = 0;  //dont accelerate, we will just coast by ignoring regen (because motorMode = 1) and overwrite accel to be 0
      //Serial.println("error2");
    }


  } else {
    lockout = 0;  //listen to throttle again after take foot off of brake
  }
  //Serial.println(regenPct);


  cruise();


}



void cruise() {

  if (cruiseMainOn == 1) {  //cruise on, start calculating
    Input = vss;
    myPID.Compute();
    cruiseTorqueCmd = map(Output, 0, 255, 0, 35);  //limit to 30 pct throttle

  }

  //Serial.println("func");
  if (cruiseSet == 1 && br > 300 && cruiseMainOn == 1 && vss > 5) {  //if cruise set and off the brake pedal
    accelPct = max(accelPct, cruiseTorqueCmd);
    accelPct2 = max(accelPct2, cruiseTorqueCmd);
    //Serial.println(accelPct);
    // Serial.println("cruiseSet = 1");
    if (cruiseOn == 0) {
      Setpoint = vss;
      Serial.println("set");
    }

    cruiseOn = 1; // lockout pedal opperation and lockin set point
    motorMode = 1; //no regen, only foward, prevents the motor from getting stuck in regen if off gas when cruise set
  }


  if (resOn == 1 && cruiseMainOn == 1) { //if hit resume and cruiseMain is still on
    resOn = 2; //wait for next release before another loop is set

    if (cruiseSet == 0) {
      cruiseSet = 1; //"hit" the set button
      cruiseOn = 1;  //prevent new setpoint from getting set
      Serial.println("res");
    } else {
      Setpoint++; //if cruise already on and we hit resOn, raise set point by 1
      Serial.println("res+");
    }

  }


  if (br < 200 || cruiseMainOn == 0 || vss < 5) { //if step on brake, cruise off, most important thing/prioirty
    if (cruiseOn == 1) {
      accelPct = 0;  //set throttle to 0 if cruise was on
      accelPct2 = 0;
    }

    cruiseSet = 0;  //turn off cruise
    cruiseOn = 0;
    Setpoint = 0;

    // Serial.println("off");
  }




}


void security1() {

  if (rdm6300.get_new_tag_id()) {
    Serial.println("tag detected");
    //  Serial.println(rdm6300.get_tag_id());
    if (rdm6300.get_tag_id() == 8121238 || rdm6300.get_tag_id() == 9785893) {
      tagsecurity = 1;
    }
  }


  // aux auth instrucutions, the switches should be on, then just flip off in order of 1 then 2
  //if they were off, you will have to turn both on , then flip them off in order of 1 then 2
  if (analogRead(auxSw1) < 200  && authcounter == 0) { //if aux 1 on
    authcounter = 1;
  }
  if (analogRead(auxSw1) > 200 && authcounter == 1) { //if aux1 then switched off
    authcounter = 2;
  }
  if (analogRead(auxSw2) < 200 && authcounter == 2) { //if aux2 on
    authcounter = 3;
  }
  if (analogRead(auxSw2) > 200 && authcounter == 3) { //if aux2 then switched off
    authcounter = 0;
    auxsecurity = 1;
  }


  //keyfob auth
  if (analogRead(unlock) < 200) { //unlock signal sent, turn off secuirty
    security = 1;
    IECUtimer = millis(); //when unlock pulse recieved, reset the ECU timer so that it wont shutoff
  }
  if (analogRead(lock) < 200) { //lock signal sent, turn on secuirty
    security = 0;  //read this flag for key missing from vehicle message?
  }



  //when to force security to = 1;
  if (tagsecurity == 1 || auxsecurity == 1) { // if either manual auth method active, overwrite any keyless commands
    security = 1;
    vehicleOnWithManAuth = 1;

    //Serial.println(security);
  }

  //Serial.println(security);

}


void dash() {
  //all little endian stuff so ex. BMS DTC is bit 0
  byte buf[8] = {0};
  byte buf1[8] = {0};
  byte buf2[8] = {0};


  if (DTC1 != 0x00 || DTC2 != 0x00 || DTC3 != 0x00 || DTC4 != 0x00) { // actually the 1 in the PRNDL
    buf[0] |= 0b00000001;  //set BMS DTC Bit, GOT IT
    //Serial.println(buf[0]);
  }

  if (brakeWarning == 1) {
    buf[0] |= 0b00000010;  //set air in brake/pkb Bit, GOT IT
  }
  if (absWarning == 1) {
    buf[0] |= 0b00000100;  //set ABS DTC Bit, GOT IT
  }
  if (seatBeltWarn == 1) { // actully the oil light on test dash, GOT IT
    buf[0] |= 0b00001000;  // seatbelt
  }
  //Serial.println(seatBeltWarn);
  if (prechrgFinished == 0) { //driver door on test dash, GOT IT
    buf[0] |= 0b00010000;  //set wait to start Bit (reserved)
  }
  if (highbeamOn == 1) {
    buf[0] |= 0b00100000;  //set highbeam Bit, GOT IT
  }
  if (parkLampsOn == 1) {
    buf[0] |= 0b01000000;  //set parklamp Bit, GOT IT
  }
  if (bsmWarn == 1 || bsmWarn == 3 || bsmWarn == 2) {  //trunk ajar light on test dash, GOT IT
    buf[0] |= 0b10000000;  //set bsmWarn Bit  MAKE THIS CHECK ENGINE, move BSM to another one
  }
  // Serial.println(bsmWarn);


  if (leftBlink == 1) {
    buf[1] |= 0b00000001;  //set leftTurn Bit, GOT IT
  }
  if (rightBlink == 1) {
    buf[1] |= 0b00000010;  //set rightTurn Bit, GOT IT
  }
  if (igState >= 2) {  //nothing on test dash, GOT IT
    buf[1] |= 0b00000100;  //set ig2  (set when tablet gets power)
  }
  if (igState == 3) {  //park brake indicator test dash, GOT IT
    buf[1] |= 0b00001000;  //set ig on bit
  }
  if (gear == 0) {
    buf[1] |= 0b00010000;  //set park bit, GOT IT
    // Serial.println(gear);
  }
  if (gear == 1) {
    buf[1] |= 0b00100000;  //set reverse bit, GOT IT
  }
  if (gear == 2) {
    buf[1] |= 0b01000000;  //set neutral bit, GOT IT
  }
  if (gear == 3) {
    buf[1] |= 0b10000000;  //set drive bit, GOT IT
  }

  if (motorMode == 1) {
    invCurrfloat = invCurr * 0.01 * 2;  //scale apprpriately to 1amp, double for 2 motors
    invCurrSend = round(invCurrfloat + 1310.68);  //off set for discharging
  }
  if (motorMode == 2) {  //regen
    invCurrfloat = invCurr * 0.01 * 2;  //scale apprpriately to 1amp, double for 2 motors
    invCurrSend = round(1310.68 - invCurrfloat);  //off set for discharging
  }

  //Serial.println(buf[0], HEX);
  //fast stuff
  //memcpy(buf, &batteryLight, 2);
  memcpy(buf + 2, &CCL, 2);
  memcpy(buf + 4, &DCL, 2);
  memcpy(buf + 6, &invCurrSend, 2);

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);
  //Serial.println(static_cast<int>(vss));


  //may have to break this out sepreate from turn signals


  if (cruiseMainOn) {
    buf1[4] |= 0b00000001;  //set cruiseMainOn, Passenger Door, test dash
  }
  if (cruiseOn) {
    buf1[4] |= 0b00000010;  //set cruiseOn, Overdrive off light test dash
  }
  if (bsmWarn & 0b00000001) {
    buf1[4] |= 0b00000100;  //left BSM
  }
  if (bsmWarn & 0b00000010) {
    buf1[4] |= 0b00001000;  //right BSM
  }
  if (bsmWarn & 0b10000000) {
    buf1[4] |= 0b00010000;  //BSM CHIME
  }

  vssDash = static_cast<int>(vss * 100);
  memcpy(buf1, &invVol, 2);
  memcpy(buf1 + 2, &vssDash, 2);
  memcpy(buf1 + 5, &moduleVoltage, 1);
  memcpy(buf1 + 6, &regenLevel, 1);
  SendCANFrameToSerial(3201, buf1);




  //SOC = 50;
  // Checks if the dashCounter has reached or exceeded 4 counts, which could correspond to a 600ms interval assuming dashCounter is incremented periodically (e.g., every 150ms).
if (dashCounter >= 4) { // 600 ms interval
    // Copies the State Of Charge (SOC) value into the first 2 bytes of the buffer (buf2).
    memcpy(buf2, &SOC, 2);
    // Copies the highest temperature value into the third byte of the buffer.
    memcpy(buf2 + 2, &highTemp, 1);
    // Copies the lowest temperature value into the fourth byte of the buffer.
    memcpy(buf2 + 3, &lowTemp, 1);
    // Copies the Ambient Air Temperature (AAT) into the fifth byte of the buffer.
    memcpy(buf2 + 4, &AAT, 1);
    // Copies the average temperature into the sixth byte of the buffer.
    memcpy(buf2 + 5, &averageTemp, 1);
    // Copies the inverter temperature into the seventh byte of the buffer.
    memcpy(buf2 + 6, &invTemp, 1);
    // Copies the motor temperature into the eighth byte of the buffer.
    memcpy(buf2 + 7, &motTemp, 1);
    // Sends the filled buffer as a CAN frame with ID 3202, which might be designated for telemetry data.
    SendCANFrameToSerial(3202, buf2);
    // Resets the dashCounter to 0, starting the cycle over for the next interval.
    dashCounter = 0;
}


    if (analogRead(seatBelt) < 300) {
      seatBeltWarn = 1;
    } else
      seatBeltWarn = 0;

    moduleVoltage = ((analogRead(A0)) * 0.146627) + 8.5;  // this is ((1000ohm+2000ohm)/1000ohm) * (analogRead/1023)*5*   100 (scaling)
    //Serial.println(moduleVoltage);


  }


  //.println("going");


  dashCounter++;


}

void SendCANFrameToSerial(unsigned long canFrameId, const byte * frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial2.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial2.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial2.write(frameData, 8);
}
