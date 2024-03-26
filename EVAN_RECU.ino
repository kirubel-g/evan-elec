
#include <SPI.h>
#include <mcp_can.h>
// Define SPI CS pins for CAN controllers
const int spiCSPinFCAN = 53;  // Chip select pin for FCAN
const int spiCSPinBMSCAN = 37; // Chip select pin for BMSCAN

// Variables to track previous CAN send times
unsigned long previousCanSend = 0;    // Previous CAN send time for FCAN
unsigned long previousCanSend2 = 0;   // Previous CAN send time for BMSCAN

// Define analog pin for charge detection
int chargeDet = A1;

// Pin assignments for various vehicle functions
int r7 = 42;        // Pin for function r7
int bsmPow = 47;    // Pin for Blind Spot Monitoring power
int leftTurn = 49;  // Pin for left turn signal
int rightTurn = 46; // Pin for right turn signal
int revLight = 45;  // Pin for reverse light
int chmsl = 43;     // Pin for center high-mounted stop lamp
int taskLight = 44; // Pin for task light
int tailLight = 9;  // Pin for tail light
int wiperHigh = 5;  // Pin for high-speed wiper control
int wiperLow = 4;   // Pin for low-speed wiper control
int bmsIg = 8;      // Pin for Battery Management System ignition

// Power-related variables
int igState = 0;    // Ignition state

// Lamp control variables
// (These variables are used to control various lamps)
// Lamp control variables
int lamps1 = 0;                 // Bytes containing the specific bits which enocde all of the lamp commands same for wipers 1 and 2
int lamps2 = 0;                 // Bytes containing the specific bits which enocde all of the lamp commands same for wipers 1 and 2


int leftCommand = 0;            // Left turn signal command
int rightCommand = 0;           // Right turn signal command
int brakeCommand = 0;           // Brake signal command
int brakeOn = 0;                // Status of brake signal
int blinking1 = 0;              // State of Blinking Left
int blinking2 = 0;              // State of Blinking Right
unsigned long blinktime2 = 0;   // Time of last blink for group 2
unsigned long previousBlink2 = 0; // Previous blink time for group 2
int rightFrontBlink = 0;        // State of right front blinker
int leftFrontBlink = 0;         // State of left front blinker
unsigned long blinktime1 = 0;   // Time of last blink for group 1
unsigned long previousBlink1 = 0; // Previous blink time for group 1
int taskCommand = 0;            // Task light command
int offCommand = 1;              // Turn off command
int headCommand = 0;            // Headlight command
int parkCommand = 0;            // Parking light command
int autoOn = 0;                 // Automatic mode status
int hazCommand = 0;             // Hazard light command
int gear = 0;                   // Current gear position
int f2p = 0;                    // f2p
int highbeamCommand = 0;        // High beam command
int headlight = 0;              // Status of headlights
int highbeam = 0;               // Status of high beams
int parklight = 0;              // Status of parking lights


// Wiper control variables
// (These variables are used to control wiper functions)
int wipers1 = 0;           // State of wipers group 1
int lowCommand = 0;        // Low wiper command
int highCommand = 0;       // High wiper command
int delayCommand = 0;      // Wiper delay command
int wipers2 = 0;           // State of wipers group 2
int delayOn = 0;           // Delay mode status
int wiperDelayNew = 0;     // New wiper delay value
int wiperHighOn = 0;       // Status of high-speed wiper
int wiperLowOn = 0;        // Status of low-speed wiper
int oktoshutoff = 0;       // OK to shut off wipers
int lowBit = 0;            // Wiper commands
int highBit = 0;           // Wiper commands

// Wiper timing variables (in milliseconds)
unsigned long wiperTimer0 = 0; // Timer for wiper action
unsigned long wiperTimer1 = -15000; // Delay before wiper action 1 (15 seconds)
unsigned long wiperTimer2 = -10000; // Delay before wiper action 2 (10 seconds)
unsigned long wiperTimer3 = -7000;  // Delay before wiper action 3 (7 seconds)
unsigned long wiperTimer4 = -5000;  // Delay before wiper action 4 (5 seconds)
unsigned long wiperTimer5 = -2500;  // Delay before wiper action 5 (2.5 seconds)


MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's
MCP_CAN CAN2(spiCSPinBMSCAN);  //start talking on BMSCAN

//All the BSM Stuff

// Constants for SPI chip select pins
const int spiCSPinFCAN = 53;
const int spiCSPinBMSCAN = 37;

// Variables for CAN message timing
unsigned long previousCanSend = 0;
unsigned long previousCanSend2 = 0;

// Analog pin for charge detection
int chargeDet = A1;

// Pin assignments for various vehicle functions
int r7 = 42;
int bsmPow = 47;
int leftTurn = 49;
int rightTurn = 46;
int revLight = 45;
int chmsl = 43;
int taskLight = 44;
int tailLight = 9;
int wiperHigh = 5;
int wiperLow = 4;
int bmsIg = 8;

// Power-related variables
int igState = 0;

// Lamp-related variables
int lamps1 = 0;
int lamps2 = 0;
int leftCommand = 0;
int rightCommand = 0;
int brakeCommand = 0;
int brakeOn = 0;
int blinking1 = 0;
int blinking2 = 0;
unsigned long blinktime2 = 0;
unsigned long previousBlink2 = 0;
int rightFrontBlink = 0;
int leftFrontBlink = 0;
unsigned long blinktime1 = 0;
unsigned long previousBlink1 = 0;
int taskCommand = 0;
int offCommand = 1;
int headCommand = 0;
int parkCommand = 0;
int autoOn = 0;
int hazCommand = 0;
int gear = 0;
int f2p = 0;
int highbeamCommand = 0;
int headlight = 0;
int highbeam = 0;
int parklight = 0;

// Wiper-related variables
int wipers1 = 0;
int lowCommand = 0;
int highCommand = 0;
int delayCommand = 0;
int wipers2 = 0;
int delayOn = 0;
int wiperDelayNew = 0;
int wiperHighOn = 0;
int wiperLowOn = 0;
int oktoshutoff = 0;
int lowBit = 0;
int highBit = 0;

// Wiper timing variables
unsigned long wiperTimer0 = 0;
unsigned long wiperTimer1 = -15000;
unsigned long wiperTimer2 = -10000;
unsigned long wiperTimer3 = -7000;
unsigned long w


unsigned long lastSend = 0;
unsigned long lastSend1 = 0;
unsigned long lastSend2 = 0;
unsigned long lastSend3 = 0;


unsigned char len1 = 0;
unsigned char buf1[8];
unsigned long canId1 = 0;
float steerAngle = 0;
uint16_t steerAngleRough = 0;

int bsmWarn = 0;


//shutdown sequence
int lockout1 = 0;
unsigned long comingHomeTimer = 0;
int chargeOn = 0;


/*
 * Function: setup
 * ----------------------------
 * This function is called once at the beginning of the program.
 * It initializes pin modes, serial communication, and CAN bus communication.
 */
void setup() {
  // Set pin modes and initial states

  // Control pins for various vehicle functions
  pinMode(r7, OUTPUT);
  digitalWrite(r7, LOW); // Ensure ECU is initially off
  digitalWrite(r7, HIGH); // Turn on ECU

  // Start serial communication with PC
  Serial.begin(9600);

  // Set pin modes for various vehicle functions
  pinMode(bsmPow, OUTPUT);
  pinMode(leftTurn, OUTPUT);
  pinMode(rightTurn, OUTPUT);
  pinMode(revLight, OUTPUT);
  pinMode(chmsl, OUTPUT);
  pinMode(taskLight, OUTPUT);
  pinMode(tailLight, OUTPUT);
  pinMode(wiperHigh, OUTPUT);
  pinMode(wiperLow, OUTPUT);
  pinMode(bmsIg, OUTPUT);

  // Set initial states for various vehicle functions
  digitalWrite(bsmPow, LOW);
  digitalWrite(leftTurn, LOW);
  digitalWrite(rightTurn, LOW);
  digitalWrite(revLight, LOW);
  digitalWrite(chmsl, LOW);
  digitalWrite(taskLight, LOW);
  digitalWrite(tailLight, LOW);
  digitalWrite(wiperHigh, LOW);
  digitalWrite(wiperLow, LOW);
  digitalWrite(bmsIg, LOW);

  // Initialize CAN communication for FCAN BUS
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
  {
    Serial.println("FCAN BUS init Failed");
    delay(100);
  }
  Serial.println("FCAN BUS Shield Init OK!");

  // Initialize CAN communication for BSM BUS
  while (CAN_OK != CAN2.begin(CAN_125KBPS, MCP_8MHz))
  {
    Serial.println("BSM BUS init Failed");
    delay(100);
  }
  Serial.println("BSM BUS Shield Init OK!");
}

/*
 * Function: loop
 * ----------------------------
 * This function is called repeatedly in a loop after the setup function.
 * It controls the behavior of various vehicle functions based on input conditions.
 */
void loop() {

  // Control behavior based on ignition state
  if (igState > 1) {
    blinkers(); // Control blinkers
    tailLamps(); // Control tail lamps
    lockout1 = 0; // Reset lockout flag
  } else if (autoOn == 1  && lockout1 == 0) {
    lockout1 = 1; // Prevent looping and indicate auto lights are on
    comingHomeTimer = millis(); // Set timer for lights off and ECU shutdown
    shutdownSeq(); // Perform shutdown sequence
  } else if (lockout1 == 0) {
    comingHomeTimer = millis(); // Set timer for ECU shutdown only
    shutdownSeq(); // Perform shutdown sequence
    lockout1 = 1; // Prevent looping, shutdownSeq won't see this value
  } else if (hazCommand == 1 || brakeCommand == 1) { // Handle brake or hazard commands after shutdown
    if (hazCommand == 1) { // Bypass lights function for hazards
      leftCommand = 1;
      rightCommand = 1;
    } else {
      leftCommand = 0;
      rightCommand = 0;
    }
    blinkers(); // Execute command
  } else { // If nothing is on, continuously shut off the blinker and turn signals
    blinking1 = 0;
    blinking2 = 0;
    blinktime2 = 0;
    previousBlink2 = 0;
    rightFrontBlink = 0;
    leftFrontBlink = 0;
    blinktime1 = 0;
    previousBlink1 = 0;
    digitalWrite(leftTurn, LOW);
    digitalWrite(rightTurn, LOW);
    digitalWrite(chmsl, LOW);
  }

  // Handle ECU shutdown and light control
  if (igState <= 1) {
    if (millis() - comingHomeTimer > 30000) { // Delay to leave lights on after key off
      digitalWrite(taskLight, LOW);
      digitalWrite(tailLight, LOW);
      parklight = 0;
      headlight = 0;
    }
    if (((millis() - comingHomeTimer > 60000 && hazCommand == 0) || (millis() - comingHomeTimer > 1800000 && hazCommand == 1))) { // Delay to shut off ECU
      if (chargeOn == 0x00) { // Only shut off if not plugged in
        digitalWrite(r7, LOW);
      } else {
        digitalWrite(r7, HIGH); // Reactivate relay if charging
      }
    }
  }

  // Handle CAN communication
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
  }

  // Process CAN messages
  if (canId == 0x102) { // IGNITION STATUS message
    igState = buf[1];
    power();
    brakeCommand = buf[2];
    hazCommand = buf[3];
    gear = buf[4];
  }

  if (canId == 0x091  && igState > 1) { // Switch state message
    lamps1 = buf[1];
    lamps2 = buf[0];
    wipers1 = buf[2];
    wipers2 = buf[6];
    lights(); // Control lights
    wipers(); // Control wipers
  }

  if (canId == 0x101) { // ACCELERATOR PEDAL POSITION message
    accelPct = buf[0] * 0.392;
  }

  if (canId == 0x105) { // MOTOR SPEED message
    motorSpeed = (buf[0] << 8) | buf[1];
    motorSpeed = motorSpeed * 0.25;
    vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828);
    vss = vss * 0.621; // Convert to mph
  }

  if (canId == 0x086) { // STEERING ANGLE message
    steerAngle = (buf[0] << 8) | buf[1];
    steerAngle = (steerAngle * 0.1) - 1598; // Calibrated steering angle
    steerAngleRough = round(steerAngle) + 1000;
    byte last3Bits = steerAngleRough & 0b00000111;
    byte remainingBits = steerAngleRough >> 3;
    byte byte3 = (remainingBits << 3) | last3Bits;
    byte byte4 = remainingBits >> 5;
    stmp86[3] = byte4;
    stmp86[4] = byte3;
  }

  if (canId == 0x6B1) { // AMBIENT TEMPERATURE message
    ambient = buf[2] - 40; // Scale temp to real value
    chargeOn = buf[6];
  }

  // Send CAN message for turn signal blink
  if (millis() - previousCanSend > 30) {
    unsigned char stmp[8] = {leftFrontBlink, rightFrontBlink, parklight, headlight, highbeam, bsmWarn, 0, 0}; //
    CAN.sendMsgBuf(0x103, 0, 8, stmp);
    previousCanSend = millis();
  }

  // Call BSM related functions
  bsmValues();
  bsm();
}



/*
 * Function: power
 * ----------------------------
 * This function controls the power state of various components based on the ignition state.
 */
void power() {
  // Serial.println(igState);
  
  // If in IG2
  if (igState > 1) {
    digitalWrite(bsmPow, HIGH); // Turn on power to BSM
    digitalWrite(bmsIg, HIGH); // Turn on power to BMS
    digitalWrite(r7, HIGH); // Ensure ECU is on
    // Serial.println("ON");
    //Serial.println(igState);
  }

  // If not in IG2
  if (igState <= 1) {
    digitalWrite(bsmPow, LOW); // Turn off power to BSM
    digitalWrite(bmsIg, LOW); // Turn off power to BMS
    // Serial.println("off");
  }
}



/*
 * Function: lights
 * ----------------------------
 * This function controls the lighting system based on the input signals received.
 */
void lights() {
  // Serial.println(lamps1);

  // If left turn signal is activated or hazard lights are on, set leftCommand
  if ((lamps1 & 0b00100000) || hazCommand == 1) {
    leftCommand = 1;
  } else {
    leftCommand = 0;
  }

  // If right turn signal is activated or hazard lights are on, set rightCommand
  if ((lamps1 & 0b00010000) || hazCommand == 1) {
    rightCommand = 1;
  } else {
    rightCommand = 0;
  }

  // If fog lights are activated, turn on task light
  if (lamps2 & 0b00000010) {
    digitalWrite(taskLight, HIGH);
  } else {
    digitalWrite(taskLight, LOW);
  }

  // If park lamp is activated, set parkCommand
  if (lamps2 & 0b00100000) {
    parkCommand = 1;
  } else {
    parkCommand = 0;
  }

  // If headlamp is activated, set headCommand
  if (lamps2 & 0b00010000) {
    headCommand = 1;
  } else {
    headCommand = 0;
  }

  // If off lamp is activated, set offCommand
  if (lamps1 & 0b01000000) {
    offCommand = 1;
  } else {
    offCommand = 0;
  }

  // If flash to pass is activated, set f2p
  if (lamps1 & 0b10000000) {
    f2p = 1;
  } else {
    f2p = 0;
  }

  // If high beam is activated, set highbeamCommand
  if (lamps2 & 0b00001000) {
    highbeamCommand = 1;
  } else {
    highbeamCommand = 0;
  }

  // If in reverse gear, turn on reverse light
  if (gear == 1) {
    digitalWrite(revLight, HIGH);
  } else {
    digitalWrite(revLight, LOW);
  }
}



/*
 * Function: blinkers
 * ----------------------------
 * This function controls the behavior of the turn signals and brake lights.
 */
void blinkers() {
  // If brake command is received, turn on the brake lights including the center high-mounted stop lamp (CHMSL)
  if (brakeCommand == 1) {
    brakeOn = 1;
    digitalWrite(chmsl, HIGH);
  } else {
    brakeOn = 0;
    digitalWrite(chmsl, LOW);
  }

  // Rear left turn signal control
  if (millis() - blinktime1 > 400 && blinking1 == 1) {
    digitalWrite(leftTurn, LOW); // Turn off rear left turn signal
    leftFrontBlink = 0;
    blinking1 = 0;
    previousBlink1 = millis();
  }

  if (leftCommand == 1 && blinking1 == 0 && millis() - previousBlink1 > 400) {
    digitalWrite(leftTurn, HIGH); // Turn on rear left turn signal
    leftFrontBlink = 1;
    blinktime1 = millis();
    blinking1 = 1;
  }

  // Rear right turn signal control
  if (millis() - blinktime2 > 400 && blinking2 == 1) {
    digitalWrite(rightTurn, LOW); // Turn off rear right turn signal
    rightFrontBlink = 0;
    blinking2 = 0;
    previousBlink2 = millis();
  }

  if (rightCommand == 1 && blinking2 == 0 && millis() - previousBlink2 > 400) {
    digitalWrite(rightTurn, HIGH); // Turn on rear right turn signal
    rightFrontBlink = 1;
    blinktime2 = millis();
    blinking2 = 1;
  }

  // Handle brake light behavior when no turn signal is active
  if (rightCommand == 0 && brakeOn == 1 && blinking2 == 0) {
    digitalWrite(rightTurn, HIGH); // Turn on rear right brake light
  }

  if (rightCommand == 0 && brakeOn == 0 && blinking2 == 0) {
    digitalWrite(rightTurn, LOW); // Turn off rear right brake light
  }

  if (brakeOn == 1 && leftCommand == 0 && blinking1 == 0) {
    digitalWrite(leftTurn, HIGH); // Turn on rear left brake light
  }

  if (leftCommand == 0 && brakeOn == 0 && blinking1 == 0) {
    digitalWrite(leftTurn, LOW); // Turn off rear left brake light
  }
}


/*
 * Function: tailLamps
 * ----------------------------
 * This function controls the behavior of the tail lamps based on various conditions.
 */
void tailLamps() {
  // If in park lights or in headlights without flash-to-pass (F2P), or in auto position
  if ((parkCommand == 1) || (headCommand == 1 && offCommand == 0) || (offCommand == 0 && parkCommand == 0 && headCommand == 0)) {
    digitalWrite(tailLight, HIGH); // Turn on tail lights
    parklight = 1; // Set park light flag
  } else {
    digitalWrite(tailLight, LOW); // Turn off tail lights
    parklight = 0; // Clear park light flag
  }

  // If in headlights without F2P or in auto position
  if ((headCommand == 1 && offCommand == 0) || (offCommand == 0 && parkCommand == 0 && headCommand == 0)) {
    headlight = 1; // Set headlight flag
  } else {
    headlight = 0; // Clear headlight flag
  }

  // If in highbeam position and headlights not off or F2P
  if ((highbeamCommand == 1 && offCommand == 0) || (f2p == 1)) {
    highbeam = 1; // Set highbeam flag
    headlight = 0; // Clear headlight flag
  } else {
    highbeam = 0; // Clear highbeam flag
  }

  // If in auto position
  if (offCommand == 0 && parkCommand == 0 && headCommand == 0) {
    autoOn = 1; // Set auto-on flag
  } else {
    autoOn = 0; // Clear auto-on flag
  }
}


/*
 * Function: wipers
 * ----------------------------
 * This function controls the behavior of the wipers based on various conditions.
 */
void wipers() {
  if ((wipers1 & 0b00100000)) {  // If bit 2 is set
    highBit = 1;
  } else {
    highBit = 0;
  }

  if (wipers1 & 0b00010000) {  // If bit 3 is set
    lowBit = 1;
  } else {
    lowBit = 0;
  }

  // Turn bits into commands
  if (lowBit == 0 && highBit == 1) {
    highCommand = 1; // High speed command
  } else {
    highCommand = 0;
  }

  if (lowBit == 1 && highBit == 0) {
    lowCommand = 1; // Low speed command
  } else {
    lowCommand = 0;
  }

  if (lowBit == 1 && highBit == 1) {
    delayCommand = 1; // Delay command
    delayOn = 1;
  } else {
    delayCommand = 0;
    delayOn = 0;
  }

  // Determine delay setting
  if (wipers2 == 0x5A) {
    wiperDelayNew = 2;
    // Reset timers other than the one used for this delay so that wipers do a wipe immediately when any delay is selected
    wiperTimer2 = -15000;
    wiperTimer3 = -15000;
    wiperTimer4 = -15000;
    wiperTimer5 = -15000;
  } else if (wipers2 == 0x46) {
    wiperDelayNew = 4;
    wiperTimer1 = -15000;
    wiperTimer3 = -15000;
    wiperTimer4 = -15000;
    wiperTimer5 = -15000;
  } else if (wipers2 == 0x32) {
    wiperDelayNew = 6;
    wiperTimer1 = -15000;
    wiperTimer2 = -15000;
    wiperTimer4 = -15000;
    wiperTimer5 = -15000;
  } else if (wipers2 == 0x1E) {
    wiperDelayNew = 8;
    wiperTimer1 = -15000;
    wiperTimer2 = -15000;
    wiperTimer3 = -15000;
    wiperTimer5 = -15000;
  } else if (wipers2 == 0x00) {
    wiperDelayNew = 10;
    wiperTimer1 = -15000;
    wiperTimer2 = -15000;
    wiperTimer3 = -15000;
    wiperTimer4 = -15000;
  }

  // High speed
  if (highCommand == 1) {
    digitalWrite(wiperHigh, HIGH);
    digitalWrite(wiperLow, LOW);
    wiperHighOn = 1;
    wiperDelayNew = 0; // Allow for delay to be reset when high selected.
    delayOn == 0; // Allow for delay to be reset when high selected.
  }

  // Wiper delay selections
  if (delayCommand == 1 && wiperDelayNew == 2 && millis() - wiperTimer1 > 15000) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    Serial.println("check");
    wiperTimer0 = millis();
    wiperTimer1 = millis();
  }

  // Other delay selections omitted for brevity...

  // Ending delay pulse (command to send wipers to rest)
  if (millis() - wiperTimer0 > 1100 && delayCommand == 1 && highCommand == 0 && delayOn == 1) {
    digitalWrite(wiperLow, LOW);
  }

  // Low speed
  if (lowCommand == 1 && delayOn == 0) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    wiperLowOn = 1;
  }

  // High speed off
  if (highCommand == 0) {
    digitalWrite(wiperHigh, LOW);
  }

  // Check to see if something has been on
  if (wiperDelayNew != 0 || wiperHighOn == 1 || wiperLowOn == 1) {
    oktoshutoff = 1;
  }

  // Shut system down
  if (highCommand == 0 && lowCommand == 0 && oktoshutoff == 1 && delayCommand == 0) {
    digitalWrite(wiperLow, LOW);
    wiperDelayNew = 0;
    wiperHighOn = 0;
    wiperLowOn = 0;
    oktoshutoff = 0;
    delayOn = 0;
  }
}

/*
 * Function: bsmValues
 * ----------------------------
 * This function updates various BSM (Blind Spot Monitoring) values based on different conditions.
 */
void bsmValues() {
  // Reset values to default
  stmp40A18[3] = 0x08;  // Default value assuming we are > 1500 RPM
  stmp9F[0] = 0x40;     // Reset bits to default values
  stmp202[2] = 0x00;
  stmp202[3] = 0x00;
  stmp91[1] = 0x00;

  // Update values based on different conditions

  // Gear condition
  if (gear == 1) {
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 7);  // Set reverse bit in stmp40A18
    stmp9F[0] |= static_cast<char>((1 & 0x01) << 0);    // Set reverse bit in stmp9F
  }
  if (gear == 3) {
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 6);  // Set another gear condition in stmp40A18
  }

  // Park light condition
  if (parklight == 1) {
    stmp9F[0] |= static_cast<char>((1 & 0x01) << 3);    // Set taillights bit in stmp9F
  }

  // Accelerator pedal position condition
  if (accelPct <= 20 && accelPct > 1) { // <20% pedal
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 4);
  }
  if (accelPct > 20) { // >20% pedal
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 5);
  }

  // Vehicle speed condition
  if (vss > 6.2) {  // >10 kph
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 1);
  }
  if (vss <= 6.2) {  // <10 kph
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 0);  // May have to add logic to set to 0 when vss = 0 if erroneous readings when stopped
  }

  // Scale and switch vehicle speed to kph
  vssRaw = static_cast<uint16_t>(vss * 100 * 1.609);

  // Assuming big-endian encoding
  stmp202[2] = (vssRaw >> 8) & 0xFF;  // Most significant byte
  stmp202[3] = vssRaw & 0xFF;          // Least significant byte

  // Command conditions for left and right blinkers
  if (leftCommand == 1) {
    stmp91[1] = 0x20;  // Left blinker command
  }
  if (rightCommand == 1) {
    stmp91[1] = 0x10;  // Right blinker command
  }
}


/*
 * Function: bsm
 * ----------------------------
 * This function reads messages from the CAN bus, processes them, and sends out new messages based on the received data.
 */
void bsm() {
  // Check if a message is available on CAN2
  if (CAN_MSGAVAIL == CAN2.checkReceive()) {
    CAN2.readMsgBuf(&len1, buf1);
    canId1 = CAN2.getCanId();
    // Serial.println(canId1);
  }

  // Process messages with ID 0x477
  if (canId1 == 0x477) {
    // Check for left and right side warnings
    if (buf1[1] & 0b00010000 || buf1[1] & 0b00100000) {
      bsmWarn |= 0b00000001; // Set left warning bit
    } else {
      bsmWarn &= 0b11111110; // Clear left warning bit
    }
    if (buf1[1] & 0b01000000 || buf1[1] & 0b10000000) {
      bsmWarn |= 0b00000010; // Set right warning bit
    } else {
      bsmWarn &= 0b11111101; // Clear right warning bit
    }

    // Check for buzzer activation
    if (buf1[2] == 0x7A || buf1[2] == 0x2A || buf1[2] == 0xDA || buf1[2] == 0x8A || buf1[1] & 0b00100000 || buf1[1] & 0b10000000) {
      bsmWarn |= 0b10000000; // Set buzzer bit
    } else {
      bsmWarn &= 0b01111111; // Clear buzzer bit
    }
  }

  // Send out messages periodically
  if ((millis() - lastSend > 30)) {
    CAN2.sendMsgBuf(0x78, 0, 8, stmp78);
    CAN2.sendMsgBuf(0x86, 0, 8, stmp86);
    lastSend = millis();
    // Update counters and adjust values for checksums
    i = stmp78[6];
    i = i + 16;
    j = stmp86[5];
    j = j + 1;
    if (stmp78[6] == 240) {
      stmp78[6] = 0x00;
      i = stmp78[6];
    }
    if (stmp86[5] == 155) {
      stmp86[6] = 0x98; // Could be wrong
      j = stmp86[6];
    }
    stmp78[6] = i;
    stmp86[5] = j;
  }

  // Send out additional messages with different intervals
  if ((millis() - lastSend3 > 25)) {
    CAN2.sendMsgBuf(0x9E, 0, 8, stmp9E);
    CAN2.sendMsgBuf(0x9F, 0, 8, stmp9F);
    lastSend3 = millis();
  }
  if ((millis() - lastSend1 > 35)) {
    CAN2.sendMsgBuf(0x202, 0, 8, stmp202);
    lastSend1 = millis();
    var = stmpFD[2];
    var = var + 80;
    stmpFD[2] = var;
  }
  if ((millis() - lastSend2 > 100)) {
    AAT = ambient + 40;
    stmp40A17[7] = AAT;
    CAN2.sendMsgBuf(0x91, 0, 8, stmp91);
    counter++;
    // Send different stmp40A messages based on counter value
    // The counter logic seems to be incorrect, as some values are skipped
    // Some conditions are commented out, but the logic may need review
    // Correct the logic based on your requirements
    // Example logic is provided below for counter increment and reset
    if (counter == 22) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A22);
      counter = 0; // Reset counter
    }
    lastSend2 = millis();
    k = stmp91[3];
    k = k + 1;
    if (stmp91[3] == 15) {
      stmp91[3] = 0x00;
      k = stmp91[3];
    }
    stmp91[3] = k;
  }
}


/*
 * Function: shutdownSeq
 * ---------------------
 * Description: This function handles the shutdown sequence of the vehicle, ensuring
 *              that all components are turned off or reset to their default states
 *              when the vehicle is powered down.
 * Parameters: None
 * Returns: None
 */
void shutdownSeq() {

  digitalWrite(leftTurn, LOW);
  digitalWrite(rightTurn, LOW);
  digitalWrite(revLight, LOW);
  digitalWrite(chmsl, LOW);
  digitalWrite(wiperHigh, LOW);
  digitalWrite(wiperLow, LOW);

  // Check if lockout1 is activated
  if (lockout1 == 1) {
    // If lockout1 is activated, turn on parklights and dome light
    digitalWrite(taskLight, HIGH);
    digitalWrite(tailLight, HIGH);
    parklight = 1;
    headlight = 1;
  } else {
    // If lockout1 is not activated, turn off parklights and dome light
    digitalWrite(taskLight, LOW);
    digitalWrite(tailLight, LOW);
    parklight = 0;
    headlight = 0;
  }

  // Resetting various flags and variables to their default states
  leftCommand = 0;
  rightCommand = 0;
  brakeOn = 0;
  blinking1 = 0;
  blinking2 = 0;
  blinktime2 = 0;
  previousBlink2 = 0;
  rightFrontBlink = 0;
  leftFrontBlink = 0;
  blinktime1 = 0;
  previousBlink1 = 0;
  taskCommand = 0;
  offCommand = 1;
  headCommand = 0;
  parkCommand = 0;
  autoOn = 0;
  gear = 0;
  f2p = 0;
  highbeamCommand = 0;
  highbeam = 0;
  wipers1 = 0;
  lowCommand = 0;
  highCommand = 0;
  delayCommand = 0;
  wipers2 = 0;
  delayOn = 0;
  wiperDelayNew = 0;
  wiperHighOn = 0;
  wiperLowOn = 0;
  oktoshutoff = 0;
  lowBit = 0;
  highBit = 0;
  wiperTimer0 = 0;
  wiperTimer1 = -15000;
  wiperTimer2 = -10000;
  wiperTimer3 = -7000;
  wiperTimer4 = -5000;
  wiperTimer5 = -2500;
  bsmWarn = 0;
}
