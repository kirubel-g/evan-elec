
#include <SPI.h>
#include <mcp_can.h>
#include <PID_v1.h>
const int spiCSPinFCAN = 53;

unsigned long previousCanSend = 0;

//int
// Define variables for pins used in the project

int r7 = 42;           // Pin for some purpose, labeled as 'd'
int heatTemp = A4;     // Pin for reading heat temperature sensor
int acPress = A5;      // Pin for reading AC pressure sensor
int brakeWarn = A2;    // Pin for brake warning indicator, labeled as 'd'
int absLight = A1;     // Pin for ABS warning light, labeled as 'd'

// Define pins for various components or signals

int mainIgPow = 6;     // Pin for main ignition power
int batPumpPwm = 10;  // Pin for battery pump PWM control, labeled as 'd'
int heatPwm = 8;       // Pin for controlling heating PWM
int throtVal = 11;     // Pin for throttle value reading
int invPumpPwm = 12;   // Pin for inverter pump PWM control, labeled as 'd'
int radFan = 7;        // Pin for radiator fan control
int epkbPow = 44;      // Pin for EPKB power, labeled as 'd'
int epkb1 = 49;        // Pin for EPKB1 signal, labeled as 'd'
int epkb2 = 47;        // Pin for EPKB2 signal, labeled as 'd'
int absVss = 9;        // Pin for ABS vehicle speed sensor, labeled as 'd'
int headlamp = 4;      // Pin for controlling headlamp, labeled as 'd'
int leftTurn = 46;     // Pin for left turn signal, labeled as 'd'
int rightTurn = 45;    // Pin for right turn signal, labeled as 'd'
int horn = 43;         // Pin for controlling horn, labeled as 'd'
int highbeam = 5;      // Pin for controlling high beam, labeled as 'd'



// Power-related variables
int igState = 0;                    // Ignition state
int lockout2 = 0;                   // Lockout status
unsigned long epkbTimer2 = 0;       // Timer for EPKB

// Lights-related variables
int leftBlink = 0;                  // Left turn signal status
int rightBlink = 0;                 // Right turn signal status
int headlampOn = 0;                 // Headlamp status
int highbeamOn = 0;                 // High beam status
int hornOn = 0;                     // Horn status

// Pumps-related variables
float pumpTimeRequest = 0;          // Pump time request
float invpumpPct = 0;               // Inverter pump percentage
unsigned long startTime = 0;        // Start time for pump
int sigOn = 0;                      // Signal status

float pumpTimeRequest2 = 0;         // Second pump time request
float batpumpPct = 0;               // Battery pump percentage
unsigned long startTime2 = 0;       // Start time for second pump
int sigOn2 = 0;                     // Second signal status

// EPKB-related variables
int gear = 0;                       // Gear position
int applied1 = 0;                   // Applied status 1
int applied2 = 0;                   // Applied status 2
unsigned long epkbTimer = 0;        // EPKB timer
unsigned long soundTimer = 0;       // Sound timer
int pkbApplied = 0;                 // EPKB applied status

// ABS-related variables
float absFreq = 0;                  // ABS frequency
int brakeWarnFlag = 0;              // Brake warning flag
int absWarnFlag = 0;                // ABS warning flag
uint16_t motorSpeed = 0;            // Motor speed
float vss = 0;                      // Vehicle speed
uint16_t vssRaw = 0;                // Raw vehicle speed

// Temperature Control-related variables (TEMPCTRL)
double Setpoint, Input, Output;     // PID control variables
double Kp = 2, Ki = 5, Kd = 1;      // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // PID controller

// HVAC-related variables
int acRequest = 0;                  // AC request
int heatReq = 0;                    // Heat request
float blowerSpeed = 0;              // Blower speed
int acOn = 0;                       // AC status
uint16_t acDuty = 0;                // AC duty cycle
int batCooling = 0;                 // Battery cooling status
int invTemp = 30;                   // Inverter temperature (default: cooling off)
int motTemp = 30;                   // Motor temperature
int motorCooling = 0;               // Motor cooling status
int batHeating = 0;                 // Battery heating status
int cabinHeating = 0;               // Cabin heating status
int rawClt = 0;                     // Raw coolant temperature
float cltRes = 0;                   // Coolant resistance
float clt = 0;                      // Coolant temperature
int fanByte = 0;                    // Fan byte
int highTemp = 35;                  // High temperature threshold for battery water cooling
int lowTemp = 20;                   // Low temperature threshold
uint16_t acPower = 6000;            // Default AC power (up to 6kW)
unsigned long lastCanSend = 0;      // Last CAN message sent time
byte duty1 = 0;                     // Duty cycle 1
byte duty2 = 0;                     // Duty cycle 2
byte power1 = 0;                    // Power 1
byte power2 = 0;                    // Power 2
int acPressVal = 0;                 // AC pressure value
unsigned long lastHeater = 0;       // Last heater operation time
float batpumpPctSend = 0;          // Battery pump percentage to send
float invpumpPctSend = 0;          // Inverter pump percentage to send
int acHighTripOff = 0;              // AC high trip status
int passiveSetPoint = 30;           // Passive set point
int fanSetPoint = 50;               // Fan set point
int fanOffSetPoint = 47;            // Fan off set point
int passiveOffSetPoint = 28;        // Passive off set point
int batCoolPumpOn = 0;              // Battery cooling pump status
int batFanOn = 0;                   // Battery fan status
int fanOn = 0;                      // Fan status
int lockout10 = 0;                  // Lockout status
unsigned long fastCoolTimer = 0;    // Fast cooling timer

// Charging-related variables
int chargeOn = 0x00;                // Charging status
int lockout1 = 0;                   // Lockout status
unsigned long FECUTimer = 0;        // FECU timer

MCP_CAN CAN(spiCSPinFCAN);         // Initialize CAN communication on FCAN


/*
* A function to initialize the entire 
*/
void setup() {
  // Set up pin modes and initial states

  // Set pin mode for r7 as OUTPUT and initially set it LOW
  pinMode(r7, OUTPUT);
  digitalWrite(r7, LOW);
  
  // Turn on the ECU by setting r7 pin HIGH
  digitalWrite(r7, HIGH);

  // Start serial communication with PC and Front Ped Alert System
  Serial.begin(9600);
  Serial3.begin(9600);

  // Set pin modes for various components
  pinMode(mainIgPow, OUTPUT);
  pinMode(batPumpPwm, OUTPUT);
  pinMode(heatPwm, OUTPUT);
  pinMode(throtVal, OUTPUT);
  pinMode(invPumpPwm, OUTPUT);
  pinMode(radFan, OUTPUT);
  pinMode(epkbPow, OUTPUT);
  pinMode(epkb1, OUTPUT);
  pinMode(epkb2, OUTPUT);
  pinMode(absVss, OUTPUT);
  pinMode(headlamp, OUTPUT);
  pinMode(leftTurn, OUTPUT);
  pinMode(rightTurn, OUTPUT);
  pinMode(horn, OUTPUT);
  pinMode(highbeam, OUTPUT);

  // Set initial states of components to LOW
  digitalWrite(mainIgPow, LOW);
  digitalWrite(batPumpPwm, LOW);
  digitalWrite(heatPwm, LOW);
  digitalWrite(throtVal, LOW);
  digitalWrite(invPumpPwm, LOW);
  digitalWrite(radFan, LOW);
  digitalWrite(epkbPow, LOW);
  digitalWrite(epkb1, LOW);
  digitalWrite(epkb2, LOW);
  digitalWrite(absVss, LOW);
  digitalWrite(headlamp, LOW);
  digitalWrite(leftTurn, LOW);
  digitalWrite(rightTurn, LOW);
  digitalWrite(horn, LOW);
  digitalWrite(highbeam, LOW);

  // Initialize PID controller in AUTOMATIC mode
  myPID.SetMode(AUTOMATIC);

  // Initialize CAN communication
  // Keep trying until successful initialization
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  // Print initialization success message
  Serial.println("CAN BUS Shield Init OK!");
}

/*
* Function: loop
* Description: Main loop function that continuously executes the control logic.
*              Manages various system states, inputs, and outputs based on received CAN messages.
*/
void loop() {
  // Allow cooling system power if charging
  if (chargeOn == 0x01) {
    digitalWrite(mainIgPow, HIGH);
  }

  // If ignition state is on or charging
  if (igState > 1 || chargeOn == 0x01) {
    // Allow thermal control system to calculate
    thermalControl();

    // Default cooling
    if (igState > 1) {
      invpumpPct = 85; // Set inverter pump percentage
    } else {
      invpumpPct = 0; // Turn off inverter pump
    }

    pumps(); // Allow pumps to run
    lockout1 = 0; // Reset lockout flag
  } else if (lockout1 == 0) {
    lockout1 = 1; // Set lockout to prevent looping
    FECUTimer = millis(); // Set timer for ECU shutdown
    shutdownSeq(); // Initiate shutdown sequence
  } else if (headlampOn == 1 || leftBlink == 1) {
    FECUTimer = millis(); // Reset timer continuously if hazards are on or headlamps are on
  }

  // Turn off FECU if ignition is off and not charging
  if (igState <= 1 && chargeOn == 0x00) {
    if (millis() - FECUTimer > 120000) { // Delay to leave FECU after no lighting commands received and key off
      digitalWrite(r7, LOW); // Turn off FECU
    }
  } else {
    digitalWrite(r7, HIGH); // Keep FECU on
  }

  // Execute ABS control
  ABS();

  // Receive CAN messages
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
  }

  // Process received CAN messages
  if (canId == 0x102) {
    igState = buf[1]; // Ignition state
    power(); // Manage power states
    gear = buf[4]; // Gear state
    gears(); // Control gears
    hornOn = buf[5]; // Horn state
    if (hornOn == 1) {
      digitalWrite(horn, HIGH); // Turn on horn
    } else {
      digitalWrite(horn, LOW); // Turn off horn
    }
    acRequest = buf[0]; // Air conditioning request
    fanByte = buf[6]; // Fan speed request
    heatReq = buf[7]; // Heater request
  }

  if (canId == 0x103) {
    leftBlink = buf[0]; // Left turn signal
    rightBlink = buf[1]; // Right turn signal
    headlampOn = buf[3]; // Headlamp state
    highbeamOn = buf[4]; // High beam state
    lights(); // Control lights
  }

  if (canId == 0x230) {
    pkbApplied = buf[1]; // Parking brake applied state
  }

  if (canId == 0x105) {
    motorSpeed = (buf[0] << 8) | buf[1]; // Motor speed
    motorSpeed = motorSpeed * 0.25;
    vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828); // Vehicle speed
  }

  if (canId == 0x6B1) {
    highTemp = buf[0] - 40; // High temperature
    lowTemp = buf[1] - 40; // Low temperature
    chargeOn = buf[6]; // Charging state
  }

  if (canId == 0x106) {
    motTemp = buf[0] - 40; // Motor temperature
    invTemp = buf[1] - 40; // Inverter temperature
  }

  // Send CAN messages
  if (millis() - previousCanSend > 300) {
    unsigned char stmp[8] = {absWarnFlag, brakeWarnFlag, 0, 0, 0, 0, 0, 0}; // ABS and brake warning flags
    CAN.sendMsgBuf(0x108, 0, 8, stmp); // Send ABS and brake warning flags on ID 0x108
    previousCanSend = millis();
  }
}


/*
* Function: power
* Description: Manages power states of various components based on the ignition state.
*              Turns on power to EPKB and main ignition when ignition state is greater than 1.
*              Turns off EPKB power after a delay when ignition state is less than or equal to 1.
*              Sets inverter and motor cooling temperatures to default values when EPKB power is turned off.
*/
void power() {
  // Check ignition state
  if (igState > 1) {
    // Turn on EPKB and main ignition power
    digitalWrite(epkbPow, HIGH);
    digitalWrite(mainIgPow, HIGH);
    lockout2 = 0; // Reset lockout flag
    // Serial.println("ON");
    // Serial.println(igState);
  }

  // If ignition state is less than or equal to 1
  if (igState <= 1) {
    // If not already in lockout state, start timer and set lockout
    if (lockout2 == 0) {
      epkbTimer2 = millis(); // Start timer
      lockout2 = 1; // Set lockout flag
      Serial.println("on");
    }

    // After delay, turn off EPKB power
    if (millis() - epkbTimer2 >= 15000 && lockout2 == 1) {
      digitalWrite(epkbPow, LOW); // Turn off EPKB power
      lockout2 = 2; // Set lockout flag
      Serial.println("off");
      // Set inverter and motor cooling temperatures to default values
      invTemp = 20;
      motTemp = 20;
    }

    // Turn off main ignition power if charging is not active
    if (chargeOn == 0x00) {
      digitalWrite(mainIgPow, LOW);
    }
  }
}

/*
* Function: lights
* Description: Controls various lights based on system states and inputs.
*              Sets the state of left turn signal, right turn signal, high beam headlights,
*              and headlamps based on corresponding input flags.
*/
void lights() {
  // Control left turn signal
  if (leftBlink == 1) {
    digitalWrite(leftTurn, HIGH); // Turn left turn signal on
  } else {
    digitalWrite(leftTurn, LOW); // Turn left turn signal off
  }

  // Control right turn signal
  if (rightBlink == 1) {
    digitalWrite(rightTurn, HIGH); // Turn right turn signal on
  } else {
    digitalWrite(rightTurn, LOW); // Turn right turn signal off
  }

  // Control high beam headlights
  if (highbeamOn == 1) {
    digitalWrite(highbeam, HIGH); // Turn high beam headlights on
  } else {
    digitalWrite(highbeam, LOW); // Turn high beam headlights off
  }

  // Control headlamps
  if (headlampOn == 1) {
    digitalWrite(headlamp, HIGH); // Turn headlamps on
    // Serial3.print("#2\n");
  } else {
    digitalWrite(headlamp, LOW); // Turn headlamps off
  }
}


/*
* Function: pumps
* Description: Controls the operation of inverter and battery pumps based on percentage input.
*              The function maps percentage input to PWM duty cycle, calculates pump runtime,
*              and controls pump operation accordingly. Additionally, it manages carrier frequency
*              for smooth operation.
*/
void pumps() {

  // Inverter pump control
  invpumpPctSend = map(invpumpPct, 0, 100, 20, 80); // Map percentage to PWM range
  if (invpumpPct == 0) { // If no pump command, set duty to 10% (off). Range 20-80 is for running.
    invpumpPctSend = 10;
  }

  // Calculate pump time request based on percentage
  pumpTimeRequest = (invpumpPctSend / 100) * 500;

  // Set pump to high, when the start time is within the pumptime request
  if (millis() - startTime < pumpTimeRequest && sigOn == 0) {
    digitalWrite(invPumpPwm, HIGH); // Turn pump on
    startTime = millis();
    sigOn = 1;
  }

  // If the start time is greater the pumptime request, then set pump to low
  if (millis() - startTime > pumpTimeRequest) {
    digitalWrite(invPumpPwm, LOW); // Turn pump off
  }

  // If the start time is greater than 500 ms, the set sigOn to 0
  if (millis() - startTime > 500) { // Set carrier frequency.
    startTime = millis();
    sigOn = 0;
  }

  // Battery pump control
  // Create a map percentage of the battery pump
  batpumpPctSend = map(batpumpPct, 0, 100, 20, 80); // Map percentage to PWM range
  if (batpumpPct == 0) { // If no pump command, set duty to 10% (off). Range 20-80 is for running.
    batpumpPctSend = 10;
  }

  // Calculate pump time request based on percentage
  pumpTimeRequest2 = (batpumpPctSend / 100) * 500;

  // Control battery pump
  if (millis() - startTime2 < pumpTimeRequest2 && sigOn2 == 0) {
    digitalWrite(batPumpPwm, HIGH); // Turn pump on
    startTime2 = millis();
    sigOn2 = 1;
  }

  if (millis() - startTime2 > pumpTimeRequest2) {
    digitalWrite(batPumpPwm, LOW); // Turn pump off
  }

  if (millis() - startTime2 > 500) { // Set carrier frequency.
    startTime2 = millis();
    sigOn2 = 0;
  }

}

/*
* Function: gears
* Description: Controls the operation of brakes based on gear status.
*              It applies or releases brakes as necessary when the gear
*              is shifted between neutral and non-neutral positions.
*              Also, it manages brake release after a certain
*              duration and plays sounds if the car is not in park.
*/
void gears() {

  // Check if the gears are in neutral
  if (gear == 0 && applied1 == 0) {
    applied2 = 0;
    unsigned char stmp2[8] = {0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Command to stop releasing
    CAN.sendMsgBuf(0x254, 0, 8, stmp2); // Send command via CAN bus
    digitalWrite(epkb1, HIGH); // Apply brake 1
    digitalWrite(epkb2, LOW); // Release brake 2
    epkbTimer = millis(); // Start timer
    applied1 = 1; // Mark brake 1 as applied
    Serial3.print("q"); // Send command to silence sounds
  }

  // Check if the gears are not in neutral
  if (gear != 0 && applied2 == 0) {
    applied1 = 0;
    digitalWrite(epkb1, LOW); // Release brake 1
    digitalWrite(epkb2, HIGH); // Apply brake 2
    unsigned char stmp[8] = {0x03, 0xAE, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00}; // Command to release
    CAN.sendMsgBuf(0x254, 0, 8, stmp); // Send command via CAN bus
    epkbTimer = millis(); // Start timer
    applied2 = 1; // Mark brake 2 as applied
  }

  // If brakes have been applied for too long, release them
  if (millis() - epkbTimer > 10000) {  // Increase this time
    digitalWrite(epkb1, LOW); // Release brake 1
    digitalWrite(epkb2, LOW); // Release brake 2
  }

  // Check if the car is in park and play sounds if not
  if (applied2 == 1 && millis() - soundTimer > 2000) { // If not in park, play sounds
    Serial3.print("#0\n"); // Send command to play sounds
    soundTimer = millis(); // Reset timer
  }
}

/*
* Function: ABS
* Description: Controls the Anti-lock Braking System (ABS) functionality.
*              Calculates ABS frequency based on vehicle speed (VSS), generates
*              ABS tone, and monitors brake and ABS warning lights.
*              Sets corresponding flags if warnings are triggered.
*/
void ABS() {
  // Convert vehicle speed to ABS frequency
  absFreq = (vss * 128000) / 3600;

  // Generate ABS tone based on frequency
  tone(absVss, absFreq);

  // Check if brake warning light is triggered or parking brake is applied
  if (analogRead(brakeWarn) < 300 || pkbApplied != 0x00) {
    brakeWarnFlag = 1; // Set brake warning flag
  } else {
    brakeWarnFlag = 0; // Clear brake warning flag
  }

  // Check if ABS warning light is triggered
  if (analogRead(absLight) < 400) {
    absWarnFlag = 1; // Set ABS warning flag
  } else {
    absWarnFlag = 0; // Clear ABS warning flag
  }
}



/*
* Function: thermalControl
* Description: Manages the thermal control system, including controlling
*              the operation of the air conditioning, heating, and cooling
*              components based on various temperature and system states.
*              Handles battery and cabin heating, AC cooling, fan operation,
*              and CAN communication for AC control.
*/
void thermalControl() {
  // Read analog value of AC pressure sensor
  acPressVal = analogRead(acPress);

  // Check if fast cooling timer has expired
  if (millis() - fastCoolTimer > 60000) {
    acHighTripOff = 1; // Set AC high trip off flag
  } else {
    acHighTripOff = 0; // Clear AC high trip off flag
  }

  // Check if AC is requested and AC pressure is below threshold
  if (acRequest == 1 && acPressVal < 300) {
    // Turn on AC and fan
    acOn = 1;
    fanOn = 1;

    // Start fast cooling timer if not already locked out
    if (lockout10 == 0) {
      fastCoolTimer = millis();
      lockout10 = 1;
    }

    // Adjust blower speed based on HVAC settings
    if (fanByte != 0x0F) {
      blowerSpeed = fanByte; // Set blower speed to HVAC specified value
    } else {
      blowerSpeed = 0; // Turn off blower if HVAC specifies off
    }

    // Calculate AC duty cycle based on blower speed and operating conditions
    if (heatReq == 0) {
      acDuty = round((blowerSpeed / 7) * 100); // Scale duty cycle based on blower speed
      if (acHighTripOff == 0) {
        acDuty = 100; // Set duty cycle to maximum for fast cooling
      }
    } else {
      acDuty = round((blowerSpeed / 7) * 100); // Scale duty cycle based on blower speed
    }
  } else if (acPressVal > 500 || acRequest == 0) {
    acDuty = 0; // Turn off AC if pressure is high or AC is not requested
    acOn = 0;
    acHighTripOff = 0; // Clear AC high trip off flag
    lockout10 = 0; // Clear lockout flag for fast cooling
  }

  // Determine setpoints for passive and fan-assisted battery cooling
  if (chargeOn == 0x01) {
    // Setpoints for charging mode
    passiveSetPoint = 45;
    fanSetPoint = 45;
    fanOffSetPoint = 42;
    passiveOffSetPoint = 42;
  } else {
    // Setpoints for normal operation
    passiveSetPoint = 30;
    fanSetPoint = 50;
    fanOffSetPoint = 47;
    passiveOffSetPoint = 28;
  }

  // Control battery cooling based on temperature setpoints
  if (highTemp > passiveSetPoint) {
    // Activate passive and fan-assisted cooling
    batpumpPct = 85;
    batCoolPumpOn = 1;

    // Control fan based on temperature setpoint and hysteresis
    if (highTemp > fanSetPoint) {
      fanOn = 1; // Turn on fan if temperature exceeds setpoint
      batFanOn = 1; // Request fan priority
    } else if (highTemp < fanOffSetPoint) {
      batFanOn = 0; // Request fan off if temperature falls below hysteresis setpoint
    }
  } else if (highTemp < passiveOffSetPoint) {
    // Deactivate battery cooling
    batCoolPumpOn = 0;
    batFanOn = 0;
  }

  // Control inverter/motor cooling based on temperature thresholds
  if ((invTemp > 50 || motTemp > 50) && igState >= 2) {
    // Activate inverter/motor cooling if temperature exceeds threshold
    motorCooling = 1;
    fanOn = 1; // Turn on fan
  } else if ((invTemp < 47 && motTemp < 47) || igState < 2) {
    // Deactivate inverter/motor cooling if temperature falls below threshold
    motorCooling = 0;
  }

  // Control battery heating based on temperature
  if (lowTemp < 5) {
    // Activate battery heating if temperature falls below threshold
    batHeating = 1;
    batpumpPct = 85;
  } else if (lowTemp > 7) {
    // Deactivate battery heating if temperature rises above threshold
    batHeating = 0;
  }

  // Control cabin heating based on HVAC settings
  if (heatReq == 1) {
    // Activate cabin heating if HVAC requests it
    cabinHeating = 1;
    if (batCoolPumpOn == 0 && batHeating == 0) {
      // Adjust battery cooling pump if neither battery cooling nor heating is active
      batpumpPct = 30; // Adjust battery cooling pump for cabin heating
    }
  } else if (batCoolPumpOn == 0 && batHeating == 0) {
    // Deactivate cabin heating if HVAC does not request it and no other thermal control is active
    batpumpPct = 0;
    cabinHeating = 0;
  } else {
    // Deactivate cabin heating if thermal control is not required
    cabinHeating = 0;
  }

  // Control heater operation using PID control
  if ((cabinHeating == 1 || batHeating == 1) && millis() - lastHeater > 100) {
    // Start PID control loop if cabin or battery heating is active
    Setpoint = 49; // Set PID setpoint to desired temperature
    rawClt = analogRead(heatTemp); // Read analog value of heat sensor
    // Convert raw sensor value to resistance using known circuit parameters
    cltRes = (rawClt * 1.7595) / (4.68 - (rawClt * .00488759));
    if (cltRes < 0) {
      cltRes = 60000; // Set minimum resistance to prevent division by zero
    }
    // Calculate temperature using Steinhart formula
    clt = cltRes / 10000;
    clt = log(clt);
    clt = (1.0 / 5500) * clt;
    clt = (1.0 / 298.15) + clt;
    clt = 1.0 / clt;
    clt = clt - 273.15;
    Input = clt; // Set PID input to calculated temperature
    myPID.Compute(); // Compute PID control output
    analogWrite(heatPwm, Output); // Adjust heater power based on PID output
    lastHeater = millis(); // Update last heater operation timestamp
  }

  // Turn on radiator fan if conditions require it
  if (fanOn == 1 && vss < 40) {
    digitalWrite(radFan, HIGH);
  }

  // Turn off radiator fan if conditions allow it
  if ((batFanOn == 0 && motorCooling == 0 && acOn == 0) || vss > 42) {
    digitalWrite(radFan, LOW);
  }


  //heater off
  if (batHeating == 0 && cabinHeating == 0) {
    digitalWrite(heatPwm, LOW);
    //Serial.println("off");
  }


  if (millis() - lastCanSend > 50) {
    //Serial.println(acDuty);
    acDuty = acDuty * 10;
    // Extract the least significant byte
    duty1 = acDuty & 0xFF;
    // Extract the most significant byte
    duty2 = (acDuty >> 8) & 0xFF;

    // Extract the least significant byte
    power1 = acPower & 0xFF;
    // Extract the most significant byte
    power2 = (acPower >> 8) & 0xFF;

    unsigned char stmp[8] = {duty1, duty2, power1, power2, 0, acOn, 0, 0}; //
    CAN.sendMsgBuf(0x28A, 0, 8, stmp);
    lastCanSend = millis();

    //  Serial.println(duty1, HEX);
    //Serial.println(duty2, HEX);
    // Serial.println(power1, HEX);
    // Serial.println(power2, HEX);
    // Serial.println(batpumpPct);
  }

  //Serial.println(batpumpPct);
}


/*
* Function: shutdownSeq
* Description: Performs a shutdown sequence by turning off various components
*              and resetting control variables to default values.
*/
void shutdownSeq() {
  // Turn off various components
  digitalWrite(batPumpPwm, LOW); // Battery pump PWM
  digitalWrite(heatPwm, LOW); // Heater PWM
  digitalWrite(throtVal, LOW); // Throttle valve
  digitalWrite(invPumpPwm, LOW); // Inverter pump PWM
  digitalWrite(radFan, LOW); // Radiator fan
  // digitalWrite(epkb1, LOW); // Electropneumatic brake 1
  // digitalWrite(epkb2, LOW); // Electropneumatic brake 2
  digitalWrite(absVss, LOW); // ABS vehicle speed sensor
  digitalWrite(horn, LOW); // Horn
  digitalWrite(highbeam, LOW); // High beam headlights

  // Reset control variables to default values
  highbeamOn = 0; // High beam headlights status
  hornOn = 0; // Horn status
  //gear = 0; // Default gear to park

  pumpTimeRequest = 0; // Pump time request
  invpumpPct = 0; // Inverter pump percentage
  startTime = 0; // Start time
  sigOn = 0; // Signal status

  pumpTimeRequest2 = 0; // Second pump time request
  batpumpPct = 0; // Battery pump percentage
  startTime2 = 0; // Second start time
  sigOn2 = 0; // Second signal status

  acRequest = 0; // Air conditioning request
  heatReq = 0; // Heater request
  blowerSpeed = 0; // Blower speed
  acOn = 0; // Air conditioning status
  acDuty = 0; // Air conditioning duty cycle
  batCooling = 0; // Battery cooling status
  invTemp = 30; // Inverter temperature (default to off)
  motTemp = 30; // Motor temperature (default to off)
  motorCooling = 0; // Motor cooling status
  batHeating = 0; // Battery heating status
  cabinHeating = 0; // Cabin heating status
  rawClt = 0; // Raw coolant temperature sensor value
  cltRes = 0; // Coolant resistance
  clt = 0; // Coolant temperature
  fanByte = 0; // Fan speed byte
  highTemp = 35; // High temperature threshold (default to on)
  lowTemp = 20; // Low temperature threshold
  acPower = 6000; // Air conditioning power consumption
  lastCanSend = 0; // Last CAN message send time
  duty1 = 0; // Duty cycle byte 1
  duty2 = 0; // Duty cycle byte 2
  power1 = 0; // Power byte 1
  power2 = 0; // Power byte 2
  acPressVal = 0; // Air conditioning pressure value
  lastHeater = 0; // Last heater operation time
  batCoolPumpOn = 0; // Battery cooling pump status
  batFanOn = 0; // Battery fan status
  passiveSetPoint = 30; // Passive cooling setpoint
  fanSetPoint = 50; // Fan-assisted cooling setpoint
  fanOffSetPoint = 47; // Fan off setpoint
  passiveOffSetPoint = 28; // Passive cooling off setpoint
  fanOn = 0; // Fan status
  lockout10 = 0; // Lockout status for fast cooling
  fastCoolTimer = 0; // Fast cooling timer
}

