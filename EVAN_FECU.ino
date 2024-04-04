#include <SPI.h>
#include <mcp_can.h>
#include <PID_v1.h>
const int spiCSPinFCAN = 53;

unsigned long previousCanSend = 0;

//int
int r7 = 42;  //d
int heatTemp = A4;
int acPress = A5;
int brakeWarn = A2;  //d
int absLight = A1;  //d

int mainIgPow = 6;  //d
int batPumpPwm = 10;  //d
int heatPwm = 8;
int throtVal = 11;
int invPumpPwm = 12;  //d
int radFan = 7;
int epkbPow = 44; //d
int epkb1 = 49;  //d
int epkb2 = 47;  //d
int absVss = 9;  //d
int headlamp = 4;  //d
int leftTurn = 46;  //d
int rightTurn = 45;  //d
int horn = 43; //d
int highbeam = 5;  //d


//power
int igState = 0;
int lockout2 = 0;
unsigned long epkbTimer2 = 0;

//lights
int leftBlink = 0;
int rightBlink = 0;
int headlampOn = 0;
int highbeamOn = 0;
int hornOn = 0;

//pumps
float pumpTimeRequest = 0;
float invpumpPct = 0;
unsigned long startTime = 0;
int sigOn = 0;

float pumpTimeRequest2 = 0;
float batpumpPct = 0;
unsigned long startTime2 = 0;
int sigOn2 = 0;


//epkb
int gear = 0;
int applied1 = 0;
int applied2 = 0;
unsigned long epkbTimer = 0;
unsigned long soundTimer = 0;
int pkbApplied = 0;

//abs
float absFreq = 0;
//float vss = 38;
int brakeWarnFlag = 0;
int absWarnFlag = 0;
uint16_t motorSpeed = 0;
float vss = 0;
uint16_t vssRaw = 0;

//TEMPCTRL
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int acRequest = 0;
int heatReq = 0;
float blowerSpeed = 0;
int acOn = 0;
uint16_t  acDuty = 0;
int batCooling = 0;
int invTemp = 30;  //default to inverter cooling off
int motTemp = 30;
int motorCooling = 0;
int batHeating = 0;
int cabinHeating = 0;
int rawClt = 0;
float cltRes = 0;
float clt = 0;
int fanByte = 0;
int highTemp = 35; // default to battery water cooling on
int lowTemp = 20;
uint16_t acPower = 6000; // ac uses up to 6kw by default
unsigned long lastCanSend = 0;
byte duty1 = 0;
byte duty2 = 0;
byte power1 = 0;
byte power2 = 0;
int acPressVal = 0;
unsigned long lastHeater = 0;
float batpumpPctSend = 0;
float invpumpPctSend = 0;
int acHighTripOff = 0;
int passiveSetPoint = 30;
int fanSetPoint = 50;
int fanOffSetPoint = 47;
int passiveOffSetPoint = 28;
int batCoolPumpOn = 0;
int batFanOn = 0;
int fanOn = 0;
int lockout10 = 0;
unsigned long fastCoolTimer = 0;

int chargeOn = 0x00;
int lockout1 = 0;
unsigned long FECUTimer = 0;



MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's


/*
* Function: Setup
* Description: This function will go through an initialize all the key variables.
*               It is going to set all the appropriate output pins to connect them 
*               for PWM. In addition, it is going to create the necessary CANbus communication.
*/
void setup() {

  // The following section csets the respective pins as output to control GPIO or PWM
  pinMode(r7, OUTPUT);
  digitalWrite(r7, LOW);
  digitalWrite(r7, HIGH); //turn on ECU
  Serial.begin(9600); //start talking to PC
  Serial3.begin(9600); //start talking to Front Ped Alert Sys.

  //pinModesp
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

  //digitalWrites

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

  //digitalWrite(epkb2, HIGH);

  myPID.SetMode(AUTOMATIC);


  // Open a CAN message and initialize the CAN bus
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");

}


/*
* Function: loop
* Description: This function iterate through the different functionalities 
*               of the van such as the thermal control, ignition status, pumps, and lights.
*/
void loop() {

  if (chargeOn == 0x01) {
    digitalWrite(mainIgPow, HIGH);  //allow cooling system power if charging
  }

  if (igState > 1 || chargeOn == 0x01) { //if key on or charging
    thermalControl();  //allow cooling system to calculate

    //default cooling
    if (igState > 1) {
      invpumpPct = 85;
      //batpumpPct = 85;
    } else {
      // batpumpPct = 0;
      invpumpPct = 0;
    }

    pumps(); // allow pumps to run
    lockout1 = 0;
  } else if (lockout1 == 0) {
    lockout1 = 1; //prevents looping
    FECUTimer = millis(); // sets timer for ECU shutdown
    shutdownSeq();
  } else if (headlampOn == 1 || leftBlink == 1) {  //once we have shutdown, come here if we have a headlamp (autohome) or hazzard command
    FECUTimer = millis();  // reset timer continiously if hazzards are on or headlamps are on.
  }


  if (igState <= 1 && chargeOn == 0x00) { // only shutoff if not plugged in otherwise we stay awake
    if (millis() - FECUTimer > 120000) {  //delay to leave FECU after no lighting commands recieved and key off
      digitalWrite(r7, LOW); //it'll reboot when you step on the brake again so no worries if it shuts off during ig1 or unplugging vehicle
    }
  } else {
    digitalWrite(r7, HIGH);
  }



  ABS(); // Call the automatic braking system.

  //thermalControl();

  //CAN RECIEVE on ID 71
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  // Check for a CAN bus message and extract the can id bits
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
  }


  // Check for the CAN frame message associated with the ignition state, power, gears, aacReqquest, and fan byte
  if (canId == 0x102) {
    igState = buf[1];

    power();

    gear = buf[4];
    gears();

    hornOn = buf[5];
    if (hornOn == 1) {  //put here so that the horn blows as soon as command recieved
      digitalWrite(horn, HIGH);
    } else {
      digitalWrite(horn, LOW);
    }

    acRequest = buf[0];
    fanByte = buf[6];
    heatReq = buf[7];


  }

  // Check for the CAN ID for the left, right, headlamp, and high beam On
  if (canId == 0x103) {
    leftBlink = buf[0];
    rightBlink = buf[1];
    headlampOn = buf[3];
    highbeamOn = buf[4];
    lights();
  }

  // Check for the CAN ID for the brake applied
  if (canId == 0x230) {
    pkbApplied = buf[1];
    // Serial.println(buf[0]);
  }

  // Check for the CAN ID to get the current motor speed
  if (canId == 0x105) {
    motorSpeed = (buf[0] << 8) | buf[1];
    motorSpeed = motorSpeed * 0.25;
    vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828);
  }

  // Check for CAN ID for high/low temp and charge ON
  if (canId == 0x6B1) {
    highTemp = buf[0] - 40; // -40 from all these temps for realvalue
    lowTemp = buf[1] - 40;
    chargeOn = buf[6];
  }

  // Check for the motor and inverter temperature
  if (canId == 0x106) {
    motTemp = buf[0] - 40;
    invTemp = buf[1] - 40;
  }


  if (canId == 0x223) {
    //  Serial.println(buf[5]);
    //  Serial.println(buf[6]);
    // Serial.println(buf[7]);
  }


  //CAN SEND on ID 72
  if (millis() - previousCanSend > 300) {
    unsigned char stmp[8] = {absWarnFlag, brakeWarnFlag, 0, 0, 0, 0, 0, 0}; //
    CAN.sendMsgBuf(0x108, 0, 8, stmp);
    previousCanSend = millis();
  }



}

/*
* Function: loop
* Description: This function checks for the igintiion state and powers on the mainIginition power and 
*              epkbPow power.
*/
void power() {
  // Serial.println(igState);
  if (igState > 1) {
    digitalWrite(epkbPow, HIGH);
    digitalWrite(mainIgPow, HIGH);
    lockout2 = 0;
    //    Serial.println("ON");
    // Serial.println(igState);
  }


  // Handle the case when the ignition state is less than 1
  if (igState <= 1) {

    if (lockout2 == 0) {
      epkbTimer2 = millis();
      lockout2 = 1;
      Serial.println("on");
    }

    // For every 15 seconds set the epkbpower to low and lockout to 2.
    // Set the inverter and motor temp to 20F 
    if (millis() - epkbTimer2 >= 15000 && lockout2 == 1) {
      digitalWrite(epkbPow, LOW);
      lockout2 = 2;
      Serial.println("off");
      //put something here to set inverter cooling command to 0 because if plugged in and inverter off, it could keep on running trying to cool inverter
      invTemp = 20;
      motTemp = 20;

    }
    if (chargeOn == 0x00) {
      digitalWrite(mainIgPow, LOW);
    }
  }

}


/*
* Function: loop
* Description: This function checks for the igintiion state and powers on the mainIginition power and 
*              epkbPow power.
*/
void lights() {
  
  //Turn ON the left blink light or turn it off
  if (leftBlink == 1) {
    digitalWrite(leftTurn, HIGH);
  } else {
    digitalWrite(leftTurn, LOW);
  }

  // Turn on the right blink light or turn it off
  if (rightBlink == 1) {
    digitalWrite(rightTurn, HIGH);
  } else {
    digitalWrite(rightTurn, LOW);
  }

  // Turn on the high beam light or turn it off
  if (highbeamOn == 1) {
    digitalWrite(highbeam, HIGH);
  } else {
    digitalWrite(highbeam, LOW);
  }

  // Turn on the head lamp ON or turn if off
  if (headlampOn == 1) {
    digitalWrite(headlamp, HIGH);
    // Serial3.print("#2\n");
  } else {
    digitalWrite(headlamp, LOW);
  }

}

/*
* Function: pumps
* Description: This function sets the PWM output for the inverter pump and the battery pump
*/
void pumps() {

  //Need logic to determine desired pump duty cycle %

  //inverter pump
  //invpumpPct = 0;
  // set the input limit 
  invpumpPctSend = map(invpumpPct, 0, 100, 20, 80);
  if (invpumpPct == 0) { // if no pump command, set duty to 10% which is off.  20-80 is run
    invpumpPctSend = 10;
  }


  //invpumpPctSend = 10; //10% is stop, 20-80 is run
  // Set the pump PWM duty cycle
  pumpTimeRequest = (invpumpPctSend / 100) * 500;

  // Serial.println(millis()- startTime);

  // Turn on the inverter pump within the ON time
  if (millis() - startTime < pumpTimeRequest && sigOn == 0) {
    digitalWrite(invPumpPwm, HIGH);
    startTime = millis();
    sigOn = 1;
    // Serial.println("high");
  }


  // Turn it off
  if (millis() - startTime > pumpTimeRequest) {
    digitalWrite(invPumpPwm, LOW);
    // Serial.println("low");
  }

  if (millis() - startTime > 500) { //set carrier freq.
    startTime = millis();
    sigOn = 0;

  }


  //battery pump
  // Create a range for the period and duty cycle of the PWM
  batpumpPctSend = map(batpumpPct, 0, 100, 20, 80);
  if (batpumpPct == 0) { // if no pump command, set duty to 10% which is off.  20-80 is run
    batpumpPctSend = 10;
  }
  //Serial.println(batpumpPctSend);
  // batpumpPctSend = 80; //10% is stop, 20-80 is run

  pumpTimeRequest2 = (batpumpPctSend / 100) * 500;

  // Turn on the pump within the ON period of the duty cycle
  if (millis() - startTime2 < pumpTimeRequest2 && sigOn2 == 0) {
    digitalWrite(batPumpPwm, HIGH);
    startTime2 = millis();
    sigOn2 = 1;
  }

  // Turn off the pump outside of the ON cycle
  if (millis() - startTime2 > pumpTimeRequest2) {
    digitalWrite(batPumpPwm, LOW);
  }

  if (millis() - startTime2 > 500) { //set carrier freq.
    startTime2 = millis();
    sigOn2 = 0;

  }

}

/*
* Function: gears
* Description: This function sets locks in the gear of when the stop and release
*/
void gears() {

  // Stop the release command for the brake
  if (gear == 0 && applied1 == 0) {
    applied2 = 0;
    unsigned char stmp2[8] = {0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //stop releasing command
    CAN.sendMsgBuf(0x254, 0, 8, stmp2);
    digitalWrite(epkb1, HIGH);
    digitalWrite(epkb2, LOW);
    epkbTimer = millis();
    applied1 = 1;
    Serial3.print("q"); //silence sounds
  }

  // Release command for the brake
  if (gear != 0 && applied2 == 0) {
    applied1 = 0;
    digitalWrite(epkb1, LOW);
    digitalWrite(epkb2, HIGH);
    unsigned char stmp[8] = {0x03, 0xAE, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00}; //release command (releases slow w/o this)
    CAN.sendMsgBuf(0x254, 0, 8, stmp);
    epkbTimer = millis();
    applied2 = 1;
  }

  // Set the epkb1 power to low
  if (millis() - epkbTimer > 10000) {  //increase this
    digitalWrite(epkb1, LOW);
    digitalWrite(epkb2, LOW);
  }


  if (applied2 == 1 && millis() - soundTimer > 2000) { //if not in park play sounds
    Serial3.print("#0\n");
    soundTimer = millis();
  }



}


/*
* Function: ABS
* Description: This function is to setup the automatic braking system. It reads the 
*               brake warn pin and sets the appropriate brake warn flag. It reads the 
*              the abs light and also sets the necessary brake warn flag.
*/
void ABS() {
  //Serial.println(analogRead(absLight)); //270ish is off 670 is on
  //Serial.println(vss);
  //vss to pulses
  absFreq = (vss * 128000) / 3600;

  tone(absVss, absFreq);
  //  Serial.println(absFreq);

  //Serial.println(pkbApplied);
  if (analogRead(brakeWarn) < 300 || pkbApplied != 0x00) {
    brakeWarnFlag = 1;
  } else
    brakeWarnFlag = 0;

  if (analogRead(absLight) < 400) {
    absWarnFlag = 1;
  } else
    absWarnFlag = 0;
}

/*
* Function: Thermal Control
* Description: This function is to manages the thermal control. It goes through 
*              and reads through the the different temperatures. Based off of the 
*              sensor reading, it returns the different modes.
*/
void thermalControl() {

  acPressVal = analogRead(acPress);

  if (millis() - fastCoolTimer > 60000) {
    acHighTripOff = 1;
  } else {
    acHighTripOff = 0;
  }

  if (acRequest == 1 && acPressVal < 300) {
    acOn = 1;
    fanOn = 1;

    if (lockout10 == 0) {
      fastCoolTimer = millis();
      lockout10 = 1;
    }
    // digitalWrite(radFan, HIGH);
    //Serial.println("a/c");

    if (fanByte != 0x0F) {  //if fan not off, set comp speed to whatever the hvac said blwoer speed is
      blowerSpeed = fanByte;
    } else {
      blowerSpeed = 0;
    }


    if (heatReq == 0) {

      acDuty = round((blowerSpeed / 7) * 100);  //may need to scale differently later

      if (acHighTripOff == 0) {  //cool down fast
        acDuty = 100;
      }

      //  Serial.println(acDuty);
    } else {
      // acDuty = 20; //a/c speed for defrost/defrost when the cabin heat is on, SHUT OFF BECAUSE HEATER IS WEAK
      acDuty = round((blowerSpeed / 7) * 100);  //may need to scale differently later, same as with heat not on except no cool down fast

    }


  } else if (acPressVal > 500) {
    acDuty = 0;  //if no one wants ac, shut off a/c
    acOn = 0;
    //acHighTripOff = 1;
  } else if (acRequest == 0) {
    acDuty = 0;  //if no one wants ac, shut off a/c
    acOn = 0;
    acHighTripOff = 0;
    lockout10 = 0;
  }


  //bat cool pump
  

  if (chargeOn == 0x01) {
    passiveSetPoint = 45;
    fanSetPoint = 45;
    fanOffSetPoint = 42;
    passiveOffSetPoint = 42;
  } else {
    passiveSetPoint = 30;
    fanSetPoint = 50;
    fanOffSetPoint = 47;
    passiveOffSetPoint = 28;
  }

  if (highTemp > passiveSetPoint) { //passive battery cooling target, 17 l/min target

    batpumpPct = 85;
    batCoolPumpOn = 1;

    // digitalWrite(radFan, HIGH);
    //    Serial.println("batCool");

    if (highTemp > fanSetPoint) {
      // digitalWrite(radFan, HIGH);  //pumps will be running in deafult state , so if we get hot just turn on the fans
      fanOn = 1;
      batFanOn = 1; // request fan priority

    } else if (highTemp < fanOffSetPoint) { //hysteresis
      // digitalWrite(radFan, LOW);
      batFanOn = 0;  //request fan off
    }

  } else if (highTemp < passiveOffSetPoint) { //hysteresis
    // digitalWrite(throtVal, LOW);
    batCoolPumpOn = 0;  //request fan off
    batFanOn = 0;
  }


  //inv cool pump
  // Serial.println(invTemp);
  //invTemp = 55;
  if ((invTemp > 50 || motTemp > 50) && igState >= 2) {  //inverter/motor cooling target, should be near battery target becuase coolant mixes battery should already be near this value
    // invpumpPct = 85;
    motorCooling = 1;
    // Serial.println("inv cooling");
    // digitalWrite(radFan, HIGH);
    fanOn = 1;
  } else if ((invTemp < 47 && motTemp < 47) || igState < 2) {
    //invpumpPct = 0;
    motorCooling = 0;
    // Serial.println(motorCooling);

  }
  //Serial.println(digitalRead(radFan));

  //bat heating
 // lowTemp = 3;
// Serial.println(highTemp);
  if (lowTemp < 5) {
    batHeating = 1;
    batpumpPct = 85;
    //Serial.println("heating");

  } else if (lowTemp > 7) {
    batHeating = 0;
  }

//Serial.println(heatReq);
  //cabin heating
  if (heatReq == 1) {
    cabinHeating = 1;
    if (batCoolPumpOn == 0 && batHeating == 0) {  //changed from || to &&
      batpumpPct = 30; //7 lpm target, might need to bump up if insufficicent flow rate
    }
  } else if (batCoolPumpOn == 0 && batHeating == 0) {
    batpumpPct = 0;
    cabinHeating = 0;
  } else {
    cabinHeating = 0;
  }



  if ((cabinHeating == 1 || batHeating == 1)  && millis() - lastHeater > 100) {
    //start PID control of heater at 50C
    Setpoint = 49;

    rawClt = analogRead(heatTemp);
    cltRes = (rawClt * 1.7595) / (4.68 - (rawClt * .00488759));  // ((analogread / 1023) * 5 * 360 ohm)/(5-(analogread/1023)*5)

    if (cltRes < 0) {
      cltRes = 60000;  //pullup resistor value isnt ideal, so this will allow for heater to turn on regardless.  Once resistance decreaes, fn becomes more accurate.
    }
    //Serial.println(rawClt);
    //using Steinhart formula
    //cltRes = 3000;
    //  Serial.println(rawClt);
    clt = cltRes / 10000; //R/Ro
    clt = log(clt); //ln(R/Ro)
    clt = (1.0 / 5500) * clt;  //1/B * ln(R/Ro)
    clt = (1.0 / 298.15) + clt; //1/To + rest of Eq.
    clt = 1.0 / clt; // invert
    clt = clt - 273.15;

    //Serial.println(rawClt);
    //Serial.println(clt);
    Input = clt;
    myPID.Compute();
    //Serial.println(clt);
    // Serial.println(Output);
    analogWrite(heatPwm, Output); //TEMP SHUTOFF FOR TESTING
    lastHeater = millis();
  }


  // rad fan on
  if (fanOn == 1 && vss < 40) {
    digitalWrite(radFan, HIGH);
  }


  //rad fan off
  if ((batFanOn == 0 && motorCooling == 0 && acOn == 0) || vss > 42) { //fan LOW
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


void shutdownSeq() {

  digitalWrite(batPumpPwm, LOW);
  digitalWrite(heatPwm, LOW);
  digitalWrite(throtVal, LOW);
  digitalWrite(invPumpPwm, LOW);
  digitalWrite(radFan, LOW);
  // digitalWrite(epkb1, LOW);
  // digitalWrite(epkb2, LOW);
  digitalWrite(absVss, LOW);
  digitalWrite(horn, LOW);
  digitalWrite(highbeam, LOW);

  highbeamOn = 0;
  hornOn = 0;
  //gear = 0; // default to park

  pumpTimeRequest = 0;
  invpumpPct = 0;
  startTime = 0;
  sigOn = 0;

  pumpTimeRequest2 = 0;
  batpumpPct = 0;
  startTime2 = 0;
  sigOn2 = 0;


  acRequest = 0;
  heatReq = 0;
  blowerSpeed = 0;
  acOn = 0;
  acDuty = 0;
  batCooling = 0;
  invTemp = 30;  //default to inverter cooling off
  motTemp = 30;
  motorCooling = 0;
  batHeating = 0;
  cabinHeating = 0;
  rawClt = 0;
  cltRes = 0;
  clt = 0;
  fanByte = 0;
  highTemp = 35; // default to battery water cooling on
  lowTemp = 20;
  acPower = 6000; // ac uses up to 6kw by default
  lastCanSend = 0;
  duty1 = 0;
  duty2 = 0;
  power1 = 0;
  power2 = 0;
  acPressVal = 0;
  lastHeater = 0;
  batCoolPumpOn = 0;
  batFanOn = 0;
  passiveSetPoint = 30;
  fanSetPoint = 50;
  fanOffSetPoint = 47;
  passiveOffSetPoint = 28;
  fanOn = 0;
  lockout10 = 0;
  fastCoolTimer = 0;


}
