
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
int invTemp = 60;  //default to inverter cooling on
int motTemp = 60;
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

int chargeOn = 0;
int lockout1 = 0;
unsigned long FECUTimer = 0;



MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's



void setup() {

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


  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");





}

void loop() {

  if (chargeOn == 0x01) {
    digitalWrite(mainIgPow, HIGH);  //allow cooling system power if charging
  }

  if (igState > 1 || chargeOn == 0x01) { //if key on or charging
    thermalControl();  //allow cooling system to calculate
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



  ABS();

  //thermalControl();

  //CAN RECIEVE on ID 71
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
  }


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

  if (canId == 0x103) {
    leftBlink = buf[0];
    rightBlink = buf[1];
    headlampOn = buf[3];
    highbeamOn = buf[4];
    lights();

  }

  if (canId == 0x230) {
    pkbApplied = buf[1];
    // Serial.println(buf[0]);
  }

  if (canId == 0x105) {
    motorSpeed = (buf[0] << 8) | buf[1];
    motorSpeed = motorSpeed * 0.25;
    vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828);
  }



  if (canId == 0x6B1) {
    highTemp = buf[0] - 40; // -40 from all these temps for realvalue
    lowTemp = buf[1] - 40;
    chargeOn = buf[6];
  }
  if (canId == 0x106) {
    motTemp = buf[0] - 40;
    invTemp = buf[1] - 40;
  }




  //CAN SEND on ID 72
  if (millis() - previousCanSend > 300) {
    unsigned char stmp[8] = {absWarnFlag, brakeWarnFlag, 0, 0, 0, 0, 0, 0}; //
    CAN.sendMsgBuf(0x108, 0, 8, stmp);
    previousCanSend = millis();
  }


}

void power() {
  // Serial.println(igState);
  if (igState > 1) {
    digitalWrite(epkbPow, HIGH);
    digitalWrite(mainIgPow, HIGH);
    //   Serial.println("ON");
    // Serial.println(igState);
  }

  if (igState <= 1) {
    digitalWrite(epkbPow, LOW);
    digitalWrite(mainIgPow, LOW);
    //  Serial.println("off");
  }

}

void lights() {
  //Serial.println(leftBlink);
  if (leftBlink == 1) {
    digitalWrite(leftTurn, HIGH);
  } else {
    digitalWrite(leftTurn, LOW);
  }

  if (rightBlink == 1) {
    digitalWrite(rightTurn, HIGH);
  } else {
    digitalWrite(rightTurn, LOW);
  }


  if (highbeamOn == 1) {
    digitalWrite(highbeam, HIGH);
  } else {
    digitalWrite(highbeam, LOW);
  }


  if (headlampOn == 1) {
    digitalWrite(headlamp, HIGH);
    // Serial3.print("#2\n");
  } else {
    digitalWrite(headlamp, LOW);
  }

}

void pumps() {

  //Need logic to determine desired pump duty cycle %

  //inverter pump
  invpumpPct = 10; //10% is stop, 20-80 is run
  pumpTimeRequest = (invpumpPct / 100) * 500;

  if (millis() - startTime < pumpTimeRequest && sigOn == 0) {
    digitalWrite(invPumpPwm, HIGH);
    startTime = millis();
    sigOn = 1;
  }

  if (millis() - startTime > pumpTimeRequest) {
    digitalWrite(invPumpPwm, LOW);
  }

  if (millis() - startTime > 500) { //set carrier freq.
    startTime = millis();
    sigOn = 0;

  }


  //battery pump

  batpumpPct = 10; //10% is stop, 20-80 is run
  pumpTimeRequest2 = (batpumpPct / 100) * 500;

  if (millis() - startTime2 < pumpTimeRequest2 && sigOn2 == 0) {
    digitalWrite(batPumpPwm, HIGH);
    startTime2 = millis();
    sigOn2 = 1;
  }

  if (millis() - startTime2 > pumpTimeRequest2) {
    digitalWrite(batPumpPwm, LOW);
  }

  if (millis() - startTime2 > 500) { //set carrier freq.
    startTime2 = millis();
    sigOn2 = 0;

  }

}

void gears() {
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
  if (gear != 0 && applied2 == 0) {
    applied1 = 0;
    digitalWrite(epkb1, LOW);
    digitalWrite(epkb2, HIGH);
    unsigned char stmp[8] = {0x03, 0xAE, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00}; //release command (releases slow w/o this)
    CAN.sendMsgBuf(0x254, 0, 8, stmp);
    epkbTimer = millis();
    applied2 = 1;
  }

  if (millis() - epkbTimer > 10000) {
    digitalWrite(epkb1, LOW);
    digitalWrite(epkb2, LOW);
  }

  if (applied2 == 1 && millis() - soundTimer > 2000) { //if not in park play sounds
    Serial3.print("#0\n");
    soundTimer = millis();
  }



}


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

void thermalControl() {



  acPressVal = analogRead(acPress);

  // acPressVal = 800; //testval
  // highTemp = 30;
  // lowTemp = 6;
  // invTemp = 60;
  //Serial.println(invTemp);

  if (acRequest == 1 && acPressVal < 300) {
    acOn = 1;
    digitalWrite(radFan, HIGH);

    if (fanByte != 0x0F) {  //if fan not off, set comp speed to whatever the hvac said blwoer speed is
      blowerSpeed = fanByte;
    } else {
      blowerSpeed = 0;
    }


    if (heatReq == 0) {
      acDuty = round((blowerSpeed / 7) * 100);  //may need to scale differently later
      //  Serial.println(acDuty);
    } else {
      acDuty = 20; //a/c speed for defrost/defrost + battery cooling when the cabin heat is on
    }



  } else if (highTemp > 49 && acPressVal < 300) {  //active battery cooling target,
    //turn on ac comp if not on already
    acDuty = 15;

  } else if (highTemp < 48 || acPressVal > 300) {  //hysteresis turned low to prevent getting stuck at high compressor duty from cabin cooling call, bms should have histeresis
    acDuty = 0;  //if no one wants ac, shut off a/c
    acOn = 0;
  }


  //bat cool pump
  if (highTemp > 30) { //passive battery cooling target, 17 l/min target
    batCooling = 1;
    batpumpPct = 74;
    digitalWrite(radFan, HIGH);
    //Serial.println("cooling");

    if (highTemp > 49) {
      digitalWrite(throtVal, HIGH); // can be analog

    } else if (highTemp < 48) { //hysteresis
      digitalWrite(throtVal, LOW);
    }

  } else if (highTemp < 28) { //hysteresis
    digitalWrite(throtVal, LOW);
    batCooling = 0;
  }


  //inv cool pump
  //  Serial.println(invTemp);
  if (invTemp > 45 || motTemp > 45) {  //inverter/motor cooling target, should be near battery target becuase coolant mixes battery should already be near this value
    invpumpPct = 70;
    motorCooling = 1;
    // Serial.println("cooling");
    digitalWrite(radFan, HIGH);
  } else if (invTemp < 43 && motTemp < 43) {
    invpumpPct = 0;
    motorCooling = 0;
    // Serial.println(motorCooling);

  }


  //bat heating
  if (lowTemp < 5) {
    batHeating = 1;
    batpumpPct = 74;
    //Serial.println("heating");

  } else if (lowTemp > 7) {
    batHeating = 0;
  }


  //cabin heating
  if (heatReq == 1) {
    cabinHeating = 1;
    if (batCooling == 0 || batHeating == 0) {
      batpumpPct = 30; //7 lpm target
    }
  } else if (batCooling == 0 && batHeating == 0) {
    batpumpPct = 0;
    cabinHeating = 0;
  } else {
    cabinHeating = 0;
  }



  if ((cabinHeating == 1 || batHeating == 1)  && millis() - lastHeater > 100) {
    //start PID control of heater at 50C
    Setpoint = 50;

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
    //Serial.println(Output);
    analogWrite(heatPwm, Output);
    lastHeater = millis();
  }

  //rad fan off
  if (batCooling == 0 && motorCooling == 0 && acOn == 0) { //fan LOW
    digitalWrite(radFan, LOW);
  }

  //heater off
  if (batHeating == 0 && cabinHeating == 0) {
    digitalWrite(heatPwm, LOW);
    //Serial.println("off");
  }


  if (millis() - lastCanSend > 50) {
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





}


void shutdownSeq() {

  digitalWrite(batPumpPwm, LOW);
  digitalWrite(heatPwm, LOW);
  digitalWrite(throtVal, LOW);
  digitalWrite(invPumpPwm, LOW);
  digitalWrite(radFan, LOW);
  digitalWrite(epkb1, LOW);
  digitalWrite(epkb2, LOW);
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
  invTemp = 60;  //default to inverter cooling on
  motTemp = 60;
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


}
