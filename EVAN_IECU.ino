
#include <SPI.h>
#include <mcp_can.h>

#include <PID_v1.h>
const int spiCSPinFCAN = 35;
const int spiCSPinFCAN2 = 53;
const int spiCSPinBCAN = 37;

unsigned long previousCanSend = 0;
unsigned long previousCanSend2 = 0;

int horn = A1;
int hazRet = A4;
int auxSw1 = A5;
int auxSw2 = A6;
int igSw1 = A2;
int igSw2 = A3;
int app1 = A15;
int app2 = A14;
int brake = A13;
int lock = A10;
int unlock = A9;


int r7 = 42;
int accPow = 47;
int illumi = 49;
int motPow = 5;
int contactorPos = 45;
int contactorNeg = 43;
int precharge = 4;
int dashPow = 6;
int revSig = 46;


//Ignition
int pushCount = 0;
int keyOn = 0;
int security = 1;
int IECUshutdown = 0;
int genericFlag1 = 0;
int ignite = 0;
int holdECUOn = 0;
int igState = 0;

unsigned long pushTimer = 0;
unsigned long IECUtimer = 0;

//steering
int i = 0;
int j = 0;
int counter = 0;
unsigned char stmp202[8] = {0x30, 0x03, 0, 0x00, 0x00, 0, 0x19, 0x93};//TACHO
unsigned char stmp2170[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xFC};//STEERING/VSS SPEED
unsigned long lastSend = 0;

//climate
byte acByte = 0;
byte fanByte = 0;
byte tempByte = 0;

int acReqOff = 0;
int heatReq = 0;
int fan = 0;


//lights
int brakeCommand = 0;
int hazCommand = 0;
int hornCommand = 0;
int leftBlink = 0;
int rightBlink = 0;
int headlampOn = 0;
int highbeamOn = 0;
int parkLampsOn = 0;

//gears
int rawGear = 0;
int gear = 0;


//motor
int prechrgFinished = 0;
int maxDisc = 500;
int maxChrg = 500;
int vehicleState = 0;
int keyState = 0;
int mainRelayCmd = 1;
int maxDiscRaw = 0;
int maxChrgRaw = 0;
int gearLvrPos = 0;
int accelRaw = 0;
float accelPct = 0;
float accelPct2 = 0;
float regenPct = 0;
int workMode = 0;
int motorMode = 1;
int rollingCounter = 0;
int acCommand = 1;
int motorRotation = 3;
int motorDat1 = 0;


uint16_t motorSpeed = 0;
float vss = 0;


int preChargeStart = 0;
unsigned long contactorTimer = 0;
unsigned long previousCanMotor1 = 10;  //offset from powersteering commands
unsigned long timer = 0;


//accel pedal, brake pedal
int total4 = 0;
const int numReadings4 =  10;
int readings4[numReadings4];
int readIndex4 = 0;
//int averagefuel = 0;
unsigned long passedtime3 = 3;
float gasoline = 0;

int total5 = 0;
const int numReadings5 =  10;
int readings5[numReadings5];
int readIndex5 = 0;
float gasoline2 = 0;

int throttleError = 0;

uint16_t brakePos = 0;
int brakePosInt = 0;
float stopCommand = 0;
int lockout = 0;


//cruise
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
float cruiseTorqueCmd = 0;
int cruise1 = 0;
int cruise2 = 0;
int cruiseSet = 0;
int cruiseMainOn = 0;


MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's
MCP_CAN CAN2(spiCSPinFCAN2);  //start talking on FCAN2 which is motor 2
MCP_CAN CAN3(spiCSPinBCAN);  //start talking on BCAN which is HVAC


void setup() {
  pinMode(r7, OUTPUT);
  digitalWrite(r7, LOW);
  digitalWrite(r7, HIGH); //turn on ECU

  Serial.begin(9600); //start talking to PC
  Serial3.begin(9600); //start talking to RFID reciever

  //pinModes

  pinMode(accPow, OUTPUT);
  pinMode(illumi, OUTPUT);
  pinMode(motPow, OUTPUT);
  pinMode(contactorPos, OUTPUT);
  pinMode(contactorNeg, OUTPUT);
  pinMode(precharge, OUTPUT);
  pinMode(dashPow, OUTPUT);
  pinMode(revSig, OUTPUT);


  //digitalWrite

  digitalWrite(accPow, LOW);
  digitalWrite(illumi, LOW);
  digitalWrite(motPow, LOW);
  digitalWrite(contactorPos, LOW);
  digitalWrite(contactorNeg, LOW);
  digitalWrite(precharge, LOW);
  digitalWrite(dashPow, LOW);
  digitalWrite(revSig, LOW);

  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
  {
    Serial.println("FCAN BUS init Failed");
    delay(100);
  }
  Serial.println("FCAN BUS Shield Init OK!");



  while (CAN_OK != CAN2.begin(CAN_500KBPS, MCP_8MHz))
  {
    Serial.println("FCAN2 BUS init Failed");
    delay(100);
  }
  Serial.println("FCAN2 BUS Shield Init OK!");



  while (CAN_OK != CAN3.begin(CAN_125KBPS, MCP_8MHz))
  {
    Serial.println("BCAN BUS init Failed");
    delay(100);
  }
  Serial.println("BCAN BUS Shield Init OK!");

  myPID.SetMode(AUTOMATIC);


}

void loop() {

  power();

  if (igState > 0) {
    steering();
  }

  hvac();

  lights();


  motah();

  preChargeLoop();

  //cruise();



  //CAN RECIEVE on ID 71
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;



  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
    // Serial.println(canId);
  }

  if (canId == 0x199) {
    rawGear = buf[0];
    gears();

  }

  if (canId == 0x103) {
    leftBlink = buf[0];
    rightBlink = buf[1];
    headlampOn = buf[3];
    highbeamOn = buf[4];
    parkLampsOn = buf[2];

  }

  if (canId == 0x105) {
    motorSpeed = (buf[0] << 8) | buf[1];
    motorSpeed = motorSpeed * 0.25;
    vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828);  //motor speed * gearboxratio * reargear ratio * tire diameter in in. * pi / conversion factor

    motorDat1 = buf[5];
    if ((motorDat1 & 0b10000000)) {   //first bit of rotation direction
      motorRotation = 2;
    }

    if ((motorDat1 & 0b01000000)) {   //seccond bit of rotation direction
      motorRotation = 1;
    }

    if ((motorDat1 & 0b01000000) && (motorDat1 & 0b10000000)) {   //first+seccond bit of rotation direction=error
      motorRotation = 3;
    }


  }


  if (canId == 0xE7) {
    brakePos = (buf[0] << 8) | buf[1];
    brakePosInt = (brakePos * .01) - 62;

  }

  if (canId == 0x9D) {

    cruise1 = buf[0];
    if ((cruise1 & 0b00100000)) {
      cruiseSet = 1;
    }

    cruise2 = buf[1];
    if ((cruise1 & 0b01000000)) {
      cruiseMainOn = 1;
    } else {
      cruiseMainOn = 0;
      cruiseSet = 0;
    }


  }





  //CAN SEND on ID 102
  if (millis() - previousCanSend > 50) {
    unsigned char stmp[8] = {ignite, igState, brakeCommand, hazCommand, gear, hornCommand, 0, 0}; //
    CAN.sendMsgBuf(0x102, 0, 8, stmp);
    previousCanSend = millis();
  }



}

void gears() {
  if (rawGear == 0xC0) {
    gear = 0;  //Park
  }
  if (rawGear == 0x20) {
    gear = 1;  //Reverse
  }
  if (rawGear == 0x10) {
    gear = 2;  //Neut
  }
  if (rawGear == 0x08) {
    gear = 3;  //Drive
  }

  if (gear == 1) {
    digitalWrite(revSig, HIGH);
    gearLvrPos = 1;
  } else {
    digitalWrite(revSig, LOW);
    gearLvrPos = 3;
  }

}



void lights() {
  if (analogRead(brake) < 200) {
    brakeCommand = 1;
  } else {
    brakeCommand = 0;
  }
  //Serial.println(brakeCommand);
  if (analogRead(hazRet) < 200) {
    hazCommand = 1;
  } else {
    hazCommand = 0;
  }
  if (analogRead(horn) < 200) {
    hornCommand = 1;
  } else {
    hornCommand = 0;
  }
  //Serial.println(analogRead(brake));
  if (parkLampsOn == 1) {
    digitalWrite(illumi, HIGH);
  } else {
    digitalWrite(illumi, LOW);
  }


}



void hvac() {
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  if (CAN_MSGAVAIL == CAN3.checkReceive())
  {
    CAN3.readMsgBuf(&len, buf);
    canId = CAN3.getCanId();
    // Serial.println(canId);
  }

  if (canId == 251231057) {  //EF97B51 in hex
    acByte = buf[4];
    fanByte = buf[0];
    tempByte = buf[1];

  }
  if (acByte & 0b00010000) {
    acReqOff = 1;
  } else {
    acReqOff = 0;
  }

  if (fanByte == 0x0F) {
    fan = 0;
  } else {
    fan = 1;
  }

  if (tempByte == 0x04) {
    heatReq = 1;
  } else {
    heatReq = 0;
  }
}





void steering() {

  if (igState < 3) { //if not enigne running power steer off
    stmp202[0] = 0x00;
  }

  if (igState == 3) { //if enigne running power steer on
    stmp202[0] = 0x30;
  }


  if ((millis() - lastSend > 20)) {


    CAN.sendMsgBuf(0x202, 0, 8, stmp202);
    CAN.sendMsgBuf(0x217, 0, 8, stmp2170);


    lastSend = millis();


    i = stmp2170[6];
    i = i + 1;
    j = stmp2170[7];
    j = j - 1;


    if (stmp2170[6] == 223) {
      stmp2170[6] = 0xD0;
      stmp2170[7] = 0xFC;
      i = stmp2170[6];
      j = stmp2170[7];

    }


    stmp2170[6] = i;
    stmp2170[7] = j;


  }

}




void power() {
  // Serial.println(pushCount);

  if ((analogRead(igSw1) < 200 || analogRead(igSw2) < 200) && pushCount == 0 && ignite == 0 && security == 1) {   //turn on if key is not being used

    if ((analogRead(igSw1) < 200 && analogRead(igSw2) > 200) || (analogRead(igSw1) > 200 && analogRead(igSw2) < 200) ) {
      Serial.println("Possible IG switch failure, invalid key switch signal detected");
      //Serial.println(analogRead(igSw1));
      Serial.println(analogRead(igSw2));
    }

    ignite = 1; //some ignition on atleast
    IECUshutdown = 0; // prevent board from shutting itself off with car on
    pushCount = 1;
    genericFlag1 = 0; //prevent the lcm and scm from going to sleep when key is cycled back on after off
    pushTimer = millis();

    digitalWrite(accPow, HIGH);// ig1 power
    igState = 1;
    Serial.println("ig1");

  }

  /*

    if ((analogRead(igSw1) < 200 || analogRead(igSw2) < 200) && pushCount == 1 && millis() - pushTimer > 3000) { //manual start loop (no brake req.)
      //for permit start on single igntion switch circuit or failed brake switch

      ignite = 1; //Ignition on, straight to IgOn
      digitalWrite(motPow, HIGH);
      digitalWrite(dashPow, HIGH);
      digitalWrite(accPow, HIGH);
      pushCount = 6;// vehicle running
      igState = 3;
    }

  */


  if ((analogRead(igSw2) > 200 && analogRead(igSw1) > 200)  && pushCount == 1) {  //button was not held down, so now it knows no manual starting
    pushCount = 3;
  }



  if ((analogRead(igSw2) < 200 || analogRead(igSw1) < 200)  && pushCount == 3 && analogRead(brake) > 300) {  //ig2 on

    ignite = 1; //Ignition on
    digitalWrite(motPow, HIGH);
    digitalWrite(dashPow, HIGH);
    digitalWrite(accPow, HIGH);
    pushCount = 5;// ig2 running
    igState = 2;

  }
  if ((analogRead(igSw2) > 200 && analogRead(igSw1) > 200) && pushCount == 5) {  //when button is released after cranking stops, prepare for shutdown
    pushCount = 9;
    Serial.println("why");
  }




  // system state is now either auto start or ig2

  if (((analogRead(igSw2) > 200 && analogRead(igSw1) > 200) && pushCount == 3 && analogRead(brake) < 200) || ((analogRead(igSw2) < 200 || analogRead(igSw1) < 200)  && pushCount == 9 && analogRead(brake) < 200)) {  //auto start on

    ignite = 1; //Ignition on
    digitalWrite(motPow, HIGH);
    digitalWrite(dashPow, HIGH);
    digitalWrite(accPow, HIGH);
    pushCount = 6;// vehicle running
    igState = 3;

  }


  if ((analogRead(igSw2) > 200 && analogRead(igSw1) > 200) && pushCount == 6) {  //when button is released after cranking stops, prepare for shutdown
    pushCount = 7;
  }




  if ((analogRead(igSw2) < 200 || analogRead(igSw1) < 200) && pushCount == 7) {  //ig off
    pushCount = 8;
    digitalWrite(motPow, LOW);
    digitalWrite(accPow, LOW);
    digitalWrite(dashPow, LOW);
    ignite = 0;
    igState = 0;
    //genericFlag2 = 0; // allow for the turn signals and such to be shut off when key off
  }


  if ((analogRead(igSw2) < 200 || analogRead(igSw1) < 200)  && pushCount == 9  && analogRead(brake) > 300) {  //ig2 off
    pushCount = 8;
    digitalWrite(motPow, LOW);
    digitalWrite(accPow, LOW);
    digitalWrite(dashPow, LOW);
    ignite = 0;
    igState = 0;
    //genericFlag2 = 0; // allow for the turn signals and such to be shut off when key off
  }

  if ((analogRead(igSw2) > 200 && analogRead(igSw1) > 200) && pushCount == 8) {  //when button is released after cranking stops, prepare for shutdown
    pushCount = 0;
  }



  if (ignite == 0 && genericFlag1 == 0) { //shutdown LCMs timer
    IECUtimer = millis();
    genericFlag1 = 1;
  }
  if (ignite == 0 && genericFlag1 == 1 && millis() - IECUtimer > 60000 && holdECUOn == 0) { //shutdown LCMs timer after 5 mins
    digitalWrite(r7, LOW);
    IECUshutdown = 1;
  }


}


void preChargeLoop() {



  //PRECHARGE
  if (igState == 3 && preChargeStart == 0) { //if enigne running
    contactorTimer = millis();
    preChargeStart = 1;
  }

  if (millis() - contactorTimer > 500 && preChargeStart == 1) { //if enigne running
    digitalWrite(contactorNeg, HIGH);
    preChargeStart = 2;
  }

  if (millis() - contactorTimer > 1000 && preChargeStart == 2) { //if enigne running
    digitalWrite(precharge, HIGH);
    preChargeStart = 3;
  }

  if (millis() - contactorTimer > 4000 && preChargeStart == 3) { //if enigne running
    digitalWrite(contactorPos, HIGH);
    preChargeStart = 4;
  }
  if (millis() - contactorTimer > 5000 && preChargeStart == 4) { //if enigne running
    digitalWrite(precharge, LOW);
    preChargeStart = 5;
    prechrgFinished = 1;
  }


  //DISCHARGE
  if (igState != 3 && preChargeStart <= 5 && preChargeStart > 0 && motorSpeed == 0) { //if enigne not running and precharged or precharging
    contactorTimer = millis();
    preChargeStart = 6;
    prechrgFinished = 0;
    // digitalWrite(precharge, LOW);
  }

  if (millis() - contactorTimer > 4000 && preChargeStart == 6) { //if enigne running
    digitalWrite(contactorPos, LOW);
    digitalWrite(precharge, LOW);
    preChargeStart = 7;
  }

  if (millis() - contactorTimer > 5500 && preChargeStart == 7) { //if enigne running
    digitalWrite(contactorNeg, LOW);
    preChargeStart = 8;
  }

  if (millis() - contactorTimer > 6500 && preChargeStart == 8) { //delay till another precharge can begin

    preChargeStart = 0;
  }


}


void motah() {

  if (igState == 3 && prechrgFinished == 1  && millis() - contactorTimer > 6000) {  //turn on READY after precharge done for 1 sec
    vehicleState = 1;
    keyState = 2;

    if (gear == 1 || gear == 3) {  //if ready to go AND in gear, allow throttle
      if (cruiseSet == 0) {
        pedals();
      }
      if (cruiseSet == 1) {

      }
    } else {
      accelPct = 0;
      regenPct = 0;
      accelPct2 = 0;
      regenPct = 0;
    }

  } else {
    vehicleState = 0;
    accelPct = 0; //if key not on, throttle = 0
    regenPct = 0;
    regenPct = 0;
    accelPct2 = 0;
    throttleError = 0; //reset throttle
    keyState = 0;



  }


  if (millis() - previousCanMotor1 > 8 && vehicleState == 1) {




    //if (prechrgFinished == 1) {
    unsigned char stmp[8] = {0, 0, 0, 0x14, 0, 0, 0, 0}; //setup precharge finished command and precharge relay open command
    CAN.sendMsgBuf(0x1A0, 0, 8, stmp);
    CAN2.sendMsgBuf(0x1A0, 0, 8, stmp);
    // }


    maxDiscRaw = (maxDisc + 500) * 10;
    maxChrgRaw = (maxChrg + 500) * 10;

    unsigned char stmp1[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //setup max charge + discharge command
    stmp1[0] = static_cast<char>((maxDiscRaw >> 8) & 0xFF);
    stmp1[1] = static_cast<char>(maxDiscRaw & 0xFF);
    stmp1[2] = static_cast<char>((maxChrgRaw >> 8) & 0xFF);
    stmp1[3] = static_cast<char>(maxChrgRaw & 0xFF);
    CAN.sendMsgBuf(0x1A1, 0, 8, stmp1);
    CAN2.sendMsgBuf(0x1A1, 0, 8, stmp1);



    //aux relay command
    unsigned char stmp2[8] = {0, 0, 0, 0, 0, 0, 0x10, 0}; //setup ID 101, aux relay active
    // take first 3 bits (0x07) of number and put them in locations 4,5,6
    stmp2[4] |= static_cast<char>((gearLvrPos & 0x07) << 4);  //|= if either things bits are set, set the bits in the results
    // take first 1 bits (0x01) of number and put them in locations 0
    stmp2[4] |= static_cast<char>((vehicleState & 0x01) << 0);
    stmp2[4] |= static_cast<char>((mainRelayCmd & 0x01) << 3);
    //key position
    stmp2[5] |= static_cast<char>((keyState & 0x03) << 6);
    stmp2[5] |= static_cast<char>((motorMode & 0x03) << 2);
    stmp2[5] |= static_cast<char>((workMode & 0x01) << 1);
    stmp2[5] |= static_cast<char>((acCommand & 0x01) << 0);
    // stmp2[5] |= static_cast<char>((workMode & 0x03) << 6);


    //accel pedal
    if (motorMode == 1) {
      accelRaw = round(accelPct / 0.392);
    }
    if (motorMode == 2) {
      accelRaw = round(regenPct / 0.392);
    }

    stmp2[0] = static_cast<char>(accelRaw & 0xFF);


    //Rolling Counter and Checksum

    rollingCounter++;
    if (rollingCounter == 16) {
      rollingCounter = 0;
    }

    stmp2[6] |= static_cast<char>((rollingCounter & 0x0F) << 0);

    char sum = stmp2[0] + stmp2[1] + stmp2[2] + stmp2[3] + stmp2[4] + stmp2[5] + stmp2[6];
    char checksum = sum ^ 0xFF;
    stmp2[7] = checksum;

    //Serial.println(motorMode);
    CAN.sendMsgBuf(0x101, 0, 8, stmp2);
    CAN2.sendMsgBuf(0x101, 0, 8, stmp2);


    previousCanMotor1 = millis();
  }



}


void pedals() {
  //.println(analogRead(app1));  //215 - 975
  //Serial.println(analogRead(app2));  //100-485

  if (millis() - passedtime3 > 12 && throttleError == 0) {  //accel pedal delay
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
      accelPct = 0; //disable Throttle
      accelPct2 = 0;
      throttleError = 1;
    }



    passedtime3 = millis();
    // Serial.println(abs(accelPct2 - accelPct));
  }


  if (accelPct == 0) {  //OFF throttle regen going Forward/Reverse

    if (lockout == 0) {
      motorMode = 2;
    }

    if (motorSpeed > 100 && ((motorRotation == 1 && gear == 3) || (motorRotation == 2 && gear == 1))) {  //if we are above 100rpms and motor is rotating same direction as requested gear, then do regen
      regenPct = 1;  //goes from 0-5 pct regen when you take foot off gas
    } else {
      regenPct = 0;
    }

  } else {
    motorMode = 1;  //this forces system to read accel pedal if on gas
    regenPct = 0;  //sets regen to 0 if on pedal
  }



  if (brakePosInt > 10) {  //progressive regen with brake pedal, will cause motor not to function if EBB goes offline during brake push
    stopCommand = map(brakePosInt, 20, 150, 5, 25); //20 mm to 150 mm of travel goes to 5-25 pct regen command
    stopCommand = constrain(stopCommand, 0, 25);
    //Serial.println("error");

    if (motorSpeed > 100 && ((motorRotation == 1 && gear == 3) || (motorRotation == 2 && gear == 1))) {  //if we are above 100rpms and motor is rotating same direction as requested gear, then do regen
      regenPct = stopCommand;  //overwrite off throttle regen command if brake pedal pressed
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




}



void cruise() {

  if (cruiseMainOn == 1 && vss > 20) {  //cruise on and vss > 20 mph, start calculating
    Input = vss;
    myPID.Compute();
    cruiseTorqueCmd = map(Output, 0, 255, 0, 100);
    //Serial.println(cruiseTorqueCmd);

  }

  if (cruiseSet == 1 && analogRead(brake) > 300 && cruiseMainOn == 1) {  //if cruise set and off the brake pedal
    accelPct = cruiseTorqueCmd;
    accelPct2 = cruiseTorqueCmd;
  }

  if (analogRead(brake) < 200) { //if step on brake, cruise off
    cruiseSet == 0;
  }


}
