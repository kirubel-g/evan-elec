
#include <SPI.h>
#include <mcp_can.h>
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
int keyState = 2;
int mainRelayCmd = 0;
int maxDiscRaw = 0;
int maxChrgRaw = 0;
int gearLvrPos = 3;
int accelRaw = 0;
float accelPct = 0;


int preChargeStart = 0;
unsigned long contactorTimer = 0;
unsigned long previousCanMotor1 = 10;  //offset from powersteering commands



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


}

void loop() {

  power();

  if (igState > 0) {
    steering();
  }

  hvac();

  lights();


 // motah();

  preChargeLoop();


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
  } else {
    digitalWrite(revSig, LOW);
  }


  //Serial.println(rawGear);
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


  if (analogRead(igSw2) > 200 && pushCount == 1) {  //button was not held down, so now it knows no manual starting
    pushCount = 3;
  }



  if (analogRead(igSw2) < 200 && pushCount == 3 && analogRead(brake) > 300) {  //ig2 on

    ignite = 1; //Ignition on
    digitalWrite(motPow, HIGH);
    digitalWrite(dashPow, HIGH);
    digitalWrite(accPow, HIGH);
    pushCount = 5;// ig2 running
    igState = 2;

  }
  if ((analogRead(igSw2) > 200) && pushCount == 5) {  //when button is released after cranking stops, prepare for shutdown
    pushCount = 9;
    Serial.println("why");
  }




  // system state is now either auto start or ig2

  if ((analogRead(igSw2) > 200 && pushCount == 3 && analogRead(brake) < 200) || (analogRead(igSw2) < 200 && pushCount == 9 && analogRead(brake) < 200)) {  //auto start on

    ignite = 1; //Ignition on
    digitalWrite(motPow, HIGH);
    digitalWrite(dashPow, HIGH);
    digitalWrite(accPow, HIGH);
    pushCount = 6;// vehicle running
    igState = 3;

  }


  if ((analogRead(igSw2) > 200) && pushCount == 6) {  //when button is released after cranking stops, prepare for shutdown
    pushCount = 7;
  }




  if ((analogRead(igSw1) < 200 || analogRead(igSw2) < 200) && pushCount == 7) {  //ig off
    pushCount = 8;
    digitalWrite(motPow, LOW);
    digitalWrite(accPow, LOW);
    digitalWrite(dashPow, LOW);
    ignite = 0;
    igState = 0;
    //genericFlag2 = 0; // allow for the turn signals and such to be shut off when key off
  }


  if (analogRead(igSw2) < 200 && pushCount == 9  && analogRead(brake) > 300) {  //ig2 off
    pushCount = 8;
    digitalWrite(motPow, LOW);
    digitalWrite(accPow, LOW);
    digitalWrite(dashPow, LOW);
    ignite = 0;
    igState = 0;
    //genericFlag2 = 0; // allow for the turn signals and such to be shut off when key off
  }

  if ((analogRead(igSw2) > 200) && pushCount == 8) {  //when button is released after cranking stops, prepare for shutdown
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
  if (igState != 3 && preChargeStart <= 5 && preChargeStart > 0) { //if enigne not running and precharged or precharging
    contactorTimer = millis();
    preChargeStart = 6;
    prechrgFinished = 0;
    // digitalWrite(precharge, LOW);
  }

  if (millis() - contactorTimer > 1500 && preChargeStart == 6) { //if enigne running
    digitalWrite(contactorPos, LOW);
    digitalWrite(precharge, LOW);
    preChargeStart = 7;
  }

  if (millis() - contactorTimer > 3000 && preChargeStart == 7) { //if enigne running
    digitalWrite(contactorNeg, LOW);
    preChargeStart = 8;
  }

  if (millis() - contactorTimer > 4000 && preChargeStart == 8) { //delay till another precharge can begin

    preChargeStart = 0;
  }


}


void motah() {

  if (igState == 3) {
    mainRelayCmd = 1;
    vehicleState = 1;
  } else {
    mainRelayCmd = 0;
    vehicleState = 0;
  }



  if (igState >= 2 && prechrgFinished == 1 && millis() - previousCanMotor1 > 20) {
   // Serial.println("working");

    //if (prechrgFinished == 1) {
      unsigned char stmp[8] = {0, 0, 0, 0x10, 0, 0, 0, 0}; //setup precharge finished command
      CAN.sendMsgBuf(0x1A0, 0, 8, stmp);
   // }


    maxDiscRaw = (maxDisc + 500) * 10;
    maxChrgRaw = (maxChrg + 500) * 10;

    unsigned char stmp1[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //setup max charge + discharge command
    stmp1[0] = static_cast<char>((maxDiscRaw >> 8) & 0xFF);
    stmp1[1] = static_cast<char>(maxDiscRaw & 0xFF);
    stmp1[2] = static_cast<char>((maxChrgRaw >> 8) & 0xFF);
    stmp1[3] = static_cast<char>(maxChrgRaw & 0xFF);
    CAN.sendMsgBuf(0x1A1, 0, 8, stmp1);



    //aux relay command
    unsigned char stmp2[8] = {0, 0, 0, 0, 0, 0, 0x10, 0}; //setup ID 101, aux relay active
    // take first 3 bits (0x07) of number and put them in locations 4,5,6
    stmp2[4] |= static_cast<char>((gearLvrPos & 0x07) << 4);  //|= if either things bits are set, set the bits in the results
    // take first 1 bits (0x01) of number and put them in locations 0
    stmp2[4] |= static_cast<char>((vehicleState & 0x01) << 0);
    stmp2[4] |= static_cast<char>((mainRelayCmd & 0x03) << 3);
    //key position
    stmp2[5] |= static_cast<char>((keyState & 0x02) << 6);



    //accel pedal
    accelRaw = round(accelPct / 0.392);
    stmp2[0] = static_cast<char>(accelRaw & 0xFF);
    CAN.sendMsgBuf(0x101, 0, 8, stmp2);






    previousCanMotor1 = millis();
  }





}
