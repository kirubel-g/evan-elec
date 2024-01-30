
#include <SPI.h>
#include <mcp_can.h>
const int spiCSPinFCAN = 53;
const int spiCSPinBMSCAN = 37;

unsigned long previousCanSend = 0;
unsigned long previousCanSend2 = 0;

//int

int chargeDet = A1;

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

//power
int igState = 0;

//lamps
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
int offCommand = 0;
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

//wipers
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

unsigned long wiperTimer0 = 0;
unsigned long wiperTimer1 = -15000;
unsigned long wiperTimer2 = -10000;
unsigned long wiperTimer3 = -7000;
unsigned long wiperTimer4 = -5000;
unsigned long wiperTimer5 = -2500;

MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's
MCP_CAN CAN2(spiCSPinBMSCAN);  //start talking on BMSCAN



void setup() {

  pinMode(r7, OUTPUT);
  digitalWrite(r7, LOW);
  digitalWrite(r7, HIGH); //turn on ECU


  Serial.begin(9600); //start talking to PC

  //pinModes)

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

  //digitalWrites

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





  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
  {
    Serial.println("FCAN BUS init Failed");
    delay(100);
  }
  Serial.println("FCAN BUS Shield Init OK!");



  while (CAN_OK != CAN2.begin(CAN_125KBPS, MCP_8MHz))
  {
    Serial.println("BSM BUS init Failed");
    delay(100);
  }
  Serial.println("BSM BUS Shield Init OK!");


}

void loop() {

  blinkers();

  tailLamps();





  //CAN RECIEVE on ID 102
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;



  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
    //Serial.println(canId);
  }



  if (canId == 0x102) {
    igState = buf[1];
    power();
    brakeCommand = buf[2];
    hazCommand = buf[3];  //if message not recieved from SSU, then haz wont work
    gear = buf[4];  //if message not recieved from SSU, gear won't work because func wont be called

  }


  //Serial.println("loop");
  if (canId == 0x091) {
    lamps1 = buf[1];
    lamps2 = buf[0];
    wipers1 = buf[2];
    wipers2 = buf[6];
    lights();
    wipers();
  }





  //CAN SEND on ID 103 for turn signal blink
  if (millis() - previousCanSend > 30) {
    unsigned char stmp[8] = {leftFrontBlink, rightFrontBlink, parklight, headlight, highbeam, 0, 0, 0}; //
    CAN.sendMsgBuf(0x103, 0, 8, stmp);
    previousCanSend = millis();
  }


}


void power() {
  // Serial.println(igState);
  if (igState > 1) {   //if in IG2
    digitalWrite(bsmPow, HIGH);
    digitalWrite(bmsIg, HIGH);
    // Serial.println("ON");
    //Serial.println(igState);
  }

  if (igState <= 1) { //if not IG2
    digitalWrite(bsmPow, LOW);
    digitalWrite(bmsIg, LOW);
    // Serial.println("off");
  }

}


void lights() {
  // Serial.println(lamps1);
  if ((lamps1 & 0b00100000) || hazCommand == 1) {  //if left turn bit is set then...
    leftCommand = 1;
  } else {
    leftCommand = 0;
  }

  if ((lamps1 & 0b00010000) || hazCommand == 1) {  //if right turn bit is set then...
    rightCommand = 1;
  } else {
    rightCommand = 0;
  }

  if (lamps2 & 0b00000010) {  //if fog bit bit is set then...
    digitalWrite(taskLight, HIGH);
  } else {
    digitalWrite(taskLight, LOW);
  }




  if (lamps2 & 0b00100000) {  //if parklamp bit is set
    parkCommand = 1;
  } else {
    parkCommand = 0;
  }
  if (lamps2 & 0b00010000) {  //if headlamp bit is set
    headCommand = 1;
  } else {
    headCommand = 0;
  }
  if (lamps1 & 0b01000000) {  //if offlamp bit is set
    offCommand = 1;
  } else {
    offCommand = 0;
  }

  
  if (lamps1 & 0b10000000) {  //if flash2pass bit is set
    f2p = 1;
  } else {
    f2p = 0;
  }
  
 if (lamps2 & 0b00001000) {  //if highbeam bit is set
    highbeamCommand = 1;
  } else {
    highbeamCommand = 0;
  }


  if (gear == 1) { // if in reverse turn on reverse light
    digitalWrite(revLight, HIGH);
  } else {
    digitalWrite(revLight, LOW);
  }


}


void blinkers()

{
  if (brakeCommand == 1) {
    brakeOn = 1;
    digitalWrite(chmsl, HIGH);
  }


  if (brakeCommand == 0) {
    brakeOn = 0;
    digitalWrite(chmsl, LOW);
  }

  if (millis() - blinktime1 > 400 && blinking1 == 1) {  //rear left blinking off

    digitalWrite(leftTurn, LOW);
    leftFrontBlink = 0;

    // dashLeft = 0;
    blinking1 = 0;
    previousBlink1 = millis();
  }

  if (leftCommand == 1 && blinking1 == 0 && millis() - previousBlink1 > 400) {  //rear left blink on

    digitalWrite(leftTurn, HIGH);
    leftFrontBlink = 1;

    //dashLeft = 1;
    blinktime1 = millis();
    blinking1 = 1;
  }

  if (millis() - blinktime2 > 400 && blinking2 == 1) {  //rear right blinking off

    digitalWrite(rightTurn, LOW);
    rightFrontBlink = 0;

    //dashRight = 0;
    blinking2 = 0;
    previousBlink2 = millis();
  }

  if (rightCommand == 1 && blinking2 == 0 && millis() - previousBlink2 > 400) {  //rear right blink on

    digitalWrite(rightTurn, HIGH);
    rightFrontBlink = 1;

    // dashRight = 1;
    blinktime2 = millis();
    blinking2 = 1;
  }

  if (rightCommand == 0 && brakeOn == 1 && blinking2 == 0) {  //brake light stuff?
    digitalWrite(rightTurn, HIGH);
  }

  if (rightCommand == 0 && brakeOn == 0 && blinking2 == 0) {
    digitalWrite(rightTurn, LOW);
  }

  if (brakeOn == 1 && leftCommand == 0 && blinking1 == 0) {
    digitalWrite(leftTurn, HIGH);
  }

  if (leftCommand == 0 && brakeOn == 0 && blinking1 == 0) {
    digitalWrite(leftTurn, LOW);
  }
}


void tailLamps() {
  if ((parkCommand == 1) || (headCommand == 1 && offCommand == 0) || (offCommand == 0 && parkCommand == 0 && headCommand == 0)) { //if in park lights or in headlights without f2pass or in auto position
    digitalWrite(tailLight, HIGH);
    parklight = 1;
  } else {
    digitalWrite(tailLight, LOW);
    parklight = 0;
  }

  if ((headCommand == 1 && offCommand == 0) || (offCommand == 0 && parkCommand == 0 && headCommand == 0)) { //if in headlights without f2pass or in auto position
    headlight = 1;
  } else {
    headlight = 0;
  }

    if ((highbeamCommand == 1 && offCommand == 0) || (f2p == 1)) { //if in highbeam position and headlights not off or f2p
    highbeam = 1;
    headlight = 0;
  } else {
    highbeam = 0;
  }
  

  if (offCommand == 0 && parkCommand == 0 && headCommand == 0) { //if in auto position
    autoOn = 1;
  } else {
    autoOn = 0;
  }


}

void wipers() {

  if ((wipers1 & 0b00100000)) {  //if bit2 set

    highBit = 1;
  } else {
    highBit = 0;
  }

  if (wipers1 & 0b00010000) {  //if bit3 set
    lowBit = 1;
  } else {
    lowBit = 0;
  }




  if (lowBit == 0 && highBit == 1) {   //turn bits into commands
    highCommand = 1;
  } else {
    highCommand = 0;
  }

  if (lowBit == 1 && highBit == 0) {
    lowCommand = 1;
  } else {
    lowCommand = 0;
  }

  if (lowBit == 1 && highBit == 1) {
    delayCommand = 1;
    delayOn = 1;
  } else {
    delayCommand = 0;
    delayOn = 0;
  }





  if (wipers2 == 0x5A) {  //determine delay setting
    wiperDelayNew = 2;
    wiperTimer2 = -15000; //reset timers other than the one used for this delay so that wipers do a wipe immediately when any delay is selected
    wiperTimer3 = -15000;
    wiperTimer4 = -15000;
    wiperTimer5 = -15000;

  }
  if (wipers2 == 0x46) {
    wiperDelayNew = 4;
    wiperTimer1 = -15000;
    wiperTimer3 = -15000;
    wiperTimer4 = -15000;
    wiperTimer5 = -15000;

  }
  if (wipers2 == 0x32) {
    wiperDelayNew = 6;
    wiperTimer1 = -15000;
    wiperTimer2 = -15000;
    wiperTimer4 = -15000;
    wiperTimer5 = -15000;

  }

  if (wipers2 == 0x1E) {
    wiperDelayNew = 8;
    wiperTimer1 = -15000;
    wiperTimer2 = -15000;
    wiperTimer3 = -15000;
    wiperTimer5 = -15000;


  }

  if (wipers2 == 0x00) {
    wiperDelayNew = 10;
    wiperTimer1 = -15000;
    wiperTimer2 = -15000;
    wiperTimer3 = -15000;
    wiperTimer4 = -15000;

  }


  if (highCommand == 1) {  // 1 = on  high speed
    digitalWrite(wiperHigh, HIGH);
    digitalWrite(wiperLow, LOW);
    wiperHighOn = 1;
    wiperDelayNew = 0; //allow for delay to be "reset" when high selected.
    delayOn == 0; //allof for delay be "reset" when high selected.
  }

  //wiper delay selections, in order of time
  //Serial.println(wiperDelayNew);
  if (delayCommand == 1 && wiperDelayNew == 2 && millis() - wiperTimer1 > 15000) {  //delay wipers
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    Serial.println("check");
    wiperTimer0 = millis();
    wiperTimer1 = millis();
  }

  if (delayCommand == 1 && wiperDelayNew == 4 && millis() - wiperTimer2 > 10000) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    wiperTimer0 = millis();
    wiperTimer2 = millis();
  }

  if (delayCommand == 1 && wiperDelayNew == 6 && millis() - wiperTimer3 > 7000) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    wiperTimer0 = millis();
    wiperTimer3 = millis();
  }

  if (delayCommand == 1 && wiperDelayNew == 8 && millis() - wiperTimer4 > 5000) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    wiperTimer0 = millis();
    wiperTimer4 = millis();
  }
  if (delayCommand == 1 && wiperDelayNew == 10 && millis() - wiperTimer5 > 2500) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    wiperTimer0 = millis();
    wiperTimer5 = millis();
  }



  //ending delay pulse (command to send wipers to rest)

  if (millis() - wiperTimer0 > 1100 && delayCommand == 1 && highCommand == 0 && delayOn == 1) {  //>500 on low is on, >500 on high is off, was 1200
    digitalWrite(wiperLow, LOW); // might need to change this if the wipers don't want to move back home in delay

  }


  // low speed
  if (lowCommand == 1 && delayOn == 0) {
    digitalWrite(wiperLow, HIGH);
    digitalWrite(wiperHigh, LOW);
    wiperLowOn = 1;
  }

  //high speed off
  if (highCommand == 0) {
    digitalWrite(wiperHigh, LOW);
  }

  //check to see if something has been on
  if (wiperDelayNew != 0 || wiperHighOn == 1 || wiperLowOn == 1) {
    oktoshutoff = 1;
  }

  //shut system down
  if (highCommand == 0 && lowCommand == 0 && oktoshutoff == 1 && delayCommand == 0) {
    digitalWrite(wiperLow, LOW);
    wiperDelayNew = 0;
    wiperHighOn = 0;
    wiperLowOn = 0;
    oktoshutoff = 0;
    delayOn = 0;
  }




}
