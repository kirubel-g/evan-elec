
#include <SPI.h>
#include <mcp_can.h>

// Chip Select for the CAN Bus
const int spiCSPinFCAN = 53;
const int spiCSPinBMSCAN = 37;

unsigned long previousCanSend = 0;
unsigned long previousCanSend2 = 0;

//int

int chargeDet = A1;


// Default values for basic functaionalihts of the van
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
int lowBit = 0;                         // wiper commands (i.e. they include the delay command too)
int highBit = 0;                        // wiper commands (i.e. they include the delay command too)

unsigned long wiperTimer0 = 0;          // different counters for each ammount of delay time the driver might select, not some sort of actio
unsigned long wiperTimer1 = -15000;     //  different counters for each ammount of delay time the driver might select, not some sort of actio
unsigned long wiperTimer2 = -10000;     //  different counters for each ammount of delay time the driver might select, not some sort of actio
unsigned long wiperTimer3 = -7000;      //  different counters for each ammount of delay time the driver might select, not some sort of actio
unsigned long wiperTimer4 = -5000;      //  different counters for each ammount of delay time the driver might select, not some sort of actio
unsigned long wiperTimer5 = -2500;      //  different counters for each ammount of delay time the driver might select, not some sort of actio

MCP_CAN CAN(spiCSPinFCAN);  //start talking on FCAN which is motor 1 + all other ECU's
MCP_CAN CAN2(spiCSPinBMSCAN);  //start talking on BMSCAN




//All the BSM Stuff

int AAT = 0;
int ambient = 34; // default value


int APP_Status = 0;
float accelPct = 0;
uint16_t motorSpeed = 0;
float vss = 0;
uint16_t vssRaw = 0;

int IL_mode = 0;
int shift_R = 0;
int shift_status = 0;


int i = 0;
int j = 0;
int k = 0;
int counter = 0;
byte var = 0;

unsigned char stmp78[8] = {0xDF, 0x3C, 0xFF, 0xE0, 0x98, 0xC0, 0x00, 0x00};
unsigned char stmp79[8] = {0xDF, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp86[8] = {0x3E, 0x83, 0x81, 0x5B, 0xE9, 0x98, 0x08, 0x00};
unsigned char stmpFD[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp165[8] = {0x40, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x00};
unsigned char stmp166[8] = {0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp215[8] = {0x27, 0x10, 0x27, 0x10, 0x27, 0x10, 0x27, 0x10};
unsigned char stmp21C[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp240[8] = {0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp245[8] = {0x80, 0x17, 0xE8, 0x01, 0x80, 0x00, 0x00, 0x00};
unsigned char stmp25D[8] = {0x07, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp420[8] = {0x4D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF2, 0x80};
unsigned char stmp4F0[8] = {0x14, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


unsigned char stmp9F[8] = {0x40, 0x64, 0x04, 0x09, 0x7F, 0x00, 0x69, 0x02};  //Reverse = 0x41, PND = 0x40
unsigned char stmp202[8] = {0x00, 0x03, 0x08, 0x00, 0x00, 0x00, 0x18, 0x93};

unsigned char stmp50[8] = {0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp91[8] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp9A[8] = {0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00};
unsigned char stmp9B[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp9E[8] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
unsigned char stmp217[8] = {0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp228[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp340[8] = {0x00, 0x00, 0x01, 0x0C, 0x00, 0x00, 0x02, 0x00};
unsigned char stmp40A[8] = {0xBF, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp413[8] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp43D[8] = {0x00, 0x10, 0x29, 0x31, 0x36, 0x24, 0x00, 0x00};
unsigned char stmp4F5[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp4F8[8] = {0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp511[3] = {0x03, 0x02, 0x00};

unsigned char stmp40A1[8] = {0xC1, 0x00, 0x4A, 0x4D, 0x31, 0x42, 0x4E, 0x31};
unsigned char stmp40A2[8] = {0xC1, 0x01, 0x4C, 0x37, 0x33, 0x48, 0x31, 0x31};
unsigned char stmp40A3[8] = {0xC1, 0x02, 0x31, 0x33, 0x35, 0x30, 0x35, 0x00};
unsigned char stmp40A4[8] = {0xC1, 0x03, 0x38, 0x00, 0x1B, 0x09, 0x94, 0x00};
unsigned char stmp40A5[8] = {0xC1, 0x04, 0x00, 0x00, 0x07, 0xBD, 0x00, 0x00};
unsigned char stmp40A6[8] = {0xC0, 0x00, 0x82, 0xDD, 0xC3, 0x09, 0x06, 0x33};
unsigned char stmp40A7[8] = {0xC0, 0x01, 0x00, 0x03, 0x5A, 0x18, 0x00, 0x49};
unsigned char stmp40A8[8] = {0xBF, 0x00, 0x3D, 0x00, 0x00, 0x88, 0xFF, 0x00};
unsigned char stmp40A9[8] = {0xBF, 0x01, 0x21, 0x0D, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp40A10[8] = {0xBF, 0x02, 0x22, 0x00, 0x00, 0x00, 0x00, 0x0F};
unsigned char stmp40A11[8] = {0xBF, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp40A12[8] = {0xBF, 0x04, 0x31, 0x00, 0x00, 0x10, 0x00, 0x00};
unsigned char stmp40A13[8] = {0xBF, 0x05, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stmp40A14[8] = {0xBF, 0x06, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00};
unsigned char stmp40A15[8] = {0xBF, 0x07, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00};
unsigned char stmp40A16[8] = {0xC0, 0x00, 0x82, 0xDD, 0xC3, 0x13, 0x06, 0x33};
unsigned char stmp40A17[8] = {0xC0, 0x01, 0x00, 0x03, 0x5A, 0x18, 0x00, 0x4A};
unsigned char stmp40A18[8] = {0xBF, 0x00, 0x3D, 0x6A, 0x00, 0x89, 0xFF, 0x00};
unsigned char stmp40A19[8] = {0xBF, 0x08, 0x21, 0x20, 0x01, 0x00, 0x00, 0x00};
unsigned char stmp40A20[8] = {0xBF, 0x09, 0x21, 0x00, 0x00, 0x40, 0x00, 0x00};
unsigned char stmp40A21[8] = {0xBF, 0x10, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00};
unsigned char stmp40A22[8] = {0xBF, 0x11, 0x30, 0x03, 0xA4, 0x00, 0x00, 0x00};

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
 * 
 */
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


/*
 * Function: loop
 * ----------------------------
 * This function is called repeatedly in a loop after the setup function.
 * It controls the behavior of various vehicle functions based on input conditions.
 */
void loop() {

  if (igState > 1) {
    blinkers();
    tailLamps();
    lockout1 = 0;
  } else if (autoOn == 1  && lockout1 == 0) {
    lockout1 = 1; //prevents looping and also lets shutdownSeq know that the auto lights is on
    comingHomeTimer = millis(); // sets timer for lights off and ECU shutdown
    shutdownSeq();
    //set taillight/headlight delay timer and shut everything else (and commands) down manually, and if haz command is on turn on hazzards

  } else if (lockout1 == 0) {
    comingHomeTimer = millis(); // sets timer for just ECU shutdown
    shutdownSeq();
    lockout1 = 1;  //just dont loop, shutdownseq wont see this value, so it wont keep lights on
    //shut everything down manually (and commands), and if haz command is on turn on hazzards

  } else if (hazCommand == 1 || brakeCommand == 1) {  //once we have shutdown, come here if we have a brake or hazzard command

    if (hazCommand == 1) { //bypass lights function
      leftCommand = 1;
      rightCommand = 1;
    } else {
      leftCommand = 0;
      rightCommand = 0;
    }
    blinkers();  //do the command

  } else {   //else if nothing on, continuously shut off the blinker and turnsignals
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


  if (igState <= 1) {
    if (millis() - comingHomeTimer > 30000) {  //delay to leave lights on after key off
      digitalWrite(taskLight, LOW);
      digitalWrite(tailLight, LOW);
      parklight = 0;
      headlight = 0;
    }

    if (((millis() - comingHomeTimer > 60000 && hazCommand == 0) || (millis() - comingHomeTimer > 1800000 && hazCommand == 1))) {  //delay to shut off ECU, hold on longer if hazz on (30min)
      if (chargeOn == 0x00) { // only shutoff if not plugged in otherwise we stay awake
        digitalWrite(r7, LOW); //it'll reboot when you step on the brake again so no worries if it shuts off during ig1
      } else {
        digitalWrite(r7, HIGH); //if timer expires as you are plugging it in, ecu could shutoff but still have power.  this prevents this because it will reactivate relay if chargeOn = 1
      }
    }
  }

  //Serial.println(chargeOn);
  //blinkers();

  //tailLamps();







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
  if (canId == 0x091  && igState > 1) { //ignore the switches if the igntion isn't ig2 or IGON
    lamps1 = buf[1];
    lamps2 = buf[0];
    wipers1 = buf[2];
    wipers2 = buf[6];
    lights();
    wipers();
  }


  //for BSM
  if (canId == 0x101) {
    accelPct = buf[0] * 0.392;
  }

  if (canId == 0x105) {
    motorSpeed = (buf[0] << 8) | buf[1];
    motorSpeed = motorSpeed * 0.25;
    vss = (motorSpeed * 1.7 * 4.56 * 29 * 3.14 * 0.0000157828282828);
    vss = vss * 0.621; //convert to a number that when converted to kph is actually mph.
  }

  if (canId == 0x086) {  //steering angle decode and recode (bc diff between cx-9 and m3 EPS ecu)
    steerAngle = (buf[0] << 8) | buf[1];
    steerAngle = (steerAngle * 0.1) - 1598;  //was 1600, calbrated now, 1577 worked
    steerAngleRough = round(steerAngle) + 1000;

    // Serial.println(steerAngle);

    byte last3Bits = steerAngleRough & 0b00000111;
    byte remainingBits = steerAngleRough >> 3;
    byte byte3 = (remainingBits << 3) | last3Bits;
    byte byte4 = remainingBits >> 5;
    stmp86[3] = byte4;
    stmp86[4] = byte3;

    //Serial.println(steerAngleRough);
  }


  if (canId == 0x6B1) {
    ambient = buf[2] - 40; // scale temp to real val
    //ambient = 34; // for now bc ambient not wired in bms
    chargeOn = buf[6];
  }


  //CAN SEND on ID 103 for turn signal blink
  if (millis() - previousCanSend > 30) {
    unsigned char stmp[8] = {leftFrontBlink, rightFrontBlink, parklight, headlight, highbeam, bsmWarn, 0, 0}; //
    CAN.sendMsgBuf(0x103, 0, 8, stmp);
    previousCanSend = millis();
  }


  bsmValues();

  bsm();


}

/*
 * Function: power
 * ----------------------------
 * This function controls the power state of various components based on the ignition state.
 * Checks the current state, turn on/off of the blind spot monitoring 
 */
void power() {
  // Serial.println(igState);
  if (igState > 1) {   //if in IG2
    digitalWrite(bsmPow, HIGH);
    digitalWrite(bmsIg, HIGH);
    digitalWrite(r7, HIGH); // make sure ECU is on
    // Serial.println("ON");
    //Serial.println(igState);
  }

  if (igState <= 1) { //if not IG2
    digitalWrite(bsmPow, LOW);
    digitalWrite(bmsIg, LOW);
    // Serial.println("off");
  }

}


/*
 * Function: lights
 * ----------------------------
 * This function controls the lighting system based on the input signals received.
 * Peforms bit manipulation to set the left,right, reverse, and task light blinker flags.
 */
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

/*
 * Function: blinkers
 * ----------------------------
 * This function controls the behavior of the turn signals and brake lights.
 * Takes the brake command flag and sets the necessary blinking patterns
 */
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


/*
 * Function: tailLamps
 * ----------------------------
 * This function controls the behavior of the tail lamps based on various conditions.
 * Checks for the park command, head comand, and off command to turn on and off the respected light pins
 * Sets the appropriate highbeam, head light flags
 */
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


/*
 * Function: wipers
 * ----------------------------
 * This function controls the behavior of the wipers based on various conditions.
 * Performs bit manipulation of the wipers1 to set the state of the wipers
 * Checks for the wiper counter for delay and decodes the wipers1 information for its respective flag
 */
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


/*
 * Function: bsmValues
 * ----------------------------
 * This function updates various BSM (Blind Spot Monitoring) values based on different conditions.
 * 
 */
void bsmValues() {

  stmp40A18[3] = 0x08; //reset stmp40A18, default value being set here is that we are > 1500RPM
  stmp9F[0] = 0x40;  //Reset bits to default values
  stmp202[2] = 0x00;
  stmp202[3] = 0x00;
  stmp91[1] = 0x00;
  // stmp40A18[3]

  if (gear == 1) {
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 7);
    stmp9F[0] |= static_cast<char>((1 & 0x01) << 0);  //reverse bit
  }

  if (gear == 3) {
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 6);
  }


  if (parklight == 1) {
    stmp9F[0] |= static_cast<char>((1 & 0x01) << 3);  //taillights bit
  }



  if (accelPct <= 20 && accelPct > 1) { //<20% pedal
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 4);
  }
  if (accelPct > 20) { //>20% pedal
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 5);
  }

  //vss = 8.12;
  if (vss > 6.2) { //>10kph
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 1);
  }
  if (vss <= 6.2) { //<10kph
    stmp40A18[3] |= static_cast<char>((1 & 0x01) << 0);  //may have to add logic to set to 0 when vss = 0 if erroneous readings when stopped

  }



  vssRaw = static_cast<uint16_t>(vss * 100 * 1.609);  //scale and switch to kph

  // Assuming big-endian encoding
  stmp202[2] = (vssRaw >> 8) & 0xFF;  // Most significant byte
  stmp202[3] = vssRaw & 0xFF;          // Least significant byte




  if (leftCommand == 1) {
    stmp91[1] = 0x20;
  }
  if (rightCommand == 1) {
    stmp91[1] = 0x10;
  }




}


void bsm() {


  if (CAN_MSGAVAIL == CAN2.checkReceive())
  {
    CAN2.readMsgBuf(&len1, buf1);
    canId1 = CAN2.getCanId();
    //   Serial.println(canId1);
  }

  if (canId1 == 0x477) {
    // Serial.println(buf1[1], HEX);
    //if ((buf1[1] & 0b00010000) || (buf1[1] & 0b01000000) || (buf1[1] & 0b10000000) || (buf[1] & 0b00100000)) {

    if (buf1[1] & 0b00010000 || buf1[1] & 0b00100000) {
      //  bsmWarn = 1;   //somebody there
      bsmWarn |= 0b00000001;
      // Serial.println("left");
    } else {
      bsmWarn &= 0b11111110;
    }


    if (buf1[1] & 0b01000000 || buf1[1] & 0b10000000) {
      //bsmWarn = 2;  //somebody there other side
      bsmWarn |= 0b00000010;
      //  Serial.println("right");
    } else {
      bsmWarn &= 0b11111101;
    }

    if (buf1[2] == 0x7A || buf1[2] == 0x2A || buf1[2] == 0xDA || buf1[2] == 0x8A || buf1[1] & 0b00100000 || buf1[1] & 0b10000000) {
      bsmWarn |= 0b10000000; //  buzzer
      // Serial.println("buzzer");
    } else {
      bsmWarn &= 0b01111111;
    }

 //   if ((buf1[1] == 0x02 || buf1[1] == 0x01) && (buf1[2] == 0x01 || buf1[2] == 0x02)) {
      //bsmWarn = 2;  //nothing anywhere
 //     bsmWarn = 0;
      //Serial.println("nothing");
 //   }



    //Serial.println(bsmWarn);
    //    Serial.print(buf1[0], HEX);
    //    Serial.print(" ");
    //    Serial.print(buf1[1], HEX);
    //    Serial.print(" ");
    //    Serial.print(buf1[2], HEX);
    //    Serial.print(" ");
    //    Serial.print(buf1[3], HEX);
    //    Serial.print(" ");
    //    Serial.print(buf1[4], HEX);
    //    Serial.print(" ");
    //    Serial.println("");
  }


  //Checksums and MSG 1
  if ((millis() - lastSend > 30)) {


    CAN2.sendMsgBuf(0x78, 0, 8, stmp78);
    CAN2.sendMsgBuf(0x86, 0, 8, stmp86);
    //  CAN2.sendMsgBuf(0x9E, 0, 8, stmp9E);
    //  CAN2.sendMsgBuf(0x9F, 0, 8, stmp9F);


    lastSend = millis();

    i = stmp78[6];
    i = i + 16;

    j = stmp86[5];
    j = j + 1;
    //j = stmp2170[7];
    // j = j - 1;


    if (stmp78[6] == 240) {
      stmp78[6] = 0x00;
      i = stmp78[6];

    }


    if (stmp86[5] == 155) {
      stmp86[6] = 0x98;  // could be wrong
      j = stmp86[6];

    }

    stmp78[6] = i;
    stmp86[5] = j;
  }

  if ((millis() - lastSend3 > 25)) {

    CAN2.sendMsgBuf(0x9E, 0, 8, stmp9E);
    CAN2.sendMsgBuf(0x9F, 0, 8, stmp9F);
    lastSend3 = millis();
  }




  //More checksums and speed message
  if ((millis() - lastSend1 > 35)) {

    CAN2.sendMsgBuf(0x202, 0, 8, stmp202);


    lastSend1 = millis();

    var = stmpFD[2];
    var = var + 80;

    stmpFD[2] = var;

  }



  if ((millis() - lastSend2 > 100)) {
    //Serial.println(ambient);

    AAT = ambient + 40;  //rescale ambient for sending
    stmp40A17[7] = AAT;





    CAN2.sendMsgBuf(0x91, 0, 8, stmp91);
    counter++;

    if (counter == 1) {
      // CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A1);
    }

    if (counter == 2) {
      // CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A2);
    }

    if (counter == 3) {
      //CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A3);
    }

    if (counter == 4) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A4);
    }

    if (counter == 5) {
      // CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A5);
    }

    if (counter == 6) {
      // CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A6);
    }

    if (counter == 7) {
      // CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A7);
    }

    if (counter == 8) {
      //  CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A8);
    }

    if (counter == 9) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A9);
    }

    if (counter == 10) {
      //  CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A10);
    }

    if (counter == 11) {
      //  CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A11);
    }

    if (counter == 12) {
      //  CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A12);
    }

    if (counter == 13) {
      //  CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A13);
    }

    if (counter == 14) {
      //  CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A14);
    }

    if (counter == 15) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A15);
    }

    if (counter == 16) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A16);
    }

    if (counter == 17) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A17);
    }
    if (counter == 18) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A18);
    }
    if (counter == 19) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A19);
    }
    if (counter == 200) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A20);
    }
    if (counter == 21) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A21);
    }
    if (counter == 22) {
      CAN2.sendMsgBuf(0x40A, 0, 8, stmp40A22);
      counter = 0;
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

void shutdownSeq() {

  digitalWrite(leftTurn, LOW);
  digitalWrite(rightTurn, LOW);
  digitalWrite(revLight, LOW);
  digitalWrite(chmsl, LOW);
  digitalWrite(wiperHigh, LOW);
  digitalWrite(wiperLow, LOW);



  if (lockout1 == 1) {
    // turn on all the parklights and dome light,  we will shut them off in the main loop
    digitalWrite(taskLight, HIGH);
    digitalWrite(tailLight, HIGH);
    parklight = 1;
    headlight = 1;

  } else {
    digitalWrite(taskLight, LOW);
    digitalWrite(tailLight, LOW);
    parklight = 0;
    headlight = 0;

  }


  leftCommand = 0;
  rightCommand = 0;
  //brakeCommand = 0;
  //lamps1 = 0;
  //lamps2 = 0; //prevent any hazzard activation when step on brake when car off
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
  //hazCommand = 0;
  gear = 0; //put back in park when key is off
  f2p = 0;
  highbeamCommand = 0;
  // headlight = 0;
  highbeam = 0;
  //parklight = 0;

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
