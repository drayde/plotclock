
// Plotclock
// cc - by Johannes Heberlein 2014
// v 1.02
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface

// modifiactions for UV plot clock by Andreas Kahler, FabLab München

// time library see https://github.com/PaulStoffregen/Time
// SerialUI v2 see https://github.com/psychogenic/SerialUI
// RTC  library see https://github.com/PaulStoffregen/Time 
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html  


// delete or mark the next line as comment if you don't need these
#define REALTIMECLOCK    // enable real time clock



// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
int SERVOFAKTORLEFT;
int SERVOFAKTORRIGHT;

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
int SERVOLEFTNULL;
int SERVORIGHTNULL;

int SERVO_DELAY;
int SERVO_DELAY_LONG;


#define SERVOPINLEFT  3
#define SERVOPINRIGHT 2

#define LED_PIN 5

// length of arms
#define L1 35
#define L2 55.1
#define L3 13.2

// origin points of left and right servo 
#define O1X 22
#define O1Y -25
#define O2X 47
#define O2Y -25

#include <TimeLib.h> 
#include <Servo.h>
#include <SerialUI.h>
#include <EEPROM.h>

#ifdef REALTIMECLOCK
// for instructions on how to hook up a real time clock,
// see here -> http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// DS1307RTC works with the DS1307, DS1337 and DS3231 real time clock chips.

  #include <Wire.h>
  #include <DS1307RTC.h> // see http://playground.arduino.cc/Code/time    
#endif


Servo servoLeft;  // 
Servo servoRight;  // 

volatile double lastX = 75;
volatile double lastY = 47.5;

int offsetX = 11;

const int bottom = 19;
const double top = 45;
const double origSize = 20;
const double scale = (top-bottom)/origSize;

int last_min = 0;


// our global-scope SerialUI object
SUI::SerialUI mySUI;


void setup() 
{ 
  readParamsFromEeprom();
  
#ifdef REALTIMECLOCK
  getTimeFromRTC();
  setSyncProvider(RTC.get);
#else  
  // Set current time only the first to values, hh,mm are needed
  setTime(16,59,0,0,0,0);
#endif

  setupMenu();

  
  home();
  servoLeft.attach(SERVOPINLEFT);  //  left servo
  servoRight.attach(SERVOPINRIGHT);  //  right servo
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 

  delay(1000);

} 

void loop() 
{ 

  if (mySUI.checkForUser(150))
  {
    mySUI.enter();
    while (mySUI.userPresent())
    {
      mySUI.handleRequests();
    }    
  }  

  int i = 0, j = 0;
  if (last_min != minute()) 
  {
    if (!servoLeft.attached()) servoLeft.attach(SERVOPINLEFT);
    if (!servoRight.attached()) servoRight.attach(SERVOPINRIGHT);

    setLed(false);    
    
    hour();
    while ((i+1)*10 <= hour())
    {
      i++;
    }

    j=0;
    while ((j+1)*10 <= minute())
    {
      j++;
    }

    writeTime(i, (hour()-i*10), j, (minute()-j*10));

    home();
    last_min = minute();

    servoLeft.detach();
    servoRight.detach();
  }
} 

void writeTime(int d1, int d2, int d3, int d4)
{
    number(offsetX-20+1, bottom, d1, scale);
    number(offsetX-20+19, bottom, d2, scale);
    
    number(offsetX-20+35, bottom, 11, scale);

    number(offsetX-20+44, bottom, d3, scale);
    number(offsetX-20+62, bottom, d4, scale);
}

void home()
{
    setLed(false);
    //drawTo(74.2, 47.5);
    drawTo(55, -4); // HOME
}

void testServo()
{
  setLed(false);
  if (!servoLeft.attached()) servoLeft.attach(SERVOPINLEFT);
  if (!servoRight.attached()) servoRight.attach(SERVOPINRIGHT);

  for (int i=0; i<=3; ++i)
  {
    // Servohorns will have 90° between movements, parallel to x and y axis
    drawTo(-3, 29.2);
    delay(500);
    drawTo(74.1, 28);
    delay(500);  
  }
  
  servoLeft.detach();
  servoRight.detach();
}

void write0000()
{
  if (!servoLeft.attached()) servoLeft.attach(SERVOPINLEFT);
  if (!servoRight.attached()) servoRight.attach(SERVOPINRIGHT);

  writeTime(0,0,0,0);

  home();

  servoLeft.detach();
  servoRight.detach();
}
    
// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {

  switch (num) {

  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    setLed(true);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    setLed(false);
    break;
  case 1:

    drawTo(bx + 3 * scale, by + 15 * scale);
    setLed(true);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 10 * scale, by + 0 * scale);
    setLed(false);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    setLed(true);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawTo(bx + 1 * scale, by + 0 * scale);
    drawTo(bx + 12 * scale, by + 0 * scale);
    setLed(false);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    setLed(true);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    setLed(false);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    setLed(true);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 6 * scale);
    drawTo(bx + 12 * scale, by + 6 * scale);
    setLed(false);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    setLed(true);
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawTo(bx + 5 * scale, by + 20 * scale);
    drawTo(bx + 12 * scale, by + 20 * scale);
    setLed(false);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    setLed(true);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    setLed(false);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    setLed(true);
    drawTo(bx + 12 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 0);
    setLed(false);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    setLed(true);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    setLed(false);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    setLed(true);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    setLed(false);
    break;
  case 11:
    drawTo(bx + 5 * scale, by + 15 * scale);
    setLed(true);
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    setLed(false);
    drawTo(bx + 5 * scale, by + 5 * scale);
    setLed(true);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    setLed(false);
    break;

  }
}



void setLed(bool isOn) 
{
  delay(SERVO_DELAY_LONG);
  digitalWrite(LED_PIN, isOn ? HIGH : LOW); 
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));
    delay(SERVO_DELAY);
  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servoLeft.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  servoRight.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));
}


void setupMenu()
{
  mySUI.setGreeting(F("+++ Welcome to Plot Clock +++\r\nEnter ? for help."));
  mySUI.begin(115200); 
  mySUI.setTimeout(20000);      // timeout for reads (in ms), same as for Serial.
  mySUI.setMaxIdleMs(30000);    // timeout for user (in ms)  
  
  SUI::Menu * mainMenu = mySUI.topLevelMenu();
  mainMenu->setName(SUI_STR("Main Menu"));
  mainMenu->addCommand(SUI_STR("set"), setTheTime, SUI_STR("sets the clock's time"));
  mainMenu->addCommand(SUI_STR("show"), getTheTime, SUI_STR("show the clock's time"));
  mainMenu->addCommand(SUI_STR("t"), write0000, SUI_STR("test - write 00:00"));
  mainMenu->addCommand(SUI_STR("c"), testServo, SUI_STR("calibration movement"));
  mainMenu->addCommand(SUI_STR("i"), showInfo, SUI_STR("info - show current settings"));
  mainMenu->addCommand(SUI_STR("l0"), left0, SUI_STR("set left zero"));
  mainMenu->addCommand(SUI_STR("lf"), leftF, SUI_STR("set left factor"));
  mainMenu->addCommand(SUI_STR("r0"), right0, SUI_STR("set right zero"));
  mainMenu->addCommand(SUI_STR("rf"), rightF, SUI_STR("set right factor"));
  mainMenu->addCommand(SUI_STR("off"), offset, SUI_STR("set x offset"));
  mainMenu->addCommand(SUI_STR("save"), saveParamsToEeprom, SUI_STR("save configuration to EEPROM"));
  mainMenu->addCommand(SUI_STR("read"), readParamsFromEeprom, SUI_STR("read configuration from EEPROM"));  
#ifdef REALTIMECLOCK
  mainMenu->addCommand(SUI_STR("rtc"), getTimeFromRTCMenu, SUI_STR("get time from RTC"));  
#endif  
}

void left0(){ setVar(&SERVOLEFTNULL); }
void leftF(){ setVar(&SERVOFAKTORLEFT); }
void right0(){ setVar(&SERVORIGHTNULL); }
void rightF(){ setVar(&SERVOFAKTORRIGHT); }
void offset(){ setVar(&offsetX); }

void setVar(int* var)
{
  mySUI.print(F("OLD VALUE: "));
  mySUI.println(*var);
  mySUI.showEnterNumericDataPrompt();
  int val = mySUI.parseInt();
  mySUI.println(val);

  if (val >= 0)
  {      
    *var = val;
    mySUI.returnOK();
    mySUI.print(F("NEW VALUE: "));
    mySUI.println(*var);
    return;
  }
  mySUI.returnError();  
}

void showInfo()
{
  mySUI.print(F("SERVOFAKTORLEFT:"));
  mySUI.println(SERVOFAKTORLEFT);
  mySUI.print(F("SERVOFAKTORRIGHT:"));
  mySUI.println(SERVOFAKTORRIGHT);
  mySUI.print(F("SERVOLEFTNULL:"));
  mySUI.println(SERVOLEFTNULL);
  mySUI.print(F("SERVORIGHTNULL:"));
  mySUI.println(SERVORIGHTNULL);
  mySUI.print(F("SERVO_DELAY:"));
  mySUI.println(SERVO_DELAY);
  mySUI.print(F("SERVO_DELAY_LONG:"));
  mySUI.println(SERVO_DELAY_LONG);
  mySUI.print(F("offsetX:"));
  mySUI.println(offsetX);
}

void setTheTime()
{
  mySUI.print(F("Enter hours: "));
  mySUI.showEnterNumericDataPrompt();
  int hours = mySUI.parseInt();
  mySUI.println(hours);

  mySUI.print(F("Enter minutes: "));
  mySUI.showEnterNumericDataPrompt();
  int minutes = mySUI.parseInt();
  mySUI.println(minutes);

  if (hours >= 0 && hours <= 23 && minutes >= 0 && minutes <= 59)
  {      
    setTime(hours,minutes,0, 0, 0, 0);
    mySUI.returnOK();

#ifdef REALTIMECLOCK
    copyTimeToRTC();
#endif
    return;
  }
  mySUI.returnError();
}

void getTheTime()
{
  mySUI.print(F("hours: "));
  mySUI.println(hour());
  mySUI.print(F("minutes: "));
  mySUI.println(minute());  
}

void saveParamsToEeprom()
{
  int adr = 0;
  writeToEeprom(adr, SERVOLEFTNULL);
  writeToEeprom(adr, SERVOFAKTORLEFT);
  writeToEeprom(adr, SERVORIGHTNULL);
  writeToEeprom(adr, SERVOFAKTORRIGHT);
  writeToEeprom(adr, SERVO_DELAY);
  writeToEeprom(adr, SERVO_DELAY_LONG);
  writeToEeprom(adr, offsetX);
}

void readParamsFromEeprom()
{
  int adr = 0;
  readFromEeprom(adr, SERVOLEFTNULL, 1950);
  readFromEeprom(adr, SERVOFAKTORLEFT, 660);
  readFromEeprom(adr, SERVORIGHTNULL, 1100);
  readFromEeprom(adr, SERVOFAKTORRIGHT, 700);
  readFromEeprom(adr, SERVO_DELAY, 5);
  readFromEeprom(adr, SERVO_DELAY_LONG, 100);
  readFromEeprom(adr, offsetX, 11);
}

void readFromEeprom(int& adr, int& param, int defaultValue)
{
  int val;
  EEPROM.get(adr, val);
  param = (val >= 0) ? val : defaultValue;
  adr += sizeof(int);
}

void writeToEeprom(int& adr, int param)
{
  EEPROM.put(adr, param);
  adr += sizeof(int);
}

#ifdef REALTIMECLOCK

bool getTimeFromRTC()
{
  time_t t = RTC.get();
  if (t > 0) 
  {
    setTime(t);
    return true;
  } 
  return false;
}

void getTimeFromRTCMenu()
{
  if (getTimeFromRTC()) 
  {
    mySUI.println(F("DS1307 time retrieved."));
    getTheTime();
    return;
  } 

  if (RTC.chipPresent())
  {
    mySUI.println(F("DS1307 must be initialized. Set time first!"));
  } 
  else 
  {
    mySUI.println(F("DS1307 read error!  Please check the circuitry."));
  } 
}

void copyTimeToRTC()
{  
  RTC.set(now());
}

#endif 
