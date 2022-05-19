/*
  FSLP_mouse
  Copyright (C) 2016 - 2017:
    B. Kazemi, ebaykazemi@googlemail.com

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 3
  of the License, or (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

*/
/*
  need to: if possible, set on screen buttons on android
*/

/**********************************************************************************************************
  LIBRARIES
**********************************************************************************************************/
#include <bluefruit.h>
#include <MD_KeySwitch.h>

#include <Wire.h>
#include <Adafruit_DRV2605.h>

/**********************************************************************************************************
  PIN DEFS / MACROS
**********************************************************************************************************/
Adafruit_DRV2605 drv;
BLEDis bledis;
BLEHidAdafruit blehid;
#define FSLP_T1                     A4 //A0   // FSLP Terminal 1 pin  
#define FSLP_T1_RES                 15//A1    // FSLP Terminal 1 resistor end pin 
#define FSLP_T2                     16//A2    // FSLP Terminal 2 pin 
#define FSLP_T3                     29//A3    // FSLP Terminal 3 pin 
#define BUTTON_PIN                  A2//  2     // active cursor/scroll mode toggle pin 
#define RED_LED_PIN                 A0//  A8    // cathode of red LED 
#define BLUE_LED_PIN                A3//A9    // cathode of blue LED 
#define SENSOR_DISCHARGE_DELAY      7 // in ms, the value for the timer interrupt when capturing sensor data (decouples capacitance)
#define NOT_BEGUN_READING_SENSOR    (!forceEntered && !positionEntered) // used for the timer interrupt capture of sensor data
#define SERIAL_BAUD_RATE            115200  // baud rate
#define PER_CYCLE_DELAY             25    // dy,dx and scroll values are processed at this interval in ms
#define TOUCH_THRESH                2   // must actuate this much force to constitute a touch 
#define Y_AXIS_FORCE_THRESH         3   // must actuate this much force for a dy / scroll value to be valid
#define ERR_HEIGHT_MIN              3   // eliminate noise from position reading 
#define ERR_HEIGHT_HOLD             18    // compare against the difference between a dy/scroll hold to get out of hold
#define ERR_HEIGHT_MAX              18    // eliminate large fluctuating position readings (noise on the other side)
#define ERR_WIDTH_HOLD              4   // in cursor mode compare to get out of a hold 
#define ERR_WIDTH_MAX               5   // in cursor mode, used to protect from light force fluctuations to stay within a hold once in it
#define ANDROID_RESOLUTION_WIDTH    1080  // when in scroll feature mode only, used together with height to instantly move the cursor out of the way 
#define ANDROID_RESOLUTION_HEIGHT   1920
#define WINDOWS_RESOLUTION_WIDTH    1920
#define WINDOWS_RESOLUTION_HEIGHT   1080
#define TIME_TO_DRAG                1000  // must have been 'clicking' for this long before triggering a drag command, which is only stopped by releasing the sensor
#define SCROLL_LED_FADE_PERIOD      1000  // the period for the blue LED fader when the scroll mode is enabled 
#define FORCE_CLICK_TOLERANCE       6     // cross ref against abs dx to assert a click hold in cursor mode 
#define MAX_FORCE_POSSIBLE          175   // maximum input mapped force possible in cursor mode 
#define MAX_MAPPED_DX               50    // maximum output mapped force possible in cursor mode 
#define TIME_TO_CLICK               500   // need to hold in the threshold for this long in ms to trigger a click 
#define HEIGHT_DIVIDER              5   // used to condition the difference height when scrolling 
#define DX_REDUCTION_DELAY          2500  // we exponentially move dx when not touching for this long (hack)  
#define MOUSE_RENDER_DELAY          25    // how often do we want to render dy and dx values in cursor mode? 
#define MOUSE_RENDER_SMOOTH_DIVIDER 5   // within each cycle of above, we divide above, dy and dx by this to render smoother 
#define MID_TO_MAX_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY 95 // lower boundary of mid to max scroll acc segment
#define MIN_TO_MID_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY 65 // lower boundary of min to mid scroll acc segment
#define MOTOR_PWM_PIN               A1       // the PWM pin the motor is attached to
//#define MOTOR_PWM_FREQ              18000   // frequency for motor PWMing, set high enough to not hear the screech 


/**********************************************************************************************************
  GLOBALS
**********************************************************************************************************/
boolean dragEnable = false;         // true when 'hasClicked' is true AND you've waited a set time
boolean isProcessingDyDx = false;       // capture sensor data when this is false, is false when processing dy, dx and scroll
boolean isFirstReading = true;        // we skip the first sensor reading as the system relies on a mealy machine
boolean isHold = false;           // used in cursor mode to know when we intend to click
boolean hasClicked = false;         // in cursor mode used to avoid a second click if stayed in hold after initial click
boolean isAndroidScrollOnly = false;    // used when the project is showcasing purely the android scroller mode
boolean isScrollOn = isAndroidScrollOnly; // scroll mode or cursor mode - always start latter if in full mouse showcase else only showing scroll ability.
boolean isScrollHold = false;         // are we holding steady in the hold? if so then set this to avoid noise
boolean haveShownHold = false;        // used to start the hold counter
boolean haveStoppedTouching = false;    // true when you release the sensor
boolean amIncreasing = false;       // in cursor mode used to assert direction of force
boolean isRetractingDx = false;       // when released the cursor moves back, this is used to know if you've started retracting
boolean haveStoppedRetracting = false;    // used to know when you've stopped retracting
boolean blueLEDLatch = false;         // used to call mouse events only once (keeps it from being retriggered)
boolean hasClickedMotorToggle = false;    // when in click or drag hold, used to toggle motor RPM
boolean motorTriggerClick = true;     // used with timer to turn off max RPM motor when triggered a click after a set time
boolean clickIfRelease = false;       // used to halt any retriggering of mouse click events
volatile boolean forceEntered = false;    // we have begun capturing force data from sensor, including the delay
volatile boolean positionEntered = false;   // we have begun capturing position data from sensor, including the delay
volatile uint8_t currentForceInt = 0;   // current force, written to from within a timer interrupt
volatile uint8_t currentPositionInt = 0;  // current position, written to from within a timer interrupt
uint8_t currentForce = 0;         // current force interrupt safe
uint8_t currentPosition = 0;        // current position interrupt safe
uint8_t forceAtStartOfClick = 0;      // used as reference point when rendering dx speed when in drag mode
uint8_t forceAtStartOfHold = 0;       // capture initial hold force and use that as reference instead of oldForce to assert when to exit hold
uint8_t oldForce = 0;           // last iterations force value
uint8_t oldPosition = 0;          // last iterations position value
uint8_t greatestForce = 0;          // in cursor mode, we capture the greatest force only written when abs(diffWidth) > n
uint8_t startPositionValue = 0;       // in scrollMode capture initial hold position and use that as reference to assert when to exit hold
uint8_t previousDxForce = 0;        // cursor mode, used this as in_min so a retriggered force at 75% is outputted from 0 to 100 %
uint8_t previousDifferenceHeight = 0;   // in cursor mode, capture the difference height (start hold)
uint8_t previoudDiffHeightCounter = 0;    // count number of times you've got the same previous height readings,
uint8_t motorScrollMotorPWMVal = 0;     // holds the duty cycle of the motor
int8_t dx = 0;                // conditioned x axis value in cursor mode
int8_t dy = 0;                // conditioned y axis value in cursor mode
int8_t dyRegion = 1;            // determines what dy region we're in (including the dead zone)
int8_t motorScrollIncVal = 0;         // when mode but is pressed, this changes depending on which mode you enter
unsigned long previousFullCycleMillis = 0;
unsigned long previousScrollCycleMillis = 0;
unsigned long previousDxReductionMillis = 0;
unsigned long previousDxIncreaseMillis = 0;
unsigned long previousHoldMillis = 0;
unsigned long previousScrollMillis = 0;
unsigned long previousDxProcess = 0;
unsigned long lastMouseRenderTime = 0;
unsigned long timeOfClick = 0;        // used to know if you've held for a set time to enter drag mode
unsigned long currentMillis = 0;      // measure of current time used throughout the main loop
//Bounce scrollBtnDebouncer = Bounce();     // Object used to debounce mode toggle button
MD_KeySwitch S(BUTTON_PIN, LOW);  //MIGHT NEED TO CHANGE THIS TO HIGH

SoftwareTimer positionTimer;        // timer used to use interrupt driven sensor acquisition
SoftwareTimer forceTimer;         // timer used to use interrupt driven sensor acquisition
SoftwareTimer hasClickedMotorTimer;
SoftwareTimer triggerClickMotorTimer;
SoftwareTimer motorScrollToggleTimer;



/**********************************************************************************************************
  setup()
**********************************************************************************************************/
void setup(void)
{
  Serial.begin(SERIAL_BAUD_RATE);
  
  drv.begin();
//  drv.pwm_passthrough();
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);
  //  Serial.println("setup start");
  S.begin();
  S.enableDoublePress(true);
  S.enableLongPress(true);
  S.enableRepeat(true);
  S.enableRepeatResult(true);

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setName("Orbitronics");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Orbitronics Corp.");
  bledis.setModel("V1");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.Advertising.start();


  //  Mouse.screenSize(ANDROID_RESOLUTION_WIDTH, ANDROID_RESOLUTION_HEIGHT);  // configure screen size to use with absolute positioning when in scroll only mode

  //  analogWriteFrequency(MOTOR_PWM_PIN, MOTOR_PWM_FREQ);// PWM Freq for motor
  // pinMode(MOTOR_PWM_PIN, OUTPUT);//set the motor PWM pin as an output
  pinMode(RED_LED_PIN, OUTPUT);

  // pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);  //we set the two LED's high since they're active low. (flashes briefly at boot otherwise)
  analogWrite(BLUE_LED_PIN, 0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  previousScrollCycleMillis = millis();
  previousHoldMillis = millis();
  previousFullCycleMillis = millis();
  previousScrollMillis = millis();
  previousDxIncreaseMillis = millis();
  previousDxReductionMillis = millis();
  lastMouseRenderTime = millis();
  previousDxProcess = millis();
  timeOfClick = millis();
  currentMillis = millis();
  //  scrollBtnDebouncer.attach(BUTTON_PIN);
  //  scrollBtnDebouncer.interval(30);
  if (isAndroidScrollOnly)
  {
    delay(500);
    blehid.mouseMove(ANDROID_RESOLUTION_WIDTH, ANDROID_RESOLUTION_HEIGHT / 2);
  }
  //  Serial.println("setup end");
  
  
  positionTimer.begin(SENSOR_DISCHARGE_DELAY * 2, readPositionADC);
  forceTimer.begin(SENSOR_DISCHARGE_DELAY, readForceADC);
  motorScrollToggleTimer.begin(2000 / 1000, motorScrollSlide);
  triggerClickMotorTimer.begin(60000 / 1000, motorTriggerClickVibration);
  hasClickedMotorTimer.begin(100000 / 1000, toggleMotorAtHold);
  Scheduler.startLoop(updateSensorModel);
  Scheduler.startLoop(handleButton);
  Scheduler.startLoop(processSensorData);
  
}

/**********************************************************************************************************
  function definitions
**********************************************************************************************************/
void setupAdv(void)
{
  //  Serial.println("setup ADv start");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for the name in the advertising packet
  Bluefruit.Advertising.addName();
  //  Serial.println("Setup adv End");
}

void setMotorPWM(uint8_t dutyCycle)
{
//  Serial.print("writing to motor: ");
//  Serial.println(dutyCycle);
  analogWrite(MOTOR_PWM_PIN, dutyCycle);
}

// // set thhis when red light is lit, indicateding that you're holding to trigger a clicked
void armMotorPreHasClick(void)
{
  setMotorPWM(0.5 * 255);
}

void turnMotorOff(void)
{
//  Serial.println("Turned off Motor");
  setMotorPWM(0);
}

void toggleMotorAtHold(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  Serial.println("am in toggleMotorAtHold called by timer");

  //noInterrupts();
  if (hasClickedMotorToggle)
    setMotorPWM(0.90 * 255);
  else
    setMotorPWM(0.45 * 255);
  hasClickedMotorToggle = !hasClickedMotorToggle;
 // interrupts();
}

void motorTriggerClickVibration(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  //noInterrupts();
  if (motorTriggerClick)
  {
    turnMotorOff();
    triggerClickMotorTimer.stop();
  }
  motorTriggerClick = !motorTriggerClick;
 // interrupts();
}

//after you've triggered a click you continue to hold for a drag, or release for a a click
void motorIsHolding(void)
{
  hasClickedMotorTimer.start();
//  Serial.println("motorIsHolding");
//  setMotorPWM(0.75 * 255);
}

void motorEnterDragOrStoppedHoldingClick(void)
{
  hasClickedMotorTimer.stop();
}

void motorTriggeredClick(void)
{
  motorEnterDragOrStoppedHoldingClick();
  setMotorPWM(255);

  triggerClickMotorTimer.start();
}

void motorEnterDrag(void)
{
  motorEnterDragOrStoppedHoldingClick();
  setMotorPWM(0.65 * 255);
}

void motorExitDrag(void)
{
  turnMotorOff();
}

void motorScrollSlide(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  //noInterrupts();
  if (motorScrollMotorPWMVal == 255 || motorScrollMotorPWMVal == 50) // if we've reached the end of the slide
  {
    turnMotorOff();
    motorScrollToggleTimer.stop();
  }
  else
  {
    setMotorPWM(motorScrollMotorPWMVal);
    motorScrollMotorPWMVal = motorScrollMotorPWMVal + motorScrollIncVal;
  }
 // interrupts();
}

void motorToggleScroll(boolean amEnteringScroll)
{
  turnMotorOff();
  if (amEnteringScroll)
  {
    motorScrollIncVal = 1;
    motorScrollMotorPWMVal = 51;
  }
  else // exiting scroll
  {
    motorScrollIncVal = -1;
    motorScrollMotorPWMVal = 254;
  }
  motorScrollToggleTimer.start();
}


void turnOnLed(boolean turnOn, int LED)
{
  if (turnOn)
  {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
  }
  else // turn off led
  {
    pinMode(LED, INPUT); // hi Z
    digitalWrite(LED, HIGH);
  }
}

void getSensors(void)
{
//  Serial.println("getting sensors");
  //  noInterrupts();
  forceEntered = true;
  // // interrupts();
  //  setup force pins; setup appropriate driveline voltages
  pinMode(FSLP_T2, OUTPUT);
  digitalWrite(FSLP_T2, HIGH);
  pinMode(FSLP_T3, OUTPUT);
  digitalWrite(FSLP_T3, HIGH);
  pinMode(FSLP_T1_RES, OUTPUT);
  digitalWrite(FSLP_T1_RES, LOW);
  pinMode(FSLP_T1, INPUT);
  forceTimer.start();
  positionTimer.start();
}

void readForceADC(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  forceTimer.stop();
//  Serial.println("getting force ADC");
  currentForceInt = analogRead(FSLP_T1) / 4;
  //  noInterrupts();
  positionEntered = true;
  forceEntered = false;
  // // interrupts();
    //setup position pins; clear charge on sensor
  pinMode(FSLP_T1, OUTPUT);
  pinMode(FSLP_T1_RES, OUTPUT);
  pinMode(FSLP_T2, OUTPUT);
  pinMode(FSLP_T3, OUTPUT);
  digitalWrite(FSLP_T1, LOW);
  digitalWrite(FSLP_T1_RES, LOW);
  digitalWrite(FSLP_T2, LOW);
  digitalWrite(FSLP_T3, LOW);
  //setup appropriate driveline voltages
  pinMode(FSLP_T2, OUTPUT);
  digitalWrite(FSLP_T2, HIGH);
  pinMode(FSLP_T3, OUTPUT);
  digitalWrite(FSLP_T3, LOW);
  pinMode(FSLP_T1_RES, OUTPUT);
  digitalWrite(FSLP_T1_RES, LOW);
  pinMode(FSLP_T1_RES, INPUT);
  digitalWrite(FSLP_T1_RES, LOW);
  pinMode(FSLP_T1, INPUT);

  

}

void readPositionADC(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  positionTimer.stop();
  positionEntered = false;
//  Serial.println("getting position ADC");
  currentPositionInt = analogRead(FSLP_T1) / 4;
  //  noInterrupts();
  
  // // interrupts();
  

}

/*
  continously read the sensor using a timer interrupt for a non blocking delay, and dma
*/
void updateSensorModel(void)
{

//    Serial.print("forceEntered:" );Serial.println(forceEntered);
//    Serial.print("positionEntered:" );Serial.println(!positionEntered);
  //  noInterrupts();
  if (NOT_BEGUN_READING_SENSOR && !isProcessingDyDx) //  we haven't entered either queries (disable interrupts for this check)
  {
    //   // interrupts();
//    Serial.println("am processing model");

    getSensors();
    //    noInterrupts();
    currentPosition = currentPositionInt; // we save the values written from the interrupts to an interrupt safe copy
//    Serial.print("currentPosition: "); Serial.println(currentPosition);
    currentForce = currentForceInt;
    //   // interrupts();
//    Serial.print("Force: "); Serial.print(currentForce); Serial.print("   Position: "); Serial.println(currentPosition);

//
//    Serial.print("forceEntered:" );Serial.println(forceEntered);
//    Serial.print("positionEntered:" );Serial.println(!positionEntered);

  }
  //  else // are currently retrieving either sensor so don't do anything except renable interrupts
  //  {
  //   // interrupts();
  //  }
}


int mapInt(int x, int in_min, int in_max, int out_min, int out_max)
{
  if (x > in_max) x = in_max;
  if (x < in_min) x = in_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (x > in_max) x = in_max;
  if (x < in_min) x = in_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
  turn on 'click trigger' LED and register click if hasClicked is true, drag if you've continued holding a set time
*/
void triggerMouseEvent(void)
{
  if (!isScrollOn)  // we are in cursor mode and in a click trigger and no other click can be triggered in this state
  {
    //    Serial.println("scroll not on");
    if (!dragEnable && hasClicked) // we've clicked but not enabled drag
    {
      Serial.println("have not clicked");
      updateCurrentMillis();
      if (currentMillis - timeOfClick > TIME_TO_DRAG)  //has it been A Set time
      {
        Serial.println("Dragging");
        dragEnable = true;
        clickIfRelease = false;
        //       Mouse.press();
        blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
        turnOnLed(true, RED_LED_PIN);
        motorEnterDrag();
      }
    }
    if (hasClicked)
    {
      if (!dragEnable)
      {
        if (!blueLEDLatch)
        {
          blueLEDLatch = true;
          motorIsHolding();
        }
        clickIfRelease = true;
        turnOnLed(false, RED_LED_PIN);
        analogWrite(BLUE_LED_PIN, 255);
      }
    }
    else   // we have not triggered a click yet
    {
      if (clickIfRelease)
      {
        //       Mouse.click();
        Serial.println("CLICK");
        blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
        delay(10);
        blehid.mouseButtonRelease();
        motorTriggeredClick();
      }
      blueLEDLatch = false;
      clickIfRelease = false;
      if (dragEnable)
      {
        Serial.println("Release Mouse buttons!");
        //       Mouse.release();
        blehid.mouseButtonRelease();
      }
      dragEnable = false;
      hasClicked = false;
      if (dragEnable)
      {
        turnOnLed(false, RED_LED_PIN);
      }
      analogWrite(BLUE_LED_PIN, 0);
    }
  }
}

/*
  Handle button to change between scroll mode and xy mouse mode here
*/
void triggerModeButEvent(void)
{
  boolean temp_has_clicked = false;
  if (S.read() == MD_KeySwitch::KS_PRESS)
    temp_has_clicked = true;
  // scrollBtnDebouncer.update();
  if (temp_has_clicked && isScrollOn)
  {
    Serial.println("first but click if");
    isScrollOn = !isScrollOn;
    motorToggleScroll(false);
  }
  else if (temp_has_clicked && !isScrollOn)
  {
    Serial.println("sceond but click if");
    isScrollOn = !isScrollOn;
    motorToggleScroll(true);
    dx = 0;
    dy = 0;
  }
  temp_has_clicked = false;
}

/*
  fade in the 'scroll mode on' LED
*/
void handleButton(void)
{
  triggerModeButEvent();
  /**
   * handle the led, fade led for scroll mode, else full on
   */
  if (isScrollOn) // we are scrolling
  {

    updateCurrentMillis();
    analogWrite(BLUE_LED_PIN, (255 * cos(2 * PI / SCROLL_LED_FADE_PERIOD * currentMillis)));
  }
  else if (!isScrollOn && !hasClicked) // we are not scrolling
  {
    analogWrite(BLUE_LED_PIN, 0);
  }
}

void updateCurrentMillis(void)
{
  currentMillis = millis();
  //  Serial.println(currentMillis);
}

/*
  deal with cursor (dx, dy) render here - t his is called 'MOUSE_RENDER_SMOOTH_DIVIDER' times for smooth ness with the d values subdivided
*/
void renderCursor(void)
{
  if (!isScrollOn)
  {
    updateCurrentMillis();
    if (currentMillis - lastMouseRenderTime > MOUSE_RENDER_DELAY / MOUSE_RENDER_SMOOTH_DIVIDER) // go into this main loop if it's been perCycleDelay ms since last iteration
    {
      //      Serial.println("doing mouse shit");
      lastMouseRenderTime = currentMillis;
      if (dx != 0 || dy != 0)
      {

        int8_t tempDy = 0;
        if (dy < 0)
          tempDy = floor((float)dy / MOUSE_RENDER_SMOOTH_DIVIDER);
        else if (dy > 0)
          tempDy = ceil((float)dy / MOUSE_RENDER_SMOOTH_DIVIDER);
        int8_t tempDx = 0;
        if (dx < 0)
          tempDx = floor((float)dx / MOUSE_RENDER_SMOOTH_DIVIDER);
        else if (dx > 0)
          tempDx = ceil((float)dx / MOUSE_RENDER_SMOOTH_DIVIDER);
        // Serial.print("tempDx; TempDy: "); Serial.print(tempDx); Serial.print("; "); Serial.println(tempDy);
        blehid.mouseMove( tempDx, tempDy);//0);
      }
    }
  }
}


















/*
  Process dx and dy, or scrollValue here (and render scroll if scrolling)
*/
void processSensorData(void)
{
  updateCurrentMillis();
  if (currentMillis - previousFullCycleMillis > PER_CYCLE_DELAY) // go into this main loop if it's been perCycleDelay ms since last iteration
  {
    //    Serial.println("processing sensor data");
    previousFullCycleMillis = currentMillis;
    isProcessingDyDx = true;
    if (currentForce >= TOUCH_THRESH) // we are touching the Sensor
    {
      if (isFirstReading) // if it's the first reading then you don't do anything as you need to capture the previous and current force / position values (consider putting a break here )
      {
        isFirstReading = false;
      }
      else // is not first reading and we are touching
      {
        /*
          processes dy and generate scrollValue and render scroll here
        */
        int differenceHeight = currentPosition - oldPosition;
        if (currentForce >= Y_AXIS_FORCE_THRESH) // touching greater than thresh and not first reading
        {
          if (abs(differenceHeight) > ERR_HEIGHT_MIN)  // diff is not noise
          {
            if (abs(differenceHeight) < ERR_HEIGHT_MAX) // diff is not erratic WHEN SCROLLING
            {
              if (differenceHeight != 0)  // There IS a change in difference
              {
                int8_t scrollValue;   // used to save the conditioned data value
                /*
                  Process scroll and slow scroll (hold) here
                  using a non blocking version of below generated jerky performance, would require too much modification of main code
                */
                if (isScrollOn)
                {
                  if (!isScrollHold)   // if you're not already scrolling and this is the first scroll event
                  {
                    isScrollHold = true;
                    startPositionValue = currentPosition;
                  }
                  else // i am already scrolling and i want to come out of the scroll tolerance if the difference of start hold val and current hold is greater than a tolerance
                  {
                    if (abs(currentPosition - startPositionValue) > ERR_HEIGHT_HOLD) { // moved enough to come out of the hold
                      isScrollHold = false;
                      haveShownHold = false;
                      differenceHeight = (ceil( (float) differenceHeight / 2));
                    }
                  }
                  if (!isScrollHold) // got here by valid difference - not in a scroll hold, do scroll action
                  {
                    if (differenceHeight < 0)// round up or down depending on sign
                      scrollValue = int (floor( (float) differenceHeight / HEIGHT_DIVIDER)); // capture initial speed and round down
                    else
                      scrollValue = int (ceil( (float) differenceHeight / HEIGHT_DIVIDER)); // capture initial speed and round up
                    if (currentForce >= MIN_TO_MID_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY) // are you in the min to mid acc range?
                    {
                      if (currentForce >= MID_TO_MAX_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY) //are you in the mid to max acc range?
                      {
                        scrollValue *= mapInt(currentForce, MID_TO_MAX_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY, 130, 5, 10);
                      }
                      else // in the min to mid range
                      {
                        scrollValue *= mapInt(currentForce, MIN_TO_MID_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY, MID_TO_MAX_SEGMENT_ACC_SCROLL_FORCE_LOW_BOUNDARY, 3, 6);
                      }
                    }
                    else // we are in the 1 to min acc range
                    {
                      if (scrollValue > 2) // anything above 2 should revert to 2
                        scrollValue = 2;
                      else if (scrollValue < -2) // anything below - 2 should revert to -2
                      {
                        scrollValue = -2;
                      }
                    }

                    for (int i = 0; i < abs(scrollValue); i++) // render the scroll event
                    {
                      if (scrollValue < 0)
                        blehid.mouseScroll(-1); // change this around
                      else
                        blehid.mouseScroll(1);
                      delay(2);
                    }
                  }
                }
              }
            }

          }
          /*
            dy cursor processed here
          */
          if (!isScrollOn) // in cursor mode
          {
            if (abs(currentPosition - oldPosition) <= 1) // don't bother processing if we've not actually moved
            {
              dyRegion = 1;
              if (currentPosition < 200)
              {
                if (currentPosition < 60)
                  dyRegion = -1;
                else
                  dyRegion = 0;
              }
              if (dyRegion != 0)
              {
                if (dyRegion == 1) // in upper region
                {
                  dy = -1 * int(round(1.00 + 0.001 * pow(currentPosition - 199, 3)));
                }
                else if (dyRegion == -1) // in lower region
                {
                  dy = int(round(1.00 + 0.00022 * pow(60 - currentPosition, 3)));
                }
              }
              else // we're in dead zone so set dy to 0
              {
                dy = 0;
              }
            }
          }
        }
        else if (currentForce < Y_AXIS_FORCE_THRESH)// we're not touching the sensor hard enough to trigger a scroll event
        {
          isScrollHold = false;
          haveShownHold = false;
          dy = 0;
        }

        /*
           process values for dx in cursor mode
        */
        if (!isScrollOn) // we are in cursor mode, we're touching
        {
          int differenceWidth = currentForce - oldForce;
          /*
             take the difference of the force. if it's +- 2 then you're holding! if holding then stay still.
             map 0 to 100 force to the width of the screen, take the force and place cursor to linearly mapped position on screen. no hold means shit moves back slowly
          */
          if (currentForce > oldForce)
            amIncreasing = true;
          else
            amIncreasing = false;
          if (amIncreasing) // pushing harder
          {
            if (currentForce > greatestForce || currentForce - oldForce > 2)
              greatestForce = currentForce;
          }
          /*
            process dx and dx hold here
          */
          if (abs(differenceWidth) <= ERR_WIDTH_MAX) // if the difference is within a hold (tolerance and )
          {

            //            dealing with the pressure hold - if holding set time then click

            if (!isHold)   // if you're not already holding SET HOLD
            {
              isHold = true;
              forceAtStartOfHold = currentForce;
            }
            else // i am already holding and i want to come out of the hold if the difference of start hold val and current hold is greater than a tolerance
            {
              if (greatestForce <= currentForce && abs(forceAtStartOfHold - currentForce) >= ERR_WIDTH_HOLD)//(startHoldVal - currentForce > ERR_WIDTH_HOLD )//&& abs(forceAtStartOfHold - currentForce) > ERR_WIDTH_HOLD) //(abs(currentForce - startHoldVal) > ERR_WIDTH_HOLD)
                isHold = false;
            }
          }
          // generate a right drag!
          // Serial.print("     dif: ");Serial.println(abs(currentForce - oldForce));
          if (dragEnable && currentForce < (greatestForce ) && abs(currentForce - oldForce) < 15) // AND THE DIFFERENCE IN FORCE IS SLOW to avoid an overshoot when releasing sensor from drag!
          {
            dx = 1;
            // Serial.print("Dx before: ");Serial.println(dx);
            dx = mapInt(forceAtStartOfClick - currentForce, 0, forceAtStartOfClick, 0, MAX_MAPPED_DX);
            // Serial.print("Dx after: ");Serial.println(dx);

          }
          //generate dx value based on force and state
          if (currentForce + 8 > greatestForce) //currentForce >= greatestForce)
          {
            // Serial.print("Dx before: ");Serial.println(dx);
            int tempDx = mapInt(currentForce, previousDxForce, MAX_FORCE_POSSIBLE, 0, MAX_MAPPED_DX);
            if (tempDx < 15) //hack alert - we're making it so cursor doesn't move when holding
              dx = 0;
            else
              dx = -1 * (tempDx - 15);
            // Serial.print("Dx after: ");Serial.println(dx);
          }
          else //you aren't pushing hard enough to generate dx movement
          {
            previousDxForce = currentForce;
            // dx = 0;
          }
        }
      }
      if (currentForce < TOUCH_THRESH) //the moment you've released the sensor
      {
        isFirstReading = true;
        oldPosition = 0; // this is recaptured in 'isFirstReading'
        isHold = false;
        hasClicked = false;
        haveShownHold = false;
        dy = 0;
      }
      else // have not released sensor yet so capture old position and old force
      {
        oldPosition = currentPosition;
      }
      oldForce = currentForce; // capture the last force if you're actuating the sensor
    }
    else  // i am not touching the sensor
    {
      isHold = false;
      hasClicked = false;
      haveShownHold = false;
      dy = 0;
    }

    /*
      deal with cursor mode when not touching, so algorithmically generate dx
    */
    if (haveStoppedTouching && !isHold && currentForce < TOUCH_THRESH ) // in cursor mode and not touching ease the cursor to the side
    {
      updateCurrentMillis();
      /*
        decrease dx nicely here -- start from beginning if you go into scroll mode while retracting or if seg == 0
      */
      if (isScrollOn)
      {
        dx = 0;
        haveStoppedRetracting = true;
        isRetractingDx = false;
        previousDxReductionMillis = currentMillis;
      }
      else
      {
        if (!isRetractingDx && (currentMillis - previousDxReductionMillis) <= DX_REDUCTION_DELAY) // go into this main loop if it's been perCycleDelay ms since last iteration
        {
          int moveVal = (pow(   1.5 * (((float)currentMillis - previousDxReductionMillis ) / 750), 4));
          if (moveVal >= 127)
            moveVal = 127;
          haveStoppedRetracting = true;
          dx = moveVal;
        }
        else
        {
          if (haveStoppedRetracting)
          {
            dx = 0;
            haveStoppedRetracting = false;
          }
          isRetractingDx = true;
        }
      }
    }










   /*
      deal with the cursor left click, click LED and training LED here
   */
   if (currentForce >= TOUCH_THRESH && !isScrollOn)  // we are touching
   {
     if (!hasClicked && abs(currentPosition - oldPosition) < 3)
     {
       haveStoppedTouching = false;
       if (isHold && dyRegion == 0)  // we are holding
       {
         if (!haveShownHold)  // we haven't shown a hold, so show a hold and START THE HOLD COUNTER
         {
           previousHoldMillis = millis();
           haveShownHold = true;
         }
         else   // we have shown a hold so start the hold counter
         {
           if (currentForce < 120 && !hasClicked && abs(greatestForce - currentForce) <= 3) //we've not triggered a click
           {
             if (dx == 0)//abs(abs(dx) - FORCE_CLICK_TOLERANCE) < 5)
             {
               turnOnLed(true, RED_LED_PIN);
               armMotorPreHasClick();
             }
             // digitalWrite(RED_LED_PIN, HIGH);
             updateCurrentMillis();
             if (dx == 0 && currentMillis - previousHoldMillis > TIME_TO_CLICK)  // it's been > n ms
             {
               previousHoldMillis = currentMillis;
               haveShownHold = false;
               hasClicked = true;
               forceAtStartOfClick = currentForce;
               timeOfClick = currentMillis;
               // Mouse.click();
               // Serial.println("CLICK");
             }
           }
           // else if (hasClicked || (currentForce - greatestForce) < 4)
           // {
           //  previousHoldMillis = currentMillis;
           //  // digitalWrite(RED_LED_PIN, LOW);
           // }
          }
       }
       else  // we are not holding
       {
         updateCurrentMillis();
         previousHoldMillis = currentMillis;
         // if (!hasClicked)
         //  hasClicked = false;
         // digitalWrite(RED_LED_PIN, LOW);
         turnOnLed(false, RED_LED_PIN);
         turnMotorOff();
         haveShownHold = false;
       }
      }
   }
   else if (currentForce < TOUCH_THRESH && !isScrollOn) // not touching
   {
     if (!haveStoppedTouching)
     {
       updateCurrentMillis();
       previousDxReductionMillis = currentMillis;
       isRetractingDx = false;
       greatestForce = 0;
       haveStoppedTouching = true;
       // digitalWrite(RED_LED_PIN, LOW);
       turnOnLed(false, RED_LED_PIN);
       turnMotorOff();
       haveShownHold = false;
       hasClicked = false;
     }
   }
    isProcessingDyDx = false;
  }
}






















/**********************************************************************************************************
  loop()
**********************************************************************************************************/
void loop(void)
{
  motorIsHolding(); 
  triggerMouseEvent();
  renderCursor();
}
