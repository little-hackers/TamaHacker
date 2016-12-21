#include <IRremote.h>
#include <Adafruit_NeoPixel.h>

const int leftbuttonPin = 1;
const int centerButtonPin = 2;
const int rightButtonPin = 4;

const int ledPin =  13;
int buttonState = 0;

      #define WAIT_INTERVAL_STATE 65535 //UInt Max
      #define BTN1   14 //A0
      #define BTN2   15 //A1
      #define DEBOUNCE_TIME  10 //ms
      int btn1State = HIGH;
      int btn1LastState = HIGH;
      unsigned long btn1DebounceTimestamp;

      int btn2State = HIGH;
      int btn2LastState = HIGH;
      unsigned long btn2DebounceTimestamp;
      
      unsigned int currentState;
      unsigned int nextState;
      unsigned long stateTimestamp;
      unsigned long stateInterval;

// Setting up the LEDs.
#define PIN 6
#define DELAY 400
#define NUMLEDS 40
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, PIN, NEO_GRB + NEO_KHZ800);

      int (*animFrames)[40];
      unsigned long* animFrameDurations;
      unsigned long animLength;
      unsigned long animLoopCount;
      unsigned long animIdx;
      unsigned long animTimestamp;
      unsigned long animLoopCounter;

#define BLANK_FRAMES 1
int BLANK[BLANK_FRAMES][40] = {
	{0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0}
};


int blink[40] = {
     1,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,0, \
     0,0,0,0,0,0,0,1
};

#define LOOK_FRAMES 2
unsigned long lookDuration[LOOK_FRAMES] = { 100, 100 };
int look[LOOK_FRAMES][40]  = {
	{0,1,1,1,1,1,1,0, \
     1,0,0,0,0,0,0,1, \
     1,0,0,0,0,0,0,1, \
     1,0,0,0,0,0,0,1, \
     0,1,1,1,1,1,1,0},
     {1,1,1,1,1,1,1,1, \
      1,0,0,0,0,0,0,1, \
      1,0,0,0,0,0,0,1, \
      1,0,0,0,0,0,0,1, \
      1,1,1,1,1,1,1,1}
};

#define DEF_FRAMES 2
unsigned long defDuration[DEF_FRAMES] = { 100, 100 };
int def[DEF_FRAMES][40]  = {
	{1,1,1,1,1,1,1,1, \
     1,0,0,0,0,0,0,1, \
     1,0,0,0,0,0,0,1, \
     1,0,0,0,0,0,0,1, \
     1,1,1,1,1,1,1,1},
     {1,0,0,0,0,0,0,1, \
      1,0,0,0,0,0,0,1, \
      1,0,0,1,0,0,0,1, \
      1,0,0,1,0,0,0,1, \
      1,1,1,1,1,1,1,1}
};

#define SPEAK_FRAMES 2
unsigned long speakDuration[DEF_FRAMES] = { 100, 100 };
int speak[DEF_FRAMES][40]  = {
	{1,1,1,1,1,1,1,1, \
     1,1,1,1,1,1,1,1, \
     1,0,0,0,0,0,0,1, \
     1,1,1,1,1,1,1,1, \
     1,1,1,1,1,1,1,1},
     {1,1,1,1,1,1,1,1, \
      1,0,0,0,0,0,0,1, \
      1,0,0,0,0,0,0,1, \
      1,0,0,0,0,0,0,1, \
      1,1,1,1,1,1,1,1}
};


int BLUE[3] = {0, 0, 255};
int RED[3] = {255, 0, 0};
int BLACK[3] = {0, 0, 0};

// STATES
#define DEFAULT_STATE 0
#define BLINK_STATE 1
#define LOOK_STATE 2
#define SPEAK_STATE 3

void setup()
{
  Serial.begin(9600);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  pinMode(BTN1, INPUT);
  digitalWrite(BTN1, HIGH);
  pinMode(BTN2, INPUT);
  digitalWrite(BTN2, HIGH);
  
  setState(DEFAULT_STATE);
  
  // Initialize random seed
  randomSeed(analogRead(0));
}

void loop() {

  strip.setBrightness(20);
  updateAnimation();
  
    // Check buttons
  if(isBtn1Pressed()){
    setState(LOOK_STATE);
  }

  if(isBtn2Pressed()){
    setState(SPEAK_STATE);
  }
  //animate(def, defDuration, DEF_FRAMES, 2);
    // Update display based on current state
  switch(getState()){

    case DEFAULT_STATE:
      if (random(0, 5) == 0) { 
        setState(LOOK_STATE); // Random blink
      } else if(random(0, 10) == 0) {
        setState(DEFAULT_STATE); // Random speak
      } else if(random(0, 5) == 0) {
        setState(LOOK_STATE); // Random look
      } else {
        animate(def, defDuration, DEF_FRAMES, 2);
        setNextState(DEFAULT_STATE, 3000);
      }
      break;
      
    case BLINK_STATE:
      showit(blink, BLUE);
      setNextState(DEFAULT_STATE, 100);
      break;
      
    case LOOK_STATE:
      animate(look, lookDuration, LOOK_FRAMES, 2);
      setNextState(DEFAULT_STATE, 2000);
      break;
      
    case SPEAK_STATE:
      animate(speak, speakDuration, SPEAK_FRAMES, 4);
      setNextState(DEFAULT_STATE, 2000);
      break;
  }
}

void animate(int frames[][40], unsigned long frameDurations[],unsigned long length, unsigned long loopCount)
{
  stopAnimation();
  animFrames = frames;
  animFrameDurations = frameDurations;
  animLength = length;
  animLoopCount = loopCount;
  animTimestamp = millis();
  playAnimationFrame(animIdx);
}

void stopAnimation() {
  animLength = 0;
  animLoopCount = 0;
  animIdx = 0;
  animLoopCounter = 0;
}

void showit(int Letter[], int RGB[])
{
  int do_delay = DELAY;	
  int count = 40;
  for (int j = 0; j < count; j++) {
    int on = Letter[j];
    strip.setPixelColor(j, RGB[0]*on, RGB[1]*on, RGB[2]*on);
  };

  strip.show();
  delay(do_delay);
}

void playAnimationFrame(int idx) {
	showit(animFrames[idx], RED);
}

// Helper method to perform animation updates. Should be called in loop()
void updateAnimation() {
  if(animLength > 0) {
    unsigned long ts = millis();
    if(ts - animTimestamp > animFrameDurations[animIdx] + 5) { // Add 5 to give tone time between calls (Seems to need it)
      unsigned int nextIdx = animIdx + 1;
      if(nextIdx < animLength){
        animIdx = nextIdx;
        animTimestamp = ts;
        playAnimationFrame(animIdx);
      } else {
        if(animLoopCounter < animLoopCount - 1) {
          animIdx = 0;
          animTimestamp = ts;
          animLoopCounter++;
          playAnimationFrame(animIdx);
        } else {
          stopAnimation();
        }
      }
    }
  }
}

// Returns true if button 1 is pressed
bool isBtn1Pressed() {
  bool changed = debounce(BTN1, btn1State, btn1LastState, btn1DebounceTimestamp);
  return btn1State == LOW && changed;
}

// Returns true while button 1 is held down
bool isBtn1Held() {
  debounce(BTN1, btn1State, btn1LastState, btn1DebounceTimestamp);
  return btn1State == LOW;
}

// Returns true if button 2 is pressed
bool isBtn2Pressed() {
  bool changed = debounce(BTN2, btn2State, btn2LastState, btn2DebounceTimestamp);
  return btn2State == LOW && changed;
}

// Returns true while button 2 is held down
bool isBtn2Held() {
  debounce(BTN2, btn2State, btn2LastState, btn2DebounceTimestamp);
  return btn2State == LOW;
}

// Check to see if the state timer has expired and thus a change of state needs to occur
bool stateIntervalExpired() {
  if (stateInterval > 0 && millis() - stateTimestamp >= stateInterval) {
    stateTimestamp = millis();
    return true;
  }
  return false;
}

// Gets the current state, switching state if next action should occur
unsigned int getState() {
  if(currentState == WAIT_INTERVAL_STATE && stateIntervalExpired()) {
    setState(nextState);
  }
  return currentState;
}

// Sets the current state without waiting, canceling any previously scheduled states
void setState(unsigned int state) {
  setNextState(state, 0);
}

// Lines up the next state to move to after the given interval
void setNextState(unsigned int state, unsigned long interval) {
  if(interval == 0) {
    currentState = state; // Apply the state imediatly
    stateInterval = 0; // Reset state interval
  } else {
    currentState = WAIT_INTERVAL_STATE; // Set to wait mode
    nextState = state; // Store next state
    stateInterval = interval; // Store interval to wait
    stateTimestamp = millis();  // Store current time
  }
}

// Read pin state with debounce
bool debounce(uint8_t pin, int &state, int &lastState, unsigned long &debounceTimestamp) {
  bool changed = false;
  int reading = digitalRead(pin);
  if(reading != lastState) {
    debounceTimestamp = millis();
  }
  if((millis() - debounceTimestamp) > DEBOUNCE_TIME)
  {
    if (reading != state) {
      state = reading;
      changed = true;
    }
  }
  lastState = reading;
  return changed;
}

