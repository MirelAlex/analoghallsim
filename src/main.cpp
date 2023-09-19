#include <Arduino.h>

// global types
typedef struct
{
  uint16 lsb;
  uint16 filteredLsb;
} Input;

typedef struct
{
  uint8 state;
  Input input;
  uint16 debounceCounter;
  boolean debounceStarted;
} Sensor;

// global data
Sensor sensor = {
    .state = 0u,
    .input = {0},
    .debounceCounter = 0u,
    .debounceStarted = false};

#define self sensor
#define IN self.input
#define STATE self.state

#define HALL_ACTIVE 1u
#define HALL_READY 2u
#define HALL_EXT_ERR 4u

#define PARAM_MIN_HYST_THRESHOLD 300u
#define PARAM_MAX_HYST_THRESHOLD 400u
#define PARAM_OPEN_LOAD_TH 600u
#define PARAM_DEBOUNCE_THRESHOLD 100u

uint32 previousMillis = 0u;
const uint8 task10ms = 10u;
// function prototypes
void readInput(void);
void filterInput(void);
void graph(void);
void taskA();
boolean calculateState();
boolean isOpenLoad();
boolean debounceState();

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);

  self.debounceCounter = 0u;
  self.debounceStarted = false;
}

void loop()
{
  taskA();
}

void readInput()
{
  IN.lsb = analogRead(A0);
}

void filterInput()
{
  float last = IN.filteredLsb * 0.97;
  float now = IN.lsb * 0.03;
  IN.filteredLsb = (uint16)((last + now));
}

void graph()
{
  Serial.print(IN.lsb);
  // Serial.print(",");
  // Serial.print(IN.filteredLsb);
  Serial.print(",");
  Serial.print(self.debounceCounter);
  Serial.print(",");
  Serial.println(STATE);
}

boolean calculateState()
{
  boolean shallSendEvent = false;

  uint8 state = STATE;
  state &= ~(HALL_EXT_ERR);
  state |= HALL_READY;

  boolean wasActive = (boolean)(state & HALL_ACTIVE);

  if (wasActive && IN.lsb < PARAM_MIN_HYST_THRESHOLD)
  {
    state &= (~HALL_ACTIVE);
    self.debounceStarted = true;
    // reset debounce counter
    // self.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
  }
  else if (!wasActive && IN.lsb > PARAM_MAX_HYST_THRESHOLD)
  {
    state |= HALL_ACTIVE;
    self.debounceStarted = true;
    // reset debounce counter
    // self.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
  }
  else
  {
    // nothing
  }

  if (debounceState() && (state != STATE))
  {
    if ((state & 1) != (STATE & 1))
    {
      shallSendEvent = true;
    }
    STATE = state;
  }

  return shallSendEvent;
}

boolean debounceState()
{

  if (self.debounceStarted)
  {
    // continue debouncing
    if (self.debounceCounter > 0u)
    {
      self.debounceCounter--;
    }
  }

  if (self.debounceCounter == 0u)
  {
    self.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
    self.debounceStarted = false;
    return true;
  }
  return false;

  /*
    boolean debounceFinished = false;
    if (self.debounceCounter > 0u)
    {
      self.debounceCounter--;
    }

    if (self.debounceCounter == 0u)
    {
      // self.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
      // self.debounceStarted = false;
      debounceFinished = true;
    }
    return debounceFinished;
  */
}

boolean isOpenLoad()
{
  boolean isOpenLoad = false;

  if (IN.filteredLsb > PARAM_OPEN_LOAD_TH)
  {
    isOpenLoad = true;
  }

  return isOpenLoad;
}

void taskA()
{
  static uint8 eventsCounter;
  uint32 currentMillis = millis();
  boolean stateChanged = false;
  if (currentMillis - previousMillis >= task10ms)
  {
    // It's time to execute the 1 ms task
    previousMillis = currentMillis;
    // Read the value from analog pin A0
    readInput();
    filterInput();

    if (!isOpenLoad())
    {
      stateChanged = calculateState();
    }
    else
    {
      STATE |= HALL_EXT_ERR;
      // self.debounceStarted = false;
      // dont need to debounce when we get out of EXT_ERR
      self.debounceCounter = 0u;
    }

    if (stateChanged)
    {
      /* send event */
      eventsCounter++;
    }
    Serial.print(eventsCounter);
    Serial.print(",");
    graph();
  }
}