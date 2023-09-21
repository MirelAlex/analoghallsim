#include <Arduino.h>

// global types
typedef struct
{
  uint16 lsb;
  uint16 filteredLsb;
} Input;

typedef enum
{
  DEBOUNCE_FINISHED = 0,
  DEBOUNCE_STARTED,
} Debounce_State;

typedef struct
{
  Debounce_State state;
  uint16 debounceCounter;
} Debounce;

typedef struct
{
  uint8 state;
  Input input;
  Debounce debounce;
} Sensor;

// global data
Sensor sensor = {
    .state = 0u,
    .input = {0},
    .debounce = {
        .state = DEBOUNCE_FINISHED,
        .debounceCounter = 0u,
    }};

#define self sensor
#define IN self.input
#define STATE self.state
#define DEBOUNCE self.debounce

#define HALL_ACTIVE 1u
#define HALL_READY 2u
#define HALL_EXT_ERR 4u

#define PARAM_MIN_HYST_THRESHOLD 300u
#define PARAM_MAX_HYST_THRESHOLD 400u
#define PARAM_OPEN_LOAD_TH 600u
#define PARAM_DEBOUNCE_THRESHOLD 300u // [ms]
#define PARAM_SHORT_TO_GND_TH 100u

// state transitioned from 0 to 1 or 1 to 0 and the READY bit was set
#define canSendEvent(state) (((state & HALL_ACTIVE) != (STATE & HALL_ACTIVE)) && (STATE & HALL_READY))

// state = 0 1
// STATE = 0 1
//

uint32 previousMillis = 0u;
const uint8 taskTime = 1u; // [ms]
// function prototypes
void hallInit();
void readInput(void);
void filterInput(void);
void graph(void);
void taskA();
boolean calculateState();
boolean isOpenLoad();
boolean isShortToGround();
Debounce_State debounceState();

void setup()
{
  // Initialize Serial communication
  Serial.begin(115200);
  // Init
  hallInit();
}

void loop()
{
  taskA();
}

void hallInit()
{
  readInput();
  IN.filteredLsb = IN.lsb;
  DEBOUNCE.debounceCounter = 0u;
  DEBOUNCE.state = DEBOUNCE_FINISHED;
  // delay(5000);
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
  Serial.print(",");
  Serial.print(IN.filteredLsb);
  Serial.print(",");
  Serial.print(DEBOUNCE.debounceCounter);
  Serial.print(",");
  Serial.println(STATE);
}

boolean calculateState()
{
  boolean shallSendEvent = false;

  uint8 state = STATE;
  state &= ~(HALL_EXT_ERR);
  state |= HALL_READY;

  // boolean wasReady = ;
  boolean wasActive = (boolean)(state & HALL_ACTIVE);

  if (wasActive && IN.lsb < PARAM_MIN_HYST_THRESHOLD)
  {
    state &= (~HALL_ACTIVE);
    DEBOUNCE.state = DEBOUNCE_STARTED;
    // reset debounce counter
    // DEBOUNCE.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
  }
  else if (!wasActive && IN.lsb > PARAM_MAX_HYST_THRESHOLD)
  {
    state |= HALL_ACTIVE;
    DEBOUNCE.state = DEBOUNCE_STARTED;
    // reset debounce counter
    // DEBOUNCE.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
  }
  else
  {
    // nothing
  }

  if (debounceState() == DEBOUNCE_FINISHED)
  {
    // update the STATE everytime we finished debouncing and the state calculated is different than previous
    // but send the event only when hall state transitioned from HIGH/LOW to LOW/HIGH
    if (canSendEvent(state))
    {
      shallSendEvent = true;
    }

    STATE = state;
  }

  return shallSendEvent;
}

Debounce_State debounceState()
{

  if (DEBOUNCE.state == DEBOUNCE_STARTED)
  {
    // continue debouncing
    if (DEBOUNCE.debounceCounter > 0u)
    {
      DEBOUNCE.debounceCounter--;
    }
  }

  if (DEBOUNCE.debounceCounter == 0u)
  {
    DEBOUNCE.debounceCounter = PARAM_DEBOUNCE_THRESHOLD;
    DEBOUNCE.state = DEBOUNCE_FINISHED;
  }

  return DEBOUNCE.state;
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

boolean isShortToGround()
{
  boolean isShortToGround = false;

  if (IN.filteredLsb < PARAM_SHORT_TO_GND_TH)
  {
    isShortToGround = true;
  }

  return isShortToGround;
}

void taskA()
{
  static uint8 eventsCounter;
  uint32 currentMillis = millis();
  boolean stateChanged = false;
  if (currentMillis - previousMillis >= taskTime)
  {
    // It's time to execute the 1 ms task
    previousMillis = currentMillis;
    // Read the value from analog pin A0
    readInput();
    filterInput();

    if (!isOpenLoad() && !isShortToGround())
    {
      stateChanged = calculateState();
    }
    else
    {
      STATE |= HALL_EXT_ERR;
      // DEBOUNCE.state = DEBOUNCE_FINISHED;
      // dont need to debounce when we get out of EXT_ERR
      DEBOUNCE.debounceCounter = 0u;
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
