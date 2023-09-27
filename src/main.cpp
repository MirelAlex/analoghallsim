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
  DEBOUNCE_IN_TRANS
} Debounce_State;

typedef struct
{
  Debounce_State state;
  uint16 debounceCounterDown;
  uint16 debounceCounterUp;
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
        .debounceCounterDown = 0u,
        .debounceCounterUp = 0u,
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
#define PARAM_DEBOUNCE_THRESHOLD 100u // [ms]
#define PARAM_SHORT_TO_GND_TH 100u
#define PARAM_STATE_INVERTED 1u

// state transitioned from 0 to 1 or 1 to 0 and the READY bit was set
#define canSendEvent(state) (((state & HALL_ACTIVE) != (STATE & HALL_ACTIVE)))
#define resetDebounceCounters()    DEBOUNCE.debounceCounterUp = PARAM_DEBOUNCE_THRESHOLD; \
                                   DEBOUNCE.debounceCounterDown = PARAM_DEBOUNCE_THRESHOLD
uint32 previousMillis = 0u;
const uint8 taskTime = 10u; // [ms]
// function prototypes
void hallInit();
void setInitialState();
void readInput(void);
void filterInput(void);
void graph(void);
void taskA();
boolean calculateState();
boolean isOpenLoad();
boolean isShortToGround();
Debounce_State debounceState();
boolean debounceStateFinished(uint16 *debounceCounter);

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
  setInitialState();
  resetDebounceCounters();
  // DEBOUNCE.state = DEBOUNCE_FINISHED;
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
  // Serial.print(IN.filteredLsb);
  // Serial.print(",");
  Serial.print(DEBOUNCE.debounceCounterDown);
  Serial.print(",");
  Serial.print(DEBOUNCE.debounceCounterUp);
  Serial.print(",");
  Serial.println(STATE);
}

boolean calculateState()
{
  boolean allowedToSendEvent = false;

  uint8 state = STATE;
  state &= ~(HALL_EXT_ERR);
  state |= HALL_READY;

  boolean wasActive = (boolean)(state & HALL_ACTIVE);
  if (PARAM_STATE_INVERTED) wasActive ^= HALL_ACTIVE;

  if (wasActive && IN.lsb < PARAM_MIN_HYST_THRESHOLD)
  {
    if (debounceStateFinished(&DEBOUNCE.debounceCounterDown))
    {
      state &= ~HALL_ACTIVE;
      if (PARAM_STATE_INVERTED) state ^= HALL_ACTIVE;
    }
  }
  else if (!wasActive && IN.lsb > PARAM_MAX_HYST_THRESHOLD)
  {
    if (debounceStateFinished(&DEBOUNCE.debounceCounterUp))
    {
      state |= HALL_ACTIVE;
      if (PARAM_STATE_INVERTED) state ^= HALL_ACTIVE;
    }
  }
  else
  {
    // nothing
    resetDebounceCounters();
  }



  if (state != STATE)
  {
    // update the STATE everytime we finished debouncing and the state calculated is different than previous
    // but send the event only when hall state transitioned from HIGH/LOW to LOW/HIGH
    if (canSendEvent(state))
    {
      allowedToSendEvent = true;
    }

    STATE = state;

  }

  return allowedToSendEvent;
}

void setInitialState()
{
  uint8 state = 0u;
  if (IN.lsb < PARAM_MIN_HYST_THRESHOLD)
  {
    // STATE = HALL_READY; // HALL_INACTIVE
  }
  else if (IN.lsb > PARAM_MAX_HYST_THRESHOLD)
  {
    state = HALL_ACTIVE;
  }
  else
  {
    // STATE = HALL_READY; // HALL_INACTIVE
  }

  if (PARAM_STATE_INVERTED) state ^= HALL_ACTIVE;
  STATE = state;
}

boolean debounceStateFinished(uint16 *debounceCounter)
{
  boolean finished = false;
  if (*debounceCounter > 0u)
  {
    (*debounceCounter)--;
  }
  else
  {
    finished = true;
  }
  return finished;
}

Debounce_State debounceState()
{
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
      resetDebounceCounters();
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
