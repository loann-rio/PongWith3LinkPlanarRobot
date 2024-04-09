#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//Constants
#define NUM_JOY 2
#define MIN_VAL 0
#define MAX_VAL 1023

//Parameters
const int joyPin[2] = {A0, A1};
const int joyOffset = 3; // from the calibration phase

int joyVal [NUM_JOY] = {0, 0};

int lastPunchVal = 0;


void setup() {
  
  Serial.begin(9600);

  for (int i = 0; i < NUM_JOY; i++) {
    pinMode(joyPin[i], INPUT);
  }
  
}

void loop() {

  for (int i = 0; i < NUM_JOY; i++) {
      joyVal[i] = map(analogRead(joyPin[i]), MIN_VAL, MAX_VAL, -100, 100) + joyOffset;
  }
  
  Serial.println(joyVal[0]);
  
  if (joyVal[1] < -50 && lastPunchVal>-50){
    Serial.println('p');
  }

  lastPunchVal = joyVal[1];
  delay(50);

}
