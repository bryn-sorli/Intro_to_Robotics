#include <Sparki.h>
#include <math.h>

#define CYCLE_TIME 100 // milliseconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define MOVING_FORWARD 10
#define MOVING_LEFT 11
#define MOVING_RIGHT 12

int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
int moving_state;

const int threshold = 700;
int lineLeft = 1000;
int lineCenter = 1000;
int lineRight = 1000;

unsigned long startMillis;
unsigned long currentMillis;

float pose_x = 0.0, pose_y = 0.0, pose_theta = 0.0;

void setup() {
  pose_x = 0.0;
  pose_y = 0.0;
  pose_theta = 0.0;
  startMillis = millis();
}

void readSensors() {
  lineLeft = sparki.lineLeft();
  lineRight = sparki.lineRight();
  lineCenter = sparki.lineCenter();
  // distance = sparki.ping();
}

// start time: 1038 end time: 10830
void measure_30cm_speed() {
  sparki.moveForward();
  sparki.clearLCD();
  sparki.print("Time elapsed: ");
  sparki.println(currentMillis - startMillis);
  sparki.updateLCD();
}

void lineFollowing() {
  readSensors();
 
  if (lineCenter < threshold) { // if line is below left line sensor
    sparki.moveForward();
    moving_state = MOVING_FORWARD;
  }
  else {
    if (lineLeft < threshold) { // if line is below left line sensor
      sparki.moveLeft(); // turn left
      moving_state = MOVING_LEFT;
    }
    if (lineRight < threshold) { // if line is below right line sensor
      sparki.moveRight(); // turn right
      moving_state = MOVING_RIGHT;
    }
  }
  
  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);
  
  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);
  
  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);
  
  sparki.updateLCD();
  
}

void updateOdometry() {
  float sparki_speed = 0.0027906976744; // cm/ms
  float radius = 4.285; // cm
  if (moving_state == MOVING_FORWARD) {
    pose_x = pose_x + sparki_speed * CYCLE_TIME * cos(pose_theta);
    pose_y = pose_y + sparki_speed * CYCLE_TIME * sin(pose_theta);
  }
  else if (moving_state == MOVING_LEFT) {
    pose_theta = pose_theta + (sparki_speed / radius) * CYCLE_TIME;
  }
  else if (moving_state == MOVING_RIGHT) {
    pose_theta = pose_theta - (sparki_speed / radius) * CYCLE_TIME;
  }

  if (lineCenter < threshold && lineRight < threshold && lineLeft < threshold)
  {
    pose_x = 0.0;
    pose_y = 0.0;
    pose_theta = 0.0;
  }
  
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("Theta: ");
  sparki.println(pose_theta);
  sparki.updateLCD();
}

void loop() {
  currentMillis = millis();
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      if (currentMillis - startMillis >= CYCLE_TIME) {
        startMillis = currentMillis;
        updateOdometry();
        displayOdometry();
        lineFollowing();
      }
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }
  
  //delay(1000*CYCLE_TIME);
}
