// This is the main program that runs the unicycle control
// gyroAccel handles getting the gyro measurements in from the gyro IMU (6DOF Spark)
// intAngVel integrates angular velocities (uses quaternions) to the roll, pitch and yaw form required

// Carl Edward Rasmussen
// Aleksi Tukiainen, 2016-05-20

// C includes
#include <math.h>

// Platform includes
#include <Arduino.h>
#include <ToneNotes.h>

// Our library includes
#include <messaging.h>

// Local includes
#include "io.h"
#include "pins.h"
#include "policy.h"
#include "intAngVel.h"
#include "gyroAccel.h"
#include "motors.h"
#include "encoders.h"
#include "quat.h"
#include "button.h"
#include "timer.h"
#include "irq_guard.h"

// Kinematic properties
const float WHEEL_CIRC = 0.222;       // circumference of the unicycle wheel (measured)
const float BELT_RATIO = 40/16;      // rotor rotations per wheel rotation
const float GEARBOX_RATIO = 225/16;  // motor rotations per rotor rotation
const float ENCODER_CPR = 512;       // counts per motor revolution

// counts per radian for the turntable and wheel
const float TT_CPRAD = ENCODER_CPR * GEARBOX_RATIO / (2*M_PI);
const float W_CPRAD = ENCODER_CPR * GEARBOX_RATIO * BELT_RATIO / (2*M_PI);
const float W_RADIUS = WHEEL_CIRC / (2 * M_PI);

// control loop properties
const float dt = 50e-3;                  // time step in seconds
const float SPEED_MEASURE_WINDOW = 5e-3; // size of the window used to measure speed

enum class LoopPhase {
  PRE,
  MAIN
};
LoopPhase phase = LoopPhase::PRE;      // phase of main loop

enum class Mode {
  CHANGING,
  IDLE,
  CONTINUOUS,
  BULK,
  MANUAL
};

volatile Mode mode = Mode::IDLE;

// for bulk recording
const int H_max = 50;      // gives the time horizon or "how many time steps will be measured"
struct {
  LogEntry logs[H_max];    //!< log storage
  size_t n = 0;            //!< total number of steps to run
  size_t i = 0;            //!< current step number
  bool run_complete = false; //!< true after a run is complete
  bool run_complete_main = false; //!< true once the main thread has seen the run complete
} bulk;

// for continuous recording
LogEntry singleLog;
bool singleLogPending = false;

// where to save the current data
LogEntry* currLog = &singleLog;

// Type A timer
CallbackTimer ctrl_tmr = io::tmr1;

//! handles computing the overall state
struct StateTracker {
  int16_t oldAngleTT = 0;  // old value of angle for turntable
  int16_t intAngleTT = 0;  // intermediate value of angle for turntable
  float AngleTT = 0.0;     // turn table angular position variable
  int16_t oldAngleW = 0;   // old value of angle for wheel
  int16_t intAngleW = 0;   // intermediate value of angle for wheel
  float AngleW = 0.0;      // wheel angular position variable

  float x_pos = 0;
  float y_pos = 0;

  quat q = quat(1, 0, 0, 0);      // identity quaternion with no rotation

  float w0[3] = {0.0, 0.0, 0.0}; // keeping track of the velocity

  void pre_update() {
    intAngleTT = getTTangle();
    intAngleW = getWangle();
  }

  void update(LogEntry& l) {
    // read the gyro
    float w[3];
    gyroRead(w[0], w[1], w[2]);

    // read the accelerometer
    float ddx, ddy, ddz;          // accelerometer readings [m/s^2]
    accelRead(ddx, ddy, ddz);

    // compute euler angles and their derivatives
    // Here roll pitch and yaw now match x,y,z orientationally (TODO: what?)
    euler_angle orient;
    euler_angle d_orient;
    intAngVel(q, w0, w, orient, d_orient);

    // Turntable angle
    int16_t newAngleTT = getTTangle();
    float dAngleTT     = (newAngleTT - intAngleTT) / (SPEED_MEASURE_WINDOW * TT_CPRAD);
    float deltaAngleTT = (newAngleTT - oldAngleTT) / TT_CPRAD;
    oldAngleTT = newAngleTT;
    AngleTT += deltaAngleTT;

    // Motorwheel angle
    int16_t newAngleW = getWangle();
    float dAngleW     = (newAngleW - intAngleW) / (SPEED_MEASURE_WINDOW * W_CPRAD);
    float deltaAngleW = (newAngleW - oldAngleW) / W_CPRAD;
    oldAngleW = newAngleW;
    AngleW += deltaAngleW;

    // Try the distance calculations (some drift due to yaw (psi))
    float dist = W_RADIUS * (deltaAngleW + d_orient.theta*dt);
    x_pos += dist*cos(orient.psi);
    y_pos += dist*sin(orient.psi);
    float xOrigin =  cos(orient.psi)*-x_pos + sin(orient.psi)*-y_pos;
    float yOrigin = -sin(orient.psi)*-x_pos + cos(orient.psi)*-y_pos;

    // Data recording starts here!
    l.droll  = d_orient.phi;   // roll angular velocity
    l.dyaw   = d_orient.psi;   // yaw angular velocity
    l.dpitch = d_orient.theta; // pitch angular velocity

    l.roll  = orient.phi;      // roll angle
    l.yaw   = orient.psi;      // yaw angle
    l.pitch = orient.theta;    // pitch angle

    l.dAngleW = dAngleW;       // Wheel angular velocity
    l.dAngleTT = dAngleTT;     // turn table angular velocity
    l.xOrigin = xOrigin;       // x position of origin in self centered coord
    l.yOrigin = yOrigin;       // y position of origin in self centered coord
    //l.dx;                    // x velocity
    //l.dy;                    // y velocity
    //l.dxOrigin;              // x velocity of origin in self centered coord
    //l.dyOrigin;              // y velocity of origin in self centered coord
    l.x = x_pos;               // x position
    l.y = y_pos;               // y position
    l.AngleW  = AngleW;        // wheel angle
    l.AngleTT = AngleTT;       // turn table angle
    l.TurntableInput = policyTurntable(l); // control torque for turntable
    l.WheelInput = policyWheel(l); // control torque for wheel
    //-0.2+((float)rand()/(float)(RAND_MAX))*0.2;

    // We may need the accelerations for calibrating the start measurements
    l.ddx = ddx;
    l.ddy = ddy;
    l.ddz = ddz;
  }
};

StateTracker state_tracker;

// Interrupt handlers begin

void __attribute__((interrupt)) mainLoop(void) {
  // main timer that keeps track of the 50ms period and perfoms the key functionality
  clearIntFlag(ctrl_tmr.irq);

  // if we're changing mode, it's not safe to access any other mode variables
  if(mode == Mode::CHANGING) return;

  //this 5ms window is used for speed measurement
  if (phase == LoopPhase::PRE) {
    ctrl_tmr.setPeriod(SPEED_MEASURE_WINDOW);
    phase = LoopPhase::MAIN;

    state_tracker.pre_update();
  }
  // the remaining window is used for collecting the rest of the data and computing output
  else {
    ctrl_tmr.setPeriod(dt - SPEED_MEASURE_WINDOW);
    phase = LoopPhase::PRE;

    // choose where to store data
    currLog = &singleLog;
    if(mode == Mode::BULK) {
      if(bulk.i < bulk.n) {
        currLog = &(bulk.logs[bulk.i++]);
      }
      else {
        mode = Mode::IDLE;
        digitalWrite(pins::LED, LOW);
        bulk.run_complete = true;
      }
    }
    state_tracker.update(*currLog);

    // update the motor outputs
    if (mode == Mode::CONTINUOUS || mode == Mode::BULK) {
      setMotorTurntable(currLog->TurntableInput);
      setMotorWheel(currLog->WheelInput);
    }
    else {
      setMotorTurntable(0);
      setMotorWheel(0);
    }

    // instruct the main thread to send a message
    if(currLog == &singleLog)
      singleLogPending = true;
  }
}

void play_starting_noise() {
  beep(NOTE_G6, 200);
  delay(50);
  beep(NOTE_D7, 200);
  delay(50);
  beep(NOTE_F7, 200);
  delay(50);
  beep(NOTE_FS7, 200);
  for(int i = 0; i < 3; i++) {
    delay(50);
    beep(NOTE_G7, 200);
  }
  delay(250);
}


void play_ending_noise() {
  beep(NOTE_G7, 100);
  delay(25);
  beep(NOTE_G6, 100);
  delay(25);
  beep(NOTE_G7, 225);
  delay(25);
}

void request_stop() {
  {
    // disable timer irq in here
    irq_guard g(ctrl_tmr.irq);
    if (mode == Mode::BULK) {
      bulk.n = bulk.i;
    }
    else {
      mode = Mode::IDLE;
    }
  }
  digitalWrite(pins::LED, LOW);
}

// set up the message handlers
auto on_go = [](const Go& go) {
  // default to the maximum number of steps
  ssize_t n = go.steps;
  if(n == 0) n = H_max;
  if(n > H_max) n = H_max;

  // lock the background loop so we can change mode
  ctrl_tmr.stop();
  mode = Mode::CHANGING;

  // reset the state
  state_tracker = StateTracker();
  phase = LoopPhase::PRE;

  resetEncoders();

  // compute the new mode
  Mode target;
  bulk.i = 0;
  bulk.run_complete = false;
  if(n < 0) {
    bulk.n = 0;
    target = Mode::CONTINUOUS;
    logging::info("Request for continuous mode");
  }
  else {
    bulk.n = n;
    target = Mode::BULK;
    logging::info("Request for bulk mode");
  }

  logging::info("Waiting for button press");
  button::awaitPress();

  logging::info("Starting");
  play_starting_noise();

  // enter the new mode
  mode = target;
  ctrl_tmr.start();
  digitalWrite(pins::LED, HIGH);
};
auto on_stop = [](const Stop& stop) {
  request_stop();
  logging::info("Stopped by remote command!");
};
auto on_get_logs = [](const GetLogs& getLogs) {
  if(bulk.run_complete) {
    logging::info("Sending test data");
    sendLogBundle(bulk.logs, bulk.n);
  }
  else {
    logging::info("No data yet");
    sendLogBundle(bulk.logs, 0);
  }
};

// main function to setup the test
void setup() {
  setupMessaging();
  logging::info("Yaw Actuated UniCycle startup");

  onMessage<Go>(&on_go);
  onMessage<Stop>(&on_stop);
  onMessage<Controller>(setPolicy);
  onMessage<GetLogs>(&on_get_logs);

  pinMode(pins::LED, OUTPUT);
  digitalWrite(pins::LED, LOW);

  button::setup();

  //srand(54321);

  logging::info("Starting PWM setup");
  setupMotors();
  setMotorTurntable(0);
  setMotorWheel(0);

  logging::info("Starting I2C setup");
  gyroAccelSetup();

  logging::info("Starting encoder setup");
  setupEncoders();

  // twitch both the turntable and wheel, so that we know things are working
  // setMotorTurntable(-0.1);
  // setMotorWheel(-0.1);
  // delay(100);
  // setMotorTurntable(0.1);
  // setMotorWheel(0.1);
  // delay(100);
  // setMotorTurntable(0);
  // setMotorWheel(0);
  // delay(100);

  delay(2000);

  float dx, dy, dz;
  gyroRead(dx, dy, dz);

  logging::info("All done");

  // start the control loop timer
  ctrl_tmr.setup();
  ctrl_tmr.attach(mainLoop);
  ctrl_tmr.start();
}

void loop() {
  updateMessaging();

  // this can't be sent in an interrupt handler
  if(singleLogPending && mode == Mode::CONTINUOUS) {
    sendLog(singleLog);
    singleLogPending = false;
  }

  // log that the test was complete
  if(bulk.run_complete && !bulk.run_complete_main) {
    logging::info("Test completed");
  }
  bulk.run_complete_main = bulk.run_complete;

  // allow e-stop
  if (mode != Mode::IDLE && button::isPressed()) {
    request_stop();
    while (button::isPressed());
    logging::warn("Stopped by on-board button!");
    play_ending_noise();
  }
}
