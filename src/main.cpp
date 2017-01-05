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
#include "policy.h"
#include "intAngVel.h"
#include "gyroAccel.h"
#include "motors.h"
#include "encoders.h"

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
  bool data_pending = false; //!< true after a run is complete
} bulk;

// for continuous recording
LogEntry singleLog;

// where to save the current data
LogEntry* currLog = &singleLog;

// Type A timer
volatile p32_timer& tmr1 = *reinterpret_cast<volatile p32_timer*>(_TMR1_BASE_ADDRESS);

// Interrupt handlers begin

void __attribute__((interrupt)) mainLoop(void) {
  // main timer that keeps track of the 50ms period and perfoms the key functionality

  static int16_t oldAngleTT = 0;  // old value of angle for turntable
  static int16_t intAngleTT = 0;  // intermediate value of angle for turntable
  static float AngleTT = 0.0;     // turn table angular position variable
  static int16_t oldAngleW = 0;   // old value of angle for wheel
  static int16_t intAngleW = 0;   // intermediate value of angle for wheel
  static float AngleW = 0.0;      // wheel angular position variable

  static float x_pos = 0;
  static float y_pos = 0;

  clearIntFlag(_TIMER_1_IRQ);

  // if we're changing mode, it's not safe to access any other mode variables
  if(mode == Mode::CHANGING) return;

  //this 5ms window is used for speed measurement
  if (phase == LoopPhase::PRE) {
    tmr1.tmxPr.reg = static_cast<uint16_t>(SPEED_MEASURE_WINDOW * F_CPU / 256);   // clock divisor is 256
    intAngleTT = getTTangle();
    intAngleW = getWangle();

    phase = LoopPhase::MAIN;
  } else {
    tmr1.tmxPr.reg = static_cast<uint16_t>((dt - SPEED_MEASURE_WINDOW) * F_CPU / 256);

    // read the gyro
    float w[3];
    gyroRead(w[0], w[1], w[2]);

    // read the accelerometer
    float ddx, ddy, ddz;          // accelerometer readings [m/s^2]
    accelRead(ddx, ddy, ddz);

    // integrate angles
    float roll, pitch, yaw;       // Euler angles of unicycle attitude
    float droll, dpitch, dyaw;    // angular velocities
    intAngVel(w, roll, pitch, yaw, droll, dpitch, dyaw); //Here roll pitch and yaw now match x,y,z orientationally

    // Turntable angle - Note: May be spinning to the wrong direction (according to convetion), but it doesn't matter for learning
    int16_t newAngleTT = getTTangle();
    float dAngleTT = (newAngleTT - intAngleTT) / (SPEED_MEASURE_WINDOW * TT_CPRAD); 
    AngleTT += static_cast<int16_t>(newAngleTT - oldAngleTT) / TT_CPRAD;
    oldAngleTT = newAngleTT;

    // Motorwheel angle
    int16_t newAngleW = getWangle();
    float dAngleW = (newAngleW - intAngleW) / (SPEED_MEASURE_WINDOW * W_CPRAD);
    AngleW += static_cast<int16_t>(newAngleW - oldAngleW) / W_CPRAD;
    oldAngleW = newAngleW;

    // Try the distance calculations (some drift due to yaw)
    float dist = W_RADIUS * ((newAngleW - oldAngleW) / W_CPRAD + dpitch*dt);
    x_pos += dist*cos(yaw);
    y_pos += dist*sin(yaw);
    float xOrigin = cos(yaw)*-x_pos + sin(yaw)*-y_pos;
    float yOrigin = -sin(yaw)*-x_pos + cos(yaw)*-y_pos;

    phase = LoopPhase::PRE;

    // choose where to store data
    currLog = &singleLog;
    if(mode == Mode::BULK) {
      if(bulk.i < bulk.n) {
        currLog = &(bulk.logs[bulk.i++]);
      }
      else {
        mode = Mode::IDLE;
        bulk.data_pending = true;
        digitalWrite(PIN_LED1, LOW);
      }
    }

    // Data recording starts here!
    LogEntry& l = *currLog;
    l.droll = droll;         // 1   roll angular velocity
    l.dyaw = dyaw;           // 2   yaw angular velocity
    l.dAngleW = dAngleW;     // 3   Wheel angular velocity
    l.dpitch = dpitch;       // 4   pitch angular velocity
    l.dAngleTT = dAngleTT;   // 5   turn table angular velocity
    l.xOrigin = xOrigin;     // 6   x position of origin in self centered coord
    l.yOrigin = yOrigin;     // 7   y position of origin in self centered coord
    l.roll = roll;           // 8   roll angle
    l.yaw = yaw;             // 9   yaw angle
    l.pitch = pitch;         // 10  pitch angle
    //l.dx;                  // 11  x velocity
    //l.dy;                  // 12  y velocity
    //l.dxOrigin;            // 13  x velocity of origin in self centered coord
    //l.dyOrigin;            // 14  y velocity of origin in self centered coord
    l.x = x_pos;             // 15  x position
    l.y = y_pos;             // 16  y position
    l.AngleW = AngleW;       // 17  wheel angle
    l.AngleTT = AngleTT;     // 18  turn table angle
    l.TurntableInput = policyTurntable(l); // 19  control torque for turntable
    l.WheelInput = policyWheel(l); // 20  control torque for wheel
    //-0.2+((float)rand()/(float)(RAND_MAX))*0.2;

    // We may need the accelerations for calibrating the start measurements
    l.ddx = ddx;
    l.ddy = ddy;
    l.ddz = ddz;
  } //end of else
}

void play_starting_noise() {
  beep(NOTE_G6, 200);
  delay(50);
  beep(NOTE_D7, 200);
  delay(50);
  beep(NOTE_F7, 200);
  delay(50);
  beep(NOTE_FS7, 200);
  delay(50);
  beep(NOTE_G7, 450);
}

// set up the message handlers
auto on_go = [](const Go& go) {
  // default to the maximum number of steps
  ssize_t n = go.steps;
  if(n == 0) n = H_max;
  if(n > H_max) n = H_max;

  // lock the background loop so we can change mode
  mode = Mode::CHANGING;

  // compute the new mode
  Mode target;
  bulk.i = 0;
  bulk.data_pending = false;
  if(n < 0) {
    bulk.n = 0;
    target = Mode::CONTINUOUS;
    debug("Starting continuous mode");
  }
  else {
    bulk.n = n;
    target = Mode::BULK;
    debug("Starting bulk mode");
  }

  play_starting_noise();

  // enter the new mode
  mode = target;
};
auto on_stop = [](const Stop& stop) {
  mode = Mode::IDLE;

  bulk.i = 0;
  bulk.n = 0;
  debug("Stop!");
};

// main function to setup the test
void setup() {
  setupMessaging();
  debug("Yaw Actuated UniCycle startup");

  onMessage<Go>(&on_go);
  onMessage<Stop>(&on_stop);
  onMessage<SetController>(setPolicy);

  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, LOW);

  //srand(54321);

  debug("Starting PWM setup");
  setupMotors();
  setMotorTurntable(0);
  setMotorWheel(0);

  debug("Starting I2C setup");
  gyroAccelSetup();

  debug("Starting encoder setup");
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

  debug("All done");

  // start the control loop timer
  tmr1.tmxCon.reg = TACON_SRC_INT | TACON_PS_256;
  tmr1.tmxTmr.reg = 0;
  tmr1.tmxPr.reg = 0xffff;
  tmr1.tmxCon.set = TACON_ON;

  // set up interrupts on the control loop timer
  clearIntFlag(_TIMER_1_IRQ);
  setIntVector(_TIMER_1_IRQ, mainLoop);
  setIntPriority(_TIMER_1_IRQ, 2, 0);
  setIntEnable(_TIMER_1_IRQ);
}

void loop() {
  updateMessaging();

  if (mode == Mode::CONTINUOUS || mode == Mode::BULK) {
    setMotorTurntable(currLog->TurntableInput);    // count-1 because at the end of it doing the maths and then storing the value
    setMotorWheel(currLog->WheelInput);            // the outputEnabled counts up. Thus the value stored and implemented match
  }
  else {
    setMotorTurntable(0);
    setMotorWheel(0);
  }
  if(bulk.data_pending) {
    // temporary until we get the terminal sending
    debug("Test complete");
    sendLogs(bulk.logs, bulk.n);
    bulk.data_pending = false;
  }
  else if(mode == Mode::CONTINUOUS) {
    sendLogs(&singleLog, 1);
  }
}
