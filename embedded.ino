// This is the main program that runs the unicycle control
// gyroAccel handles getting the gyro measurements in from the gyro IMU (6DOF Spark)
// intAngVel integrates angular velocities (uses quaternions) to the roll, pitch and yaw form required

// Carl Edward Rasmussen
// Aleksi Tukiainen, 2016-05-20

#include <math.h>
#include "policy.h"
#include "intAngVel.h"
#include "ToneNotes.h"

#include "gyroAccel.h"
#include "motors.h"
#include "encoders.h"

int mode = 'R';

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

int phase = 0;                // phase of main loop

float dx, dy, dz;             // rate gyro readings [degrees per sec]

static bool counter = false;   // this Boolean could be controller by a button (to start the experiment)
static int count = 0;         // variable to keep track of how many measurements have been done
const int H = 50;             // gives the time horizon or "how many time steps will be measured"

// Storage arrays in same order as in unicycle "doitnlds" file
struct LogEntry {
  float droll;            // 1   roll angular velocity
  float dyaw;             // 2   yaw angular velocity
  float dAngleW;          // 3   Wheel angular velocity
  float dpitch;           // 4   pitch angular velocity
  float dAngleTT;         // 5   turn table angular velocity
  float xOrigin;          // 6   x position of origin in self centered coord
  float yOrigin;          // 7   y position of origin in self centered coord
  float roll;             // 8   roll angle
  float yaw;              // 9   yaw angle
  float pitch;            // 10  pitch angle
  //float dx;             // 11  x velocity
  //float dy;             // 12  y velocity
  //float dxOrigin;       // 13  x velocity of origin in self centered coord
  //float dyOrigin;       // 14  y velocity of origin in self centered coord
  float x;                // 15  x position
  float y;                // 16  y position
  float AngleW;           // 17  wheel angle
  float AngleTT;          // 18  turn table angle
  float TurntableInput;   // 19  control torque for turntable (here we record what is demanded, not what provided due to duty maximum of 1)
  float WheelInput;       // 20  control torque for wheel (here we record what is demanded, not what provided due to duty maximum of 1)

  // We may need the accelerations for calibrating the start measurements
  float ddx;
  float ddy;
  float ddz;
};


LogEntry logArray[H];

// Type A timer
p32_timer& tmr1 = *reinterpret_cast<p32_timer*>(_TMR1_BASE_ADDRESS);

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

  if (count == 0) {
    counter = true;
  }

  //this 5ms window is used for speed measurement
  if (phase == 0) {
    tmr1.tmxPr.reg = static_cast<uint16_t>(SPEED_MEASURE_WINDOW * F_CPU / 256);   // clock divisor is 256
    intAngleTT = getTTangle();
    intAngleW = getWangle();

    phase = 1;
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

    phase = 0;
    if (count >= H) {
      counter = false;
      digitalWrite(PIN_LED1, LOW);
    } // recording only over the time horizon
    if (counter) {
      // Data recording starts here!
      logArray[count].droll = droll;         // 1   roll angular velocity
      logArray[count].dyaw = dyaw;           // 2   yaw angular velocity
      logArray[count].dAngleW = dAngleW;     // 3   Wheel angular velocity
      logArray[count].dpitch = dpitch;       // 4   pitch angular velocity
      logArray[count].dAngleTT = dAngleTT;   // 5   turn table angular velocity
      logArray[count].xOrigin = xOrigin;     // 6   x position of origin in self centered coord
      logArray[count].yOrigin = yOrigin;     // 7   y position of origin in self centered coord
      logArray[count].roll = roll;           // 8   roll angle
      logArray[count].yaw = yaw;             // 9   yaw angle
      logArray[count].pitch = pitch;         // 10  pitch angle
      //logArray[count].dx;                  // 11  x velocity
      //logArray[count].dy;                  // 12  y velocity
      //logArray[count].dxOrigin;            // 13  x velocity of origin in self centered coord
      //logArray[count].dyOrigin;            // 14  y velocity of origin in self centered coord
      logArray[count].x = x_pos;             // 15  x position
      logArray[count].y = y_pos;             // 16  y position
      logArray[count].AngleW = AngleW;       // 17  wheel angle
      logArray[count].AngleTT = AngleTT;     // 18  turn table angle
      logArray[count].TurntableInput = policyTurntable(droll, dyaw, dAngleW, dpitch, dAngleTT, xOrigin, yOrigin, roll, yaw, pitch); // 19  control torque for turntable
      logArray[count].WheelInput = policyWheel(droll, dyaw, dAngleW, dpitch, dAngleTT, xOrigin, yOrigin, roll, yaw, pitch); // 20  control torque for wheel
      //-0.2+((float)rand()/(float)(RAND_MAX))*0.2;

      // We may need the accelerations for calibrating the start measurements
      logArray[count].ddx = ddx;
      logArray[count].ddy = ddy;
      logArray[count].ddz = ddz;

      count += 1;
    }
  } //end of else
}

void debug(const char* s) {
  Serial.println(s);
}

// main function to setup the test
void setup() {
  Serial.begin(57600);
  debug("Yaw Actuated UniCycle startup");

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
  gyroRead(dx, dy, dz);
  digitalWrite(PIN_LED1, HIGH);

  debug("All done");

  // make a starting noise
  beep(NOTE_G6, 200);
  delay(50);
  beep(NOTE_D7, 200);
  delay(50);
  beep(NOTE_F7, 200);
  delay(50);
  beep(NOTE_FS7, 200);
  delay(50);
  beep(NOTE_G7, 450);

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

void printField(float LogEntry::* field, const char* name, bool debug) {
  if(debug) {
    Serial.print(name);
    Serial.print(": ");
  }
  for (int k = 0; k < H; k++) {
    Serial.print(logArray[k].*field, 4);
    Serial.print(',');
  }
  Serial.println("");
}

void loop() {

  // This part is for testing purposes
  // Serial.begin();
  // delay(1000);

  if (counter)
  {
    setMotorTurntable(logArray[count-1].TurntableInput);    // count-1 because at the end of it doing the maths and then storing the value
    setMotorWheel(logArray[count-1].WheelInput);            // the counter counts up. Thus the value stored and implemented match
  }
  else
  {
    setMotorTurntable(0);
    setMotorWheel(0);
  }

//  if (counter == false) {
//    Serial.println('c');
//    for (int k=0; k<H; k++) {
//      Serial.println(mode);
//      while (mode == 'R') {
//          mode = Serial.read();
//          Serial.println('R2');
//      }
//      if (mode == 'W') {Serial.println(logArray[k].pitch,4);}
//      mode='R';
//    }
//  }

  // End of testing

  // Controllers


  // Data to serial port
  // This makes sure we only show these after we've done the measurements
  if (Serial.available() > 0) {
    mode = Serial.read();
    bool sendSomething = false;
    bool sendDebug = false;
    if (mode == 'W') {
      sendSomething = true;
    }
    else if (mode == 'w') {
      sendSomething = true;
      sendDebug = true;
    }

    if(sendSomething) {
      Serial.println(dx, 4);
      Serial.println(dy, 4);
      Serial.println(dz, 4);
      Serial.println(' ');

      #define PRINT_FIELD(name) printField(&LogEntry:: name, #name, sendDebug)
        PRINT_FIELD(droll);
        PRINT_FIELD(dyaw);
        PRINT_FIELD(dAngleW);
        PRINT_FIELD(dpitch);
        PRINT_FIELD(dAngleTT);
        PRINT_FIELD(xOrigin);
        PRINT_FIELD(yOrigin);
        PRINT_FIELD(roll);
        PRINT_FIELD(yaw);
        PRINT_FIELD(pitch);
        PRINT_FIELD(x);
        PRINT_FIELD(y);
        PRINT_FIELD(AngleW);
        PRINT_FIELD(AngleTT);
        PRINT_FIELD(TurntableInput);
        PRINT_FIELD(WheelInput);
        PRINT_FIELD(ddx);
        PRINT_FIELD(ddy);
        PRINT_FIELD(ddz);
      #undef PRINT_FIELD

      mode = 'R';
    }
  }
}

