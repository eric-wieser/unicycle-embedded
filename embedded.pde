// This is the main program that runs the unicycle control
// gyroAccel handles getting the gyro measurements in from the gyro IMU (6DOF Spark)
// intAngVel integrates angular velocities (uses quaternions) to the roll, pitch and yaw form required

// Carl Edward Rasmussen
// Aleksi Tukiainen, 2016-05-20

#include <plib.h>
#include <math.h>
#include "policy.h"
#include "gyroAccel.h"
#include "intAngVel.h"

int mode = 'R';
const float dt = 0.05;        // time step in seconds
const float cw = 0.222;       // circumference of the unicycle wheel (measured)

int TMR3sign = 1;             // sign of the value in TMR3
int TMR4sign = 1;             // sign of the value in TMR4
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

extern "C" {

// main timer that keeps track of the 50ms period and perfoms the key funtionality
void __ISR(_TIMER_1_VECTOR, ipl2) mainLoop(void)
{
  static short int oldAngleTT = 0;  // old value of angle for turntable
  static short int intAngleTT = 0;  // intermediate value of angle for turntable
  short int newAngleTT;
  static short int oldAngleW = 0;   // old value of angle for wheel
  static short int intAngleW = 0;   // intermediate value of angle for wheel
  short int newAngleW;
  float w[3];                       // storage of the gyro spin values

  float AngleTT = 0.0;          // turn table angular position variable
  float dAngleTT = 0.0;         // turn table angular velocity variable

  float AngleW = 0.0;           // wheel angular position variable
  float dAngleW = 0.0;          // wheel angular velocity variable

  float ddx, ddy, ddz;          // accelerometer readings [m/s^2]
  float roll, pitch, yaw;       // Euler angles of unicycle attitude
  float droll, dpitch, dyaw;    // angular velocities

  float dist, x_pos, y_pos;     // distance travelled linearly at a given time step, x-position and y-position
  float xOrigin, yOrigin;       // variables for finding origin


  mT1ClearIntFlag();

  if (count == 0) {
    counter = true;
  }

  //this 5ms window is used for speed measurement
  if (phase == 0) {
    WritePeriod1(1563);   // 5 ms (1563*3.2us = 5 ms, where 3.2us is 256/80MHz)
    intAngleTT = TMR3sign * TMR3; // TMR3 and thus AngleTT are to do with the turntable
    intAngleW = TMR4sign * TMR4; // TMR4 and thus AngleW are to do with the wheel

    phase = 1;
  } else {
    WritePeriod1(14062);  // 45 ms (the period is varied between 5ms and 45ms by rewriting the interrupt period)

    gyroRead(w[0], w[1], w[2]);
    intAngVel(w, roll, pitch, yaw, droll, dpitch, dyaw); //Here roll pitch and yaw now match x,y,z orientationally
    accelRead(ddx, ddy, ddz);

    // Turntable angle - Note: May be spinning to the wrong direction (according to convetion), but it doesn't matter for learning
    newAngleTT = (short int) TMR3 * TMR3sign;
    // factor 1145.9156 = 512*225/16/(2*pi): counts/degrees = counts/revolution*[gearing_factor]/(radians/revolution)
    AngleTT += ((short int) (newAngleTT - oldAngleTT)) / 1145.9156;  // in radians
    // factor 0.1745 = 200/1145.9156: (counts/5ms*1000ms/s/[1145.9156 counts/radian])
    dAngleTT = 0.1745 * (newAngleTT - intAngleTT);          // in radians per s
    oldAngleTT = newAngleTT;

    // Motorwheel angle
    newAngleW = (short int) TMR4 * TMR4sign;
    // factor 2864.7890 = 1145.9156*40/16 (40/16 comes from additional gearing on the motor wheel belt)
    AngleW += ((short int) (newAngleW - oldAngleW)) / 2864.7890;
    // factor 0.0698 = 0.1745/40*16 (40/16 comes from additional gearing on the motor wheel belt)
    dAngleW = 0.0698 * (newAngleW - intAngleW);

    // Try the distance calculations (some drift due to yaw)
    dist=cw*(((newAngleW - oldAngleW) / 2864.7890)+dpitch*dt)/(2*M_PI);
    oldAngleW = newAngleW;
    x_pos += dist*cos(yaw);
    y_pos += dist*sin(yaw);
    xOrigin = cos(yaw)*-x_pos+sin(yaw)*-y_pos;
    yOrigin = -sin(yaw)*-x_pos+cos(yaw)*-y_pos;

    phase = 0;
    if (count >= H) {
      counter = false;
      digitalWrite(13, LOW);
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

// This interrupt flag is for keeping track of the TMR3&4 directions, flagged if direction changes
// They are counters to keep track of the two motor angles, so if direction changes,
// the counters need to count down rather than up
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) signChange3(void)
{
  int state = PORTD;        // Read state and clear mismatch condition
  mCNClearIntFlag();        // clear interrupt flag

  int negMotor1 = (state & 32) > 0 ? -1 : 1;
  int negMotor2 = (state & 64) > 0 ? -1 : 1;

  // Serial.print("____________________CN: "); // change notice vector gives information on a change
  // Serial.print(negMotor1);
  // Serial.println(negMotor2);

  if (TMR3sign != negMotor1) { // negMotor1 is the turntable motor
    TMR3 = (short int) -TMR3; // negate counter
  }
  TMR3sign = negMotor1;      // keep track of sign

  if (TMR4sign != negMotor2) { //negMotor2 is the wheel motor
    TMR4 = (short int) -TMR4;
  }
  TMR4sign = negMotor2;
}

void setupPWM() {
  INTEnableSystemMultiVectoredInt();// Enable system wide interrupt to multivectored mode.

  OC1CON = 0x0000;      // Turn off the OC1 when performing the setup
  OC1R = 0x0000;        // Initialize primary Compare register
  OC1RS = 0x0000;       // Initialize secondary Compare register
  OC1CON = 0x0006;      // Configure for PWM mode without Fault pin enabled

  OC2CON = 0x0000;      // Turn off the OC2 when performing the setup
  OC2R = 0x0000;        // Initialize primary Compare register
  OC2RS = 0x0000;       // Initialize secondary Compare register
  OC2CON = 0x0006;      // Configure for PWM mode without Fault pin enabled

  OC3CON = 0x0000;      // Turn off the OC3 when performing the setup
  OC3R = 0x0000;        // Initialize primary Compare register
  OC3RS = 0x0000;       // Initialize secondary Compare register
  OC3CON = 0x0006;      // Configure for PWM mode without Fault pin enabled

  OC4CON = 0x0000;      // Turn off the OC4 when performing the setup
  OC4R = 0x0000;        // Initialize primary Compare register
  OC4RS = 0x0000;       // Initialize secondary Compare register
  OC4CON = 0x0006;      // Configure for PWM mode without Fault pin enabled

  PR2 = 0xFFFF;         // Set period

  // Configure Timer2 interrupt. Note that in PWM mode, the
  // corresponding source timer interrupt flag is asserted.
  // OC interrupt is not generated in PWM mode.

  IFS0CLR = 0x00000100; // Clear the T2 interrupt flag
  IEC0SET = 0x00000100; // Enable T2 interrupt
  IPC2SET = 0x0000001C; // Set T2 interrupt priority to 7
  T2CONSET = 0x8000;    // Enable Timer2
  OC1CONSET = 0x8000;   // Enable OC1
  OC2CONSET = 0x8000;   // Enable OC2
  OC3CONSET = 0x8000;   // Enable OC3
  OC4CONSET = 0x8000;   // Enable OC4
}

// Example code for Timer2 ISR
void __ISR(_TIMER_2_VECTOR, ipl7) T2_IntHandler (void) {
  // The timer 2 is currently not in use (doesn't do anything)
  IFS0CLR = 0x0100;// Clearing Timer2 interrupt flag
}

} // end extern C

// setup functions and main loop below

// function definitions for setting the motor duty cycle
void setMotorTurntable(int period, float cmd) {
  int duty;
  duty = round(period * abs(cmd));

  // if cmd too small or large it will be set to minimum or maximum
  if(cmd < 0) {
    OC1RS = 0x0000;
    OC2RS = duty;
  } else {
    OC1RS = duty;
    OC2RS = 0x0000;
  }
}

void setMotorWheel(int period, float cmd) {
  int duty;
  duty = round(period * abs(cmd));

  if(cmd < 0) {
    OC3RS = 0x0000;
    OC4RS = duty;
  } else {
    OC3RS = duty;
    OC4RS = 0x0000;
  }
}

// main function to setup the test
void setup() {
  Serial.begin(57600);
//    char a = 'b';

  pinMode(13, OUTPUT); // LED pin 13 is digital output
  digitalWrite(13, LOW);

  //srand(54321);

  setupPWM();
  setMotorTurntable(PR2, 0);
  setMotorWheel(PR2, 0);

  OpenI2C1(I2C_EN, 0x062); // I2C at 400 KHz

  gyroWrite(0x3e, 0x80);  // Reset to defaults
  gyroWrite(0x16, 0x19);  // DLPF_CFG = 1 (188 Hz LP), FS_SEL = 3

  accelWrite(0x31, 0x0f); // data format, 13 bits, left justified, +/-16g range
  accelWrite(0x2c, 0x0b); // measurement rate, 200 Hz
  accelWrite(0x2d, 0x08); // power ctrl, enable measurements

  delay(1500);             // wait for gyro to get ready (setup)

  mCNOpen(CN_ON | CN_IDLE_CON, CN14_ENABLE | CN15_ENABLE, CN_PULLUP_DISABLE_ALL);
PORTD: // moved
  ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2); //should this be INT_PRIOR_2  ?
  setMotorTurntable(PR2, -0.1);
  setMotorWheel(PR2, -0.1);
  delay(100);
  setMotorTurntable(PR2, 0.1);
  setMotorWheel(PR2, 0.1);
  delay(100);
  setMotorTurntable(PR2, 0);
  setMotorWheel(PR2, 0);
  delay(100);
  OpenTimer3(T3_ON | T3_PS_1_1 | T3_SOURCE_EXT, 0xffff); // T3 external source is the pulse from the turntable
  WriteTimer3(0);
  OpenTimer4(T4_ON | T3_PS_1_1 | T4_SOURCE_EXT, 0xffff); // T4 external source is the pulse from the wheel
  WriteTimer4(0);
  delay(2000);
  gyroRead(dx, dy, dz);
  digitalWrite(13, HIGH);   // set the LED on

//    Serial.println('a');
//    while (a != 'a')
//    {
//      a = Serial.read();
//    }

  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 0xffff);
  WriteTimer1(0);
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

  INTEnableSystemMultiVectoredInt(); // setup for multi-vectored interrupts

}

void loop() {

  // This part is for testing purposes
  // Serial.begin();
  // delay(1000);

  if (counter)
  {
    setMotorTurntable(PR2, logArray[count-1].TurntableInput);    // count-1 because at the end of it doing the maths and then storing the value
    setMotorWheel(PR2, logArray[count-1].WheelInput);            // the counter counts up. Thus the value stored and implemented match
  }
  else
  {
    setMotorTurntable(PR2, 0);
    setMotorWheel(PR2, 0);
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
    if (mode == 'W') {
      Serial.println(dx, 4);
      Serial.println(dy, 4);
      Serial.println(dz, 4);
      Serial.println(' ');
      //Serial.print('1');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].droll, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('2');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].dyaw, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('3');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].dAngleW, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('4');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].dpitch, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('5');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].dAngleTT, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('6');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].xOrigin, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('7');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].yOrigin, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('8');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].roll, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('9');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].yaw, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('10');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].pitch, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('11');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].x, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('12');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].y, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('13');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].AngleW, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('14');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].AngleTT, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('15');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].TurntableInput, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('16');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].WheelInput, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('17');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].ddx, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('18');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].ddy, 4);
        Serial.print(',');
      }
      Serial.println(' ');
      //Serial.print('19');
      for (int k = 0; k < H; k++) {
        Serial.print(logArray[k].ddz, 4);
        Serial.print(',');
      }
      Serial.println();
      mode = 'R';
    }
  }
}
