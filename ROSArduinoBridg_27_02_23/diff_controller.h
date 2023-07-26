   /* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> ZL;
CAN_message_t msg;
CAN_message_t rec;
CAN_message_t rec1;
int ErrorCode = 999, timeOut = 0, timeOut1 = 0;
int estop_FB_to_NUC = 0, _maxSpeed = 150;
int16_t speedl_data = 0, speedr_data = 0;

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
    Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
    Using integrated term (ITerm) instead of integrated error (Ierror),
    to allow tuning changes,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20; //20;
int Kd = 12; //12;
int Ki = 0;
int Ko = 50; //50;

unsigned char moving = 0; // is the base in motion?

void quick_stop_response(int8_t option)
{
  /*
    5 : normal stop
    6 : decelerate to stop
    7 : Emergency stop
  */
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x5A;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = option;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
}

void en_motor()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 301;
      break;
    }
  }
  msg.buf[4] = 0x07;
  ZL.write(msg);
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  //  // Serial1.print(" en_motor()");
}

void stop_motor()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 302;
      break;
    }
  }
  //  // Serial1.print(" stop_motor()");
}

void clear_error()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0X80;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
  }
}

void init_can() {
  ZL.begin(); ZL.setBaudRate(500000);
  msg.id = 0x601;  msg.len = 8;
}


byte emergency_switch_read()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x41;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  ZL.read(rec);
  timeOut1 = millis();
  while (rec.buf[1] != 0x41 && rec.buf[2] != 0x60) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 304;
      break;
    }
  }
  if (rec.buf[4] == 0x27)
  {
    return (rec.buf[5] >> 7); // 0b0x ; x : 0 switch open ; 1 switch presssed
  }
  return 0b11; // not applicable
}

void clear_encoder()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x05;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x03;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x05) || (rec.buf[2] != 0x20)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 303;
      break;
    }
  }
  // Serial1.print("clear_encoder");
}

uint32_t speed_read()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x6C;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x03;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x6C) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x03)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 310;
      break;
    }
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4]; // High 16 left, Low 16 right
}
long encoder_read_left() {
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x64;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x64) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 317;
      break;
    }
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}

long encoder_read_right() {
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x64;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x02;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x64) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x02)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 318;
      break;
    }
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}

void drive_vel_motor_straight(int16_t speedl, int16_t speedr)
{
  /*
     controlle speed according 100 rpm. and for error solving max limit +- 50.
  */

  /* pid speed filter 24/11/22 12:43pm */
  /*if (speedl > (_maxSpeed + 30)) speedl = (_maxSpeed + 30);
  else if (speedl < (-_maxSpeed - 30)) speedl = (-_maxSpeed - 30);

  if (speedr > (_maxSpeed + 30)) speedr = (_maxSpeed + 30);
  else if (speedr < (-_maxSpeed - 30)) speedr = (-_maxSpeed - 30);

  /* final speed filter 24/11/22 12:43pm */
  /*if (speedl > 280 ) speedl = 280;
  else if (speedl < -280) speedl = -280;
  if (speedr > 280 ) speedr = 280;
  else if (speedr < -280) speedr = -280;*/


  speedl_data = speedl;
  speedr_data = speedr;
  Serial7.print("l=");
  Serial7.print(speedl);
  Serial7.print(" r=");
  Serial7.println(speedr);

  msg.buf[0] = 0x23;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x03;
  msg.buf[4] = (speedl ^ 0xFFFF) + 1;
  msg.buf[5] = ((speedl ^ 0xFFFF) + 1) >> 8;
  msg.buf[6] = speedr;
  msg.buf[7] = speedr >> 8;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0xFF) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x03)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 309;
      break;
    }
  }
}
/*
  Initialize PID variables to zero to prevent startup spikes
  when turning PID on to start moving
  In particular, assign both Encoder and PrevEnc the current encoder value
  See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
  Note that the assumption here is that PID is only turned on
  when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID() {
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = -encoder_read_left(); //readEncoder(LEFT);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = encoder_read_right(); //readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
    Avoid derivative kick and allow tuning changes,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;


  // output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  Kp = 3;
  output = (Kp * Perror) / 10;

  p->PrevEnc = p->Encoder;

  //output += p->output;

  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  /*
    if (output >= MAX_PWM)
    output = MAX_PWM;
    else if (output <= -MAX_PWM)
    output = -MAX_PWM;
    else
    /*
      allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  // p->ITerm += Ki * Perror;


  p->output = output;

  Serial7.print(" ip=");
  Serial7.print(input);
  Serial7.print(" TTF=");
  Serial7.print(p->TargetTicksPerFrame);



  Serial7.print(" fo=");
  Serial7.println(output);
  //  p->output = input;

  p->PrevInput = input;
}



/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = -encoder_read_left(); //readEncoder(LEFT);
  rightPID.Encoder = encoder_read_right(); //readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!moving) {
    /*
      Reset PIDs once, to prevent startup spikes,
      see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
      PrevInput is considered a good proxy to detect
      whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  Serial7.println("rightPID");
  doPID(&rightPID);
  Serial7.println("leftPID");
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  // setMotorSpeeds(leftPID.output, rightPID.output);
  drive_vel_motor_straight(leftPID.output, rightPID.output);
}
