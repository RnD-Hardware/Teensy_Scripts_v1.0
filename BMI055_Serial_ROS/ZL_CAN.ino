/*
  Function directory

  void init_can() // initialise CAN // ID : 0x601 , baudrate : 500kbps , data length : 8
  void init_sync_position_control(uint16_t speed, uint16_t acc, uint16_t dcc)
  void drive_pos_turn_straight(int32_t pos)
  void drive_pos_turn_clk(int32_t pos)
  void init_sync_velocity_control(uint16_t acc, uint16_t dcc)
  void drive_vel_motor_straight(int16_t speed)
  void stop_motor()
  void acc_dcc_time(uint16_t acc, uint16_t dcc)
  void acc_time(uint16_t acc)
  void dcc_time(uint16_t dcc)
  int32_t encoder_read_left()
  int32_t encoder_read_right()
  void error_check()
  void clear_error()
  void quick_stop_response(int8_t option) // set action of QS
  uint16_t read_temperature() // show temperature of driver, left , right motors
  uint8_t CAN_baudrate() // read baudrate
*/
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> ZL;
CAN_message_t msg;
CAN_message_t rec;

void init_can()
{
  ZL.begin();
  ZL.setBaudRate(500000);
  msg.id = 0x601;
  msg.len = 8;
}

void init_sync_position_control(uint16_t speed, uint16_t acc, uint16_t dcc)
{
  msg.buf[0] = 0x2F;
  msg.buf[1] = 0x60;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x01;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set position mode
  while (!ZL.read(rec));
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  ZL.write(msg);       // set left acc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  while (!ZL.read(rec));
  msg.buf[1] = 0x84;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  ZL.write(msg);       // set left dcc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right dcc
  while (!ZL.read(rec));
  msg.buf[1] = 0x81;
  msg.buf[3] = 0x01;
  msg.buf[4] = speed;
  msg.buf[5] = speed >> 8;
  ZL.write(msg);       // set left speed
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right speed
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x07;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  while (!ZL.read(rec));
}

void drive_pos_turn_straight(int32_t pos)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x7A;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = pos;
  msg.buf[5] = pos >> 8;
  msg.buf[6] = pos >> 16;
  msg.buf[7] = pos >> 24;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  msg.buf[4] = (pos ^ 0xFFFFFFFF) + 1;
  msg.buf[5] = ((pos ^ 0xFFFFFFFF) + 1) >> 8;
  msg.buf[6] = ((pos ^ 0xFFFFFFFF) + 1) >> 16;
  msg.buf[7] = ((pos ^ 0xFFFFFFFF) + 1) >> 24;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x4F;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x5F;
  ZL.write(msg);
  while (!ZL.read(rec));
}

void drive_pos_turn_clk(int32_t pos)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x7A;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = pos;
  msg.buf[5] = pos >> 8;
  msg.buf[6] = pos >> 16;
  msg.buf[7] = pos >> 24;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  msg.buf[4] = pos;
  msg.buf[5] = pos >> 8;
  msg.buf[6] = pos >> 16;
  msg.buf[7] = pos >> 24;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x4F;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x5F;
  ZL.write(msg);
  while (!ZL.read(rec));
}

void init_sync_velocity_control(uint16_t acc, uint16_t dcc)
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x0F;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x01;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set sync control
  while (!ZL.read(rec));
  msg.buf[0] = 0x2F;
  msg.buf[1] = 0x60;
  msg.buf[2] = 0x60;
  msg.buf[4] = 0x03;
  ZL.write(msg);       // set velocity mode
  while (!ZL.read(rec));
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  ZL.write(msg);       // set left acc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  while (!ZL.read(rec));
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  ZL.write(msg);       // set left dcc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right dcc
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x07;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  while (!ZL.read(rec));
}

void drive_vel_motor_straight(int16_t speedl, int16_t speedr)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x03;
  msg.buf[4] = (speedl ^ 0xFFFF) + 1;
  msg.buf[5] = ((speedl ^ 0xFFFF) + 1) >> 8;
  msg.buf[6] = speedr;
  msg.buf[7] = speedr >> 8;
  ZL.write(msg);
  while (!ZL.read(rec));
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
  while (!ZL.read(rec));
}

void acc_dcc_time(uint16_t acc, uint16_t dcc)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set left acc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  while (!ZL.read(rec));
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  ZL.write(msg);       // set left dcc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right dcc
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x07;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  while (!ZL.read(rec));
}

void acc_time(uint16_t acc)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set left acc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x07;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  while (!ZL.read(rec));
}

void dcc_time(uint16_t dcc)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set left acc
  while (!ZL.read(rec));
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  while (!ZL.read(rec));
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x07;
  ZL.write(msg);
  while (!ZL.read(rec));
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  while (!ZL.read(rec));
}

int32_t encoder_read_left()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x64;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  //  Serial.print(rec.buf[0],HEX);Serial.print("\t");  // feed back : 43 ( 4byte feedback )
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}

int32_t encoder_read_right()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x64;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x02;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  //  Serial.print(rec.buf[0],HEX);Serial.print("\t");  // feed back : 43 ( 4byte feedback )
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}

void error_check()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x3F;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  if (!rec.buf[4] && !rec.buf[5] && !rec.buf[6] && !rec.buf[7])
  {
    Serial.println("No Error");
  }
  else
  {
    switch (rec.buf[4])
    {
      case 1:
        Serial.println("Over voltage");
        break;
      case 2:
        Serial.println("Under voltage");
        break;
      default:
        if (rec.buf[5] == 1)
        {
          Serial.println("EEPROM read write error");
        }
        else
        {
          switch (rec.buf[6])
          {
            case 0x04:
              Serial.println("err_right : Over current");
              break;
            case 0x08:
              Serial.println("err_right : Over load");
              break;
            case 0x10:
              Serial.println("err_right : Current out of tolerance");
              break;
            case 0x20:
              Serial.println("err_right : Encoder out of tolerance");
              break;
            case 0x40:
              Serial.println("err_right : velocity out of tolerance");
              break;
            case 0x80:
              Serial.println("err_right : reference voltage error");
              break;
          }
          switch (rec.buf[4])
          {
            case 0x04:
              Serial.println("err_left : Over current");
              break;
            case 0x08:
              Serial.println("err_left : Over load");
              break;
            case 0x10:
              Serial.println("err_left : Current out of tolerance");
              break;
            case 0x20:
              Serial.println("err_left : Encoder out of tolerance");
              break;
            case 0x40:
              Serial.println("err_left : velocity out of tolerance");
              break;
            case 0x80:
              Serial.println("err_left : reference voltage error");
              break;
          }
        }
    }
    if (rec.buf[5] == 2)
    {
      Serial.println("err_left : Hall error");
    }
    if (rec.buf[7] == 2)
    {
      Serial.println("err_right : Hall error");
    }
  }
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
  while (!ZL.read(rec));
}

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

uint16_t read_temperature()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x32;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x03;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  Serial7.print(" ,");
  Serial7.print(((rec.buf[5] << 8) + rec.buf[4]) / 10); //Serial.print("\t"); // driver
  msg.buf[3] = 0x01;
  ZL.write(msg);
  while (!ZL.read(rec));
  Serial7.print(" ,");
  Serial7.print(((rec.buf[5] << 8) + rec.buf[4]) / 10); //Serial.print("\t"); // left motor
  msg.buf[3] = 0x02;
  ZL.write(msg);
  while (!ZL.read(rec));
  Serial7.print(" ,");
  Serial7.println(((rec.buf[5] << 8) + rec.buf[4]) / 10); // right motor
  return (rec.buf[5] << 8) + rec.buf[4];
}

uint8_t CAN_baudrate()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x0B;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  return rec.buf[4];
}
