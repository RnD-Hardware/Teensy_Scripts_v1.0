/*
  created by : - Raj Mehta
*/

/* BMI */
#include "BMI088.h"

#define TO_DEG(x) (x * 57.2957795131)

Bmi088Accel accel(Wire, 0x19);
Bmi088Gyro gyro(Wire, 0x69);
float gxr, gyr, gzr;
float accx_mss, accy_mss, accz_mss;

uint32_t t1, t2, _yaw2;
double dt;
int temp1;
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float yaw2 , _yaw;
char x;

unsigned long encl_t2 = 0, encl_t1 = millis();
unsigned long encr_t2 = 0, encr_t1 = millis();
unsigned long startMillis = millis(), ti1 = millis();
unsigned long timediff, ti2;

int32_t enc_count_left, enc_count_right;
uint16_t temperature;
int status;


void setup()
{
  /* BMI088 */
  status = gyro.begin();
  if (status < 0) {
    Serial.print("Gyro ");
    Serial.println(status);
    while (1) {}
  }
  status = accel.begin();
  if (status < 0) {
    Serial.print("Accel ");
    Serial.println(status);
    //    while (1) {}
  }
  //  status = gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_64HZ);
  Serial.begin(115200);
}


void loop()
{
  timediff = millis() - startMillis;
  /* 5 millisec which is 200Hz */
  if (timediff >= 50)
  {
    startMillis = millis();
    Serial.print("GYR: ");
    Serial.print(gxr);
    Serial.print(", ");
    Serial.print(gyr);
    Serial.print(", ");
    Serial.print(gzr);
    Serial.print(" ACC: ");
    Serial.print(accx_mss);
    Serial.print(", ");
    Serial.print(accy_mss);
    Serial.print(", ");
    Serial.println(accz_mss);
    //    Serial.print(" yaw=");
    //    Serial.println(_yaw2);
    gyroread();
    if (isnan(yaw2))
    {
      _yaw2 = 44444;
    }
    else
    {
      _yaw = TO_DEG(yaw2) + 180 ;
      _yaw2 = _yaw * 100;
    }
  }
}
