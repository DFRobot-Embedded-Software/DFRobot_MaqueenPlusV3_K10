/*!
 * @file Cosmo.cpp
 * @brief Define the basic structure of class Cosmo
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [LR]<rong.li@dfrobot.com>
 * @version V1.0.0
 * @date 2024-07-23
 * @url https://github.com/DFRobot/DFRobot_MaqueenPlusV3_K10
 */
#include "DFRobot_MaqueenPlusV3_K10.h"



DFRobot_MaqueenPlusV3_K10::DFRobot_MaqueenPlusV3_K10()
{
}

int DFRobot_MaqueenPlusV3_K10::begin(void)
{
  Serial.begin(9600);
  Wire.begin();
  // Wire.begin(K10_SDA_PIN,K10_SCL_PIN,100000);
  Wire.beginTransmission(SLAVE_ADDR);
  while(Wire.endTransmission() != 0) {
    Serial.println("i2c connect error");
    delay(50);
  }
  I2CWirte(SYSINIT ,0x01);//reset maqueen
  delay(200);//wait for reset
  Wire.begin();
  while(Wire.endTransmission() != 0) {
    Serial.println("i2c connect error");
    delay(50);
  } 
  return 1;
}

void DFRobot_MaqueenPlusV3_K10::I2CWirte(uint8_t Reg ,uint8_t data)
{
  Wire.beginTransmission(SLAVE_ADDR);                 // transmit to device Address
  Wire.write(Reg);                               // sends one byte
  Wire.write(data);
  Wire.endTransmission();
}
void DFRobot_MaqueenPlusV3_K10::I2CRead(uint8_t Reg ,uint8_t *data ,uint8_t datalen)
{
  uint8_t i = 0;
  Wire.beginTransmission(SLAVE_ADDR);                 // transmit to device Address
  Wire.write(Reg);                               // sends one byte
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)SLAVE_ADDR,datalen);
  while (Wire.available())                       // slave may send less than requested
    data[i++] = Wire.read();
}

void DFRobot_MaqueenPlusV3_K10::servoMotorCtrl(eServoNum_t servoNum ,uint8_t angle)
{ 
  switch (servoNum)
  {
  case eServo0:
    I2CWirte(SERVO_0,angle);
    break;
  case eServo1:
    I2CWirte(SERVO_1,angle);
    break;
  default:
    break;
  }
}  

void DFRobot_MaqueenPlusV3_K10::setCross(eCross_t crossId,eTurnCmd_t cmd)
{
  switch (crossId)
  {
  case eStateCrossing:
    I2CWirte(CROSS_DEFAULT ,cmd);
    break;
  case eStateLeftRight:
    I2CWirte(T1_DEFAULT ,cmd);
    break;
  case eStateLeftStright:
    I2CWirte(T2_DEFAULT ,cmd);;
    break;
  case eStateRightStright:
    I2CWirte(T3_DEFAULT ,cmd);
    break;
  default:
    break;
  }
}

void DFRobot_MaqueenPlusV3_K10::lineTraking(eCmd_t cmd,eSpeedGrade_t speed,eCmd_t cmd2)
{
  if(cmd==eOn){/*Star*/
    I2CWirte(CAR_MODE ,eModeTracking);
    I2CWirte(SPEED_GRADE ,speed);
    I2CWirte(RIGHT_ANGLE_CONTROL,cmd2);
  }else if(cmd==eOff){/*Stop*/
    I2CWirte(CAR_MODE ,eModeEmpty);
    I2CWirte(RIGHT_ANGLE_CONTROL,cmd2);
  }
}


uint8_t DFRobot_MaqueenPlusV3_K10::getCrossState(void)
{
  I2CRead(CAR_STATE ,&rxbuf[CAR_STATE] ,1);
  return rxbuf[CAR_STATE];
}

String DFRobot_MaqueenPlusV3_K10::getVersion(void)
{
  uint8_t versionlen=0;
  String version;
  I2CRead(VERSON_LEN ,&rxbuf[VERSON_LEN] ,1);
  versionlen = rxbuf[VERSON_LEN];
  I2CRead(VERSON_DATA ,&rxbuf[VERSON_DATA] ,versionlen);
  for(uint8_t j = 0; j < versionlen; j++) {
    version+=(char)rxbuf[VERSON_DATA+j];
    //  Serial.print((char)rxbuf[51+j]);
  } 
  // Serial.println();
  return version;
}

uint16_t DFRobot_MaqueenPlusV3_K10::getDistanceSum(void)
{
  I2CRead(DISTANCE_SUM_H ,&rxbuf[DISTANCE_SUM_H] ,2);
  return rxbuf[DISTANCE_SUM_H]<<8| rxbuf[DISTANCE_SUM_L];
}

void DFRobot_MaqueenPlusV3_K10::clearDistanceSum(void)
{
  I2CWirte(DISTANCE_SUM_L ,0x00);
}

uint16_t DFRobot_MaqueenPlusV3_K10::getLight(eDirectionPart_t cmd)
{
   if(cmd==eLeftP) {
     I2CRead(LIGHTL_H ,&rxbuf[LIGHTL_H] ,2); 
     return rxbuf[LIGHTL_H]<<8| rxbuf[LIGHTL_L];
   } else if(cmd==eRightP) {
     I2CRead(LIGHTR_H ,&rxbuf[LIGHTR_H] ,2); 
     return rxbuf[LIGHTR_H]<<8| rxbuf[LIGHTR_L];
   }
   return 0;
}


void DFRobot_MaqueenPlusV3_K10::setRgb(eDirection_t rgb,eRgbCmd_t cmd)
{
    if(rgb==eLeft){
      I2CWirte(RGB_L ,cmd);
    }else if(rgb==eRight){
      I2CWirte(RGB_R ,cmd);
    }else if(rgb==eAll){
      I2CWirte(RGB_L ,cmd);
      I2CWirte(RGB_R ,cmd);
    }
}

void DFRobot_MaqueenPlusV3_K10::setRgbOff(eDirection_t rgb)
{
    if(rgb==eLeft){
      I2CWirte(RGB_L ,eRgbOff);
    }else if(rgb==eRight){
      I2CWirte(RGB_R ,eRgbOff);
    }else if(rgb==eAll){
      I2CWirte(RGB_L ,eRgbOff);
      I2CWirte(RGB_R ,eRgbOff);
    }
}

uint8_t DFRobot_MaqueenPlusV3_K10::readPatrol(ePatrolNum_t num)
{
  I2CRead(BLACK_ADC_STATE ,&rxbuf[BLACK_ADC_STATE] ,1);
  if(rxbuf[BLACK_ADC_STATE]&(1<<(5-num)))//max=5
    return 1;
  else
    return 0;
}

uint16_t DFRobot_MaqueenPlusV3_K10::readPatrolVoltage(ePatrolNum_t num)
{
  I2CRead(ADC_COLLECT_0+num*2 ,&rxbuf[ADC_COLLECT_0+num*2] ,2);
  return rxbuf[ADC_COLLECT_0+num*2]*255 +rxbuf[ADC_COLLECT_0+num*2+1];
}


void DFRobot_MaqueenPlusV3_K10::setMotor(eDirection_t motor,eCarDirection_t Dir,uint8_t speed)
{
    if (motor==eLeft){
      I2CWirte(MOTOR_0,Dir);
      I2CWirte(SPEED_0,speed);
    }else if (motor==eRight){
      I2CWirte(MOTOR_1,Dir);
      I2CWirte(SPEED_1,speed);
    }else if(motor==eAll){
      I2CWirte(MOTOR_1,Dir);
      I2CWirte(SPEED_1,speed);
      I2CWirte(MOTOR_0,Dir);
      I2CWirte(SPEED_0,speed);
    }
}
void DFRobot_MaqueenPlusV3_K10::setMotorStop(eDirection_t motor)
{
    if (motor==eLeft){
      I2CWirte(MOTOR_0,0);
      I2CWirte(SPEED_0,0);
    }else if (motor==eRight){
      I2CWirte(MOTOR_1,0);
      I2CWirte(SPEED_1,0);
    }else if(motor==eAll){
      I2CWirte(MOTOR_1,0);
      I2CWirte(SPEED_1,0);
      I2CWirte(MOTOR_0,0);
      I2CWirte(SPEED_0,0);
    }
}



void DFRobot_MaqueenPlusV3_K10::motorTypeSet(uint8_t type)
{
  I2CWirte(MOTOR_TYPE,type);
}
void DFRobot_MaqueenPlusV3_K10::angleControl(eAngleDirection_t cmd,eSpeedGrade_t speed_grade,uint16_t angle,eBlocking_t blocking)
{
  I2CWirte(CAR_MODE ,eModeAccurately);// Set car mode to accurately
  I2CWirte(ANGLE_CONTROL ,cmd);  
  I2CWirte(ANGLE_SPEED ,speed_grade);
  I2CWirte(ANGLE_DATA ,angle);
  if(blocking==eBlocking){
    uint8_t count = 0;
    while (count < 10) {
        if (getPIDFinish()==true) {
            count++;
        } else {
            count = 0;
        }
    }
  }
}
void DFRobot_MaqueenPlusV3_K10::distanceControl(eCarDirection_t cmd,eSpeedGrade_t speed_grade,uint16_t distance,eBlocking_t blocking)
{
  I2CWirte(CAR_MODE ,eModeAccurately);// Set car mode to accurately
  I2CWirte(DISTANCE_CONTROL ,cmd);  
  I2CWirte(DISTANCE_SPEED ,speed_grade);
  I2CWirte(DISTANCE_DATA_H ,distance>>8);
  I2CWirte(DISTANCE_DATA_L ,distance);
  if(blocking==eBlocking){
    uint8_t count = 0;
    while (count < 10) {
        if (getPIDFinish()==true) {
            count++;
        } else {
            count = 0;
        }
    }
  }
}
uint8_t DFRobot_MaqueenPlusV3_K10::getRealSpeed(eDirectionPart_t motor)
{
  if(motor==eLeftP){
    I2CRead(SPEED_DATAL ,&rxbuf[SPEED_DATAL] ,1);
    return rxbuf[SPEED_DATAL];
  }else if(motor==eRightP){
    I2CRead(SPEED_DATAR ,&rxbuf[SPEED_DATAR] ,1);
    return rxbuf[SPEED_DATAR];
  }
  return 0;
}
void DFRobot_MaqueenPlusV3_K10::setRightAngleControl(eCmd_t cmd)
{
  I2CWirte(RIGHT_ANGLE_CONTROL ,cmd);
}

uint8_t DFRobot_MaqueenPlusV3_K10::getPIDFinish(void)
{
  I2CRead(PID_CONTROL_FINISH ,&rxbuf[PID_CONTROL_FINISH] ,1);
  delay(5);
  return rxbuf[PID_CONTROL_FINISH];
}