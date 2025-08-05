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
  Serial.println("1");
  Wire.begin();
  // Wire.begin(K10_SDA_PIN,K10_SCL_PIN,100000);
  Wire.beginTransmission(SLAVE_ADDR);
  Serial.println("1");
  while(Wire.endTransmission() != 0) {
    Serial.println("i2c connect error");
    delay(100);
  } 
  // I2CWirte(MY_SYS_INIT ,0x01);//Reset
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

void DFRobot_MaqueenPlusV3_K10::servoMotorCtrl( uint8_t servoMotorPin,uint8_t angle)
{ 
 
  switch (servoMotorPin)
  {
  case 1:
    I2CWirte(SERVO_1,angle);
    break;
  case 2:
    I2CWirte(SERVO_2,angle);
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

void DFRobot_MaqueenPlusV3_K10::lineTraking(eTrakingTurn_t cmd)
{
  if(cmd==eTrakingOn){/*Star*/
    I2CWirte(CAR_MODE ,0x01);
  }else/*Stop*/
    I2CWirte(CAR_MODE ,0x00);
}


uint8_t DFRobot_MaqueenPlusV3_K10::getCross(void)
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
uint16_t DFRobot_MaqueenPlusV3_K10::getLight(eLightSelect_t cmd)
{
   uint16_t temp_light=0;
   I2CRead(LIGHTL_H ,&rxbuf[LIGHTL_H] ,4); 
   if(cmd==0)  temp_light= rxbuf[LIGHTL_H]<<8| rxbuf[LIGHTL_L];
   else  temp_light= rxbuf[LIGHTR_H]<<8| rxbuf[LIGHTR_L];
   return temp_light;
}


void DFRobot_MaqueenPlusV3_K10::rgbSet(eRgbSelect_t rgb,eRgbCmd_t cmd)
{
    if(rgb==eRgbLeft){
      I2CWirte(RGB_L ,cmd);
    }else if(rgb==eRgbRight){
      I2CWirte(RGB_R ,cmd);
    }else if(rgb==eRgbAll){
      I2CWirte(RGB_L ,cmd);
      I2CWirte(RGB_R ,cmd);
    }
}

uint8_t DFRobot_MaqueenPlusV3_K10::readPatrol(ePatrolNum_t num)
{
  I2CRead(BLACK_ADC_STATE ,&rxbuf[BLACK_ADC_STATE] ,1);
  if(rxbuf[BLACK_ADC_STATE]&(1<<(4-num)))//max=4
    return 1;
  else
    return 0;
}

uint16_t DFRobot_MaqueenPlusV3_K10::readPatrolVoltage(ePatrolNum_t num)
{
  uint16_t temp_data=0;
  I2CRead(ADC_COLLECT_0+num*2 ,&rxbuf[ADC_COLLECT_0+num*2] ,2);
  temp_data= rxbuf[ADC_COLLECT_0+num*2]*255 +rxbuf[ADC_COLLECT_0+num*2+1];
  return temp_data;
}


void DFRobot_MaqueenPlusV3_K10::motorSet(eMotorSelect_t motor,eDirection_t Dir,uint8_t speed)
{
    if (motor==eMotorLeft){
      I2CWirte(MOTOR_0,Dir);
      I2CWirte(SPEED_0,speed);
    }else if (motor==eMotorRight){
      I2CWirte(MOTOR_1,Dir);
      I2CWirte(SPEED_1,speed);
    }else if(motor==eMotorAll){
      I2CWirte(MOTOR_1,Dir);
      I2CWirte(SPEED_1,speed);
      I2CWirte(MOTOR_0,Dir);
      I2CWirte(SPEED_0,speed);
    }
}

void DFRobot_MaqueenPlusV3_K10::motorStop(eMotorSelect_t motor)
{
    if (motor==eMotorLeft){
      I2CWirte(SPEED_0,0);
    }else if (motor==eMotorRight){   
      I2CWirte(SPEED_1,0);
    }else if(motor==eMotorAll){
      I2CWirte(SPEED_0,0);
      I2CWirte(SPEED_1,0);
    }
} 

void DFRobot_MaqueenPlusV3_K10::motorTypeSet(uint8_t type)
{
  I2CWirte(MOTOR_TYPE,type);
}
