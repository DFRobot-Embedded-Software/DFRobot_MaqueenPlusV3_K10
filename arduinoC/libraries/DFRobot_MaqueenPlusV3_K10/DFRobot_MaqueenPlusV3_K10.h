#ifndef __DFROBOT_MAQUEENPLUS_V3_K10_H_
#define __DFROBOT_MAQUEENPLUS_V3_K10_H_

#include "Arduino.h"
#include "Wire.h"

/**
 * @enum eCross_t
 * @brief Crossing
 */
typedef enum {
  eStateCrossing =1,       /**< a crossroads */ 
  eStateLeftRight =2,     /**< a T-junction where you can only turn left or right */ 
  eStateLeftStright=3,    /**< a T-intersection that can only turn left or go straight */ 
  eStateRightStright=4    /**< a T-intersection that can only turn right or go straight */ 
}eCross_t;

typedef  enum {
  eNoBlocking = 0,    /**< no blocking */ 
  eBlocking = 1,     /**< blocking */ 
}eBlocking_t;



typedef enum {
  eLeftP = 0,     /**< LEFT */ 
  eRightP = 1,    /**< RIGHT */ 
}eDirectionPart_t;

typedef enum {
  eLeft = 0,     /**< LEFT */ 
  eRight = 1,    /**< RIGHT */ 
  eAll= 2,      /**< ALL */
}eDirection_t;

/**
 * @enum eCmd_t
 * @brief turn on or turn off
 */
typedef enum {
  eOff = 0,    /**< OFF */ 
  eOn = 1,     /**< ON */ 
}eCmd_t;

/**
 * @enum eServoNum_t
 */
typedef enum {
  eServo0 = 0,    /**< servo0 */ 
  eServo1 = 1,     /**< servo1 */ 
}eServoNum_t;


/**
 * @enum ePatrolNum_t
 * @brief select partrol sensor
 */
typedef enum {
  eL3 = 0,    /**< L3 */ 
  eL2 = 1,    /**< L2 */ 
  eL1 = 2,     /**<L1*/ 
  eR1 = 3,    /**< R1 */ 
  eR2 = 4,    /**< R2 */ 
  eR3 = 5,    /**< R3 */ 
}ePatrolNum_t;

/**
 * @enum eSpeedGrade_t  
 * @brief FORWARD or REVERSE
 */
typedef enum {
  eSpeedGrade1 =1,    
  eSpeedGrade2 =2,
  eSpeedGrade3 =3,
  eSpeedGrade4 =4,
  eSpeedGrade5 =5    
}eSpeedGrade_t;


/**
 * @enum eRgbCmd_t
 * @brief rgb command
 */
typedef enum {
  eRgbRed  = 0x01,   /**< Red */ 
  eRgbGreen =0x02,     /**< Green */ 
  eRgbYellow = 0x03,  /**< Yellow */ 
  eRgbBlue  =0x04,    /**< Blue */ 
  eRgbPurple = 0x05,  /**< Purple */ 
  eRgbCyan  =0x06,   /**< Cyan */ 
  eRgbWhite  = 0x07, /**< White */
  eRgbOff  =0x08   /**< Off */
}eRgbCmd_t;

/**
 * @enum eTurnCmd_t
 * @brief Bluetooth command
 */
typedef enum {
  eTurnLeft  =1,         /**< turn left */  
  eTurnRight =2 ,        /**< turn right */  
  eTurnStright= 3,       /**< go  stright*/  
  eTurnStop =4           /**< stop */  
}eTurnCmd_t;

typedef enum {
  eModeEmpty = 0,        
  eModeDirectly = 1,
  eModeTracking = 2,
  eModeAccurately = 3,
}eCarMode_t;

typedef enum {
  eClockwise = 0,  //顺时针
  eAnticlockwise = 1, //逆时针
}eAngleDirection_t;

typedef enum {
  eForward = 0,  //前进
  eBackward = 1, //后退
}eCarDirection_t;

class DFRobot_MaqueenPlusV3_K10
{
protected:
  #define K10_SDA_PIN 47
  #define K10_SCL_PIN 48
  uint8_t rxbuf[120]  = {0} ;     //Receive buf
  uint8_t version = 0;             // Version len
  /**
   * @fn iicWirte
   * @brief Write register function
   * @param Reg  Register address 8bits
   * @param data data to be written
   * @return None
   */
  void I2CWirte(uint8_t Reg ,uint8_t data);
  /**
   * @fn I2C_Read
   * @brief Read register function
   * @param reg  Register address 8bits
   * @param data Storage and buffer for data to be read
   * @param datalen Length of data to be read
   * @return None
   */
  void I2CRead(uint8_t Reg ,uint8_t *data ,uint8_t datalen);

public:
  #define         SLAVE_ADDR 0x10                   // Slave IIC address
  #define         MOTOR_0                     0
  #define         SPEED_0                     1
  #define         MOTOR_1                     2
  #define         SPEED_1                     3

  #define         RGB_L                       11
  #define         RGB_R                       12

  #define         SERVO_0                     20
  #define         SERVO_1                     21

  #define         BLACK_ADC_STATE             29
  #define         ADC_COLLECT_0               30
  #define         ADC_COLLECT_1               32
  #define         ADC_COLLECT_2               34
  #define         ADC_COLLECT_3               36
  #define         ADC_COLLECT_4               38
  #define         ADC_COLLECT_5               40


  #define         VERSON_LEN                  50
  #define         VERSON_DATA                 51

  /* ------------------------------ 运动控制相关寄存器 ------------------------------ */
  // 小车模式与状态
  #define CAR_MODE               60          // 小车工作模式
  #define CAR_STATE              61          // 小车运行状态
  #define MOTOR_TYPE             62          // 电机类型（用于适配不同电机参数）
  #define SPEED_GRADE             63          // 速度模式 1（低）-5（高）

  // 距离控制（如直线行走固定距离）
  #define DISTANCE_CONTROL       64          // 距离控制方向 1：前进 2：后退
  #define DISTANCE_DATA_H        65          // 目标距离高8位（单位：cm，16位数据）
  #define DISTANCE_DATA_L        66          // 目标距离低8位

  // 角度控制（如旋转固定角度）
  #define ANGLE_CONTROL          67          // 角度控制方向 1：顺时针 2：逆时针
  #define ANGLE_DATA             68          // 目标角度（单位：度，0-180）

  //
  #define CROSS_DEFAULT          69
  #define T1_DEFAULT             70
  #define T2_DEFAULT             71
  #define T3_DEFAULT             72

  // 系统初始化
  #define SYSINIT                73          // 系统状态重置（写入1触发重置，清除所有状态）


  /* ------------------------------ 编码器与光强相关寄存器 ------------------------------ */
  // 编码器速度（单位：CM/S）
  #define SPEED_DATAL            76          // 左轮速度值
  #define SPEED_DATAR            77          // 右轮速度值

  // 光强传感器数据（16位）
  #define LIGHTL_H               78          // 左光强传感器高8位
  #define LIGHTL_L               79          // 左光强传感器低8位
  #define LIGHTR_H               80          // 右光强传感器高8位
  #define LIGHTR_L               81          // 右光强传感器低8位


  /* ------------------------------ 补充控制寄存器 ------------------------------ */
  #define DISTANCE_SPEED         85          // 距离控制时的速度（用于调整行走速度）
  #define ANGLE_SPEED            86          // 角度控制时的速度（用于调整旋转速度）
  #define PID_CONTROL_FINISH     87       // 精确控制是否完成
  #define RIGHT_ANGLE_CONTROL    88          // 循迹时的直角优化

  #define DISTANCE_SUM_H       90     // 运动累计距离（高八位）	0-255
  #define DISTANCE_SUM_L       91     // 运动累计距离（低八位）	0-255

  /**
   * @fn begin
   * @brief subclass initialization function
   * @return bool type, means returning initialization status
   * @retval true is success
   */
  int begin(void);
  /**
   * @fn setMotor
   * @brief Setting motor parameters 
   * @param motor  Left motor: eMotorLeft, Right motor: eMotorRight All:eMotorAll
   * @param Dir  Forward turning: eMotorForward backward turning:eMotorReverse
   * @param speed  0-255
   * @return 
   */
  void setMotor(eDirection_t motor,eCarDirection_t Dir,uint8_t speed);

  /**
   * @fn setMotorStop
   * @brief Stop the motor
   * @param motor  Left motor: eMotorLeft, Right motor: eMotorRight All:eMotorAll
   * @return None
   */
  void setMotorStop(eDirection_t motor);

  /**
   * @fn setRgb
   * @brief Set color of the rgb
   * @param rgb  eRgbLeft or eRgbRight or eRgbAll
   * @param cmd  color:eRgbCmd_t
   * @return None
   */
  void setRgb(eDirection_t rgb,eRgbCmd_t cmd);

  /**
   * @fn setRgbOff
   * @brief Turn off the rgb
   * @param rgb  eRgbLeft or eRgbRight or eRgbAll
   * @return None
   */
  void setRgbOff(eDirection_t rgb);

  /**
   * @fn servoMotorCtrl
   * @brief Ctrl the servo motor
   * @param servoNum  servo motor num: one of eServoNum_t
   * @param angle  0 to 180
   * @return None
   */
  void servoMotorCtrl(eServoNum_t servoNum ,uint8_t angle);
  /**
   * @fn readPatrol
   * @brief Read the status of the Line patrol sensor
   * @param num   Line patrol sensor num:  of ePatrolNum_t
   * @return bright-off:0 bright-on:1
   */
  uint8_t readPatrol(ePatrolNum_t num);
  /**
   * @fn readPatrolVoltage
   * @brief Read the voltage of the Line patrol sensor
   * @param num    Line patrol sensor num: one of ePatrolNum_t
   * @return voltage :0-4095
   */
  uint16_t readPatrolVoltage(ePatrolNum_t num);
  /**
   * @fn getVersion
   * @brief Get the Version of Cosmo.
   * @return version
   */
  String getVersion(void);
  /**
   * @fn getLight
   * @brief Acquisition of light intensity
   * @param cmd  light sensor:  eLightLeft or eLightRight
   * @return light data :0-1023
   */
  uint16_t getLight(eDirectionPart_t cmd);
  /**
   * @fn lineTraking
   * @brief  Stop line patrol or Start line patrol
   * @param cmd  Stop line patrol: eTrakingOff  Start line patrol: eTrakingOn
   * @param speed  Speed of line patrol: eSpeedGrade_t
   * @param cmd2  Right angle control: eCmd_t
   * @return 
   */
  void lineTraking(eCmd_t cmd,eSpeedGrade_t speed,eCmd_t cmd2);

  /**
   * @fn angleControl
   * @brief Control the car to rotate a certain angle
   * @param cmd  eClockwise or eAnticlockwise
   * @param speed_grade  Speed of line patrol: eSpeedGrade_t
   * @param angle  Angle to rotate: 0-360
   * @param blocking  eNoBlocking or eBlocking
   * @return 
   */
  void angleControl(eAngleDirection_t cmd,eSpeedGrade_t speed_grade,uint16_t angle,eBlocking_t blocking);

  /**
   * @fn distanceControl
   * @brief Control the car to move a certain distance
   * @param cmd  eClockwise or eAnticlockwise
   * @param speed_grade  Speed of line patrol: eSpeedGrade_t
   * @param distance  Distance to move: 0-65535
   * @param blocking  eNoBlocking or eBlocking

   * @return 
  */
  void distanceControl(eCarDirection_t cmd,eSpeedGrade_t speed_grade,uint16_t distance,eBlocking_t blocking);
  /**
   * @fn getRealSpeed
   * @brief Get the real speed of the car
   * @return real speed of the car
   */
  uint8_t getRealSpeed(eDirectionPart_t motor);
  

  /**
   * @fn getCrossState
   * @brief Get the type of Cross road
   * @return  type of Cross road: eStateCrossing ,eStateLeftRight ,eStateLeftStright ,eStateRightStright
   */
  uint8_t getCrossState(void);
  /**
   * @fn setCross
   * @brief Setting the action for this intersectiond
   * @param crossId  type of Cross road
   * @param cmd  Action command  
   * @n eTurnLeft , eTurnRight , eTurnStright,eTurnStop  
   * @return 
   */
  void setCross(eCross_t crossId,eTurnCmd_t cmd);
   /**
   * @fn motorTypeSet
   * @brief Setting the motor type
   * @param type  type  
   * @n 0(266rmp) 
   * @return 
   */
  void motorTypeSet(uint8_t type);

  /**
   * @fn setRightAngleControl
   * @brief Set right-angle control
   * @param cmd  cmd  
   * @n 0(off) 
   * @n 1(on) 
   * @return 
   */
  void setRightAngleControl(eCmd_t cmd);
  /**
   * @fn getDistanceSum
   * @brief Get the distance sum
   * @return distance sum (uint16_t)cm
   */
  uint16_t getDistanceSum(void);

  /**
   * @fn clearDistanceSum
   * @brief Clear the distance sum
   * @return None
   */
  void clearDistanceSum(void);

  /**
   * @fn getPIDFinish
   * @brief Get the PID finish
   * @return PID finish (uint8_t)
   */
  uint8_t getPIDFinish(void);

  DFRobot_MaqueenPlusV3_K10();
  // ~DFRobot_MaqueenPlusV3_K10();

};





#endif