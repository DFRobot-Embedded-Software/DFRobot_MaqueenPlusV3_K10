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

/**
 * @enum eRgbSelect_t
 * @brief Right and Left and All
 */
typedef enum {
  eRgbLeft = 0,     /**< LEFT */ 
  eRgbRight = 1,    /**< RIGHT */ 
  eRgbAll = 2,      /**< ALL */ 
}eRgbSelect_t;

/**
 * @enum eMotorSelect_t
 * @brief Right and Left and All
 */
typedef enum {
  eMotorLeft = 0,     /**< LEFT */ 
  eMotorRight = 1,    /**< RIGHT */ 
  eMotorAll = 2,      /**< ALL */ 
}eMotorSelect_t;


/**
 * @enum eMotorTurn_t
 * @brief turn on or turn off
 */
typedef enum {
  eMotorOn = 1,     /**< ON */ 
  eMotoroff = 0,    /**< OFF */ 
}eMotorTurn_t;

/**
 * @enum eTrakingTurn_t
 * @brief turn on or turn off
 */
typedef enum {
  eTrakingOn = 1,     /**< ON */ 
  eTrakingOff = 0,    /**< OFF */ 
}eTrakingTurn_t;

/**
 * @enum eLightSelect_t
 * @brief Right and Left
 */
typedef enum {
  eLightLeft = 0,     /**< LEFT */ 
  eLightRight = 1,    /**< RIGHT */ 
}eLightSelect_t;



/**
 * @enum ePatrolNum_t
 * @brief select partrol sensor
 */
typedef enum {
  eL2 = 0,    /**< L2 */ 
  eL1 = 1,    /**< L1 */ 
  eM = 2,     /**< M */ 
  eR1 = 3,    /**< R1 */ 
  eR2 = 4,    /**< R2 */ 
}ePatrolNum_t;

/**
 * @enum eDirection_t  
 * @brief FORWARD or REVERSE
 */
typedef enum {
  eMotorForward = 0,    /**< FORWARD */ 
  eMotorReverse =1,     /**< REVERSE */ 
}eDirection_t;


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

/**
 * @enum eBatteryType_t
 * @brief type of battery
 */
typedef enum {
  eLithium  =   0 ,        //Lithium ion battery
  eAlkaline =   1        //Alkaline battery
}eBatteryType_t;


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

  #define         SERVO_1                     20
  #define         SERVO_2                     21

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
  #define SPEED_MODE             63          // 速度模式 1（低）-5（高）

  // 距离控制（如直线行走固定距离）
  #define DISTANCE_CONTROL       64          // 距离控制使能（0=禁用，1=启用）
  #define DISTANCE_DATA_H        65          // 目标距离高8位（单位：cm，16位数据）
  #define DISTANCE_DATA_L        66          // 目标距离低8位

  // 角度控制（如旋转固定角度）
  #define ANGLE_CONTROL          67          // 角度控制使能（0=禁用，1=启用）
  #define ANGLE_DATA             68          // 目标角度（单位：度，0-360）

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

  /**
   * @fn begin
   * @brief subclass initialization function
   * @return bool type, means returning initialization status
   * @retval true is success
   */
  int begin(void);
  /**
   * @fn servoMotorCtrl
   * @brief Ctrl the servo motor
   * @param servoMotorPin  pin of servo motor
   * @n Optional pin：D3 D6 D9 D10
   * @param angle  0 to 180
   * @return None
   */
  void servoMotorCtrl(uint8_t servoMotorPin,uint8_t angle);

  /**
   * @fn rgbSet
   * @brief Set color of the rgb
   * @param rgb  eRgbLeft or eRgbRight or eRgbAll
   * @param cmd  color:eRgbCmd_t
   * @n RGB_R  
   * @n RGB_G 
   * @n RGB_B  
   * @n RGB_RB 
   * @n RGB_RG 
   * @n RGB_GB 
   * @n RGB_RGB
   * @n RGB_OFF
   * @return None
   */
  void rgbSet(eRgbSelect_t rgb,eRgbCmd_t cmd);
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
   * @fn getLight
   * @brief Acquisition of light intensity
   * @param cmd  light sensor:  eLightLeft or eLightRight
   * @return light data :0-1023
   */
  uint16_t getLight(eLightSelect_t cmd);
  /**
   * @fn getVersion
   * @brief Get the Version of Cosmo.
   * @return version
   */
  String getVersion(void);
  /**
   * @fn lineTraking
   * @brief  Stop line patrol or Start line patrol
   * @param cmd  Stop line patrol: eTrakingOff  Start line patrol: eTrakingOn
   * @return 
   */
  void lineTraking(eTrakingTurn_t cmd);
  /**
   * @fn BLEModule
   * @brief Enable or disable the Bluetooth module
   * @n 0:disable 1:enable
   */
  uint8_t getCross(void);
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
   * @fn motorSet
   * @brief Setting motor parameters 
   * @param motor  Left motor: eMotorLeft, Right motor: eMotorRight All:eMotorAll
   * @param Dir  Forward turning: eMotorForward backward turning:eMotorReverse
   * @param speed  0-255
   * @return 
   */
  void motorSet(eMotorSelect_t motor,eDirection_t Dir,uint8_t speed);
  /**
   * @fn motorStop
   * @brief Stop motor operation
   * @param motor  Left motor: eMotorLeft, Right motor: eMotorRight All:eMotorAll
   * @return 
   */
  void motorStop(eMotorSelect_t motor);
  /**
   * @fn motorTypeSet
   * @brief Set the type of motor 
   * @return 
   */
  void motorTypeSet(uint8_t type);

  DFRobot_MaqueenPlusV3_K10();
  // ~DFRobot_MaqueenPlusV3_K10();

};





#endif