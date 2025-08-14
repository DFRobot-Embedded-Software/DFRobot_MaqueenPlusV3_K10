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
 * @enum ePIDCmd_t
 * @brief suspend  or continue
 */
typedef enum {
  eSuspend = 0,    /**< suspend */ 
  eContinue = 1,     /**< continue */ 
}ePIDCmd_t;

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
  eClockwise = 0,  
  eAnticlockwise = 1, 
}eAngleDirection_t;

typedef enum {
  eForward = 0,  
  eBackward = 1, 
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

/* ------------------------------ Motion control related related registers related registers ------------------------------ */
// Car mode and status
  #define CAR_MODE      60 // Car working mode
  #define CAR_STATE     61 // Car running status
  #define MOTOR_TYPE    62 // Motor type (used to adapt to different motor parameters)
  #define SPEED_GRADE   63 // Speed mode 1 (low) -5 (high)

// Distance control (e.g., walking a fixed distance in a straight line)
  #define DISTANCE_CONTROL  64 // Distance control direction 0: forward 1: backward
  #define DISTANCE_DATA_H   65 // Target distance high 8 bits (unit: cm, 16-bit data)
  #define DISTANCE_DATA_L   66 // Target distance low 8 bits

// Angle control (e.g., rotating a fixed angle)
  #define ANGLE_CONTROL     67 // Angle control direction 0: clockwise 1: counterclockwise
  #define ANGLE_DATA        68 // Target angle (unit: degree, 0-180)

//
  #define CROSS_DEFAULT    69
  #define T1_DEFAULT       70
  #define T2_DEFAULT       71
  #define T3_DEFAULT       72

// System initialization
  #define SYSINIT         73 // System state reset (write 1 to trigger reset, clear all states)

/* ------------------------------ Encoder and light intensity related registers ------------------------------ */
// Encoder speed (unit: CM/S)
  #define SPEED_DATAL     76 // Left wheel speed value
  #define SPEED_DATAR     77 // Right wheel speed value

// Light intensity sensor data (16-bit)
  #define LIGHTL_H      78 // Left light intensity sensor high 8 bits
  #define LIGHTL_L      79 // Left light intensity sensor low 8 bits
  #define LIGHTR_H      80 // Right light intensity sensor high 8 bits
  #define LIGHTR_L      81 // Right light intensity sensor low 8 bits

/* ------------------------------ Supplementary control registers ------------------------------ */
  #define DISTANCE_SPEED      85 // Speed during distance control (used to adjust walking speed)
  #define ANGLE_SPEED         86 // Speed during angle control (used to adjust rotation speed)
  #define PID_CONTROL_FINISH  87 // Whether precise control is completed

  #define DISTANCE_SUM_H    90 // Cumulative movement distance (high eight bits) 0-255
  #define DISTANCE_SUM_L    91 // Cumulative movement distance (low eight bits) 0-255






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
   * @return 
   */
  void lineTraking(eCmd_t cmd,eSpeedGrade_t speed);

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

  void PIDControl(ePIDCmd_t cmd);

  DFRobot_MaqueenPlusV3_K10();
  // ~DFRobot_MaqueenPlusV3_K10();

};





#endif