/*!
 * @file        motorCtrl.ino
 * @brief       this demo demonstrates how to control motor of Cosmo .
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [lr](rong.li@dfrobot.com)
 * @version     V1.0
 * @date        2024-10-16
 * @url         https://github.com/DFRobot/DFRobot_MaqueenPlusV3_K10
 */
#include <DFRobot_MaqueenPlusV3_K10.h>
DFRobot_MaqueenPlusV3_K10 MaqueenPlusV3;


void setup() {
  MaqueenPlusV3.begin();
}
 
void loop() {
  MaqueenPlusV3.motorSet(eMotorLeft,eMotorForward,100);
  MaqueenPlusV3.motorSet(eMotorRight,eMotorForward,100);
  delay(1000);
  MaqueenPlusV3.motorSet(eMotorAll,eMotorForward,0);
  delay(100);
  MaqueenPlusV3.motorSet(eMotorLeft,eMotorForward,100);
  MaqueenPlusV3.motorSet(eMotorRight,eMotorReverse,100);
  delay(300);
  MaqueenPlusV3.motorSet(eMotorAll,eMotorForward,0);
  delay(100);
}