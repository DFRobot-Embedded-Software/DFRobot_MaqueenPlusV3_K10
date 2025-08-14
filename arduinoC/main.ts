
enum BLOCKING {
    //% block="NoBlocking"
  eNoBlocking ,   
    //% block="Blocking"
  eBlocking ,    
};

enum DIRECTION_PART  {
    //% block="Left"
    eLeftP,
    //% block="Right"
    eRightP,
}


enum PIDCmd {
    //% block="Suspend"
    eSuspend ,   
    //% block="Continue"
    eContinue ,   
};

enum DIRECTION  {
    //% block="Left"
    eLeft,
    //% block="Right"
    eRight,
    //% block="All"
    eAll,
}
enum CAR_DIRECTION {
    //% block="GoForward"
    eForward,  
    //% block="Backward"
    eBackward,
}
enum RGB_COLOR{
  //% block="Red"
  eRgbRed  ,   
  //% block="Green"
  eRgbGreen ,    
  //% block="Yellow"
  eRgbYellow ,  
  //% block="Blue"
  eRgbBlue  ,    
  //% block="Purple"
  eRgbPurple ,  
  //% block="Cyan"
  eRgbCyan  ,   
  //% block="White"
  eRgbWhite  , 
  //% block="Off"
  eRgbOff    
};

//% block="Crossing"
enum CROSS {
    //% block="Crossroads"
    eStateCrossing,
    //% block="T-junction (Left/Right)"
    eStateLeftRight,
    //% block="T-junction (Left/Straight)"
    eStateLeftStright,
    //% block="T-junction (Right/Straight)"
    eStateRightStright
}

//% block="On/Off"
enum CMD {
    //% block="Off"
    eOff,
    //% block="On"
    eOn
}

//% block="Servo Number"
enum SERVO_NUM {
    //% block="S0"
    eServo0
    //% block="S1"
    eServo1,

}

//% block="Patrol Number"
enum PATROL_NUM {
    //% block="L3"
    eL3,
    //% block="L2"
    eL2,
    //% block="L1"
    eL1,
    //% block="R1"
    eR1,
    //% block="R2"
    eR2,
    //% block="R3"
    eR3
}

//% block="Speed Grade"
enum SPEED_GRADE {
    //% block="1"
    eSpeedGrade1,
    //% block="2"
    eSpeedGrade2,
    //% block="3"
    eSpeedGrade3,
    //% block="4"
    eSpeedGrade4,
    //% block="5"
    eSpeedGrade5
}

//% block="Turn Command"
enum TURN_CMD {
    //% block="Turn Left"
    eTurnLeft,
    //% block="Turn Right"
    eTurnRight,
    //% block="Go Straight"
    eTurnStright,
    //% block="Stop"
    eTurnStop
}

//% block="Car Mode"
enum CAR_MODE {
    //% block="Empty"
    eModeEmpty,
    //% block="Directly"
    eModeDirectly,
    //% block="Tracking"
    eModeTracking,
    //% block="Accurately"
    eModeAccurately
}

//% block="Angle Direction"
enum ANGLE_DIRECTION {
    //% block="Clockwise"
    eClockwise,
    //% block="Anticlockwise"
    eAnticlockwise
}


//% color="#48ce9b" iconWidth=50 iconHeight=40
namespace DFRobot_MaqueenPlusV3_K10 {
    //% block="MaqueenPlus_V3_K10_Init" blockType="command"
    export function DFRobot_MaqueenPlusV3_K10_init() {
        Generator.addInclude('DFRobot_MaqueenPlusV3_K10', '#include <DFRobot_MaqueenPlusV3_K10.h>');
        Generator.addObject('maqueen', 'DFRobot_MaqueenPlusV3_K10', 'maqueen;');
        Generator.addSetup('maqueen.begin', 'maqueen.begin();');
    }
    //% block="set the motor[DIRECTION] direction[CAR_DIRECTION] speed[SPEED]" blockType="command"
    //% DIRECTION.shadow="dropdown" DIRECTION.options="DIRECTION" DIRECTION.defl="DIRECTION.eAll"
    //% CAR_DIRECTION.shadow="dropdown" CAR_DIRECTION.options="CAR_DIRECTION" CAR_DIRECTION.defl="CAR_DIRECTION.eForward"
    //% SPEED.shadow="range" SPEED.params.min=0 SPEED.params.max=255 SPEED.defl=0
    export function setMotor(parameter: any, block: any) {
        let motor = parameter.DIRECTION.code;
        let direction = parameter.CAR_DIRECTION.code;
        let speed = parameter.SPEED.code;
        Generator.addCode(`maqueen.setMotor(${motor}, ${direction}, ${speed});`);
    }

    //% block="set the motor [DIRECTION] stop" blockType="command"
    //% DIRECTION.shadow="dropdown" DIRECTION.options="DIRECTION" DIRECTION.defl="DIRECTION.eAll"
    export function setMotorStop(parameter: any, block: any) {
        let motor = parameter.DIRECTION.code;
        Generator.addCode(`maqueen.setMotorStop(${motor});`);
    }

    //% externalFunc
    export function getColorsFunc_() {
        return [
            "#f00",   // eRgbRed
            "#0f0",   // eRgbGreen
            "#ff0",   // eRgbYellow
            "#00f",   // eRgbBlue
            "#f0f",   // eRgbPurple
            "#0ff",   // eRgbCyan
            "#fff",   // eRgbWhite
            "#000"    // eRgbOff
        ]
    }
    //% block="set the [DIRECTION]CAR_RGB  color[COLOR]" blockType="command"
    //% DIRECTION.shadow="dropdown" DIRECTION.options="DIRECTION" DIRECTION.defl="DIRECTION.eAll"
    //% COLOR.shadow="colorPalette" COLOR.params.column=4
    //% COLOR.params.colorsFunc="getColorsFunc_" COLOR.defl="#f00"
    export function setRgb(parameter: any, block: any) {
        let direction = parameter.DIRECTION.code;
        const colorMap: { [key: string]: string } = {
            "0xFF0000": "eRgbRed",
            "0x00FF00": "eRgbGreen",
            "0xFFFF00": "eRgbYellow",
            "0x0000FF": "eRgbBlue",
            "0xFF00FF": "eRgbPurple",
            "0x00FFFF": "eRgbCyan",
            "0xFFFFFF": "eRgbWhite",
            "0x000000": "eRgbOff"
        };
        let rgbColorRaw = parameter.COLOR.code;
        let rgbColor = colorMap[rgbColorRaw]; //change
        Generator.addCode(`maqueen.setRgb(${direction}, ${rgbColor});`);
    }

    //% block="set the [DIRECTION]CAR_RGB  off" blockType="command"
    //% DIRECTION.shadow="dropdown" DIRECTION.options="DIRECTION" DIRECTION.defl="DIRECTION.eAll"
    export function setRgbOff(parameter: any, block: any) {
        let direction = parameter.DIRECTION.code;
        Generator.addCode(`maqueen.setRgbOff(${direction});`);
    }

    //% block="set servo [SERVO_NUM] angle [angle]" blockType="command"
    //% SERVO_NUM.shadow="dropdown" SERVO_NUM.options="SERVO_NUM" SERVO_NUM.defl="SERVO_NUM.eServo0"
    //% angle.shadow="range" angle.params.min=0 angle.params.max=180 angle.defl=0
    export function servoMotorCtrl(parameter: any, block: any) {
        let servoNum = parameter.SERVO_NUM.code;
        let angle = parameter.angle.code;
        Generator.addCode(`maqueen.servoMotorCtrl(${servoNum}, ${angle});`);
    }

    //% block="set cross [CROSS] command [TURN_CMD]" blockType="command"
    //% CROSS.shadow="dropdown" CROSS.options="CROSS" CROSS.defl="CROSS.eStateCrossing"
    //% TURN_CMD.shadow="dropdown" TURN_CMD.options="TURN_CMD" TURN_CMD.defl="TURN_CMD.eTurnLeft"
    export function setCross(parameter: any, block: any) {
        let cross = parameter.CROSS.code;
        let turnCmd = parameter.TURN_CMD.code;
        Generator.addCode(`maqueen.setCross(${cross}, ${turnCmd});`);
    }

    //% block="line tracking [CMD] speed grade [SPEED_GRADE]" blockType="command"
    //% CMD.shadow="dropdown" CMD.options="CMD" CMD.defl="CMD.eOn"
    //% SPEED_GRADE.shadow="dropdown" SPEED_GRADE.options="SPEED_GRADE" SPEED_GRADE.defl="SPEED_GRADE.eSpeedGrade1"
    export function lineTraking(parameter: any, block: any) {
        let cmd = parameter.CMD.code;
        let speed = parameter.SPEED_GRADE.code;
        Generator.addCode(`maqueen.lineTraking(${cmd}, ${speed});`);
    }

    //% block="get cross state" blockType="reporter"
    export function getCrossState(parameter: any, block: any) {
        Generator.addCode(`maqueen.getCrossState()`);
    }

    //% block="get version" blockType="reporter"
    export function getVersion(parameter: any, block: any) {
        Generator.addCode(`maqueen.getVersion()`);
    }

    //% block="get [DIRECTION_PART] light intensity" blockType="reporter"
    //% DIRECTION_PART.shadow="dropdown" DIRECTION_PART.options="DIRECTION_PART" DIRECTION_PART.defl="DIRECTION_PART.eLeftP"
    export function getLight(parameter: any, block: any) {
        let directionPart = parameter.DIRECTION_PART.code;
        Generator.addCode(`maqueen.getLight(${directionPart})`);
    }

    //% block="read patrol sensor [PATROL_NUM]" blockType="reporter"
    //% PATROL_NUM.shadow="dropdown" PATROL_NUM.options="PATROL_NUM" PATROL_NUM.defl="PATROL_NUM.eL3"
    export function readPatrol(parameter: any, block: any) {
        let patrolNum = parameter.PATROL_NUM.code;
        Generator.addCode(`maqueen.readPatrol(${patrolNum})`);
    }

    //% block="read patrol voltage [PATROL_NUM]" blockType="reporter"
    //% PATROL_NUM.shadow="dropdown" PATROL_NUM.options="PATROL_NUM" PATROL_NUM.defl="PATROL_NUM.eL3"
    export function readPatrolVoltage(parameter: any, block: any) {
        let patrolNum = parameter.PATROL_NUM.code;
        Generator.addCode(`maqueen.readPatrolVoltage(${patrolNum})`);
    }

    //% block="angle control speed grade [SPEED_GRADE] angle [angle] blocking [BLOCKING]" blockType="command"
    //% SPEED_GRADE.shadow="dropdown" SPEED_GRADE.options="SPEED_GRADE" SPEED_GRADE.defl="SPEED_GRADE.eSpeedGrade1"
    //% angle.shadow="range" angle.params.min=-180 angle.params.max=180 angle.defl=0
    //% BLOCKING.shadow="dropdown" BLOCKING.options="BLOCKING" BLOCKING.defl="BLOCKING.eNoBlocking"
    export function angleControl(parameter: any, block: any) {
        let speedGrade = parameter.SPEED_GRADE.code;
        let angle = parameter.angle.code;
        let blocking = parameter.BLOCKING.code
        if( angle >= 0) {
            Generator.addCode(`maqueen.angleControl(eClockwise, ${speedGrade}, ${angle}, ${blocking});`);
        } else {
            angle = -angle;
            Generator.addCode(`maqueen.angleControl(eAnticlockwise, ${speedGrade}, ${angle}, ${blocking});`);
        }
    }

    //% block="distance control [CAR_DIRECTION] speed grade [SPEED_GRADE] distance [distance] blocking [BLOCKING]" blockType="command"
    //% CAR_DIRECTION.shadow="dropdown" CAR_DIRECTION.options="CAR_DIRECTION" CAR_DIRECTION.defl="CAR_DIRECTION.eForward"
    //% SPEED_GRADE.shadow="dropdown" SPEED_GRADE.options="SPEED_GRADE" SPEED_GRADE.defl="SPEED_GRADE.eSpeedGrade1"
    //% distance.shadow="range" distance.params.min=0 distance.params.max=65535 distance.defl=0
    //% BLOCKING.shadow="dropdown" BLOCKING.options="BLOCKING" BLOCKING.defl="BLOCKING.eNoBlocking"
    export function distanceControl(parameter: any, block: any) {
        let carDirection = parameter.CAR_DIRECTION.code;
        let speedGrade = parameter.SPEED_GRADE.code;
        let distance = parameter.distance.code;
        let blocking = parameter.BLOCKING.code;
        Generator.addCode(`maqueen.distanceControl(${carDirection}, ${speedGrade}, ${distance}, ${blocking});`);
    }
    //% block="get [DIRECTION_PART] real speed" blockType="reporter"
    //% DIRECTION_PART.shadow="dropdown" DIRECTION_PART.options="DIRECTION_PART" DIRECTION_PART.defl="DIRECTION_PART.eLeftP"
    export function getRealSpeed(parameter: any, block: any) {
        let directionPart = parameter.DIRECTION_PART.code;
        Generator.addCode(`maqueen.getRealSpeed(${directionPart})`);
    }


    //% block="get total distance" blockType="reporter"
    export function getDistanceSum(parameter: any, block: any) {
        Generator.addCode(`maqueen.getDistanceSum()`);
    }

    //% block="PID Control [PIDCmd]" blockType="command"
    //% PIDCmd.shadow="dropdown" PIDCmd.options="PIDCmd" PIDCmd.defl="PIDCmd.eSuspend"
    export function PIDControl(parameter: any, block: any) {
        let cmd = parameter.PIDCmd.code;
        Generator.addCode(`maqueen.PIDControl(${cmd});`);
    }

}
