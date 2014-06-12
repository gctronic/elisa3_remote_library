
#include "elisa3-lib.h"
#ifdef _WIN32
    #include "windows.h"
#endif

// macro for handling flags byte
#define FRONT_IR_ON(x) ((x) |= (1 << 1))
#define BACK_IR_ON(x) ((x) |= (1 << 0))
#define ALL_IR_ON(x) ((x) |= (1<<0) | (1 << 1))
#define TV_REMOTE_ON(x) ((x) |= (1<<2))
#define SLEEP_ON(x) ((x) = 0x08)
#define CALIBRATION_ON(x) ((x) |= (1<<4))
#define OBSTACLE_AVOID_ON(x) ((x) |= (1<<6))
#define CLIFF_AVOID_ON(x) ((x) |= (1<<7))
#define FRONT_IR_OFF(x) ((x) &= ~(1 << 1))
#define BACK_IR_OFF(x) ((x) &= ~(1 << 0))
#define ALL_IR_OFF(x) ((x) &= ~(1 << 0) & ~(1 << 1))
#define TV_REMOTE_OFF(x) ((x) &= ~(1 << 2))
#define SLEEP_OFF(x) ((x) &= ~(1 << 3))
#define CALIBRATION_OFF(x) ((x) &= ~(1 << 4))
#define OBSTACLE_AVOID_OFF(x) ((x) &= ~(1 << 6))
#define CLIFF_AVOID_OFF(x) ((x) &= ~(1 << 7))

#define RAD_2_DEG 57.2957796

#define NUM_ROBOTS 4
#define PAYLOAD_SIZE 13
#define ADDR_SIZE 2
#define ROBOT_PACKET_SIZE (PAYLOAD_SIZE+ADDR_SIZE)
#define PACKETS_SIZE 64
#define OVERHEAD_SIZE (2*NUM_ROBOTS+1)
#define UNUSED_BYTES 3

// The usb buffer between the pc and the base-station is 64 bytes.
// Each packet exchanged with the bast-station must contain as the
// first byte the "command id" that at the moment can be either
// "change robot state" (0x27) or "goto base-station bootloader" (0x28).
// In order to optimize the throughput the packet exchanged with the radio
// base-station contains informations to send to four different robots
// simultaneously.
// Each robot must be identified by a 2 byte address, thus we have:
// 64 - 1 - 2*4 = 55 / 4 = 13 bytes usable for the payload of each robot.
//
// Payload content for each robot:
// --------------------------------------------------------------------------
// R | B | G | IR/flags | Right | Left | Leds | ...remaining 6 bytes not used
// --------------------------------------------------------------------------
//
// * R, B, G: values from 0 (OFF) to 100 (ON max power)
// * IR/flags:
//   - first two bits are dedicated to the IRs:
//     0x00 => all IRs off
//     0x01 => back IR on
//     0x02 => front IRs on
//     0x03 => all IRs on
//   - third bit is used for enabling/disablng IR remote control (0=>diabled, 1=>enabled)
//   - fourth bit is used for sleep (1 => go to sleep for 1 minute)
//   - fifth bit is used to calibrate all sensors (proximity, ground, accelerometer)
//   - sixth bits is reserved (used by radio station)
//   - seventh bit is used for enabling/disabling onboard obstacle avoidance
//   - eight bit is used for enabling/disabling onboard cliff avoidance
// * Right, Left: speed (in percentage); MSBit indicate direction: 1=forward, 0=backward; values from 0 to 100
// * Leds: each bit define whether the corresponding led is turned on (1) or off(0); e.g. if bit0=1 then led0=on
// * remaining bytes free to be used
//
// Overhead content :
// - command: 1 byte, indicates which command the packet refer to
// - address: 2 bytes per robot


// robots
int robotAddress[4];
char leftSpeed[4];
char rightSpeed[4];
char redLed[4], greenLed[4], blueLed[4];
unsigned int proxValue[4][8];
unsigned int proxAmbientValue[4][8];
unsigned int groundValue[4][4];
unsigned int groundAmbientValue[4][4];
unsigned int batteryAdc[4];
unsigned int batteryPercent[4];
signed int accX[4], accY[4], accZ[4];
unsigned char selector[4];
unsigned char tvRemote[4];
unsigned char flagsRX[4];
unsigned char flagsTX[2][4];
unsigned char smallLeds[4];
signed long int leftMotSteps[4], rightMotSteps[4];
signed int robTheta[4], robXPos[4], robYPos[4];
unsigned char sleepEnabledFlag[4];

// Communication
char RX_buffer[64]={0};         // Last packet received from base station
char TX_buffer[64]={0};         // Next packet to send to base station
DWORD commThreadId;
HANDLE commThread;
double numOfErrors[4], numOfPackets=0, errorPercentage[4];

// functions declaration
DWORD WINAPI CommThread( LPVOID lpParameter);


char speed(char value) {
    if(value >= 0) {
        return (value|0x80);
    } else {
        return ((-value)&0x7F);
    }
}

int computeVerticalAngle(signed int x, signed int y) {

    int currentAngle = 0;

	currentAngle = (signed int)(atan2f((float)x, (float)y)*RAD_2_DEG);

	if(currentAngle<0) {
		currentAngle = 360+currentAngle;	// angles from 0 to 360
	}

    return currentAngle;

}

int getIdFromAddress(int address) {
    int i=0;
    for(i=0; i<4; i++) {
        if(address == robotAddress[i]) {
            return i;
        }
    }
    return -1;
}

void openRobotComm() {
    openCommunication();
    TX_buffer[0]=0x27;
    commThread = CreateThread(NULL, 0, CommThread, NULL, 0, &commThreadId);
}

void closeRobotComm() {
    closeCommunication();
    TerminateThread(commThread, 0);
    CloseHandle(commThread);
}

void setLeftSpeed(int robotAddr, char value) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        leftSpeed[id] = value;
    }
}

void setRightSpeed(int robotAddr, char value) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        rightSpeed[id] = value;
    }
}

void setRed(int robotAddr, unsigned char value) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if(value < 0) {
            value = 0;
        }
        if(value > 100) {
            value = 100;
        }
        redLed[id] = value;
    }
}

void setGreen(int robotAddr, unsigned char value) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if(value < 0) {
            value = 0;
        }
        if(value > 100) {
            value = 100;
        }
        greenLed[id] = value;
    }
}

void setBlue(int robotAddr, unsigned char value) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if(value < 0) {
            value = 0;
        }
        if(value > 100) {
            value = 100;
        }
        blueLed[id] = value;
    }
}

void turnOnFrontIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        FRONT_IR_ON(flagsTX[0][id]);
    }
}

void turnOffFrontIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        FRONT_IR_OFF(flagsTX[0][id]);
    }
}

void turnOnBackIR(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        BACK_IR_ON(flagsTX[0][id]);
    }
}

void turnOffBackIR(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        BACK_IR_OFF(flagsTX[0][id]);
    }
}

void turnOnAllIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        ALL_IR_ON(flagsTX[0][id]);
    }
}

void turnOffAllIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        ALL_IR_OFF(flagsTX[0][id]);
    }
}

void enableTVRemote(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        TV_REMOTE_ON(flagsTX[0][id]);
    }
}

void disableTVRemote(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        TV_REMOTE_OFF(flagsTX[0][id]);
    }
}

void enableSleep(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        SLEEP_ON(flagsTX[0][id]);
        sleepEnabledFlag[id] = 1;
    }
}

void disableSleep(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        SLEEP_OFF(flagsTX[0][id]);
        sleepEnabledFlag[id] = 0;
    }
}

void enableObstacleAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        OBSTACLE_AVOID_ON(flagsTX[0][id]);
    }
}

void disableObstacleAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        OBSTACLE_AVOID_OFF(flagsTX[0][id]);
    }
}

void enableCliffAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        CLIFF_AVOID_ON(flagsTX[0][id]);
    }
}

void disableCliffAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        CLIFF_AVOID_OFF(flagsTX[0][id]);
    }
}

void setRobotAddress(int robotIndex, int robotAddr) {
    robotAddress[robotIndex] = robotAddr;
}

unsigned int getProximity(int robotAddr, int proxId) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return proxValue[id][proxId];
    }
    return -1;
}

unsigned int getProximityAmbient(int robotAddr, int proxId) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return proxAmbientValue[id][proxId];
    }
    return -1;
}

unsigned int getGround(int robotAddr, int groundId) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return groundValue[id][groundId];
    }
    return -1;
}

unsigned int getGroundAmbient(int robotAddr, int groundId) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return groundAmbientValue[id][groundId];
    }
    return -1;
}

void getAllProximity(int robotAddr, unsigned int* proxArr) {
    int id = getIdFromAddress(robotAddr);
    int i = 0;
    if(id>=0) {
        for(i=0; i<8; i++) {
            proxArr[i] = proxValue[id][i];
        }
    }
}

void getAllProximityAmbient(int robotAddr, unsigned int* proxArr) {
    int id = getIdFromAddress(robotAddr);
    int i = 0;
    if(id>=0) {
        for(i=0; i<8; i++) {
            proxArr[i] = proxAmbientValue[id][i];
        }
    }
}

void getAllGround(int robotAddr, unsigned int* groundArr) {
    int id = getIdFromAddress(robotAddr);
    int i = 0;
    if(id>=0) {
        for(i=0; i<8; i++) {
            groundArr[i] = groundValue[id][i];
        }
    }
}

void getAllGroundAmbient(int robotAddr, unsigned int* groundArr) {
    int id = getIdFromAddress(robotAddr);
    int i = 0;
    if(id>=0) {
        for(i=0; i<8; i++) {
            groundArr[i] = groundAmbientValue[id][i];
        }
    }
}

unsigned int getBatteryAdc(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if(batteryAdc[id] >= 934) {           // 934 is the measured adc value when the battery is charged
            batteryPercent[id] = 100;
        } else if(batteryAdc[id] <= 780) {    // 780 is the measrued adc value when the battery is discharged
            batteryPercent[id] = 0;
        } else {
            batteryPercent[id] = (unsigned int)((float)((batteryAdc[id]-780.0)/(934.0-780.0))*100.0);
        }
        return batteryAdc[id];
    }
    return -1;
}

unsigned int getBatteryPercent(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return batteryPercent[id];
    }
    return -1;
}

signed int getAccX(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return accX[id];
    }
    return -1;
}

signed int getAccY(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return accY[id];
    }
    return -1;
}

signed int getAccZ(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return accZ[id];
    }
    return -1;
}

unsigned char getSelector(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return selector[id];
    }
    return -1;
}

unsigned char getTVRemoteCommand(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return tvRemote[id];
    }
    return -1;
}

signed int getOdomTheta(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return robTheta[id];
    }
    return -1;
}

signed int getOdomXpos(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return robXPos[id];
    }
    return -1;
}

signed int getOdomYpos(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return robYPos[id];
    }
    return -1;
}

void setSmallLed(int robotAddr, int ledId, int state) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if(state==0) {
            smallLeds[id] &= ~(1<<ledId);
        } else {
            smallLeds[id] |= (1<<ledId);
        }
    }
}

void turnOffSmallLeds(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        smallLeds[id] = 0;
    }
}

void turnOnSmallLeds(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        smallLeds[id] = 0xFF;
    }
}

int getVerticalAngle(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return computeVerticalAngle(accX[id], accY[id]);
    }
    return -1;
}

void calibrateSensors(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        CALIBRATION_ON(flagsTX[0][id]);
    }
}

void startOdometryCalibration(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        flagsTX[1][id] |= (1<<0);
    }
}

unsigned char robotIsCharging(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if((flagsRX[id]&0x01) == 0x01) {
            return 1;
        } else {
            return 0;
        }
    }
    return 0;
}

unsigned char robotIsCharged(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if((flagsRX[id]&0x04) == 0x04) {
            return 1;
        } else {
            return 0;
        }
    }
    return 0;
}

unsigned char buttonIsPressed(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if((flagsRX[id]&0x02) == 0x02) {
            return 1;
        } else {
            return 0;
        }
    }
    return 0;
}

void resetFlagTX(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        flagsTX[0][id] = 0;
        flagsTX[1][id] = 0;
    }
}

unsigned char getFlagTX(int robotAddr, int flagInd) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return flagsTX[flagInd][id];
    }
    return -1;
}

unsigned char getFlagRX(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return flagsRX[id];
    }
    return -1;
}

signed long int getLeftMotSteps(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return leftMotSteps[id];
    }
    return -1;
}

signed long int getRightMotSteps(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return rightMotSteps[id];
    }
    return -1;
}

double getRFQuality(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return 100.0-errorPercentage[id];
    }
    return -1;
}

void transferData() {

    int err=0;

    // first robot
    if(sleepEnabledFlag[0] == 1) {
        TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = flagsTX[0][0];                 // activate IR remote control
        TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(0*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (robotAddress[0]>>8)&0xFF;     // address of the robot
        TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = robotAddress[0]&0xFF;
    } else {
        TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = redLed[0];                     // R
        TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = blueLed[0];    	            // B
        TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = greenLed[0];                   // G
        TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = flagsTX[0][0];                    // flags
        TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[0]);                     // speed right
        TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[0]);                     // speed left
        TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = smallLeds[0];                   // small green leds
        TX_buffer[(0*ROBOT_PACKET_SIZE)+8] = flagsTX[1][0];
        TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (robotAddress[0]>>8)&0xFF;     // address of the robot
        TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = robotAddress[0]&0xFF;
    }

    // second robot
    if(sleepEnabledFlag[1] == 1) {
        TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = flagsTX[0][1];                       // activate IR remote control
        TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(1*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = ((robotAddress[1])>>8)&0xFF; // address of the robot
        TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = (robotAddress[1])&0xFF;
    } else {
        TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = redLed[1];                     // R
        TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = blueLed[1];    	            // B
        TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = greenLed[1];                   // G
        TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = flagsTX[0][1];                    // flags
        TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[1]);                     // speed right
        TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[1]);                     // speed left
        TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = smallLeds[1];                   // small green leds
        TX_buffer[(1*ROBOT_PACKET_SIZE)+8] = flagsTX[1][1];
        TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = (robotAddress[1]>>8)&0xFF;     // address of the robot
        TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = robotAddress[1]&0xFF;
    }

    // third robot
    if(sleepEnabledFlag[2] == 1) {
        TX_buffer[(2*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(2*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(2*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(2*ROBOT_PACKET_SIZE)+4] = flagsTX[0][2];                       // activate IR remote control
        TX_buffer[(2*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(2*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(2*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(2*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(2*ROBOT_PACKET_SIZE)+14] = ((robotAddress[2])>>8)&0xFF; // address of the robot
        TX_buffer[(2*ROBOT_PACKET_SIZE)+15] = (robotAddress[2])&0xFF;
    } else {
        TX_buffer[(2*ROBOT_PACKET_SIZE)+1] = redLed[2];                     // R
        TX_buffer[(2*ROBOT_PACKET_SIZE)+2] = blueLed[2];    	            // B
        TX_buffer[(2*ROBOT_PACKET_SIZE)+3] = greenLed[2];                   // G
        TX_buffer[(2*ROBOT_PACKET_SIZE)+4] = flagsTX[0][2];                    // flags
        TX_buffer[(2*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[2]);                     // speed right
        TX_buffer[(2*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[2]);                     // speed left
        TX_buffer[(2*ROBOT_PACKET_SIZE)+7] = smallLeds[2];                   // small green leds
        TX_buffer[(2*ROBOT_PACKET_SIZE)+8] = flagsTX[1][2];
        TX_buffer[(2*ROBOT_PACKET_SIZE)+14] = (robotAddress[2]>>8)&0xFF;     // address of the robot
        TX_buffer[(2*ROBOT_PACKET_SIZE)+15] = robotAddress[2]&0xFF;
    }

    // fourth robot
    if(sleepEnabledFlag[3] == 1) {
        TX_buffer[(3*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(3*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(3*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(3*ROBOT_PACKET_SIZE)+4] = flagsTX[0][3];                       // activate IR remote control
        TX_buffer[(3*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(3*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(3*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(3*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(3*ROBOT_PACKET_SIZE)+14] = ((robotAddress[3])>>8)&0xFF; // address of the robot
        TX_buffer[(3*ROBOT_PACKET_SIZE)+15] = (robotAddress[3])&0xFF;
    } else {
        TX_buffer[(3*ROBOT_PACKET_SIZE)+1] = redLed[3];                     // R
        TX_buffer[(3*ROBOT_PACKET_SIZE)+2] = blueLed[3];    	            // B
        TX_buffer[(3*ROBOT_PACKET_SIZE)+3] = greenLed[3];                   // G
        TX_buffer[(3*ROBOT_PACKET_SIZE)+4] = flagsTX[0][3];                    // flags
        TX_buffer[(3*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[3]);                     // speed right
        TX_buffer[(3*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[3]);                     // speed left
        TX_buffer[(3*ROBOT_PACKET_SIZE)+7] = smallLeds[3];                   // small green leds
        TX_buffer[(3*ROBOT_PACKET_SIZE)+8] = flagsTX[1][3];
        TX_buffer[(3*ROBOT_PACKET_SIZE)+14] = (robotAddress[3]>>8)&0xFF;     // address of the robot
        TX_buffer[(3*ROBOT_PACKET_SIZE)+15] = robotAddress[3]&0xFF;
    }

    // transfer the data to the base-station
    err = usb_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
    if(err < 0) {
        printf("send error!\n");
    }

    RX_buffer[0] = 0;
    RX_buffer[16] = 0;
    RX_buffer[32] = 0;
    RX_buffer[48] = 0;
    err = usb_receive(RX_buffer, 64);     // receive the ack payload for 4 robots at a time (16 bytes for each one)
    if(err < 0) {
        printf("receive error!\n");
    }

    // the base-station returns this "error" codes:
    // - 0 => transmission succeed (no ack received though)
    // - 1 => ack received (should not be returned because if the ack is received, then the payload is read)
    // - 2 => transfer failed
    if((int)((unsigned char)RX_buffer[0])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 0, currAddress[0]);
        numOfErrors[0]++;
    } else {
        // extract the sensors data for the first robot based on the packet id (first byte):
        // id=3 | prox0         | prox1         | prox2         | prox3         | prox5         | prox6         | prox7         | flags
        // id=4 | prox4         | gound0        | ground1       | ground2       | ground3       | accX          | accY          | tv remote
        // id=5 | proxAmbient0  | proxAmbient1  | proxAmbient2  | proxAmbient3  | proxAmbient5  | proxAmbient6  | proxAmbient7  | selector
        // id=6 | proxAmbient4  | goundAmbient0 | goundAmbient1 | goundAmbient2 | goundAmbient3 | accZ          | battery       | free byte
        switch((int)((unsigned char)RX_buffer[0])) {
            case 3:
                proxValue[0][0] = (((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1]);
                proxValue[0][1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                proxValue[0][2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                proxValue[0][3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                proxValue[0][5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                proxValue[0][6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                proxValue[0][7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                flagsRX[0] = (unsigned char)RX_buffer[15];
                break;

            case 4:
                proxValue[0][4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                groundValue[0][0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                groundValue[0][1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                groundValue[0][2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                groundValue[0][3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                accX[0] = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
                accY[0] = (int)((RX_buffer[14]<<8)|(RX_buffer[13]));
                tvRemote[0] = (unsigned char)RX_buffer[15];
                break;

            case 5:
                proxAmbientValue[0][0] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                proxAmbientValue[0][1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                proxAmbientValue[0][2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                proxAmbientValue[0][3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                proxAmbientValue[0][5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                proxAmbientValue[0][6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                proxAmbientValue[0][7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                selector[0] = (unsigned char)RX_buffer[15];
                break;

            case 6:
                proxAmbientValue[0][4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                groundAmbientValue[0][0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                groundAmbientValue[0][1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                groundAmbientValue[0][2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                groundAmbientValue[0][3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                accZ[0] = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
                batteryAdc[0] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                // RX_buffer[15] is free
                break;

            case 7:
                leftMotSteps[0] = ((signed long)((unsigned char)RX_buffer[4]<<24)| ((unsigned char)RX_buffer[3]<<16)| ((unsigned char)RX_buffer[2]<<8)|((unsigned char)RX_buffer[1]));
                rightMotSteps[0] = ((signed long)((unsigned char)RX_buffer[8]<<24)| ((unsigned char)RX_buffer[7]<<16)| ((unsigned char)RX_buffer[6]<<8)|((unsigned char)RX_buffer[5]));
                robTheta[0] = ((((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9])/10);//%360;
                robXPos[0] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                robYPos[0] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                break;
        }
    }

    if((int)((unsigned char)RX_buffer[16])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 1, currAddress[1]);
        numOfErrors[1]++;
    } else {
        switch((int)((unsigned char)RX_buffer[16])) {
            case 3:
                proxValue[1][0] = (((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17]);
                proxValue[1][1] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                proxValue[1][2] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                proxValue[1][3] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                proxValue[1][5] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                proxValue[1][6] = ((signed int)RX_buffer[28]<<8)|(unsigned char)RX_buffer[27];
                proxValue[1][7] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                flagsRX[1] = (unsigned char)RX_buffer[31];
                break;

            case 4:
                proxValue[1][4] = ((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17];
                groundValue[1][0] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                groundValue[1][1] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                groundValue[1][2] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                groundValue[1][3] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                accX[1] = (int)((RX_buffer[28]<<8)|(RX_buffer[27]));
                accY[1] = (int)((RX_buffer[30]<<8)|(RX_buffer[29]));
                tvRemote[1] = (unsigned char)RX_buffer[31];
                break;

            case 5:
                proxAmbientValue[1][0] = ((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17];
                proxAmbientValue[1][1] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                proxAmbientValue[1][2] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                proxAmbientValue[1][3] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                proxAmbientValue[1][5] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                proxAmbientValue[1][6] = ((signed int)RX_buffer[28]<<8)|(unsigned char)RX_buffer[27];
                proxAmbientValue[1][7] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                selector[1] = (unsigned char)RX_buffer[31];
                break;

            case 6:
                proxAmbientValue[1][4] = ((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17];
                groundAmbientValue[1][0] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                groundAmbientValue[1][1] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                groundAmbientValue[1][2] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                groundAmbientValue[1][3] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                accZ[1] = (int)((RX_buffer[28]<<8)|(RX_buffer[27]));
                batteryAdc[1] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                // RX_buffer[31] is free
                break;

            case 7:
                leftMotSteps[1] = ((signed long)((unsigned char)RX_buffer[20]<<24)| ((unsigned char)RX_buffer[19]<<16)| ((unsigned char)RX_buffer[18]<<8)|((unsigned char)RX_buffer[17]));
                rightMotSteps[1] = ((signed long)((unsigned char)RX_buffer[24]<<24)| ((unsigned char)RX_buffer[23]<<16)| ((unsigned char)RX_buffer[22]<<8)|((unsigned char)RX_buffer[21]));
                robTheta[1] = ((((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25])/10);//%360;
                robXPos[1] = ((signed int)RX_buffer[28]<<8)|(unsigned char)RX_buffer[27];
                robYPos[1] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                break;
        }
    }

    if((int)((unsigned char)RX_buffer[32])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 2, currAddress[2]);
        numOfErrors[2]++;
    } else {
        switch((int)((unsigned char)RX_buffer[32])) {
            case 3:
                proxValue[2][0] = (((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33]);
                proxValue[2][1] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                proxValue[2][2] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                proxValue[2][3] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                proxValue[2][5] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                proxValue[2][6] = ((signed int)RX_buffer[44]<<8)|(unsigned char)RX_buffer[43];
                proxValue[2][7] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                flagsRX[2] = (unsigned char)RX_buffer[47];
                break;

            case 4:
                proxValue[2][4] = ((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33];
                groundValue[2][0] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                groundValue[2][1] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                groundValue[2][2] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                groundValue[2][3] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                accX[2] = (int)((RX_buffer[44]<<8)|(RX_buffer[43]));
                accY[2] = (int)((RX_buffer[46]<<8)|(RX_buffer[45]));
                tvRemote[2] = (unsigned char)RX_buffer[47];
                break;

            case 5:
                proxAmbientValue[2][0] = ((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33];
                proxAmbientValue[2][1] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                proxAmbientValue[2][2] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                proxAmbientValue[2][3] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                proxAmbientValue[2][5] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                proxAmbientValue[2][6] = ((signed int)RX_buffer[44]<<8)|(unsigned char)RX_buffer[43];
                proxAmbientValue[2][7] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                selector[2] = (unsigned char)RX_buffer[47];
                break;

            case 6:
                proxAmbientValue[2][4] = ((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33];
                groundAmbientValue[2][0] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                groundAmbientValue[2][1] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                groundAmbientValue[2][2] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                groundAmbientValue[2][3] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                accZ[2] = (int)((RX_buffer[44]<<8)|(RX_buffer[43]));
                batteryAdc[2] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                // RX_buffer[47] is free
                break;

            case 7:
                leftMotSteps[2] = ((signed long)((unsigned char)RX_buffer[36]<<24)| ((unsigned char)RX_buffer[35]<<16)| ((unsigned char)RX_buffer[34]<<8)|((unsigned char)RX_buffer[33]));
                rightMotSteps[2] = ((signed long)((unsigned char)RX_buffer[40]<<24)| ((unsigned char)RX_buffer[39]<<16)| ((unsigned char)RX_buffer[38]<<8)|((unsigned char)RX_buffer[37]));
                robTheta[2] = ((((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41])/10);//%360;
                robXPos[2] = ((signed int)RX_buffer[44]<<8)|(unsigned char)RX_buffer[43];
                robYPos[2] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                break;
        }
    }

    if((int)((unsigned char)RX_buffer[48])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 3, currAddress[3]);
        numOfErrors[3]++;
    } else {
        switch((int)((unsigned char)RX_buffer[48])) {
            case 3:
                proxValue[3][0] = (((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49]);
                proxValue[3][1] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                proxValue[3][2] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                proxValue[3][3] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                proxValue[3][5] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                proxValue[3][6] = ((signed int)RX_buffer[60]<<8)|(unsigned char)RX_buffer[59];
                proxValue[3][7] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                flagsRX[3] = (unsigned char)RX_buffer[63];
                break;

            case 4:
                proxValue[3][4] = ((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49];
                groundValue[3][0] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                groundValue[3][1] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                groundValue[3][2] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                groundValue[3][3] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                accX[3] = (int)((RX_buffer[60]<<8)|(RX_buffer[59]));
                accY[3] = (int)((RX_buffer[62]<<8)|(RX_buffer[61]));
                tvRemote[3] = (unsigned char)RX_buffer[63];
                break;

            case 5:
                proxAmbientValue[3][0] = ((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49];
                proxAmbientValue[3][1] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                proxAmbientValue[3][2] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                proxAmbientValue[3][3] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                proxAmbientValue[3][5] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                proxAmbientValue[3][6] = ((signed int)RX_buffer[60]<<8)|(unsigned char)RX_buffer[59];
                proxAmbientValue[3][7] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                selector[3] = (unsigned char)RX_buffer[63];
                break;

            case 6:
                proxAmbientValue[3][4] = ((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49];
                groundAmbientValue[3][0] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                groundAmbientValue[3][1] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                groundAmbientValue[3][2] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                groundAmbientValue[3][3] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                accZ[3] = (int)((RX_buffer[60]<<8)|(RX_buffer[59]));
                batteryAdc[3] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                // RX_buffer[15] is free
                break;

            case 7:
                leftMotSteps[3] = ((signed long)((unsigned char)RX_buffer[52]<<24)| ((unsigned char)RX_buffer[51]<<16)| ((unsigned char)RX_buffer[50]<<8)|((unsigned char)RX_buffer[49]));
                rightMotSteps[3] = ((signed long)((unsigned char)RX_buffer[56]<<24)| ((unsigned char)RX_buffer[55]<<16)| ((unsigned char)RX_buffer[54]<<8)|((unsigned char)RX_buffer[53]));
                robTheta[3] = ((((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57])/10);//%360;
                robXPos[3] = ((signed int)RX_buffer[60]<<8)|(unsigned char)RX_buffer[59];
                robYPos[3] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                break;
        }
    }

    CALIBRATION_OFF(flagsTX[0][0]);
    CALIBRATION_OFF(flagsTX[0][1]);
    CALIBRATION_OFF(flagsTX[0][2]);
    CALIBRATION_OFF(flagsTX[0][3]);
    flagsTX[1][0] &= ~(1<<0);
    flagsTX[1][1] &= ~(1<<0);
    flagsTX[1][2] &= ~(1<<0);
    flagsTX[1][3] &= ~(1<<0);

}

DWORD WINAPI CommThread( LPVOID lpParameter) {

    SYSTEMTIME currTimeRF;
    FILETIME currTimeRFF;
    ULONGLONG currTimeRF64;
    SYSTEMTIME txTimeRF;
    FILETIME txTimeRFF;
    ULONGLONG txTimeRF64;
    SYSTEMTIME exitTime;
    FILETIME exitTimeF;
    ULONGLONG exitTime64;

    GetSystemTime(&currTimeRF);
    SystemTimeToFileTime(&currTimeRF, &currTimeRFF);
    currTimeRF64 = (((ULONGLONG) currTimeRFF.dwHighDateTime) << 32) + currTimeRFF.dwLowDateTime;
    GetSystemTime(&txTimeRF);
    SystemTimeToFileTime(&txTimeRF, &txTimeRFF);
    txTimeRF64 = (((ULONGLONG) txTimeRFF.dwHighDateTime) << 32) + txTimeRFF.dwLowDateTime;
    GetSystemTime(&exitTime);
    SystemTimeToFileTime(&exitTime, &exitTimeF);
    exitTime64 = (((ULONGLONG) exitTimeF.dwHighDateTime) << 32) + exitTimeF.dwLowDateTime;

    while(1) {
        transferData();

        while(1) {
            GetSystemTime(&currTimeRF);
            SystemTimeToFileTime(&currTimeRF, &currTimeRFF);
            currTimeRF64 = (((ULONGLONG) currTimeRFF.dwHighDateTime) << 32) + currTimeRFF.dwLowDateTime;    // 100 nsec resolution

            if(((currTimeRF64-txTimeRF64)/10000 > 4)) {   // 4 ms => transfer @ 250 Hz
                GetSystemTime(&txTimeRF);
                SystemTimeToFileTime(&txTimeRF, &txTimeRFF);
                txTimeRF64 = (((ULONGLONG) txTimeRFF.dwHighDateTime) << 32) + txTimeRFF.dwLowDateTime;
                break;
            }
        }

        numOfPackets++;

        if(((currTimeRF64-exitTime64)/10000 > 5000)) { // 5 seconsd
            GetSystemTime(&exitTime);
            SystemTimeToFileTime(&exitTime, &exitTimeF);
            exitTime64 = (((ULONGLONG) exitTimeF.dwHighDateTime) << 32) + exitTimeF.dwLowDateTime;
            errorPercentage[0] = numOfErrors[0]/numOfPackets*100.0;
            errorPercentage[1] = numOfErrors[1]/numOfPackets*100.0;
            errorPercentage[2] = numOfErrors[2]/numOfPackets*100.0;
            errorPercentage[3] = numOfErrors[3]/numOfPackets*100.0;
            numOfErrors[0] = 0;
            numOfErrors[1] = 0;
            numOfErrors[2] = 0;
            numOfErrors[3] = 0;
            numOfPackets = 0;
        }
    }

    return 0;
}

