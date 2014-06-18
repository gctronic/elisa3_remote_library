
#include "elisa3-lib.h"
#ifdef _WIN32
    #include "windows.h"
#endif

#if defined(__linux__) || defined(__APPLE__)
    #include "pthread.h"
	#include <time.h>
	#include <sys/time.h>
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
int robotAddress[100];
char leftSpeed[100];
char rightSpeed[100];
char redLed[100], greenLed[100], blueLed[100];
unsigned int proxValue[100][8];
unsigned int proxAmbientValue[100][8];
unsigned int groundValue[100][4];
unsigned int groundAmbientValue[100][4];
unsigned int batteryAdc[100];
unsigned int batteryPercent[100];
signed int accX[100], accY[100], accZ[100];
unsigned char selector[100];
unsigned char tvRemote[100];
unsigned char flagsRX[100];
unsigned char flagsTX[100][2];
unsigned char smallLeds[100];
signed long int leftMotSteps[100], rightMotSteps[100];
signed int robTheta[100], robXPos[100], robYPos[100];
unsigned char sleepEnabledFlag[100];

// Communication
char RX_buffer[64]={0};         // Last packet received from base station
char TX_buffer[64]={0};         // Next packet to send to base station
#ifdef _WIN32
DWORD commThreadId;
HANDLE commThread;
#endif
#if defined(__linux__) || defined(__APPLE__)
pthread_t commThread;
#endif
double numOfErrors[100], numOfPackets=0, errorPercentage[100];
unsigned char lastMessageSentFlag[100];
unsigned char calibrationSent[100];
unsigned char calibrateOdomSent[100];
unsigned char stopTransmissionFlag = 0;
unsigned int currNumRobots = 0;
unsigned int currPacketId = 0;

// functions declaration
#ifdef _WIN32
DWORD WINAPI CommThread( LPVOID lpParameter);
#endif
#if defined(__linux__) || defined(__APPLE__)
void *CommThread(void *arg);
#endif

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
    for(i=0; i<currNumRobots; i++) {
        if(address == robotAddress[i]) {
            return i;
        }
    }
    return -1;
}

void startCommunication(int *robotAddr, int numRobots) {
    openCommunication();
    TX_buffer[0]=0x27;

#ifdef _WIN32
    commThread = CreateThread(NULL, 0, CommThread, NULL, 0, &commThreadId);
#endif

#if defined(__linux__) || defined(__APPLE__)
    if(pthread_create(&commThread, NULL, CommThread, NULL)) {
        fprintf(stderr, "Error creating thread\n");
    }
#endif

    setRobotAddresses(robotAddr, numRobots);

}

void stopCommunication() {
    closeCommunication();

#ifdef _WIN32
    TerminateThread(commThread, 0);
    CloseHandle(commThread);
#endif

#if defined(__linux__) || defined(__APPLE__)
	pthread_cancel(commThread);
#endif

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

void setLeftSpeedForAll(char *value) {
    int i = 0;
    for(i=0; i<currNumRobots; i++) {
        leftSpeed[i] = value[i];
    }
}

void setRightSpeedForAll(char *value) {
    int i = 0;
    for(i=0; i<currNumRobots; i++) {
        rightSpeed[i] = value[i];
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

void setRedForAll(unsigned char *value) {
    int i = 0;
    for(i=0; i<currNumRobots; i++) {
        if(value[i] < 0) {
            value[i] = 0;
        }
        if(value[i] > 100) {
            value[i] = 100;
        }
        redLed[i] = value[i];
    }
}

void setGreenForAll(unsigned char *value) {
    int i = 0;
    for(i=0; i<currNumRobots; i++) {
        if(value[i] < 0) {
            value[i] = 0;
        }
        if(value[i] > 100) {
            value[i] = 100;
        }
        greenLed[i] = value[i];
    }
}

void setBlueForAll(unsigned char *value) {
    int i = 0;
    for(i=0; i<currNumRobots; i++) {
        if(value[i] < 0) {
            value[i] = 0;
        }
        if(value[i] > 100) {
            value[i] = 100;
        }
        blueLed[i] = value[i];
    }
}

void turnOnFrontIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        FRONT_IR_ON(flagsTX[id][0]);
    }
}

void turnOffFrontIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        FRONT_IR_OFF(flagsTX[id][0]);
    }
}

void turnOnBackIR(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        BACK_IR_ON(flagsTX[id][0]);
    }
}

void turnOffBackIR(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        BACK_IR_OFF(flagsTX[id][0]);
    }
}

void turnOnAllIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        ALL_IR_ON(flagsTX[id][0]);
    }
}

void turnOffAllIRs(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        ALL_IR_OFF(flagsTX[id][0]);
    }
}

void enableTVRemote(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        TV_REMOTE_ON(flagsTX[id][0]);
    }
}

void disableTVRemote(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        TV_REMOTE_OFF(flagsTX[id][0]);
    }
}

void enableSleep(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        SLEEP_ON(flagsTX[id][0]);
        sleepEnabledFlag[id] = 1;
    }
}

void disableSleep(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        SLEEP_OFF(flagsTX[id][0]);
        sleepEnabledFlag[id] = 0;
    }
}

void enableObstacleAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        OBSTACLE_AVOID_ON(flagsTX[id][0]);
    }
}

void disableObstacleAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        OBSTACLE_AVOID_OFF(flagsTX[id][0]);
    }
}

void enableCliffAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        CLIFF_AVOID_ON(flagsTX[id][0]);
    }
}

void disableCliffAvoidance(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        CLIFF_AVOID_OFF(flagsTX[id][0]);
    }
}

void setRobotAddress(int robotIndex, int robotAddr) {
    robotAddress[robotIndex] = robotAddr;
}

void setRobotAddresses(int *robotAddr, int numRobots) {
    int i = 0;
    for(i=0; i<numRobots; i++) {
        robotAddress[i] = robotAddr[i];
    }
    currNumRobots = numRobots;
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
        for(i=0; i<4; i++) {
            groundArr[i] = groundValue[id][i];
        }
    }
}

void getAllGroundAmbient(int robotAddr, unsigned int* groundArr) {
    int id = getIdFromAddress(robotAddr);
    int i = 0;
    if(id>=0) {
        for(i=0; i<4; i++) {
            groundArr[i] = groundAmbientValue[id][i];
        }
    }
}

void getAllProximityFromAll(unsigned int proxArr[][8]) {
    int i = 0, j = 0;
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<8; j++) {
            proxArr[i][j] = proxValue[i][j];
        }
    }
}

void getAllProximityAmbientFromAll(unsigned int proxArr[][8]) {
    int i = 0, j = 0;
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<8; j++) {
            proxArr[i][j] = proxAmbientValue[i][j];
        }
    }
}

void getAllGroundFromAll(unsigned int groundArr[][4]) {
    int i = 0, j = 0;
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<4; j++) {
            groundArr[i][j] = groundValue[i][j];
        }
    }
}

void getAllGroundAmbientFromAll(unsigned int groundArr[][4]) {
    int i = 0, j = 0;
    for(i=0; i<currNumRobots; i++) {
        for(j=0; j<4; j++) {
            groundArr[i][j] = groundAmbientValue[i][j];
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
        calibrationSent[id] = 0;
        CALIBRATION_ON(flagsTX[id][0]);
    }
}

void calibrateSensorsForAll() {
    int i = 0;
    for(i=0; i<currNumRobots; i++) {
        calibrationSent[i] = 0;
        CALIBRATION_ON(flagsTX[i][0]);
    }
}

void startOdometryCalibration(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
		calibrateOdomSent[id] = 0;
        flagsTX[id][1] |= (1<<0);
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
        flagsTX[id][0] = 0;
        flagsTX[id][1] = 0;
    }
}

unsigned char getFlagTX(int robotAddr, int flagInd) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        return flagsTX[id][flagInd];
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

void resetMessageIsSentFlag(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        lastMessageSentFlag[id] = 0;
    }
}

unsigned char messageIsSent(int robotAddr) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        if(lastMessageSentFlag[id]==3) {
            return 1;
        } else {
            return 0;
        }
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

void stopTransferData() {
    stopTransmissionFlag = 1;
}

void resumeTransferData() {
    stopTransmissionFlag = 0;
}

void setCompletePacket(int robotAddr, char red, char green, char blue, char flags[2], char left, char right, char leds) {
    int id = getIdFromAddress(robotAddr);
    if(id>=0) {
        redLed[id] = red;
        blueLed[id] = blue;
        greenLed[id] = green;
        flagsTX[id][0] = flags[0];
        flagsTX[id][1] = flags[1];
        leftSpeed[id] = left;
        rightSpeed[id] = right;
        smallLeds[id] = leds;
    }
}

void setCompletePacketForAll(int *robotAddr, char *red, char *green, char *blue, char flags[][2], char *left, char *right, char *leds) {
    int i=0;
    for(i=0; i<currNumRobots; i++) {
        redLed[i] = red[i];
        blueLed[i] = blue[i];
        greenLed[i] = green[i];
        flagsTX[i][0] = flags[i][0];
        flagsTX[i][1] = flags[i][1];
        leftSpeed[i] = left[i];
        rightSpeed[i] = right[i];
        smallLeds[i] = leds[i];
        robotAddress[i] = robotAddr[i];
    }
}

unsigned char sendMessageToRobot(int robotAddr, char red, char green, char blue, char flags[2], char left, char right, char leds, unsigned long us) {

    stopTransferData();

    currNumRobots = 1;
    robotAddress[0] = robotAddr;
    redLed[0] = red;
    blueLed[0] = blue;
    greenLed[0] = green;
    flagsTX[0][0] = flags[0];
    flagsTX[0][1] = flags[1];
    leftSpeed[0] = left;
    rightSpeed[0] = right;
    smallLeds[0] = leds;

    resumeTransferData();

    return waitForUpdate(robotAddr, us);
}

unsigned char waitForUpdate(int robotAddr, unsigned long us) {
#ifdef _WIN32
    SYSTEMTIME startTime;
    FILETIME startTimeF;
    ULONGLONG startTime64;
    SYSTEMTIME exitTime;
    FILETIME exitTimeF;
    ULONGLONG exitTime64;
    GetSystemTime(&startTime);
    SystemTimeToFileTime(&startTime, &startTimeF);
    startTime64 = (((ULONGLONG) startTimeF.dwHighDateTime) << 32) + startTimeF.dwLowDateTime;
#endif

#if defined(__linux__) || defined(__APPLE__)
	struct timeval startTime, exitTime;
    gettimeofday(&startTime, NULL);
	gettimeofday(&exitTime, NULL);
#endif

    resetMessageIsSentFlag(robotAddr);

    while(messageIsSent(robotAddr)==0) {
#ifdef _WIN32
        GetSystemTime(&exitTime);
        SystemTimeToFileTime(&exitTime, &exitTimeF);
        exitTime64 = (((ULONGLONG) exitTimeF.dwHighDateTime) << 32) + exitTimeF.dwLowDateTime;
        if(((exitTime64-startTime64)/10 > us)) {
            return 1;
        }
#endif

#if defined(__linux__) || defined(__APPLE__)
        gettimeofday(&exitTime, NULL);
        if((((exitTime.tv_sec * 1000000 + exitTime.tv_usec)-(startTime.tv_sec * 1000000 + startTime.tv_usec)) > us)) {
            return 1;
        }
#endif
    }

    return 0;
}

void transferData() {

    int err=0;

    // first robot
    if(sleepEnabledFlag[0] == 1) {
        TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+0][0];                 // activate IR remote control
        TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(0*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (robotAddress[currPacketId*4+0]>>8)&0xFF;     // address of the robot
        TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = robotAddress[currPacketId*4+0]&0xFF;
    } else {
        TX_buffer[(0*ROBOT_PACKET_SIZE)+1] = redLed[currPacketId*4+0];                     // R
        TX_buffer[(0*ROBOT_PACKET_SIZE)+2] = blueLed[currPacketId*4+0];    	            // B
        TX_buffer[(0*ROBOT_PACKET_SIZE)+3] = greenLed[currPacketId*4+0];                   // G
        TX_buffer[(0*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+0][0];                    // flags
        TX_buffer[(0*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[currPacketId*4+0]);                     // speed right
        TX_buffer[(0*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[currPacketId*4+0]);                     // speed left
        TX_buffer[(0*ROBOT_PACKET_SIZE)+7] = smallLeds[currPacketId*4+0];                   // small green leds
        TX_buffer[(0*ROBOT_PACKET_SIZE)+8] = flagsTX[currPacketId*4+0][1];
        TX_buffer[(0*ROBOT_PACKET_SIZE)+14] = (robotAddress[currPacketId*4+0]>>8)&0xFF;     // address of the robot
        TX_buffer[(0*ROBOT_PACKET_SIZE)+15] = robotAddress[currPacketId*4+0]&0xFF;
    }

    // second robot
    if(sleepEnabledFlag[1] == 1) {
        TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+1][0];                       // activate IR remote control
        TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(1*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = ((robotAddress[currPacketId*4+1])>>8)&0xFF; // address of the robot
        TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = (robotAddress[currPacketId*4+1])&0xFF;
    } else {
        TX_buffer[(1*ROBOT_PACKET_SIZE)+1] = redLed[currPacketId*4+1];                     // R
        TX_buffer[(1*ROBOT_PACKET_SIZE)+2] = blueLed[currPacketId*4+1];    	            // B
        TX_buffer[(1*ROBOT_PACKET_SIZE)+3] = greenLed[currPacketId*4+1];                   // G
        TX_buffer[(1*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+1][0];                    // flags
        TX_buffer[(1*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[currPacketId*4+1]);                     // speed right
        TX_buffer[(1*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[currPacketId*4+1]);                     // speed left
        TX_buffer[(1*ROBOT_PACKET_SIZE)+7] = smallLeds[currPacketId*4+1];                   // small green leds
        TX_buffer[(1*ROBOT_PACKET_SIZE)+8] = flagsTX[currPacketId*4+1][1];
        TX_buffer[(1*ROBOT_PACKET_SIZE)+14] = (robotAddress[currPacketId*4+1]>>8)&0xFF;     // address of the robot
        TX_buffer[(1*ROBOT_PACKET_SIZE)+15] = robotAddress[currPacketId*4+1]&0xFF;
    }

    // third robot
    if(sleepEnabledFlag[2] == 1) {
        TX_buffer[(2*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(2*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(2*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(2*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+2][0];                       // activate IR remote control
        TX_buffer[(2*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(2*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(2*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(2*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(2*ROBOT_PACKET_SIZE)+14] = ((robotAddress[currPacketId*4+2])>>8)&0xFF; // address of the robot
        TX_buffer[(2*ROBOT_PACKET_SIZE)+15] = (robotAddress[currPacketId*4+2])&0xFF;
    } else {
        TX_buffer[(2*ROBOT_PACKET_SIZE)+1] = redLed[currPacketId*4+2];                     // R
        TX_buffer[(2*ROBOT_PACKET_SIZE)+2] = blueLed[currPacketId*4+2];    	            // B
        TX_buffer[(2*ROBOT_PACKET_SIZE)+3] = greenLed[currPacketId*4+2];                   // G
        TX_buffer[(2*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+2][0];                    // flags
        TX_buffer[(2*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[currPacketId*4+2]);                     // speed right
        TX_buffer[(2*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[currPacketId*4+2]);                     // speed left
        TX_buffer[(2*ROBOT_PACKET_SIZE)+7] = smallLeds[currPacketId*4+2];                   // small green leds
        TX_buffer[(2*ROBOT_PACKET_SIZE)+8] = flagsTX[currPacketId*4+2][1];
        TX_buffer[(2*ROBOT_PACKET_SIZE)+14] = (robotAddress[currPacketId*4+2]>>8)&0xFF;     // address of the robot
        TX_buffer[(2*ROBOT_PACKET_SIZE)+15] = robotAddress[currPacketId*4+2]&0xFF;
    }

    // fourth robot
    if(sleepEnabledFlag[3] == 1) {
        TX_buffer[(3*ROBOT_PACKET_SIZE)+1] = 0x00;                          // R
        TX_buffer[(3*ROBOT_PACKET_SIZE)+2] = 0x00;				            // B
        TX_buffer[(3*ROBOT_PACKET_SIZE)+3] = 0x00;                          // G
        TX_buffer[(3*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+3][0];                       // activate IR remote control
        TX_buffer[(3*ROBOT_PACKET_SIZE)+5] = 0x00;                          // speed right (in percentage)
        TX_buffer[(3*ROBOT_PACKET_SIZE)+6] = 0x00;                          // speed left (in percentage)
        TX_buffer[(3*ROBOT_PACKET_SIZE)+7] = 0x00;                          // small green leds
        TX_buffer[(3*ROBOT_PACKET_SIZE)+8] = 0x00;
        TX_buffer[(3*ROBOT_PACKET_SIZE)+14] = ((robotAddress[currPacketId*4+3])>>8)&0xFF; // address of the robot
        TX_buffer[(3*ROBOT_PACKET_SIZE)+15] = (robotAddress[currPacketId*4+3])&0xFF;
    } else {
        TX_buffer[(3*ROBOT_PACKET_SIZE)+1] = redLed[currPacketId*4+3];                     // R
        TX_buffer[(3*ROBOT_PACKET_SIZE)+2] = blueLed[currPacketId*4+3];    	            // B
        TX_buffer[(3*ROBOT_PACKET_SIZE)+3] = greenLed[currPacketId*4+3];                   // G
        TX_buffer[(3*ROBOT_PACKET_SIZE)+4] = flagsTX[currPacketId*4+3][0];                    // flags
        TX_buffer[(3*ROBOT_PACKET_SIZE)+5] = speed(rightSpeed[currPacketId*4+3]);                     // speed right
        TX_buffer[(3*ROBOT_PACKET_SIZE)+6] = speed(leftSpeed[currPacketId*4+3]);                     // speed left
        TX_buffer[(3*ROBOT_PACKET_SIZE)+7] = smallLeds[currPacketId*4+3];                   // small green leds
        TX_buffer[(3*ROBOT_PACKET_SIZE)+8] = flagsTX[currPacketId*4+3][1];
        TX_buffer[(3*ROBOT_PACKET_SIZE)+14] = (robotAddress[currPacketId*4+3]>>8)&0xFF;     // address of the robot
        TX_buffer[(3*ROBOT_PACKET_SIZE)+15] = robotAddress[currPacketId*4+3]&0xFF;
    }

    // transfer the data to the base-station
    err = usb_send(TX_buffer, PACKETS_SIZE-UNUSED_BYTES);
    if(err < 0) {
        printf("send error!\n");
    }

    // when the flag "lastMessageSentFlag" is reset we aren't sure the current message is really sent to the radio module
    // for next transmission to the robots so we wait the message is sent twice
    if(lastMessageSentFlag[currPacketId*4+0]==0) {
        lastMessageSentFlag[currPacketId*4+0]=1;
    } else if(lastMessageSentFlag[currPacketId*4+0]==1) {
        lastMessageSentFlag[currPacketId*4+0]=2;
    }
    if(lastMessageSentFlag[currPacketId*4+1]==0) {
        lastMessageSentFlag[currPacketId*4+1]=1;
    } else if(lastMessageSentFlag[currPacketId*4+1]==1) {
        lastMessageSentFlag[currPacketId*4+1]=2;
    }
    if(lastMessageSentFlag[currPacketId*4+2]==0) {
        lastMessageSentFlag[currPacketId*4+2]=1;
    } else if(lastMessageSentFlag[currPacketId*4+2]==1) {
        lastMessageSentFlag[currPacketId*4+2]=2;
    }
    if(lastMessageSentFlag[currPacketId*4+3]==0) {
        lastMessageSentFlag[currPacketId*4+3]=1;
    } else if(lastMessageSentFlag[currPacketId*4+3]==1) {
        lastMessageSentFlag[currPacketId*4+3]=2;
    }

	calibrationSent[currPacketId*4+0]++;
	calibrationSent[currPacketId*4+1]++;
	calibrationSent[currPacketId*4+2]++;
	calibrationSent[currPacketId*4+3]++;

	calibrateOdomSent[currPacketId*4+0]++;
	calibrateOdomSent[currPacketId*4+1]++;
	calibrateOdomSent[currPacketId*4+2]++;
	calibrateOdomSent[currPacketId*4+3]++;

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
        numOfErrors[currPacketId*4+0]++;
    } else {
        if(lastMessageSentFlag[currPacketId*4+0]==2) {
            lastMessageSentFlag[currPacketId*4+0]=3;
        }
        // extract the sensors data for the first robot based on the packet id (first byte):
        // id=3 | prox0         | prox1         | prox2         | prox3         | prox5         | prox6         | prox7         | flags
        // id=4 | prox4         | gound0        | ground1       | ground2       | ground3       | accX          | accY          | tv remote
        // id=5 | proxAmbient0  | proxAmbient1  | proxAmbient2  | proxAmbient3  | proxAmbient5  | proxAmbient6  | proxAmbient7  | selector
        // id=6 | proxAmbient4  | goundAmbient0 | goundAmbient1 | goundAmbient2 | goundAmbient3 | accZ          | battery       | free byte
        switch((int)((unsigned char)RX_buffer[0])) {
            case 3:
                proxValue[currPacketId*4+0][0] = (((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1]);
                proxValue[currPacketId*4+0][1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                proxValue[currPacketId*4+0][2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                proxValue[currPacketId*4+0][3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                proxValue[currPacketId*4+0][5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                proxValue[currPacketId*4+0][6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                proxValue[currPacketId*4+0][7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                flagsRX[currPacketId*4+0] = (unsigned char)RX_buffer[15];
                break;

            case 4:
                proxValue[currPacketId*4+0][4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                groundValue[currPacketId*4+0][0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                groundValue[currPacketId*4+0][1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                groundValue[currPacketId*4+0][2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                groundValue[currPacketId*4+0][3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                accX[currPacketId*4+0] = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
                accY[currPacketId*4+0] = (int)((RX_buffer[14]<<8)|(RX_buffer[13]));
                tvRemote[currPacketId*4+0] = (unsigned char)RX_buffer[15];
                break;

            case 5:
                proxAmbientValue[currPacketId*4+0][0] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                proxAmbientValue[currPacketId*4+0][1] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                proxAmbientValue[currPacketId*4+0][2] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                proxAmbientValue[currPacketId*4+0][3] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                proxAmbientValue[currPacketId*4+0][5] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                proxAmbientValue[currPacketId*4+0][6] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                proxAmbientValue[currPacketId*4+0][7] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                selector[currPacketId*4+0] = (unsigned char)RX_buffer[15];
                break;

            case 6:
                proxAmbientValue[currPacketId*4+0][4] = ((signed int)RX_buffer[2]<<8)|(unsigned char)RX_buffer[1];
                groundAmbientValue[currPacketId*4+0][0] = ((signed int)RX_buffer[4]<<8)|(unsigned char)RX_buffer[3];
                groundAmbientValue[currPacketId*4+0][1] = ((signed int)RX_buffer[6]<<8)|(unsigned char)RX_buffer[5];
                groundAmbientValue[currPacketId*4+0][2] = ((signed int)RX_buffer[8]<<8)|(unsigned char)RX_buffer[7];
                groundAmbientValue[currPacketId*4+0][3] = ((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9];
                accZ[currPacketId*4+0] = (int)((RX_buffer[12]<<8)|(RX_buffer[11]));
                batteryAdc[currPacketId*4+0] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                // RX_buffer[15] is free
                break;

            case 7:
                leftMotSteps[currPacketId*4+0] = ((signed long)((unsigned char)RX_buffer[4]<<24)| ((unsigned char)RX_buffer[3]<<16)| ((unsigned char)RX_buffer[2]<<8)|((unsigned char)RX_buffer[1]));
                rightMotSteps[currPacketId*4+0] = ((signed long)((unsigned char)RX_buffer[8]<<24)| ((unsigned char)RX_buffer[7]<<16)| ((unsigned char)RX_buffer[6]<<8)|((unsigned char)RX_buffer[5]));
                robTheta[currPacketId*4+0] = ((((signed int)RX_buffer[10]<<8)|(unsigned char)RX_buffer[9])/10);//%360;
                robXPos[currPacketId*4+0] = ((signed int)RX_buffer[12]<<8)|(unsigned char)RX_buffer[11];
                robYPos[currPacketId*4+0] = ((signed int)RX_buffer[14]<<8)|(unsigned char)RX_buffer[13];
                break;
        }
    }

    if((int)((unsigned char)RX_buffer[16])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 1, currAddress[1]);
        numOfErrors[currPacketId*4+1]++;
    } else {
        if(lastMessageSentFlag[currPacketId*4+1]==2) {
            lastMessageSentFlag[currPacketId*4+1]=3;
        }
        switch((int)((unsigned char)RX_buffer[16])) {
            case 3:
                proxValue[currPacketId*4+1][0] = (((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17]);
                proxValue[currPacketId*4+1][1] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                proxValue[currPacketId*4+1][2] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                proxValue[currPacketId*4+1][3] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                proxValue[currPacketId*4+1][5] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                proxValue[currPacketId*4+1][6] = ((signed int)RX_buffer[28]<<8)|(unsigned char)RX_buffer[27];
                proxValue[currPacketId*4+1][7] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                flagsRX[currPacketId*4+1] = (unsigned char)RX_buffer[31];
                break;

            case 4:
                proxValue[currPacketId*4+1][4] = ((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17];
                groundValue[currPacketId*4+1][0] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                groundValue[currPacketId*4+1][1] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                groundValue[currPacketId*4+1][2] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                groundValue[currPacketId*4+1][3] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                accX[currPacketId*4+1] = (int)((RX_buffer[28]<<8)|(RX_buffer[27]));
                accY[currPacketId*4+1] = (int)((RX_buffer[30]<<8)|(RX_buffer[29]));
                tvRemote[currPacketId*4+1] = (unsigned char)RX_buffer[31];
                break;

            case 5:
                proxAmbientValue[currPacketId*4+1][0] = ((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17];
                proxAmbientValue[currPacketId*4+1][1] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                proxAmbientValue[currPacketId*4+1][2] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                proxAmbientValue[currPacketId*4+1][3] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                proxAmbientValue[currPacketId*4+1][5] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                proxAmbientValue[currPacketId*4+1][6] = ((signed int)RX_buffer[28]<<8)|(unsigned char)RX_buffer[27];
                proxAmbientValue[currPacketId*4+1][7] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                selector[currPacketId*4+1] = (unsigned char)RX_buffer[31];
                break;

            case 6:
                proxAmbientValue[currPacketId*4+1][4] = ((signed int)RX_buffer[18]<<8)|(unsigned char)RX_buffer[17];
                groundAmbientValue[currPacketId*4+1][0] = ((signed int)RX_buffer[20]<<8)|(unsigned char)RX_buffer[19];
                groundAmbientValue[currPacketId*4+1][1] = ((signed int)RX_buffer[22]<<8)|(unsigned char)RX_buffer[21];
                groundAmbientValue[currPacketId*4+1][2] = ((signed int)RX_buffer[24]<<8)|(unsigned char)RX_buffer[23];
                groundAmbientValue[currPacketId*4+1][3] = ((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25];
                accZ[currPacketId*4+1] = (int)((RX_buffer[28]<<8)|(RX_buffer[27]));
                batteryAdc[currPacketId*4+1] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                // RX_buffer[31] is free
                break;

            case 7:
                leftMotSteps[currPacketId*4+1] = ((signed long)((unsigned char)RX_buffer[20]<<24)| ((unsigned char)RX_buffer[19]<<16)| ((unsigned char)RX_buffer[18]<<8)|((unsigned char)RX_buffer[17]));
                rightMotSteps[currPacketId*4+1] = ((signed long)((unsigned char)RX_buffer[24]<<24)| ((unsigned char)RX_buffer[23]<<16)| ((unsigned char)RX_buffer[22]<<8)|((unsigned char)RX_buffer[21]));
                robTheta[currPacketId*4+1] = ((((signed int)RX_buffer[26]<<8)|(unsigned char)RX_buffer[25])/10);//%360;
                robXPos[currPacketId*4+1] = ((signed int)RX_buffer[28]<<8)|(unsigned char)RX_buffer[27];
                robYPos[currPacketId*4+1] = ((signed int)RX_buffer[30]<<8)|(unsigned char)RX_buffer[29];
                break;
        }
    }

    if((int)((unsigned char)RX_buffer[32])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 2, currAddress[2]);
        numOfErrors[currPacketId*4+2]++;
    } else {
        if(lastMessageSentFlag[currPacketId*4+2]==2) {
            lastMessageSentFlag[currPacketId*4+2]=3;
        }
        switch((int)((unsigned char)RX_buffer[32])) {
            case 3:
                proxValue[currPacketId*4+2][0] = (((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33]);
                proxValue[currPacketId*4+2][1] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                proxValue[currPacketId*4+2][2] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                proxValue[currPacketId*4+2][3] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                proxValue[currPacketId*4+2][5] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                proxValue[currPacketId*4+2][6] = ((signed int)RX_buffer[44]<<8)|(unsigned char)RX_buffer[43];
                proxValue[currPacketId*4+2][7] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                flagsRX[currPacketId*4+2] = (unsigned char)RX_buffer[47];
                break;

            case 4:
                proxValue[currPacketId*4+2][4] = ((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33];
                groundValue[currPacketId*4+2][0] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                groundValue[currPacketId*4+2][1] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                groundValue[currPacketId*4+2][2] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                groundValue[currPacketId*4+2][3] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                accX[currPacketId*4+2] = (int)((RX_buffer[44]<<8)|(RX_buffer[43]));
                accY[currPacketId*4+2] = (int)((RX_buffer[46]<<8)|(RX_buffer[45]));
                tvRemote[currPacketId*4+2] = (unsigned char)RX_buffer[47];
                break;

            case 5:
                proxAmbientValue[currPacketId*4+2][0] = ((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33];
                proxAmbientValue[currPacketId*4+2][1] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                proxAmbientValue[currPacketId*4+2][2] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                proxAmbientValue[currPacketId*4+2][3] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                proxAmbientValue[currPacketId*4+2][5] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                proxAmbientValue[currPacketId*4+2][6] = ((signed int)RX_buffer[44]<<8)|(unsigned char)RX_buffer[43];
                proxAmbientValue[currPacketId*4+2][7] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                selector[currPacketId*4+2] = (unsigned char)RX_buffer[47];
                break;

            case 6:
                proxAmbientValue[currPacketId*4+2][4] = ((signed int)RX_buffer[34]<<8)|(unsigned char)RX_buffer[33];
                groundAmbientValue[currPacketId*4+2][0] = ((signed int)RX_buffer[36]<<8)|(unsigned char)RX_buffer[35];
                groundAmbientValue[currPacketId*4+2][1] = ((signed int)RX_buffer[38]<<8)|(unsigned char)RX_buffer[37];
                groundAmbientValue[currPacketId*4+2][2] = ((signed int)RX_buffer[40]<<8)|(unsigned char)RX_buffer[39];
                groundAmbientValue[currPacketId*4+2][3] = ((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41];
                accZ[currPacketId*4+2] = (int)((RX_buffer[44]<<8)|(RX_buffer[43]));
                batteryAdc[currPacketId*4+2] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                // RX_buffer[47] is free
                break;

            case 7:
                leftMotSteps[currPacketId*4+2] = ((signed long)((unsigned char)RX_buffer[36]<<24)| ((unsigned char)RX_buffer[35]<<16)| ((unsigned char)RX_buffer[34]<<8)|((unsigned char)RX_buffer[33]));
                rightMotSteps[currPacketId*4+2] = ((signed long)((unsigned char)RX_buffer[40]<<24)| ((unsigned char)RX_buffer[39]<<16)| ((unsigned char)RX_buffer[38]<<8)|((unsigned char)RX_buffer[37]));
                robTheta[currPacketId*4+2] = ((((signed int)RX_buffer[42]<<8)|(unsigned char)RX_buffer[41])/10);//%360;
                robXPos[currPacketId*4+2] = ((signed int)RX_buffer[44]<<8)|(unsigned char)RX_buffer[43];
                robYPos[currPacketId*4+2] = ((signed int)RX_buffer[46]<<8)|(unsigned char)RX_buffer[45];
                break;
        }
    }

    if((int)((unsigned char)RX_buffer[48])<=2) { // if something goes wrong skip the data
        //printf("transfer failed to robot %d (addr=%d)\n", 3, currAddress[3]);
        numOfErrors[currPacketId*4+3]++;
    } else {
        if(lastMessageSentFlag[currPacketId*4+3]==2) {
            lastMessageSentFlag[currPacketId*4+3]=3;
        }
        switch((int)((unsigned char)RX_buffer[48])) {
            case 3:
                proxValue[currPacketId*4+3][0] = (((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49]);
                proxValue[currPacketId*4+3][1] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                proxValue[currPacketId*4+3][2] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                proxValue[currPacketId*4+3][3] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                proxValue[currPacketId*4+3][5] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                proxValue[currPacketId*4+3][6] = ((signed int)RX_buffer[60]<<8)|(unsigned char)RX_buffer[59];
                proxValue[currPacketId*4+3][7] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                flagsRX[currPacketId*4+3] = (unsigned char)RX_buffer[63];
                break;

            case 4:
                proxValue[currPacketId*4+3][4] = ((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49];
                groundValue[currPacketId*4+3][0] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                groundValue[currPacketId*4+3][1] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                groundValue[currPacketId*4+3][2] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                groundValue[currPacketId*4+3][3] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                accX[currPacketId*4+3] = (int)((RX_buffer[60]<<8)|(RX_buffer[59]));
                accY[currPacketId*4+3] = (int)((RX_buffer[62]<<8)|(RX_buffer[61]));
                tvRemote[currPacketId*4+3] = (unsigned char)RX_buffer[63];
                break;

            case 5:
                proxAmbientValue[currPacketId*4+3][0] = ((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49];
                proxAmbientValue[currPacketId*4+3][1] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                proxAmbientValue[currPacketId*4+3][2] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                proxAmbientValue[currPacketId*4+3][3] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                proxAmbientValue[currPacketId*4+3][5] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                proxAmbientValue[currPacketId*4+3][6] = ((signed int)RX_buffer[60]<<8)|(unsigned char)RX_buffer[59];
                proxAmbientValue[currPacketId*4+3][7] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                selector[currPacketId*4+3] = (unsigned char)RX_buffer[63];
                break;

            case 6:
                proxAmbientValue[currPacketId*4+3][4] = ((signed int)RX_buffer[50]<<8)|(unsigned char)RX_buffer[49];
                groundAmbientValue[currPacketId*4+3][0] = ((signed int)RX_buffer[52]<<8)|(unsigned char)RX_buffer[51];
                groundAmbientValue[currPacketId*4+3][1] = ((signed int)RX_buffer[54]<<8)|(unsigned char)RX_buffer[53];
                groundAmbientValue[currPacketId*4+3][2] = ((signed int)RX_buffer[56]<<8)|(unsigned char)RX_buffer[55];
                groundAmbientValue[currPacketId*4+3][3] = ((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57];
                accZ[currPacketId*4+3] = (int)((RX_buffer[60]<<8)|(RX_buffer[59]));
                batteryAdc[currPacketId*4+3] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                // RX_buffer[15] is free
                break;

            case 7:
                leftMotSteps[currPacketId*4+3] = ((signed long)((unsigned char)RX_buffer[52]<<24)| ((unsigned char)RX_buffer[51]<<16)| ((unsigned char)RX_buffer[50]<<8)|((unsigned char)RX_buffer[49]));
                rightMotSteps[currPacketId*4+3] = ((signed long)((unsigned char)RX_buffer[56]<<24)| ((unsigned char)RX_buffer[55]<<16)| ((unsigned char)RX_buffer[54]<<8)|((unsigned char)RX_buffer[53]));
                robTheta[currPacketId*4+3] = ((((signed int)RX_buffer[58]<<8)|(unsigned char)RX_buffer[57])/10);//%360;
                robXPos[currPacketId*4+3] = ((signed int)RX_buffer[60]<<8)|(unsigned char)RX_buffer[59];
                robYPos[currPacketId*4+3] = ((signed int)RX_buffer[62]<<8)|(unsigned char)RX_buffer[61];
                break;
        }
    }

	if(calibrationSent[currPacketId*4+0] > 2) {
	    CALIBRATION_OFF(flagsTX[currPacketId*4+0][0]);
	}
	if(calibrationSent[currPacketId*4+1] > 2) {
	    CALIBRATION_OFF(flagsTX[currPacketId*4+1][0]);
	}
	if(calibrationSent[currPacketId*4+2] > 2) {
    	CALIBRATION_OFF(flagsTX[currPacketId*4+2][0]);
	}
	if(calibrationSent[currPacketId*4+3] > 2) {
    	CALIBRATION_OFF(flagsTX[currPacketId*4+3][0]);
	}

	if(calibrateOdomSent[currPacketId*4+0] > 2) {
	    flagsTX[currPacketId*4+0][1] &= ~(1<<0);
	}
	if(calibrateOdomSent[currPacketId*4+1] > 2) {
    	flagsTX[currPacketId*4+1][1] &= ~(1<<0);
	}
	if(calibrateOdomSent[currPacketId*4+2] > 2) {
	    flagsTX[currPacketId*4+2][1] &= ~(1<<0);
	}
	if(calibrateOdomSent[currPacketId*4+3] > 2) {
	    flagsTX[currPacketId*4+3][1] &= ~(1<<0);
	}

}

#ifdef _WIN32
DWORD WINAPI CommThread( LPVOID lpParameter) {
    int i = 0;

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

        if(stopTransmissionFlag==1) {
            continue;
        }

        transferData();

        currPacketId++;
        if(currPacketId>=(currNumRobots/4)) {
            currPacketId = 0;
        }

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
            for(i=0; i<currNumRobots; i++) {
                errorPercentage[i] = numOfErrors[i]/numOfPackets*100.0;
                numOfErrors[i] = 0;
            }
            numOfPackets = 0;
        }
    }

    return 0;
}
#endif

#if defined(__linux__) || defined(__APPLE__)
void *CommThread(void *arg) {
	int i = 0;
	struct timeval currTimeRF, txTimeRF, exitTime;
	
    gettimeofday(&currTimeRF, NULL);
	gettimeofday(&txTimeRF, NULL);
	gettimeofday(&exitTime, NULL);

    while(1) {

        if(stopTransmissionFlag==1) {
            continue;
        }

        transferData();

        currPacketId++;
        if(currPacketId>=(currNumRobots/4)) {
            currPacketId = 0;
        }

        while(1) {
            gettimeofday(&currTimeRF, NULL);

            if((((currTimeRF.tv_sec * 1000000 + currTimeRF.tv_usec)-(txTimeRF.tv_sec * 1000000 + txTimeRF.tv_usec)) > 4000)) {   // 4 ms => transfer @ 250 Hz
                gettimeofday(&txTimeRF, NULL);
                break;
            }
        }

        numOfPackets++;

        if(((currTimeRF.tv_sec * 1000000 + currTimeRF.tv_usec)-(exitTime.tv_sec * 1000000 + exitTime.tv_usec) > 5000000)) { // 5 seconds
            gettimeofday(&exitTime, NULL);
            for(i=0; i<currNumRobots; i++) {
                errorPercentage[i] = numOfErrors[i]/numOfPackets*100.0;
                numOfErrors[i] = 0;
            }
            numOfPackets = 0;
        }
    }

}
#endif



