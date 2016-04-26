/*
 *  Copyright 2015, Adept MoileRobots LLC
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ADEPT MOBILEROBOTS LLC or ADEPT TECHNOLOGY INC. 
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <stdio.h>
#include <string.h>
#include <vector>

#ifndef MAX_KINOVA_DEVICE
#define MAX_KINOVA_DEVICE 20
#endif

#ifndef CODE_VERSION_COUNT
#define CODE_VERSION_COUNT 3
#endif

#ifndef API_VERSION_COUNT
#define API_VERSION_COUNT 3
#endif

#ifdef API_HEADERS_VER

#if (API_HEADERS_VER < 50104)
#warning "Using older function definitions with vector args"
#define USE_VECTOR_ARGS 1
#else
#define HAVE_GET_COMMAND_VELOCITY 1
#endif

#endif

//#define LOG_GET(msg) LOG(msg)
#define LOG_GET(msg) {}

/* Internal state */

static bool Init = false;

static AngularPosition LeftHomePosition;
static AngularPosition RightHomePosition;

void _initHomePositions()
{
  memset(&LeftHomePosition, 0, sizeof(RightHomePosition));
  memset(&RightHomePosition, 0, sizeof(LeftHomePosition));
  // XXX TODO set correct home position for left/right Jaco2 arms
  LeftHomePosition.Actuators = {44, 140, 295, 175, 240, 285};
  RightHomePosition.Actuators = {320, 220, 65, 175, 240, 285};
  LeftHomePosition.Fingers = RightHomePosition.Fingers = {0, 0, 0};
}

class EmulatedDevice {
public:
  static int MaxDeviceID;
  KinovaDevice deviceInfo;
  QuickStatus status;
  ClientConfigurations clientConfig;
  GeneralInformations genInfo;
  AngularPosition angularPos;
  CartesianPosition cartesianPos;
  AngularPosition angularVel;
  CartesianPosition cartesianVel;
  AngularPosition angularForce;
  CartesianPosition cartesianForce;
  AngularPosition angularCurrent;
  AngularPosition lastAngularCmd;
  CartesianPosition lastCartesianCmd;
  EmulatedDevice(const char *sn, int type, ArmLaterality lat = LEFTHAND) {
    strncpy(deviceInfo.SerialNumber, sn, SERIAL_LENGTH);
    deviceInfo.DeviceID = MaxDeviceID++;

    memset(&clientConfig, 0, sizeof(clientConfig));
    strncpy(clientConfig.Serial, sn, STRING_LENGTH);
    clientConfig.Laterality = lat;
    switch(type) {
      case 0:
        strcpy(clientConfig.Model, "Emulated Jaco");
        break;
      case 1:
        strcpy(clientConfig.Model, "Emulated Mico");
        break;
      case 3:
        strcpy(clientConfig.Model, "Emulated Jaco2");
        break;
      default:
        strcpy(clientConfig.Model, "Emulated");
        break;
    }    

    memset(&status, 0, sizeof(status));
    status.ControlActiveModule = CONTROL_MODULE_NONE;
    status.ForceControlStatus = 1; // off
    status.RobotType = type;

    memset(&genInfo, 0, sizeof(genInfo));
    genInfo.ConnectedActuatorCount = 6;
    
    memset(&angularPos, 0, sizeof(angularPos));
    memset(&cartesianPos, 0, sizeof(cartesianPos));
    memset(&angularVel, 0, sizeof(angularVel));
    memset(&cartesianVel, 0, sizeof(cartesianVel));
    memset(&angularForce, 0, sizeof(angularForce));
    memset(&cartesianForce, 0, sizeof(cartesianForce));
    memset(&angularCurrent, 0, sizeof(angularCurrent));
    memset(&lastAngularCmd, 0, sizeof(lastAngularCmd));
    memset(&lastCartesianCmd, 0, sizeof(lastCartesianCmd));
  }
};

int EmulatedDevice::MaxDeviceID = 0;
  

static std::vector<EmulatedDevice> Devices;

static std::vector<EmulatedDevice>::iterator ActiveDevice;

static FILE *LOGFP = stderr;

#define LOG(msg) {\
  if(Init) \
    fprintf(LOGFP, "[emulate_kinova_api] %s %s (%s active)\n", __func__, (msg), ActiveDevice->deviceInfo.SerialNumber); \
  else \
    fprintf(LOGFP, "[emulate_kinova_api] %s %s\n", __func__, (msg)); \
}

#define LOG_INT(msg, i) {\
  fprintf(LOGFP, "[emulate_kinova_api] %s %s %d (%s active)\n", __func__, (msg), (i), ActiveDevice->deviceInfo.SerialNumber); \
}

#define LOG_POS(msg, pos) {\
  fprintf(LOGFP, "[emulate_kinova_api] %s %s (px=%0.2f, py=%0.2f, pz=%0.2f, tx=%0.2f, ty=%0.2f, tz=%0.2f) (%s active)\n", __func__, (msg), (pos).X, (pos).Y, (pos).Z, (pos).ThetaX, (pos).ThetaY, (pos).ThetaZ, ActiveDevice->deviceInfo.SerialNumber);\
}

#define LOG_ANGLES(msg, a) {\
  fprintf(LOGFP, "[emulate_kinova_api] %s %s (%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f) (%s active)\n", __func__, (msg), (a).Actuator1,  (a).Actuator2,  (a).Actuator3, (a).Actuator4,  (a).Actuator5,  (a).Actuator6, ActiveDevice->deviceInfo.SerialNumber);\
}

#define LOG_LIMITS(msg, l) {\
  fprintf(LOGFP, "[emulate_kinova_api] %s %s (speed1=%0.2f, speed2=%0.2f, speed3=%0.2f, force1=%0.2f, force2=%0.2f, force3=%0.2f, acc1=%0.2f, acc2=%0.2f, acc3=%0.2f) (%s active)\n", __func__, (msg), (l).speedParameter1, (l).speedParameter2, (l).speedParameter3, (l).forceParameter1, (l).forceParameter2, (l).forceParameter3, (l).accelerationParameter1, (l).accelerationParameter2, (l).accelerationParameter3, ActiveDevice->deviceInfo.SerialNumber);\
}

#define LOG_FINGERS(msg, f) {\
  fprintf(LOGFP, "[emulate_kinova_api] %s %s (%0.2f, %0.2f, %0.2f)\n", __func__, (msg), (f).Finger1, (f).Finger2, (f).Finger3); \
}

/* External API */

#define EMULATE_KINOVA_NOT_IMPLEMENTED  2002



KINOVAAPIUSBCOMMANDLAYER_API int InitAPI() {
  Devices.push_back(EmulatedDevice("EM001", 3, LEFTHAND));
  Devices.push_back(EmulatedDevice("EM002", 3, RIGHTHAND));
  ActiveDevice = Devices.begin();
  _initHomePositions();
  Init = true;
  LOG("ok");
	return NO_ERROR_KINOVA;
}

#ifdef USE_VECTOR_ARGS

KINOVAAPIUSBCOMMANDLAYER_API int GetAPIVersion(std::vector<int> &Response)
{
puts("emu getapi");
  Response.clear();
  Response.push_back(0);
  Response.push_back(0);
  Response.push_back(0);
  return NO_ERROR_KINOVA;
}

#else

KINOVAAPIUSBCOMMANDLAYER_API int GetAPIVersion(int Response[API_VERSION_COUNT])
{
  // todo provide accurate API number
  memset(&Response, 0, API_VERSION_COUNT * sizeof(int));
  return NO_ERROR_KINOVA;
}

#endif

KINOVAAPIUSBCOMMANDLAYER_API int GetDeviceCount(int &result) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  result = Devices.size();
  return NO_ERROR_KINOVA;
}

#ifdef USE_VECTOR_ARGS
KINOVAAPIUSBCOMMANDLAYER_API int GetDevices(std::vector<KinovaDevice> &devlist, int &result) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  std::vector<KinovaDevice> dv;
  for(std::vector<EmulatedDevice>::const_iterator i = Devices.begin(); i != Devices.end(); ++i)
  {
    dv.push_back(i->deviceInfo);
  }
  devlist = dv;
  result = NO_ERROR_KINOVA;
  LOG("ok");
  return devlist.size();
}
#else


KINOVAAPIUSBCOMMANDLAYER_API int GetDevices(KinovaDevice devlist[MAX_KINOVA_DEVICE], int &result) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  int d = 0;
  //memset(&devlist, 0, MAX_KINOVA_DEVICE * sizeof(KinovaDevice));
  for(std::vector<EmulatedDevice>::const_iterator i = Devices.begin(); i != Devices.end() && d < MAX_KINOVA_DEVICE; ++i)
  {
    devlist[d++] = i->deviceInfo;
  }
  result = NO_ERROR_KINOVA;
  LOG("ok");
  return d;
}
#endif


KINOVAAPIUSBCOMMANDLAYER_API int SetActiveDevice(KinovaDevice device) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  for(std::vector<EmulatedDevice>::iterator i = Devices.begin(); i != Devices.end(); ++i) {
  	if(strcmp(i->deviceInfo.SerialNumber, device.SerialNumber) == 0) {
      ActiveDevice = i;
      LOG("found requested device");
	    return NO_ERROR_KINOVA;
	  }
  }	
  LOG("error: requested device not found");
  return ERROR_NO_DEVICE_FOUND;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetActiveDevice(KinovaDevice& device) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
  

KINOVAAPIUSBCOMMANDLAYER_API int CloseAPI(void) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Init = false;
  Devices.clear();
  ActiveDevice = Devices.begin();
  LOG("ok");
  return NO_ERROR_KINOVA;
}

#ifdef USE_VECTOR_ARGS
KINOVAAPIUSBCOMMANDLAYER_API int GetCodeVersion(std::vector<int> &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}
#else
KINOVAAPIUSBCOMMANDLAYER_API int GetCodeVersion(int Response[CODE_VERSION_COUNT]) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  memset(&Response, 0, CODE_VERSION_COUNT * sizeof(int));
  return NO_ERROR_KINOVA;
}
#endif

KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianPosition(CartesianPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->cartesianPos;
  LOG_GET("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularPosition(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularPos;
  LOG_GET("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularVelocity(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularVel;
  LOG_GET("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianForce(CartesianPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->cartesianForce;
  LOG_GET("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularForce(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularForce;
  LOG_GET("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCurrent(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularCurrent;
  LOG_GET("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCommand(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->lastAngularCmd;
  LOG_ANGLES("returning", Response.Actuators);
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetActualTrajectoryInfo(TrajectoryPoint &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetGlobalTrajectoryInfo(TrajectoryFIFO &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetSensorsInfo(SensorsInfo &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}



KINOVAAPIUSBCOMMANDLAYER_API int SetAngularControl() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  ActiveDevice->status.ControlActiveModule = CONTROL_MODULE_ANGULAR_POSITION;
  LOG("set control mode to ANGULAR_POSITION");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianControl() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  ActiveDevice->status.ControlActiveModule = CONTROL_MODULE_CARTESIAN_POSITION;
  LOG("set control mode to CARTESIAN_POSITION");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StartControlAPI() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  ActiveDevice->status.ControlEnableStatus = 0;
  LOG("set control ON");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StopControlAPI() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  ActiveDevice->status.ControlEnableStatus = 1;
  LOG("set control OFF");
  return NO_ERROR_KINOVA;
}

void _setCartPos(const CartesianInfo &coords)
{
      // todo compute joint angles
    ActiveDevice->cartesianPos.Coordinates = coords;
    ActiveDevice->lastCartesianCmd = ActiveDevice->cartesianPos;
}

void _setAngPos(const AngularInfo &angles)
{
      // todo compute cartesian position of end effector.
      ActiveDevice->angularPos.Actuators = angles;
      ActiveDevice->lastAngularCmd = ActiveDevice->angularPos;
}

void _setFingers(const FingersPosition &fingers)
{
  ActiveDevice->angularPos.Fingers = fingers;
  ActiveDevice->cartesianPos.Fingers = fingers;
}

void _setCartVel(const CartesianInfo &coords)
{
      // todo compute joint angles
    ActiveDevice->cartesianVel.Coordinates = coords;
    ActiveDevice->lastCartesianCmd = ActiveDevice->cartesianVel;
}

void _setAngVel(const AngularInfo &angles)
{
      // todo compute cartesian position
      ActiveDevice->angularVel.Actuators = angles;
      ActiveDevice->lastAngularCmd = ActiveDevice->angularVel;
}

KINOVAAPIUSBCOMMANDLAYER_API int SendAdvanceTrajectory(TrajectoryPoint traj) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  switch(traj.Position.Type)
  {
    case CARTESIAN_POSITION:
      LOG_POS("cartesian pos", traj.Position.CartesianPosition);
      _setCartPos(traj.Position.CartesianPosition);
      break;
    case ANGULAR_POSITION:
      LOG_ANGLES("angular position", traj.Position.Actuators);
      _setAngPos(traj.Position.Actuators);
      break;
    case CARTESIAN_VELOCITY:
      LOG_POS("cartesian vel", traj.Position.CartesianPosition);
      _setCartVel(traj.Position.CartesianPosition);
      break;
    case ANGULAR_VELOCITY:
      LOG_ANGLES("angular vel", traj.Position.Actuators);
      _setAngVel(traj.Position.Actuators);
      break;
    // todo the others
    default:
      LOG_INT("not yet implemented for position type", traj.Position.Type);
      return EMULATE_KINOVA_NOT_IMPLEMENTED;
  }
  LOG_FINGERS("fingers", traj.Position.Fingers);
  _setFingers(traj.Position.Fingers);
  if(traj.LimitationsActive == 1) {
    LOG_LIMITS("limits enabled: ", traj.Limitations);
  } else {
    LOG("limits NOT enabled");
  }
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SendBasicTrajectory(TrajectoryPoint traj) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  switch(traj.Position.Type)
  {
    case CARTESIAN_POSITION:
      LOG_POS("cartesian pos", traj.Position.CartesianPosition);
      _setCartPos(traj.Position.CartesianPosition);
      break;
    case ANGULAR_POSITION:
      LOG_ANGLES("angular position", traj.Position.Actuators);
      _setAngPos(traj.Position.Actuators);
      break;
    case CARTESIAN_VELOCITY:
      LOG_POS("cartesian vel", traj.Position.CartesianPosition);
      _setCartVel(traj.Position.CartesianPosition);
      break;
    case ANGULAR_VELOCITY:
      LOG_ANGLES("angular vel", traj.Position.Actuators);
      _setAngVel(traj.Position.Actuators);
      break;
    // todo the others
    default:
      LOG_INT("not yet implemented for position type", traj.Position.Type);
      return EMULATE_KINOVA_NOT_IMPLEMENTED;
  }
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetClientConfigurations(ClientConfigurations &config) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  config = ActiveDevice->clientConfig;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetClientConfigurations(ClientConfigurations config) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("changing stored ClientConfiguration");
  ActiveDevice->clientConfig = config;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int EraseAllTrajectories() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


#ifdef USE_VECTOR_ARGS
KINOVAAPIUSBCOMMANDLAYER_API int GetPositionCurrentActuators(std::vector<float> &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
#else
KINOVAAPIUSBCOMMANDLAYER_API int GetPositionCurrentActuators(float Response[POSITION_CURRENT_COUNT]) { 
  if(!Init) return ERROR_NOT_INITIALIZED;
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
#endif

KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorPID(unsigned int address, float P, float I, float D) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetControlType(int &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StartForceControl() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StopForceControl() {
  LOG("");
  if(!Init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}



KINOVAAPIUSBCOMMANDLAYER_API int GetGripperStatus(Gripper &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetQuickStatus(QuickStatus &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  Response = ActiveDevice->status;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetGeneralInformations(GeneralInformations &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  Response = ActiveDevice->genInfo;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianForceMinMax(CartesianInfo min, CartesianInfo max) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianInertiaDamping(CartesianInfo inertia, CartesianInfo damping) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int MoveHome() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  if(ActiveDevice->clientConfig.Laterality == LEFTHAND)
  {
    _setAngPos(LeftHomePosition.Actuators);
    _setFingers(LeftHomePosition.Fingers);
  }
  else
  {
    _setAngPos(RightHomePosition.Actuators);
    _setFingers(LeftHomePosition.Fingers);
  }
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetActuatorAcceleration(AngularAcceleration &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int InitFingers() {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetControlMapping(ControlMappingCharts command) {
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetProtectionZone(ZoneList command) {
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorPIDFilter(int a, float p, float i, float d) {
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetForcesInfo(ForcesInfo &resp) {
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int ProgramFlash(const char *filename) 
{
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int RefresDevicesList(void) // sic
{
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}



KINOVAAPIUSBCOMMANDLAYER_API int GetSingularityVector(SingularityVector &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int RestoreFactoryDefault(){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SendJoystickCommand(JoystickCommand joystickCommand){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}



KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianCommand(CartesianPosition &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCurrentMotor(AngularPosition &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}



KINOVAAPIUSBCOMMANDLAYER_API int StartCurrentLimitation(){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int StopCurrentLimitation(){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetSystemErrorCount(unsigned int &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetSystemError(unsigned int indexError, SystemError &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int ClearErrorLog(){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int EraseAllProtectionZones(){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetSerialNumber(char Command[STRING_LENGTH], char temp[STRING_LENGTH]){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetControlMapping(ControlMappingCharts &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetProtectionZone(ZoneList &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}




KINOVAAPIUSBCOMMANDLAYER_API int SetJointZero(int ActuatorAdress){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueZero(int ActuatorAdress){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueGain(int ActuatorAdress, int Gain){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorAddress(int ActuatorAdress, int newAddress){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetFrameType(int frameType){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetAngularTorqueMinMax(AngularInfo min, AngularInfo max){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetAngularInertiaDamping(AngularInfo inertia, AngularInfo damping){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetDevValue(std::vector<float> command){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int GetDevValue(std::vector<float> &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#ifdef USE_VECTOR_ARGS

KINOVAAPIUSBCOMMANDLAYER_API int SetSpasmFilterValues(std::vector<float> Response, int activationStatus)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetSpasmFilterValues(std::vector<float> &Response, int &activationStatus)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#else 

KINOVAAPIUSBCOMMANDLAYER_API int SetSpasmFilterValues(float Command[SPASM_FILTER_COUNT], int activationStatus)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetSpasmFilterValues(float Response[SPASM_FILTER_COUNT], int &activationStatus){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#endif

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularForceGravityFree(AngularPosition &Response){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}



#ifdef USE_VECTOR_ARGS

KINOVAAPIUSBCOMMANDLAYER_API int GetPeripheralInventory(std::vector<PeripheralInfo> &){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#else

KINOVAAPIUSBCOMMANDLAYER_API int GetPeripheralInventory(PeripheralInfo list[MAX_INVENTORY] ){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#endif

KINOVAAPIUSBCOMMANDLAYER_API int SetModel(char Command[STRING_LENGTH], char temp[STRING_LENGTH]){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetJoystickValue(JoystickCommand &joystickCommand){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetRobotConfiguration(int ConfigID){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#ifdef HAVE_GET_COMMAND_VELOCITY
KINOVAAPIUSBCOMMANDLAYER_API int GetCommandVelocity(float cartesianVelocity[CARTESIAN_SIZE], float angularVelocity[MAX_ACTUATORS]){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
#endif

KINOVAAPIUSBCOMMANDLAYER_API int GetEndEffectorOffset(unsigned int &status, float &x, float &y, float &z){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetEndEffectorOffset(unsigned int status, float x, float y, float z){
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int ScanForNewDevice() {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int InitCommunication() {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int CloseCommunication() {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API Packet SendPacket(Packet &out, Packet &in, int &result) {
	result = EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#if (API_HEADERS_VER >= 50200)
#warning including new 5.2.0 functions

 KINOVAAPIUSBCOMMANDLAYER_API int RunGravityZEstimationSequence(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]) 
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetGravityType(GRAVITY_TYPE)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
int SetGravityVector(float gravityVector[GRAVITY_VECTOR_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetGravityOptimalZParam(float Command[GRAVITY_PARAM_SIZE]) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetGravityManualInputParam(float Command[GRAVITY_PARAM_SIZE]) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorMaxVelocity(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetSwitchThreshold(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetPositionLimitDistance(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueControlType(TORQUECONTROL_TYPE type)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetGravityPayload(float Command[GRAVITY_PAYLOAD_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFeedCurrent(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFeedVelocity(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SendAngularTorqueCommand(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SendCartesianForceCommand(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueActuatorGain(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueActuatorDamping(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SwitchTrajectoryTorque(GENERALCONTROL_TYPE type)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueCommandMax(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueSafetyFactor(float factor)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueGainMax(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueInactivityTimeMainController(int time) 
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueDampingMax(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFeedVelocityUnderGain(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFeedCurrentVoltage(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueStaticFrictionMax(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

//Internal use only
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueErrorResend(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueInactivityTimeActuator(float command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueInactivityType(int)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueVibrationController(float activationStatus) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueRobotProtection(int protectionLevel)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueVelocityLimitFilter(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}
KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFeedFilter(float command[COMMAND_SIZE]) 
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueStaticFriction(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueErrorDeadband(float command[COMMAND_SIZE]) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueBrake(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueRateLimiter(float command[COMMAND_SIZE]) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorquePositionLimitDampingGain(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorquePositionLimitDampingMax(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorquePositionLimitRepulsGain(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorquePositionLimitRepulsMax(float command[COMMAND_SIZE]) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFilterVelocity(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFilterMeasuredTorque(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFilterError(float command[COMMAND_SIZE]) 
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int SetTorqueFilterControlEffort(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetTrajectoryTorqueMode(int&) {
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularTorqueCommand(float command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularTorqueGravityEstimation(float Command[COMMAND_SIZE])
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVADLLCOMMLAYER_API int RS485_Read(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVADLLCOMMLAYER_API int RS485_Write(RS485_Message* PackagesOut, int QtyToSend, int &QtySent)
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

KINOVADLLCOMMLAYER_API int RS485_Activate() 
{
	return EMULATE_KINOVA_NOT_IMPLEMENTED;
}

#endif
