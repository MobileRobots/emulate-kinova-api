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

/* Internal state */

static bool Init = false;

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
  AngularPosition angularForce;
  CartesianPosition cartesianForce;
  AngularPosition angularCurrent;
  AngularPosition lastAngularCmd;
  CartesianPosition lastCartesianCmd;
  EmulatedDevice(const char *sn, int type) {
    strncpy(deviceInfo.SerialNumber, sn, SERIAL_LENGTH);
    deviceInfo.DeviceID = MaxDeviceID++;

    memset(&clientConfig, 0, sizeof(clientConfig));
    strncpy(clientConfig.Serial, sn, STRING_LENGTH);
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
    fprintf(LOGFP, "KinovaAPI emu: %s %s (%s active)\n", __func__, msg, ActiveDevice->deviceInfo.SerialNumber); \
  else \
    fprintf(LOGFP, "KinovaAPI emu: %s %s\n", __func__, msg); \
}

#define LOG_INT(msg, i) {\
  fprintf(LOGFP, "KinovaAPI emu: %s %s %d (%s active)\n", __func__, msg, i, ActiveDevice->deviceInfo.SerialNumber); \
}

#define LOG_POS(msg, pos) {\
  fprintf(LOGFP, "KinovaAPI emu: %s %s (px=%0.2f, py=%0.2f, pz=%0.2f, tx=%0.2f, ty=%0.2f, tz=%0.2f) (%s active)\n", __func__, msg, pos.X, pos.Y, pos.Z, pos.ThetaX, pos.ThetaY, pos.ThetaZ, ActiveDevice->deviceInfo.SerialNumber);\
}

#define LOG_ANGLES(msg, a) {\
  fprintf(LOGFP, "KinovaAPI emu: %s %s (%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f) (%s active)\n", __func__, msg, a.Actuator1,  a.Actuator2,  a.Actuator3, a.Actuator4,  a.Actuator5,  a.Actuator6, ActiveDevice->deviceInfo.SerialNumber);\
}

/* External API */

#define EMULATE_KINOVA_NOT_IMPLEMENTED  2002

KINOVAAPIUSBCOMMANDLAYER_API int InitAPI() {
  Devices.push_back(EmulatedDevice("EM001", 3));
  Devices.push_back(EmulatedDevice("EM002", 3));
  ActiveDevice = Devices.begin();
  Init = true;
  LOG("ok");
	return NO_ERROR_KINOVA;
}

// was a std::vector in previous API
KINOVAAPIUSBCOMMANDLAYER_API int GetAPIVersionVector(std::vector<int> &Response)
{
  Response.clear();
  Response.push_back(0);
  Response.push_back(0);
  Response.push_back(0);
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAPIVersion(int Response[API_VERSION_COUNT])
{
  // todo provide accurate API number
  memset(&Response, 0, API_VERSION_COUNT * sizeof(int));
  return NO_ERROR_KINOVA;
}


// was a std::vector in previous api
KINOVAAPIUSBCOMMANDLAYER_API int GetDevicesVector(std::vector<KinovaDevice> &_Devices, int &result) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  std::vector<KinovaDevice> dv;
  for(std::vector<EmulatedDevice>::const_iterator i = Devices.begin(); i != Devices.end(); ++i)
  {
    dv.push_back(i->deviceInfo);
  }
  _Devices = dv;
  result = _Devices.size();
  LOG("ok");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetDeviceCount(int &result) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  result = Devices.size();
  return NO_ERROR_KINOVA;
}


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


KINOVAAPIUSBCOMMANDLAYER_API int CloseAPI(void) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Init = false;
  Devices.clear();
  ActiveDevice = Devices.begin();
  LOG("ok");
  return NO_ERROR_KINOVA;
}

// was a std::vector in previous api
KINOVAAPIUSBCOMMANDLAYER_API int GetCodeVersionVector(std::vector<int> &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetCodeVersion(int Response[CODE_VERSION_COUNT]) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  memset(&Response, 0, CODE_VERSION_COUNT * sizeof(int));
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianPosition(CartesianPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->cartesianPos;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularPosition(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularPos;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularVelocity(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularVel;
  LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianForce(CartesianPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->cartesianForce;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularForce(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularForce;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCurrent(AngularPosition &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  Response = ActiveDevice->angularCurrent;
  LOG("");
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


KINOVAAPIUSBCOMMANDLAYER_API int SendAdvanceTrajectory(TrajectoryPoint trajectory) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  // TODO log limits and other values in trajectory struct
  switch(trajectory.Position.Type)
  {
    case CARTESIAN_POSITION:
      // todo compute joint angles
      LOG_POS("cartesian pos", trajectory.Position.CartesianPosition);
      ActiveDevice->cartesianPos.Coordinates = trajectory.Position.CartesianPosition;
      ActiveDevice->lastCartesianCmd = ActiveDevice->cartesianPos;
      return NO_ERROR_KINOVA;
    case ANGULAR_POSITION:
      // todo compute cartesian position
      LOG_ANGLES("angular position", trajectory.Position.Actuators);
      ActiveDevice->angularPos.Actuators = trajectory.Position.Actuators;
      ActiveDevice->lastAngularCmd = ActiveDevice->angularPos;
      return NO_ERROR_KINOVA;
    // todo the others
    default:
      LOG_INT("not yet implemented for position type", trajectory.Position.Type);
  }
  return EMULATE_KINOVA_NOT_IMPLEMENTED;
}


KINOVAAPIUSBCOMMANDLAYER_API int SendBasicTrajectory(TrajectoryPoint trajectory) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
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


KINOVAAPIUSBCOMMANDLAYER_API int GetPositionCurrentActuators(std::vector<float> &Response) {
  if(!Init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}


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


