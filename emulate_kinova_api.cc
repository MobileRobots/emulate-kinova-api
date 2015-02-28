#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <stdio.h>
#include <string.h>
#include <vector>


/* Internal state */

static bool init = false;

class EmulatedDevice {
public:
  static int maxDeviceId;
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
  EmulatedDevice(const char *sn, int type) {
    strncpy(deviceInfo.SerialNumber, sn, SERIAL_LENGTH);
    deviceInfo.DeviceID = maxDeviceId++;

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
  }
};

int EmulatedDevice::maxDeviceId = 0;
  

static std::vector<EmulatedDevice> devices;

static std::vector<EmulatedDevice>::iterator activeDevice;

static FILE *logfp = stderr;

#define LOG(msg) {\
  if(init) \
    fprintf(logfp, "KinovaAPI emu: %s %s (%s active)\n", __func__, msg, activeDevice->deviceInfo.SerialNumber); \
  else \
    fprintf(logfp, "KinovaAPI emu: %s %s\n", __func__, msg); \
}

/* External API */

KINOVAAPIUSBCOMMANDLAYER_API int InitAPI() {
  devices.push_back(EmulatedDevice("EM001", 3));
  devices.push_back(EmulatedDevice("EM002", 3));
  activeDevice = devices.begin();
  init = true;
  LOG("ok");
	return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAPIVersion(std::vector<int> &Response)
{
  Response.clear();
  Response.push_back(0);
  Response.push_back(0);
  Response.push_back(0);
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetDevices(std::vector<KinovaDevice> &_devices, int &result) {
  if(!init) return ERROR_NOT_INITIALIZED;
  std::vector<KinovaDevice> dv;
  for(std::vector<EmulatedDevice>::const_iterator i = devices.begin(); i != devices.end(); ++i)
  {
    dv.push_back(i->deviceInfo);
  }
  _devices = dv;
  result = NO_ERROR_KINOVA;
  LOG("ok");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetActiveDevice(KinovaDevice device) {
  if(!init) return ERROR_NOT_INITIALIZED;
  for(std::vector<EmulatedDevice>::iterator i = devices.begin(); i != devices.end(); ++i) {
  	if(strcmp(i->deviceInfo.SerialNumber, device.SerialNumber) == 0) {
      activeDevice = i;
      LOG("found requested device");
	    return NO_ERROR_KINOVA;
	  }
  }	
  LOG("error: requested device not found");
  return ERROR_NO_DEVICE_FOUND;
}


KINOVAAPIUSBCOMMANDLAYER_API int CloseAPI(void) {
  if(!init) return ERROR_NOT_INITIALIZED;
  init = false;
  devices.clear();
  activeDevice = devices.begin();
  LOG("ok");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetCodeVersion(std::vector<int> &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianPosition(CartesianPosition &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  Response = activeDevice->cartesianPos;
  //LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularPosition(AngularPosition &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  Response = activeDevice->angularPos;
  //LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularVelocity(AngularPosition &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  Response = activeDevice->angularVel;
  //LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetCartesianForce(CartesianPosition &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  Response = activeDevice->cartesianForce;
  //LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularForce(AngularPosition &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  Response = activeDevice->angularForce;
  //LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetAngularCurrent(AngularPosition &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  Response = activeDevice->angularCurrent;
  //LOG("");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetActualTrajectoryInfo(TrajectoryPoint &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetGlobalTrajectoryInfo(TrajectoryFIFO &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetSensorsInfo(SensorsInfo &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}



KINOVAAPIUSBCOMMANDLAYER_API int SetAngularControl() {
  if(!init) return ERROR_NOT_INITIALIZED;
  activeDevice->status.ControlActiveModule = CONTROL_MODULE_ANGULAR_POSITION;
  LOG("set control mode to ANGULAR_POSITION");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianControl() {
  if(!init) return ERROR_NOT_INITIALIZED;
  activeDevice->status.ControlActiveModule = CONTROL_MODULE_CARTESIAN_POSITION;
  LOG("set control mode to CARTESIAN_POSITION");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StartControlAPI() {
  if(!init) return ERROR_NOT_INITIALIZED;
  activeDevice->status.ControlEnableStatus = 0;
  LOG("set control ON");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StopControlAPI() {
  if(!init) return ERROR_NOT_INITIALIZED;
  activeDevice->status.ControlEnableStatus = 1;
  LOG("set control OFF");
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SendAdvanceTrajectory(TrajectoryPoint trajectory) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SendBasicTrajectory(TrajectoryPoint trajectory) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetClientConfigurations(ClientConfigurations &config) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetClientConfigurations(ClientConfigurations config) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int EraseAllTrajectories() {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetPositionCurrentActuators(std::vector<float> &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetActuatorPID(unsigned int address, float P, float I, float D) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetControlType(int &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StartForceControl() {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int StopForceControl() {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}



KINOVAAPIUSBCOMMANDLAYER_API int GetGripperStatus(Gripper &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetQuickStatus(QuickStatus &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int GetGeneralInformations(GeneralInformations &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianForceMinMax(CartesianInfo min, CartesianInfo max) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int SetCartesianInertiaDamping(CartesianInfo inertia, CartesianInfo damping) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int MoveHome() {
  if(!init) return ERROR_NOT_INITIALIZED;
  LOG("");
  return NO_ERROR_KINOVA;
}

KINOVAAPIUSBCOMMANDLAYER_API int GetActuatorAcceleration(AngularAcceleration &Response) {
  if(!init) return ERROR_NOT_INITIALIZED;
  return NO_ERROR_KINOVA;
}


KINOVAAPIUSBCOMMANDLAYER_API int InitFingers() {
  return NO_ERROR_KINOVA;
}


