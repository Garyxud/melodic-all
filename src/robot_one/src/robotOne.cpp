#include "robotOne.h"
#include "TcpUdpSocket.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#define PORT 5046
#define VERSION 1.0
#define TIME_WAIT "Time.Wait"
#define MAX_DATA 921600

using namespace std;

TcpUdpSocket *s = NULL;
char dataIn[MAX_DATA];
char dataOut[MAX_DATA];
char dataContainer[MAX_DATA];
volatile static int retSize = 0;
volatile static float retValue = 0;
char *temp;

bool Handler(char *data)
{
	int c = (unsigned char)data[0];
	if (c == 0 || c == 1) // Get and Set sending
		return false;
	int status = (unsigned char)data[1];
	temp = data + 2;
	retValue = ((float*)temp)[0];
	if (c == 3) // Raw data
	{
		temp = data + 6;
		retSize = ((int*)temp)[0];
		temp = data + 10;
		for (int i = 0; i < retSize; i++)
			dataContainer[i] = temp[i];
	}
	return true;
}

extern_c int connectRobotOne(const char *address)
{
	if (s != NULL)
		return 1;
	s = new TcpUdpSocket(PORT, address, FALSE);
	bool connected = s->isConnected();
	return connected ? 1 : 0;
}

extern_c int disconnectRobotOne()
{
	if (s == NULL)
		return 0;
	delete s;
	s = NULL;
	return 1;
}

extern_c float versionRobotOne()
{
	return VERSION;
}

extern_c float waitRobotOne()
{
	if (s == NULL || !s->isConnected())
		return 0.0f;
	int len = strlen(TIME_WAIT);
	dataOut[0] = (char)0;
	*((int*)(dataOut + 1)) = len;
	for (int i = 0; i < len; i++)
		dataOut[5 + i] = TIME_WAIT[i];
	s->sendTo(dataOut, len + 5 + 4);
	bool done = false;
	while (!done)
	{
		s->receive(dataIn, MAX_DATA);
		done = Handler(dataIn);
	}
	return retValue;
}

extern_c char* getData()
{
	return dataContainer;
}

extern_c float* getDataFloat()
{
	return (float*)dataContainer;
}

extern_c float get(const char *name)
{
	if (s == NULL || !s->isConnected())
		return 0.0f;
	int len = strlen(name);
	dataOut[0] = (char)0;
	*((int*)(dataOut + 1)) = len;
	for (int i = 0; i < len; i++)
		dataOut[5 + i] = name[i];
	s->sendTo(dataOut, len + 5 + 4);
	bool done = false;
	while (!done)
	{
		s->receive(dataIn, MAX_DATA);
		done = Handler(dataIn);
	}
	return retValue;
}

extern_c float set(const char *name, float value)
{
	if (s == NULL || !s->isConnected())
		return 0.0f;
	int len = strlen(name);
	dataOut[0] = (char)1;
	*((int*)(dataOut + 1)) = len;
	for (int i = 0; i < len; i++)
		dataOut[5 + i] = name[i];
	temp = (char*)&value;
	for (int i = 0; i < 4; i++)
		dataOut[5 + len + i] = temp[i];
	s->sendTo(dataOut, len + 5 + 4);
	bool done = false;
	while (!done)
	{
		s->receive(dataIn, MAX_DATA);
		done = Handler(dataIn);
	}
	return retValue;
}

extern_c float setData(const char *name, int size, char *data)
{
	if (s == NULL || !s->isConnected())
		return 0.0f;
	int len = strlen(name);
	dataOut[0] = (char)1;
	*((int*)(dataOut + 1)) = len;
	for (int i = 0; i < len; i++)
		dataOut[5 + i] = name[i];
	float tmpF = (float)size;
	temp = (char*)&tmpF;
	for (int i = 0; i < 4; i++)
		dataOut[5 + len + i] = temp[i];
	int index = len + 5 + 4;
	for (int i = 0; i < size; i++)
	{
		dataOut[index] = data[i];
		index++;
		if(index >= MAX_DATA)
			break;
	}
	s->sendTo(dataOut, index);
	bool done = false;
	while (!done)
	{
		s->receive(dataIn, MAX_DATA);
		done = Handler(dataIn);
	}
	return retValue;
}

extern_c float setDataFloat(const char *name, int size, float *data)
{
	if (s == NULL || !s->isConnected())
		return 0.0f;
	int len = strlen(name);
	dataOut[0] = (char)1;
	*((int*)(dataOut + 1)) = len;
	for (int i = 0; i < len; i++)
		dataOut[5 + i] = name[i];
	float tmpF = (float)size;
	temp = (char*)&tmpF;
	for (int i = 0; i < 4; i++)
		dataOut[5 + len + i] = temp[i];
	int index = len + 5 + 4;
	char *tmpFV = (char*)data;
	for (int i = 0; i < size*4; i++)
	{
		dataOut[index] = tmpFV[i];
		index++;
		if(index >= MAX_DATA)
			break;
	}
	s->sendTo(dataOut, index);
	bool done = false;
	while (!done)
	{
		s->receive(dataIn, MAX_DATA);
		done = Handler(dataIn);
	}
	return retValue;
}

// Implementations
extern_c void initLidar(LidarData *lidarData, int size)
{
	if(lidarData->readings != NULL)
		delete[] lidarData->readings;
	if(lidarData->angles != NULL)
		delete[] lidarData->angles;
	if(lidarData->x != NULL)
		delete[] lidarData->x;
	if(lidarData->y != NULL)
		delete[] lidarData->y;
	lidarData->size = size;
	lidarData->readings = new float[size];
	lidarData->angles = new float[size];
	lidarData->x = new float[size];
	lidarData->y = new float[size];
}

extern_c int readLidar(LidarData *lidarData)
{
	int szI = (int)get("Lidar.Read");
	if(lidarData->size != szI)
		initLidar(lidarData, szI);
	float *readings = getDataFloat();
	float sz = szI;
	float inc = M_PI / sz;
	float start = -(0.5f * sz)*inc;
	float end = -start;
	float inc_ = M_PI_2;
	for(int i = 0; i < sz; i++)
	{
		lidarData->readings[i] = readings[i];
		lidarData->angles[i] = start + i*inc;
		if(lidarData->readings[i] > 0)
		{
			lidarData->x[i] = -lidarData->readings[i]*cos(inc_ + lidarData->angles[i]);
			lidarData->y[i] =  lidarData->readings[i]*sin(inc_ + lidarData->angles[i]);
		}
		else
		{
			lidarData->x[i] = lidarData->y[i] = 0;
		}
	}
	return szI;
}

extern_c void initCamera(CameraData *cameraData)
{
	if(cameraData->data != NULL)
		delete[] cameraData->data;
	cameraData->width = 320;
	cameraData->height = 240;
	cameraData->channels = 3;
	cameraData->size = cameraData->width * cameraData->height * cameraData->channels;
	cameraData->data = new char[cameraData->size];
 }


extern_c int captureCamera(CameraData *cameraData)
{
	int sz = (int)get("Camera.Capture");
	char *data = getData();
	memcpy(cameraData->data, data, cameraData->size);
	return sz;
}

extern_c void getPose(Value3 *pose)
{
	pose->values[0] = get("Pose.X");
	pose->values[1] = get("Pose.Y");
	pose->values[2] = get("Pose.Theta");
}

extern_c void setPose(Value3 *pose)
{
	set("Pose.X", pose->values[0]);
	set("Pose.Y", pose->values[1]);
	set("Pose.Theta", pose->values[2]);
}

extern_c void getOdometry(Value3 *pose)
{
	pose->values[0] = get("Odometry.X");
	pose->values[1] = get("Odometry.Y");
	pose->values[2] = get("Odometry.Theta");
}

extern_c void setOdometry(Value3 *pose)
{
	set("Odometry.X", pose->values[0]);
	set("Odometry.Y", pose->values[1]);
	set("Odometry.Theta", pose->values[2]);
}

extern_c void getOdometryStd(Value2 *odometryStd)
{
	odometryStd->values[0] = get("Odometry.Std.Linear");
	odometryStd->values[1] = get("Odometry.Std.Angular");	
}

extern_c void setOdometryStd(Value2 *odometryStd)
{
	set("Odometry.Std.Linear", odometryStd->values[0]);
	set("Odometry.Std.Angular", odometryStd->values[1]);
}

extern_c void getGPS(Value3 *gps)
{
	gps->values[0] = get("GPS.X");
	gps->values[1] = get("GPS.Y");
	gps->values[2] = get("GPS.Theta");
}

extern_c void getGPSStd(Value3 *gpsStd)
{
	gpsStd->values[0] = get("GPS.Std.X");
	gpsStd->values[1] = get("GPS.Std.Y");	
	gpsStd->values[2] = get("GPS.Std.Theta");	
}

extern_c void setGPSStd(Value3 *gpsStd)
{
	set("GPS.Std.X", gpsStd->values[0]);
	set("GPS.Std.Y", gpsStd->values[1]);
	set("GPS.Std.Theta", gpsStd->values[2]);
}

extern_c void getVelocity(Value2 *velocity)
{
	velocity->values[0] = get("Velocity.Linear");
	velocity->values[1] = get("Velocity.Angular");		
}

extern_c void setVelocity(Value2 *velocity)
{
	set("Velocity.Linear", velocity->values[0]);
	set("Velocity.Angular", velocity->values[1]);
}

extern_c bool getLowLevelControl()
{
	return get("Controller.LowLevel") > 0;
}

extern_c void setLowLevelControl(bool enabled)
{
	set("Controller.LowLevel", enabled ? 1.0f : 0.0f);
}

extern_c bool getTrace()
{
	return get("Trace") > 0;
}

extern_c void setTrace(bool enabled)
{
	set("Trace", enabled ? 1.0f : 0.0f);
}

extern_c void getWheels(Value2 *wheels)
{
	wheels->values[0] = get("Velocity.Angular.Left");
	wheels->values[1] = get("Velocity.Angular.Right");
}

extern_c void setWheels(Value2 *wheels)
{
	set("Velocity.Angular.Left", wheels->values[0]);
	set("Velocity.Angular.Right", wheels->values[1]);
}

extern_c bool getManualController()
{
	return get("Controller.Manual") > 0;
}

extern_c void setManualController(bool enabled)
{
	set("Controller.Manual", enabled ? 1.0f : 0.0f);
}