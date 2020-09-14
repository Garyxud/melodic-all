#ifndef ROBOT_ONE_H_
#define ROBOT_ONE_H_

#ifdef WIN32
#define extern_c extern "C" __declspec(dllexport)
#else
#define extern_c extern "C" 
#endif

typedef struct
{
    float values[3];
}Value3;

typedef struct
{
    float values[2];
}Value2;

typedef struct
{
    int width, height, channels, size;
    char *data;
}CameraData;

typedef struct
{
    int size;
    float *readings, *angles, *x, *y;
}LidarData;

extern_c int connectRobotOne(const char *address);
extern_c int disconnectRobotOne();
extern_c float versionRobotOne();
extern_c float waitRobotOne();
extern_c float get(const char *name);
extern_c float set(const char *name, float value);
extern_c char* getData();
extern_c float* getDataFloat();
extern_c float setData(const char *name, int size, char *data);
extern_c float setDataFloat(const char *name, int size, float *data);

extern_c void initLidar(LidarData *lidarData, int size);
extern_c int readLidar(LidarData *lidarData);
extern_c void initCamera(CameraData *cameraData);
extern_c int captureCamera(CameraData *cameraData);
extern_c void getPose(Value3 *pose);
extern_c void setPose(Value3 *pose);
extern_c void getOdometry(Value3 *pose);
extern_c void setOdometry(Value3 *pose);
extern_c void getOdometryStd(Value2 *odometryStd);
extern_c void setOdometryStd(Value2 *odometryStd);
extern_c void getGPS(Value3 *gps);
extern_c void getGPSStd(Value3 *gpsStd);
extern_c void setGPSStd(Value3 *gpsStd);
extern_c void getVelocity(Value2 *velocity);
extern_c void setVelocity(Value2 *velocity);
extern_c bool getLowLevelControl();
extern_c void setLowLevelControl(bool enabled);
extern_c bool getTrace();
extern_c void setTrace(bool enabled);
extern_c void getWheels(Value2 *wheels);
extern_c void setWheels(Value2 *wheels);
extern_c bool getManualController();
extern_c void setManualController(bool enabled);

#endif