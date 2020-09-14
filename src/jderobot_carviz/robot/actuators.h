

#include <QMutex>

#include </opt/jderobot/include/jderobot/types/cmdvel.h>

#include "../comm/motorsClient.hpp"

class Actuators
{
public:
    Actuators(Comm::Communicator* jdrc);
    
    float getMotorV();
    float getMotorW();
    float getMotorL();

    //SETS
    void setMotorV(float motorV);
    void setMotorW(float motorW);
    void setMotorL(float motorL);
    void setMotorSTOP();

private:

    QMutex mutex;

    Comm::Communicator* jdrc;

    // ICE INTERFACES
    Comm::MotorsClient* motorsClient;

};
