#include "heifu_diagnostic/Heifu_diagnostic.hpp"

using namespace HD;

Heifu_diagnostic::Heifu_diagnostic():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subDiagnostic   = n.subscribe("/diagnostics", 10, &Heifu_diagnostic::cbCheckGpsFix, this);
    
    // Publishers
    pubGpsFixState  = n.advertise < std_msgs::Int8 > (ns + "/g/f/m", 10);
    
    // Variables

    // Parameters
    n.param<double>("paramNodeRate", paramNodeRate,  5.0);
}


void Heifu_diagnostic::run(){
    
    ros::Rate go(paramNodeRate);
    while (ros::ok()){
        ros::spinOnce();
        go.sleep();
    }
}

void Heifu_diagnostic::cbCheckGpsFix(const diagnostic_msgs::DiagnosticArrayConstPtr& msg){

    diagnostic_msgs::DiagnosticArray fixMsg = *msg;
    int size = sizeof(fixMsg.status);
    for(int i=0; i<size; i++){
        if (fixMsg.status[i].message == str1)
        {
            int size2 = sizeof(fixMsg.status[i].values);
            for (int j = 0; j <= size2; j++)
            {
                if (fixMsg.status[i].values[j].key == str2)
                {
                    int fixType = stoi(fixMsg.status[i].values[j].value);
                    switch (fixType)
                    {
                        case 0:
                            gpsFixMode.data = 0;
                            ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: No GPS");
                            break;
                        case 1:
                            gpsFixMode.data = 1;
                            ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: No FIX");
                            break;
                        case 2:
                            gpsFixMode.data = 2;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: 2D FIX");
                            break;
                        case 3:
                            gpsFixMode.data = 3;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: 3D FIX");
                            break;
                        case 4:
                            gpsFixMode.data = 4;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: DGPS Fix");
                            break;
                        case 5:
                            gpsFixMode.data = 5;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: RTK FLOAT");
                            break;
                        case 6:
                            gpsFixMode.data = 6;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: RTK FIX");
                            break;
                        case 7:
                            gpsFixMode.data = 7;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: Static fixed");
                            break;
                        case 8:
                            gpsFixMode.data = 8;
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: No PPP");
                            break;
                        default:
                            //ros_utils::ROS_PRINT(ros_utils::BLUE_BOLD, "FixType: NO INFO");
                            break;
                    }
                    pubGpsFixState.publish(gpsFixMode);
                    break;
                }
            }
             
        }
        
    }
}