#include "leuze_phidget_driver.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "leuze_phidgets_driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    sleep(3); //Wait for phidgets_ik_node to come up and spawn its topics
    LeuzePhidgetDriver *driver = new LeuzePhidgetDriver(&n);

    ros::spin();
    return 0;
}
