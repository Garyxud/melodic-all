#include "radar_omnipresense/SendAPICommand.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(radar_omnipresense, testIsConnected)
{
  try 
  {
//    ASSERT_FALSE(radar_omnipresense->IsConnected());
  } 
  catch(...) 
  {
    ADD_FAILURE() << "Uncaught exception";
  } 
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "tester");
//  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
