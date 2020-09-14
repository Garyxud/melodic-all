/**
 * @file test_quaternion_operation.cpp
 * @author Masaya Kataoka ms.kataoka@gmail.com
 * @brief test code for Quaternion Operation
 * @version 0.1
 * @date 2019-04-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

//headers in Google Test
#include <gtest/gtest.h>

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include <quaternion_operation/quaternion_operation.h>

/**
 * @brief test for + operator
 * 
 */
TEST(TestSuite, testCase1)
{
    geometry_msgs::Quaternion ans,q1,q2;
    q1.x = 0;
    q1.y = 1;
    q1.z = 0;
    q1.w = 1;
    q2.x = 0;
    q2.y = 1;
    q2.z = 0;
    q2.w = 1;
    ans = q1 + q2;
    EXPECT_EQ(ans.x,0);
    EXPECT_EQ(ans.y,2);
    EXPECT_EQ(ans.z,0);
    EXPECT_EQ(ans.w,2);
}

/**
 * @brief test for * operator
 * 
 */
TEST(TestSuite, testCase2)
{
    geometry_msgs::Quaternion ans,q1,q2;
    q1.x = 0;
    q1.y = 1;
    q1.z = 0;
    q1.w = 0;
    q2.x = 0;
    q2.y = 0;
    q2.z = 0;
    q2.w = 1;
    ans = q1 * q2;
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
}

/**
 * @brief test for getRotationMatrix function
 * @sa quaternion_operation::getRotationMatrix
 */
TEST(TestSuite, testCase3)
{
    geometry_msgs::Quaternion q1;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    Eigen::MatrixXd mat = quaternion_operation::getRotationMatrix(q1);
    EXPECT_FLOAT_EQ(mat(0,0), 1.0);
    EXPECT_FLOAT_EQ(mat(0,1), 0.0);
    EXPECT_FLOAT_EQ(mat(0,2), 0.0);
    EXPECT_FLOAT_EQ(mat(1,0), 0.0);
    EXPECT_FLOAT_EQ(mat(1,1), 1.0);
    EXPECT_FLOAT_EQ(mat(1,2), 0.0);
    EXPECT_FLOAT_EQ(mat(2,0), 0.0);
    EXPECT_FLOAT_EQ(mat(2,1), 0.0);
    EXPECT_FLOAT_EQ(mat(2,2), 1.0);
}

/**
 * @brief test for getRotationMatrix function
 * @sa quaternion_operation::getRotationMatrix
 */
TEST(TestSuite, testCase4)
{
    geometry_msgs::Quaternion q1;
    q1.x = std::sqrt((double)0.5);
    q1.y = 0;
    q1.z = 0;
    q1.w = std::sqrt((double)0.5);
    Eigen::MatrixXd mat = quaternion_operation::getRotationMatrix(q1);
    EXPECT_FLOAT_EQ(mat(0,0),  1.0);
    EXPECT_FLOAT_EQ(mat(0,1),  0.0);
    EXPECT_FLOAT_EQ(mat(0,2),  0.0);
    EXPECT_FLOAT_EQ(mat(1,0),  0.0);
    //EXPECT_FLOAT_EQ(mat(1,1),  0.0);
    EXPECT_FLOAT_EQ(mat(1,2), -1.0);
    EXPECT_FLOAT_EQ(mat(2,0),  0.0);
    EXPECT_FLOAT_EQ(mat(2,1),  1.0);
    //EXPECT_FLOAT_EQ(mat(2,2),  0.0);
}

/**
 * @brief test for slerp function
 * @sa quaternion_operation::slerp
 */
TEST(TestSuite, testCase5)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 0;
    q2.y = 0;
    q2.z = 0;
    q2.w = 1;
    ans  = quaternion_operation::slerp(q1,q2,0);
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
}

/**
 * @brief test for slerp function
 * @sa quaternion_operation::slerp
 */
TEST(TestSuite, testCase6)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 1;
    q2.y = 0;
    q2.z = 0;
    q2.w = 0;
    ans  = quaternion_operation::slerp(q1,q2,0);
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
    ans  = quaternion_operation::slerp(q1,q2,1);
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
}

/**
 * @brief test for getRotation function
 * @sa quaternion_operation::getRotation
 */
TEST(TestSuite, testCase7)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 1;
    q2.y = 0;
    q2.z = 0;
    q2.w = 0;
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = 0;
    q2.y = 0;
    q2.z = 1;
    q2.w = 0;
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = std::sqrt((double)0.5);
    q2.y = std::sqrt((double)0.5);
    q2.z = 0;
    q2.w = 0;
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = 0;
    q1.y = 0;
    q1.z = 0;
    q1.w = 1;
    q2.x = std::sqrt((double)0.4);
    q2.y = std::sqrt((double)0.3);
    q2.z = 0;
    q2.w = std::sqrt((double)0.3);
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
    q1.x = std::sqrt((double)0.8);
    q1.y = 0;
    q1.z = 0;
    q1.w = std::sqrt((double)0.2);
    q2.x = std::sqrt((double)0.4);
    q2.y = std::sqrt((double)0.3);
    q2.z = 0;
    q2.w = std::sqrt((double)0.3);
    ans  = quaternion_operation::getRotation(q1,quaternion_operation::rotation(q1,q2));
    EXPECT_EQ(quaternion_operation::equals(q2,ans),true);
}

/**
 * @brief test for rotation function
 * @sa quaternion_operation::rotation
 */
TEST(TestSuite, testCase8)
{
    geometry_msgs::Quaternion q1,q2,ans;
    q1.x = 1;
    q1.y = 0;
    q1.z = 0;
    q1.w = 0;
    q2.x = 0;
    q2.y = 0;
    q2.z = 0;
    q2.w = 1;
    ans  = quaternion_operation::rotation(q1,q2);
    EXPECT_EQ(quaternion_operation::equals(q1,ans),true);
}

/**
 * @brief test for * operator
 * 
 */
TEST(TestSuite, testCase9)
{
    geometry_msgs::Quaternion q1,q2;
    q1.x = std::sqrt((double)0.8);
    q1.y = 0;
    q1.z = 0;
    q1.w = std::sqrt((double)0.2);
    q2.x = std::sqrt((double)0.4);
    q2.y = std::sqrt((double)0.3);
    q2.z = 0;
    q2.w = std::sqrt((double)0.3);
    EXPECT_EQ(quaternion_operation::equals(q1*q2,q2*q1),false);
}

/**
 * @brief test for euler to quat convertion
 * 
 */
TEST(TestSuite, testCase10)
{
    geometry_msgs::Quaternion q;
    geometry_msgs::Vector3 euler;
    euler.x = 0;
    euler.y = 0;
    euler.z = 0;
    q = quaternion_operation::convertEulerAngleToQuaternion(euler);
    EXPECT_FLOAT_EQ(q.x,0);
    EXPECT_FLOAT_EQ(q.y,0);
    EXPECT_FLOAT_EQ(q.z,0);
    EXPECT_FLOAT_EQ(q.w,1);
}

/**
 * @brief test for quat to euler convertion
 * 
 */
TEST(TestSuite, testCase11)
{
    geometry_msgs::Quaternion q;
    geometry_msgs::Vector3 euler;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.w = 1;
    euler = quaternion_operation::convertQuaternionToEulerAngle(q);
    EXPECT_FLOAT_EQ(euler.x,0);
    EXPECT_FLOAT_EQ(euler.y,0);
    EXPECT_FLOAT_EQ(euler.z,0);
}

/**
 * @brief Test for quat/euler conversion
 * 
 */
TEST(TestSuite, testCase12)
{
    geometry_msgs::Quaternion q,q1;
    geometry_msgs::Vector3 euler;
    q.x = 0;
    q.y = std::sqrt(0.5);
    q.z = 0;
    q.w = std::sqrt(0.5);
    euler = quaternion_operation::convertQuaternionToEulerAngle(q);
    q1 = quaternion_operation::convertEulerAngleToQuaternion(euler);
    EXPECT_FLOAT_EQ(q.x,q1.x);
    EXPECT_FLOAT_EQ(q.y,q1.y);
    EXPECT_FLOAT_EQ(q.z,q1.z);
    EXPECT_FLOAT_EQ(q.w,q1.w);
}

/**
 * @brief Run all the tests that were declared with TEST()
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}