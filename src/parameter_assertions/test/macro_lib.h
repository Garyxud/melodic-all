#ifndef SRC_MACROS_H
#define SRC_MACROS_H

//==================
//=  Useful Macros =
//==================
#define EXPECT_VEC_EQ(a, b)                                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    ASSERT_EQ(a.size(), b.size());                                                                                     \
    for (size_t i = 0; i < a.size(); i++)                                                                              \
    {                                                                                                                  \
      EXPECT_EQ(a[i], b[i]);                                                                                           \
    }                                                                                                                  \
  } while (false)

#define EXPECT_STR_VEC_EQ(a, b)                                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    ASSERT_EQ(a.size(), b.size());                                                                                     \
    for (size_t i = 0; i < a.size(); i++)                                                                              \
    {                                                                                                                  \
      EXPECT_STREQ(a[i].c_str(), b[i].c_str());                                                                        \
    }                                                                                                                  \
  } while (false)

#define EXPECT_FLOAT_VEC_EQ(a, b)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    ASSERT_EQ(a.size(), b.size());                                                                                     \
    for (size_t i = 0; i < a.size(); i++)                                                                              \
    {                                                                                                                  \
      EXPECT_FLOAT_EQ(a[i], b[i]);                                                                                     \
    }                                                                                                                  \
  } while (false)

#define EXPECT_ROS_ALIVE()                                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    EXPECT_TRUE(ros::ok());                                                                                            \
  } while (false)

#define EXPECT_ROS_DEAD()                                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    EXPECT_FALSE(ros::ok());                                                                                           \
  } while (false)

#endif  // SRC_MACROS_H
