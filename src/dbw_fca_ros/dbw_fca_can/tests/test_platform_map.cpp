/*********************************************************************
 * C++ unit test for dbw_fca_can/PlatformMap.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dbw_fca_can/PlatformMap.h>
using namespace dbw_fca_can;

// Test if x equals any of y
template <typename T> static bool EQ3(T x, T y0, T y1, T y2) {
  if (x == y0) return true;
  if (x == y1) return true;
  if (x == y2) return true;
  return false;
}

// Test constructors
TEST(PlatformMap, constructor)
{
  const Platform p = (Platform)1;
  const Module m = (Module)2;

  // Empty
  EXPECT_EQ(ModuleVersion(), PlatformMap().findModule(p,m));

  // Individual fields
  EXPECT_EQ(ModuleVersion(1,2,3), PlatformMap(p,m,ModuleVersion(1,2,3)).findModule(p,m));

  // PlatformVersion class
  EXPECT_EQ(ModuleVersion(1,2,3), PlatformMap(PlatformVersion(p,m, ModuleVersion(1,2,3))).findModule(p,m));

  // Vector of PlatformVersion
#if 0 // Fails to compile in ROS Indigo: "call of overloaded ‘PlatformMap(<brace-enclosed initializer list>)’ is ambiguous"
  EXPECT_EQ(ModuleVersion(1,2,3), PlatformMap({PlatformVersion(p,m, ModuleVersion(1,2,3))}).findModule(p,m));
#endif
}

// Test the findModule() method
TEST(PlatformMap, findModule)
{
  const Platform px = (Platform)0; const Module mx = (Module)0;
  const Platform py = (Platform)1; const Module my = (Module)1;
  const Platform pz = (Platform)2; const Module mz = (Module)2;
  const Platform pw = (Platform)3; const Module mw = (Module)3;

  // Construct PlatformMap with multiple platforms and modules
  const PlatformMap x({
    {PlatformVersion(px,mx,ModuleVersion(10,11,12))},
    {PlatformVersion(px,my,ModuleVersion(20,21,22))},
    {PlatformVersion(px,mz,ModuleVersion(30,31,32))},
    {PlatformVersion(py,mx,ModuleVersion(40,41,42))},
    {PlatformVersion(py,my,ModuleVersion(50,51,52))},
    {PlatformVersion(py,mz,ModuleVersion(60,61,62))},
    {PlatformVersion(pz,mx,ModuleVersion(70,71,72))},
    {PlatformVersion(pz,my,ModuleVersion(80,81,82))},
    {PlatformVersion(pz,mz,ModuleVersion(90,91,92))},
  });

  // Find entries that don't exist
  EXPECT_EQ(ModuleVersion(), x.findModule(px,mw));
  EXPECT_EQ(ModuleVersion(), x.findModule(pw,mx));
  EXPECT_EQ(ModuleVersion(), x.findModule(PlatformVersion(px,mw,0,0,0)));
  EXPECT_EQ(ModuleVersion(), x.findModule(PlatformVersion(pw,mx,0,0,0)));
  EXPECT_EQ(ModuleVersion(), x.findModule(mw));

  // Find each entry
  EXPECT_EQ(ModuleVersion(10,11,12), x.findModule(px,mx));
  EXPECT_EQ(ModuleVersion(20,21,22), x.findModule(px,my));
  EXPECT_EQ(ModuleVersion(30,31,32), x.findModule(px,mz));
  EXPECT_EQ(ModuleVersion(40,41,42), x.findModule(py,mx));
  EXPECT_EQ(ModuleVersion(50,51,52), x.findModule(py,my));
  EXPECT_EQ(ModuleVersion(60,61,62), x.findModule(py,mz));
  EXPECT_EQ(ModuleVersion(70,71,72), x.findModule(pz,mx));
  EXPECT_EQ(ModuleVersion(80,81,82), x.findModule(pz,my));
  EXPECT_EQ(ModuleVersion(90,91,92), x.findModule(pz,mz));
  EXPECT_EQ(ModuleVersion(10,11,12), x.findModule(PlatformVersion(px,mx,0,0,0)));
  EXPECT_EQ(ModuleVersion(20,21,22), x.findModule(PlatformVersion(px,my,0,0,0)));
  EXPECT_EQ(ModuleVersion(30,31,32), x.findModule(PlatformVersion(px,mz,0,0,0)));
  EXPECT_EQ(ModuleVersion(40,41,42), x.findModule(PlatformVersion(py,mx,0,0,0)));
  EXPECT_EQ(ModuleVersion(50,51,52), x.findModule(PlatformVersion(py,my,0,0,0)));
  EXPECT_EQ(ModuleVersion(60,61,62), x.findModule(PlatformVersion(py,mz,0,0,0)));
  EXPECT_EQ(ModuleVersion(70,71,72), x.findModule(PlatformVersion(pz,mx,0,0,0)));
  EXPECT_EQ(ModuleVersion(80,81,82), x.findModule(PlatformVersion(pz,my,0,0,0)));
  EXPECT_EQ(ModuleVersion(90,91,92), x.findModule(PlatformVersion(pz,mz,0,0,0)));

  // Find any module regardless of platform
  EXPECT_TRUE(EQ3(x.findModule(mx), ModuleVersion(10,11,12), ModuleVersion(40,41,42), ModuleVersion(70,71,72)));
  EXPECT_TRUE(EQ3(x.findModule(my), ModuleVersion(20,21,22), ModuleVersion(50,51,52), ModuleVersion(80,81,82)));
  EXPECT_TRUE(EQ3(x.findModule(mz), ModuleVersion(30,31,32), ModuleVersion(60,61,62), ModuleVersion(90,91,92)));
}

// Test the findPlatform() method
TEST(PlatformMap, findPlatform)
{
  const Platform px = (Platform)0; const Module mx = (Module)0;
  const Platform py = (Platform)1; const Module my = (Module)1;
  const Platform pz = (Platform)2; const Module mz = (Module)2;
  const Platform pw = (Platform)3; const Module mw = (Module)3;

  // Construct PlatformMap with multiple platforms and modules
  const PlatformMap x({
    {PlatformVersion(px,mx,ModuleVersion(10,11,12))},
    {PlatformVersion(px,my,ModuleVersion(20,21,22))},
    {PlatformVersion(px,mz,ModuleVersion(30,31,32))},
    {PlatformVersion(py,mx,ModuleVersion(40,41,42))},
    {PlatformVersion(py,my,ModuleVersion(50,51,52))},
    {PlatformVersion(py,mz,ModuleVersion(60,61,62))},
    {PlatformVersion(pz,mx,ModuleVersion(70,71,72))},
    {PlatformVersion(pz,my,ModuleVersion(80,81,82))},
    {PlatformVersion(pz,mz,ModuleVersion(90,91,92))},
  });

  // Find entries that don't exist
  EXPECT_EQ(PlatformVersion(), x.findPlatform(mw));
  EXPECT_EQ(PlatformVersion(), x.findPlatform(PlatformVersion(pw,mw,0,0,0)));

  // Find each entry
  EXPECT_TRUE(EQ3(x.findPlatform(mx), PlatformVersion(px,mx,ModuleVersion(10,11,12)), PlatformVersion(py,mx,ModuleVersion(40,41,42)), PlatformVersion(pz,mx,ModuleVersion(70,71,72))));
  EXPECT_TRUE(EQ3(x.findPlatform(my), PlatformVersion(px,my,ModuleVersion(20,21,22)), PlatformVersion(py,my,ModuleVersion(50,51,52)), PlatformVersion(pz,my,ModuleVersion(80,81,82))));
  EXPECT_TRUE(EQ3(x.findPlatform(mz), PlatformVersion(px,mz,ModuleVersion(30,31,32)), PlatformVersion(py,mz,ModuleVersion(60,61,62)), PlatformVersion(pz,mz,ModuleVersion(90,91,92))));
}

// Test operators
TEST(PlatformMap, operators)
{
  const Platform px = (Platform)0; const Module mx = (Module)0;
  const Platform py = (Platform)1; const Module my = (Module)1;
  const Platform pz = (Platform)2; const Module mz = (Module)2;
  const Platform pw = (Platform)3; const Module mw = (Module)3;

  // Construct PlatformMap with multiple platforms and modules
  const PlatformMap x({
    {PlatformVersion(px,mx,ModuleVersion(10,11,12))},
    {PlatformVersion(px,my,ModuleVersion(20,21,22))},
    {PlatformVersion(px,mz,ModuleVersion(30,31,32))},
    {PlatformVersion(py,mx,ModuleVersion(40,41,42))},
    {PlatformVersion(py,my,ModuleVersion(50,51,52))},
    {PlatformVersion(py,mz,ModuleVersion(60,61,62))},
    {PlatformVersion(pz,mx,ModuleVersion(70,71,72))},
    {PlatformVersion(pz,my,ModuleVersion(80,81,82))},
    {PlatformVersion(pz,mz,ModuleVersion(90,91,92))},
  });

  // Compare PlatformVersion with PlatformMap entries that don't exist
  EXPECT_TRUE (PlatformVersion(pw,mw,0,0,0) == x);
  EXPECT_FALSE(PlatformVersion(pw,mw,1,1,1) <  x);
  EXPECT_FALSE(PlatformVersion(pw,mw,1,1,1) <= x);
  EXPECT_TRUE (PlatformVersion(pw,mw,1,1,1) >  x);
  EXPECT_TRUE (PlatformVersion(pw,mw,1,1,1) >= x);
  EXPECT_FALSE(PlatformVersion(pw,mw,1,1,1) == x);
  EXPECT_TRUE (PlatformVersion(pw,mw,1,1,1) != x);

  // Compare PlatformVersion with PlatformMap entries
  EXPECT_TRUE (PlatformVersion(py,my,45,45,45) <  x);
  EXPECT_FALSE(PlatformVersion(py,my,55,55,55) <  x);
  EXPECT_TRUE (PlatformVersion(py,my,45,45,45) <= x);
  EXPECT_TRUE (PlatformVersion(py,my,50,51,52) <= x);
  EXPECT_FALSE(PlatformVersion(py,my,55,55,55) <= x);
  EXPECT_FALSE(PlatformVersion(py,my,45,45,45) >  x);
  EXPECT_TRUE (PlatformVersion(py,my,55,55,55) >  x);
  EXPECT_FALSE(PlatformVersion(py,my,45,45,45) >= x);
  EXPECT_TRUE (PlatformVersion(py,my,50,51,52) >= x);
  EXPECT_TRUE (PlatformVersion(py,my,55,55,55) >= x);
  EXPECT_TRUE (PlatformVersion(py,my,50,51,52) == x);
  EXPECT_FALSE(PlatformVersion(py,my,50,50,50) == x);
  EXPECT_TRUE (PlatformVersion(py,my,50,50,50) != x);
  EXPECT_FALSE(PlatformVersion(py,my,50,51,52) != x);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

