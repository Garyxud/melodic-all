#include "um6/registers.h"
#include <gtest/gtest.h>

#include <arpa/inet.h>


TEST(ByteOrder, compare_with_htons)
{
  // Arbitrary, just try a selection of values.
  for(uint16_t host_num = 0; host_num < 50000; host_num += 71)
  {
    uint16_t net_num = htons(host_num);
    uint16_t memcpy_num = 0;
    um6::memcpy_network(&memcpy_num, &host_num, sizeof(host_num));
    EXPECT_EQ(memcpy_num, net_num);
  }
}

TEST(ByteOrder, compare_with_htonl)
{
  for(uint32_t host_num = 0; host_num < 4000000000; host_num += 1299827)
  {
    uint32_t net_num = htonl(host_num);
    uint32_t memcpy_num = 0;
    um6::memcpy_network(&memcpy_num, &host_num, sizeof(host_num));
    EXPECT_EQ(memcpy_num, net_num);
  }
}

TEST(Accessor, basic_int)
{
  um6::Registers r;
  r.write_raw(5, "\x01\x02\x03\x04\x05\x06");

  um6::Accessor<uint16_t> u16(&r, 5, 3);
  EXPECT_EQ(0x0102, u16.get(0));
  EXPECT_EQ(0x0304, u16.get(1));
  EXPECT_EQ(0x0506, u16.get(2));

  um6::Accessor<uint16_t> u16n(&r, 6, 1);
  EXPECT_EQ(0x0506, u16n.get(0));

  um6::Accessor<uint32_t> u32(&r, 5, 3);
  EXPECT_EQ(0x01020304, u32.get(0));
}

TEST(Accessor, basic_float)
{
  um6::Registers r;
  r.write_raw(10, "\x01\x02\x03\x04\x05\x06\x07\x08");

  um6::Accessor<float> f(&r, 10, 2);
  union
  {
    float val;
    uint32_t bytes;
  };
  bytes = 0x01020304;
  EXPECT_FLOAT_EQ(val, f.get(0));
  bytes = 0x05060708;
  EXPECT_FLOAT_EQ(val, f.get(1));
}

TEST(Accessor, scaled_int)
{
  um6::Registers r;
  r.write_raw(11, "\x01\x02\x03\x04");

  const float scale(0.001);
  um6::Accessor<int16_t> i16(&r, 11, 2, scale);
  EXPECT_FLOAT_EQ(scale * 0x0102, i16.get_scaled(0));
  EXPECT_FLOAT_EQ(scale * 0x0304, i16.get_scaled(1));
}

TEST(Accessor, set_float)
{
  um6::Registers r;
  r.mag_ref.set(0, 0.123);
  r.mag_ref.set_scaled(1, 0.987);
  r.mag_ref.set_scaled(2, 0.555);

  float check;
  um6::memcpy_network(&check, (float*)r.mag_ref.raw(), 4);
  EXPECT_FLOAT_EQ(0.123, check);
  um6::memcpy_network(&check, (float*)r.mag_ref.raw() + 1, 4);
  EXPECT_FLOAT_EQ(0.987, check);
  um6::memcpy_network(&check, (float*)r.mag_ref.raw() + 2, 4);
  EXPECT_FLOAT_EQ(0.555, check);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
