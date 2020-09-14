// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <psen_scan/ros_parameter_handler.h>
#include <psen_scan/decrypt_password_exception.h>

using namespace psen_scan;

namespace psen_scan_test
{
TEST(decryptPasswordTest, charOutOfRange)
{
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCCDDEEFFGG"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("GGAABBCCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCGGCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBGACCDDEEFF"), DecryptPasswordException);

  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCCDDEEFF@@"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("@@AABBCCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCCDDEEFF@@"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("A@@ABBCCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AA@ABBCCDDEEFF"), DecryptPasswordException);

  EXPECT_THROW(RosParameterHandler::decryptPassword("aabbccddeeffgg"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("ggaabbccddeeff"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("``aabbccddeeff"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("aabbccddeeff``"), DecryptPasswordException);

  EXPECT_THROW(RosParameterHandler::decryptPassword("//345678901234"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("345678901234::"), DecryptPasswordException);
}

TEST(decryptPasswordTest, unevenCharacterCount)
{
  EXPECT_THROW(RosParameterHandler::decryptPassword("ac0d8d033"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("aC068d033"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("aC0d68D03"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("ac0d6 8d03"), DecryptPasswordException);
  EXPECT_NO_THROW(RosParameterHandler::decryptPassword("ac0d68d033"));
  EXPECT_NO_THROW(RosParameterHandler::decryptPassword("ac0d6 8d033"));
}

TEST(decryptPasswordTest, controlCharactersBelow32)
{
  EXPECT_THROW(RosParameterHandler::decryptPassword("CD3195"), DecryptPasswordException);  //"\0\0\0"
  EXPECT_THROW(RosParameterHandler::decryptPassword("AA3111"), DecryptPasswordException);  // In der Mitte "\0"
  EXPECT_THROW(RosParameterHandler::decryptPassword("c4fd50ca29"),
               DecryptPasswordException);  // Tab am Anfang "\TABTEST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec8cca29"),
               DecryptPasswordException);  // Tab in der Mitte "TE\TABST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec66c554"),
               DecryptPasswordException);  // Tab am Ende "TEST\TAB)"
  EXPECT_THROW(RosParameterHandler::decryptPassword("d2fd50ca29"),
               DecryptPasswordException);  // \31 am Anfang "\31TEST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec9Aca29"),
               DecryptPasswordException);  // \31 in der Mitte "TE\31ST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec66c562"), DecryptPasswordException);  // Tab am Ende "TEST/31"
}

TEST(decryptPasswordTest, correctDecryption)
{
  EXPECT_EQ(RosParameterHandler::decryptPassword("8e0987d04eadfc68c380e74a"), "Christian123");
  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b115640c70d438"), "Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword("9e1086da35a04aae1276da3e"), "Sascha??????");

  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b1 15640c70d438"), "Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b11 5640c70d438"), "Giuseppe!!!!");

  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b115640c70d438 "), "Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword(" 8a0880ea38b115640c70d438"), "Giuseppe!!!!");

  EXPECT_EQ(RosParameterHandler::decryptPassword("  8 a0880e a38b115 640c7  0d4   38   "), "Giuseppe!!!!");
}
}  // namespace psen_scan_test
