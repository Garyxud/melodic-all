#include <knowledge_representation/LongTermMemoryConduit.h>
#include <string>
#include <vector>
#include <knowledge_representation/convenience.h>

#include <gtest/gtest.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <knowledge_representation/LTMCPoint.h>
#include <knowledge_representation/LTMCPose.h>
// #include <knowledge_representation/LTMCRegion.h>

using knowledge_rep::AttributeValueType;
using knowledge_rep::Concept;
using knowledge_rep::Entity;
using knowledge_rep::EntityAttribute;
using knowledge_rep::Instance;
using knowledge_rep::Map;
using knowledge_rep::Point;
using knowledge_rep::Pose;
using knowledge_rep::Region;
using std::cout;
using std::endl;
using std::string;
using std::vector;

class EntityTest : public ::testing::Test
{
protected:
  EntityTest() : ltmc(knowledge_rep::getDefaultLTMC()), entity(ltmc.addEntity())
  {
  }

  ~EntityTest()
  {
    entity.deleteEntity();
    ltmc.deleteAllEntities();
    ltmc.deleteAllAttributes();
  }

  knowledge_rep::LongTermMemoryConduit ltmc;
  knowledge_rep::Entity entity;
};

TEST_F(EntityTest, EntityEqualityWorks)
{
  auto new_entity = ltmc.addEntity();
  ASSERT_NE(entity, new_entity);

  auto reconstructed = Entity(new_entity.entity_id, ltmc);
  EXPECT_EQ(new_entity, reconstructed);
}

TEST_F(EntityTest, EntityAssignmentWorks)
{
  auto some_entity = ltmc.addEntity();
  auto old_id = some_entity.entity_id;
  some_entity = entity;
  EXPECT_NE(old_id, some_entity.entity_id);
}

TEST_F(EntityTest, DeleteEntityWorks)
{
  entity.deleteEntity();
  EXPECT_FALSE(entity.isValid());
  EXPECT_EQ(0, entity.getAttributes().size());
}

TEST_F(EntityTest, AddEntityWorks)
{
  ASSERT_TRUE(entity.isValid());
}

TEST_F(EntityTest, IdAttributeWorks)
{
  entity.addAttribute("is_open", 1u);
  auto attrs = entity.getAttributes("is_open");
  // That's right, we aren't actually storing these as uint, because databases rarely have uint support
  ASSERT_EQ(typeid(int), attrs.at(0).value.type());
  EXPECT_EQ(1, boost::get<int>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_open"));
}

TEST_F(EntityTest, BoolAttributeWorks)
{
  entity.addAttribute("is_open", true);
  auto attrs = entity.getAttributes("is_open");
  ASSERT_EQ(typeid(bool), attrs.at(0).value.type());
  EXPECT_TRUE(boost::get<bool>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_open"));

  entity.addAttribute("is_open", false);
  attrs = entity.getAttributes("is_open");
  ASSERT_EQ(typeid(bool), attrs.at(0).value.type());
  EXPECT_FALSE(boost::get<bool>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_open"));
}

TEST_F(EntityTest, IntAttributeWorks)
{
  entity.addAttribute("count", -1);
  auto attrs = entity.getAttributes("count");
  ASSERT_EQ(typeid(int), attrs.at(0).value.type());
  EXPECT_EQ(-1, boost::get<int>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("count"));
}

TEST_F(EntityTest, FloatAttributeWorks)
{
  entity.addAttribute("is_open", 1.f);
  auto attrs = entity.getAttributes("is_open");
  ASSERT_EQ(typeid(double), attrs.at(0).value.type());
  EXPECT_EQ(1, boost::get<double>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_open"));
}

TEST_F(EntityTest, StringAttributeWorks)
{
  entity.addAttribute("is_open", "test");
  auto attrs = entity.getAttributes("is_open");
  ASSERT_EQ(typeid(string), attrs.at(0).value.type());
  EXPECT_EQ("test", boost::get<string>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_open"));
}

TEST_F(EntityTest, CantRemoveEntityAttributeTwice)
{
  entity.addAttribute("is_open", true);
  ASSERT_TRUE(entity.removeAttribute("is_open"));
  ASSERT_FALSE(entity.removeAttribute("is_open"));
}
