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
#include <knowledge_representation/LTMCRegion.h>

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

class LTMCTest : public ::testing::Test
{
protected:
  LTMCTest() : ltmc(knowledge_rep::getDefaultLTMC())
  {
  }

  void SetUp() override
  {
    ltmc.deleteAllAttributes();
    ltmc.deleteAllEntities();
  }

  knowledge_rep::LongTermMemoryConduit ltmc;
};

TEST_F(LTMCTest, InitialConfigurationIsValid)
{
  // Robot should exist
  EXPECT_TRUE(ltmc.entityExists(1));
  Instance robot = { ltmc.getEntity(1)->entity_id, ltmc };
  auto concepts = robot.getConcepts();
  EXPECT_EQ(1, concepts.size());
  EXPECT_EQ("robot", concepts[0].getName());
  EXPECT_EQ(6, ltmc.getAllEntities().size());
  EXPECT_EQ(5, ltmc.getAllConcepts().size());
  EXPECT_EQ(1, ltmc.getAllInstances().size());
  EXPECT_EQ(16, ltmc.getAllAttributes().size());
}

TEST_F(LTMCTest, GetConceptWorks)
{
  // Get concept returns the one true concept id
  Concept soda = ltmc.getConcept("soda");
  EXPECT_EQ(soda.entity_id, ltmc.getConcept("soda").entity_id);
}

TEST_F(LTMCTest, GetConceptIDWorks)
{
  // Get concept returns the one true concept id
  Concept soda = ltmc.getConcept("soda");
  auto by_id = ltmc.getConcept(soda.entity_id);
  ASSERT_TRUE(static_cast<bool>(by_id));
  EXPECT_EQ(soda.entity_id, by_id->entity_id);
}

TEST_F(LTMCTest, GetInstanceWorks)
{
  // Get concept returns the one true concept id
  Concept soda = ltmc.getConcept("soda");
  auto soda_inst = soda.createInstance("coke");
  auto by_id = ltmc.getInstance(soda_inst->entity_id);
  ASSERT_TRUE(static_cast<bool>(soda_inst));
  EXPECT_EQ(soda_inst->entity_id, by_id->entity_id);
}

TEST_F(LTMCTest, GetInstanceNamedWorks)
{
  // Get concept returns the one true concept id
  Concept soda = ltmc.getConcept("soda");
  auto soda_inst = soda.createInstance("coke");
  auto second_inst = soda.createInstance("another");
  // auto by_name = ltmc.getInstanceNamed("coke");
  ASSERT_TRUE(static_cast<bool>(soda_inst));
  // EXPECT_EQ(soda_inst->entity_id, by_name.entity_id);
}

TEST_F(LTMCTest, GetMapWorks)
{
  // Get concept returns the one true concept id
  auto fresh_map = ltmc.getMap("test map");
  auto second_map = ltmc.getMap("second test map");
  auto retrieved_map = ltmc.getMap("test map");
  EXPECT_EQ(fresh_map, retrieved_map);
}

TEST_F(LTMCTest, GetMapIdWorks)
{
  // Get concept returns the one true concept id
  auto fresh_map = ltmc.getMap("test map");
  auto retrieved_map = ltmc.getMap(fresh_map.entity_id);
  ASSERT_TRUE(static_cast<bool>(retrieved_map));
  EXPECT_EQ(fresh_map, *retrieved_map);
}

TEST_F(LTMCTest, GetPointIdWorks)
{
  // Get concept returns the one true concept id
  auto fresh_map = ltmc.getMap("test map");
  auto fresh_point = fresh_map.addPoint("test point", 0, 1);
  auto retrieved_point = ltmc.getPoint(fresh_point.entity_id);
  ASSERT_TRUE(static_cast<bool>(retrieved_point));
  EXPECT_EQ(fresh_point, *retrieved_point);
}

TEST_F(LTMCTest, GetPoseIdWorks)
{
  // Get concept returns the one true concept id
  auto fresh_map = ltmc.getMap("test map");
  auto fresh_pose = fresh_map.addPose("test pose", 0, 1, 2);
  auto retrieved_pose = ltmc.getPose(fresh_pose.entity_id);
  ASSERT_TRUE(static_cast<bool>(retrieved_pose));
  EXPECT_EQ(fresh_pose, *retrieved_pose);
}

TEST_F(LTMCTest, GetRegionIdWorks)
{
  // Get concept returns the one true concept id
  auto fresh_map = ltmc.getMap("test map");
  auto fresh_region = fresh_map.addRegion("test region", { { 0, 1 }, { 2, 3 }, { 3, 4 } });
  auto retrieved_region = ltmc.getRegion(fresh_region.entity_id);
  ASSERT_TRUE(static_cast<bool>(retrieved_region));
  EXPECT_EQ(fresh_region, *retrieved_region);
}

TEST_F(LTMCTest, SQLQueryIdWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryId("SELECT * FROM entity_attributes_id", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, SQLQueryBoolWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryBool("SELECT * FROM entity_attributes_bool", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, SQLQueryFloatWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryFloat("SELECT * FROM entity_attributes_float", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, SQLQueryStrWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_str", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, GetEntitiesWithAttributeOfValue)
{
  auto entity = ltmc.addEntity();
  bool success = entity.addAttribute("is_a", 1u);
  ASSERT_TRUE(success);
  auto entities = ltmc.getEntitiesWithAttributeOfValue("is_a", 1u);
  EXPECT_EQ(1, entities.size());

  success = entity.addAttribute("is_open", true);
  ASSERT_TRUE(success);
  entities = ltmc.getEntitiesWithAttributeOfValue("is_open", true);
  EXPECT_EQ(1, entities.size());

  success = entity.addAttribute("count", -1);
  ASSERT_TRUE(success);
  entities = ltmc.getEntitiesWithAttributeOfValue("count", -1);
  EXPECT_EQ(1, entities.size());

  success = entity.addAttribute("height", 2.01234567890123f);
  ASSERT_TRUE(success);
  entities = ltmc.getEntitiesWithAttributeOfValue("height", 2.01234567890123f);
  EXPECT_EQ(1, entities.size());

  success = entity.addAttribute("name", "never seen before name");
  ASSERT_TRUE(success);
  auto str = "never seen before name";
  entities = ltmc.getEntitiesWithAttributeOfValue("name", str);
  EXPECT_EQ(1, entities.size());
}

TEST_F(LTMCTest, ObjectAndConceptNameSpacesAreSeparate)
{
  Concept pitcher_con = ltmc.getConcept("soylent pitcher");
  knowledge_rep::Instance pitcher = *pitcher_con.createInstance("soylent pitcher");
  // Both should still be valid
  EXPECT_TRUE(pitcher_con.isValid());
  EXPECT_TRUE(pitcher.isValid());
}

TEST_F(LTMCTest, CanOnlyAddAttributeOnce)
{
  knowledge_rep::Entity drinkable = ltmc.getConcept("drinkable");
  knowledge_rep::Entity can = ltmc.addEntity();

  // Adding a second time should fail
  EXPECT_TRUE(can.addAttribute("is_a", drinkable));
  EXPECT_FALSE(can.addAttribute("is_a", drinkable));
}

TEST_F(LTMCTest, OnlyValidAttributeNamesAllowed)
{
  knowledge_rep::Entity can = ltmc.addEntity();
  EXPECT_FALSE(can.addAttribute("not a real attribute", true));
}

TEST_F(LTMCTest, GetAllEntitiesWorks)
{
  int start_num = ltmc.getAllEntities().size();
  ltmc.addEntity();
  EXPECT_EQ(ltmc.getAllEntities().size(), start_num + 1);
}

TEST_F(LTMCTest, GetAllConceptsWorks)
{
  int start_num = ltmc.getAllConcepts().size();
  ltmc.getConcept("neverseenbeforecon");
  EXPECT_EQ(ltmc.getAllConcepts().size(), start_num + 1);
}

TEST_F(LTMCTest, GetAllInstancesWorks)
{
  int start_num = ltmc.getAllInstances().size();
  ltmc.getConcept("test").createInstance();
  EXPECT_EQ(ltmc.getAllInstances().size(), start_num + 1);
}

TEST_F(LTMCTest, InstanceConceptSumToTotalEntities)
{
  knowledge_rep::Entity can = ltmc.addEntity();
  EXPECT_EQ(ltmc.getAllConcepts().size() + ltmc.getAllInstances().size(), ltmc.getAllEntities().size());
}

TEST_F(LTMCTest, GetAllAttributesWorks)
{
  int start_num = ltmc.getAllAttributes().size();
  ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Str);
  EXPECT_EQ(ltmc.getAllAttributes().size(), start_num + 1);
}

TEST_F(LTMCTest, GetAllMapsWorks)
{
  int start_num = ltmc.getAllMaps().size();
  ltmc.getMap("neverseenbeforemap");
  EXPECT_EQ(ltmc.getAllMaps().size(), start_num + 1);
}

TEST_F(LTMCTest, AddNewAttributeWorks)
{
  EXPECT_TRUE(ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Str));
  // Second add should fail
  EXPECT_FALSE(ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Str));
  // Even if it's on another type
  EXPECT_FALSE(ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Bool));
}

TEST_F(LTMCTest, RecursiveRemoveWorks)
{
  Concept parent = ltmc.getConcept("parent concept");
  Concept child = ltmc.getConcept("child concept");
  child.addAttribute("is_a", parent);
  auto instance = child.createInstance();
  ASSERT_EQ(1, parent.removeInstancesRecursive());
  EXPECT_FALSE(instance.isValid());
}

// Run all the tests
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
