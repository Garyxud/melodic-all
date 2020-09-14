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

class MapTest : public ::testing::Test
{
protected:
  MapTest()
    : ltmc(knowledge_rep::getDefaultLTMC())
    , map(ltmc.getMap("test map"))
    , point(map.addPoint("test point", 0, 1))
    , pose(map.addPose("test pose", 0, 1, 0))
    , region(map.addRegion("test region", { { 0, 1 }, { 2, 3 } }))
  {
  }

  void TearDown() override
  {
    map.deleteEntity();
    ltmc.deleteAllEntities();
    ltmc.deleteAllAttributes();
  }

  knowledge_rep::LongTermMemoryConduit ltmc;
  knowledge_rep::Map map;
  knowledge_rep::Point point;
  knowledge_rep::Pose pose;
  knowledge_rep::Region region;
};

TEST_F(MapTest, GetMap)
{
  EXPECT_EQ(map, ltmc.getMap("test map"));
  EXPECT_TRUE(map.hasConcept(ltmc.getConcept("map")));
  EXPECT_NE(map, ltmc.getMap("another map"));
}

TEST_F(MapTest, CopyMap)
{
  EXPECT_EQ(map, ltmc.getMap("test map"));
  auto copy = map.deepCopy("new map");
  EXPECT_EQ("new map", copy.getName());
  EXPECT_EQ(map.getAllPoints().size(), copy.getAllPoints().size());
  EXPECT_EQ(map.getAllPoses().size(), copy.getAllPoses().size());
  EXPECT_EQ(map.getAllRegions().size(), copy.getAllRegions().size());
}

TEST_F(MapTest, GetAllMaps)
{
  auto map_concept = ltmc.getConcept("map");

  EXPECT_TRUE(map.hasConcept(ltmc.getConcept("map")));
  EXPECT_NE(map, ltmc.getMap("another map"));
}

TEST_F(MapTest, AddPointWorks)
{
  EXPECT_EQ(0.0, point.x);
  EXPECT_EQ(1.0, point.y);
  EXPECT_EQ("test point", point.getName());
  EXPECT_TRUE(point.hasConcept(ltmc.getConcept("point")));
  auto second_point = map.addPoint("another point", 1.0, 2.0);
  EXPECT_EQ("another point", second_point.getName());
}

TEST_F(MapTest, GetAllPointsWorks)
{
  EXPECT_EQ(1, map.getAllPoints().size());
  point.deleteEntity();
  EXPECT_EQ(0, map.getAllPoints().size());
}

TEST_F(MapTest, DoubleAddPointFails)
{
  EXPECT_ANY_THROW(map.addPoint("test point", 2.0, 3.0));
}

TEST_F(MapTest, PointEqualityWorks)
{
  auto same_point = Point(point.entity_id, "test point", point.x, point.y, map, ltmc);
  EXPECT_EQ(point, same_point);
}

TEST_F(MapTest, GetPointWorks)
{
  auto retrieved = map.getPoint("test point");
  EXPECT_EQ(point, retrieved);
}

TEST_F(MapTest, AddPoseWorks)
{
  EXPECT_TRUE(pose.hasConcept(ltmc.getConcept("pose")));
  EXPECT_EQ(0, pose.x);
  EXPECT_EQ(1, pose.y);
  EXPECT_EQ(0, pose.theta);
  auto from_points = map.addPose("fromline", 0, 1, 2, 1);
  EXPECT_EQ(0, from_points.x);
  EXPECT_EQ(1, from_points.y);
  EXPECT_EQ(0, from_points.theta);
}

TEST_F(MapTest, GetPoseWorks)
{
  auto retrieved = map.getPose("test pose");
  EXPECT_EQ(pose, retrieved);
}

TEST_F(MapTest, GetAllPosesWorks)
{
  EXPECT_EQ(1, map.getAllPoses().size());
  pose.deleteEntity();
  EXPECT_EQ(0, map.getAllPoses().size());
}

TEST_F(MapTest, PoseEqualityWorks)
{
  auto second = map.addPose("test point", 2.0, 3.0, 4.0);
  pose.deleteEntity();
  EXPECT_EQ(pose, pose);
  EXPECT_EQ(second, second);
  EXPECT_NE(pose, second);
  EXPECT_NE(second, pose);
}

TEST_F(MapTest, AddRegionWorks)
{
  EXPECT_TRUE(region.hasConcept(ltmc.getConcept("region")));
  // EXPECT_THAT(region.points, ::testing::ContainerEq(std::vector<Region::Point2D>({ { 0.0, 1.0 }, { 2.0, 3.0 } })));
}

TEST_F(MapTest, GetRegionWorks)
{
  auto retrieved = map.getRegion("test region");
  EXPECT_EQ(region, retrieved);
  // EXPECT_THAT(retrieved.get().points, ::testing::ContainerEq(region.points));
}

TEST_F(MapTest, GetAllRegionsWorks)
{
  auto second = map.addRegion("second region", { { 1.1, 2.2 }, { 3.3, 4.4 } });
  auto retrieved = map.getAllRegions();
  EXPECT_TRUE(retrieved[0] == region || retrieved[1] == region);
  EXPECT_TRUE(retrieved[0] == second || retrieved[1] == second);
  second.deleteEntity();
  EXPECT_EQ(1, map.getAllRegions().size());
}

TEST_F(MapTest, RegionEqualityWorks)
{
  auto second = map.addRegion("second region", { { 1.1, 2.2 }, { 3.3, 4.4 } });
  EXPECT_EQ(region, region);
  EXPECT_EQ(second, second);
  EXPECT_NE(region, second);
  EXPECT_NE(second, region);
}

TEST_F(MapTest, MapNameWorks)
{
  EXPECT_EQ("test map", map.getName());
  ASSERT_EQ(1, map["name"].size());
  EXPECT_EQ("test map", map["name"][0].getStringValue());
  /*map.addAttribute("name", "second name");
  EXPECT_EQ("test map", map.getName());
  EXPECT_EQ("test map", map["name"][0].getStringValue());*/
  auto downcast = Entity{ map.entity_id, ltmc };
  EXPECT_EQ("test map", downcast.getName().get());
}

TEST_F(MapTest, MapRenameWorks)
{
  auto second_map = ltmc.getMap("second map");
  EXPECT_EQ("test map", map.getName());
  ASSERT_EQ(1, map["name"].size());
  EXPECT_EQ("test map", map["name"][0].getStringValue());

  // Fine to rename to current name
  ASSERT_TRUE(map.rename("test map"));
  // But otherwise map names must be unique
  ASSERT_FALSE(map.rename("second map"));
  ASSERT_TRUE(map.rename("new map"));
  EXPECT_EQ("new map", map.getName());
  auto downcast = Entity{ map.entity_id, ltmc };
  EXPECT_EQ("new map", downcast.getName().get());
}

TEST_F(MapTest, PointNameWorks)
{
  EXPECT_EQ("test point", point.getName());
  ASSERT_EQ(1, point["name"].size());
  EXPECT_EQ("test point", point["name"][0].getStringValue());
  /*map.addAttribute("name", "second name");
  EXPECT_EQ("test point", point.getName());
  EXPECT_EQ("test point", point["name"][0].getStringValue());*/
}

TEST_F(MapTest, PoseNameWorks)
{
  EXPECT_EQ("test pose", pose.getName());
  ASSERT_EQ(1, pose["name"].size());
  EXPECT_EQ("test pose", pose["name"][0].getStringValue());
  /*map.addAttribute("name", "second name");
  EXPECT_EQ("test pose", pose.getName());
  EXPECT_EQ("test pose", pose["name"][0].getStringValue());
   */
}

TEST_F(MapTest, RegionNameWorks)
{
  EXPECT_EQ("test region", region.getName());
  ASSERT_EQ(1, region["name"].size());
  EXPECT_EQ("test region", region["name"][0].getStringValue());
  /*map.addAttribute("name", "second name");
  EXPECT_EQ("test region", region.getName());
  EXPECT_EQ("test region", region["name"][0].getStringValue());*/

  Instance as_instance = region;
  EXPECT_EQ("test region", as_instance.getName().get());
}
