#pragma once

#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>

namespace knowledge_rep
{
/**
 * @brief An instance of the Map concept, which represents a single 2D frame of reference
 *
 * Unlike instances of other concepts, maps **must** be uniquely named
 * @tparam LTMCImpl
 */
template <typename LTMCImpl>
class LTMCMap : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using PointImpl = LTMCPoint<LTMCImpl>;
  using PoseImpl = LTMCPose<LTMCImpl>;
  using RegionImpl = LTMCRegion<LTMCImpl>;

  friend LTMCImpl;
  uint map_id;

public:
  /**
   * @brief Build a map from its identifying components
   *
   * Map IDs aren't exposed, so this constructor shouldn't be called outside of the library.
   * Create and retrieve maps using other methods on LTMC.
   * @param entity_id
   * @param map_id
   * @param name
   * @param ltmc
   */
  LTMCMap(uint entity_id, uint map_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : map_id(map_id), InstanceImpl(entity_id, name, ltmc)
  {
  }

  /**
   * @brief Renames a map
   *
   * Due to special uniqueness constraints on map names, you need to use
   * this method instead of manually adjusting the name attribute.
   * @param new_name
   * @return true if the rename succeeded
   */
  bool rename(const std::string& new_name)
  {
    bool rename_succeeded = this->ltmc.get().renameMap(*this, new_name);
    if (rename_succeeded)
    {
      this->name = new_name;
    }
    return rename_succeeded;
  }

  std::string getName() const
  {
    return this->name;
  }

  /**
   * @brief Get's the map's id (**not** its entity ID)
   *
   * Map's are indexed by their own space of IDs. This prevents edge cases where a plain
   * entity ID could be coerced to be an instance of a map, without respecting unique constraints for maps.
   * @return
   */
  uint getId()
  {
    return this->map_id;
  }

  /**
   * @brief Add a new point to the map
   */
  PointImpl addPoint(const std::string& name, double x, double y)
  {
    return this->ltmc.get().addPoint(*this, name, x, y);
  }

  /**
   * @brief Add a new pose to the map
   *
   * The pose is assumed to be in the map frame
   *
   * @param name name of the pose to add
   * @param x X coordinate of the pose position
   * @param y Y coordinate of the pose position
   * @param theta orientation of the pose
   */
  PoseImpl addPose(const std::string& name, double x, double y, double theta)
  {
    return this->ltmc.get().addPose(*this, name, x, y, theta);
  }

  /**
   * @brief Add a new pose using two points
   *
   * Coordinates are taken to be in the map frame. This convenience method is simply intended
   * to prevent atan2 usage bugs in client code.
   *
   * @param name name of the pose to add
   * @param x1 X coordinate of the pose position
   * @param y1 Y coordinate of the pose position
   * @param x2 X coordinate of a second point along the direction of the pose orientation
   * @param y2 Y coordinate of a second point along the direction of the pose orientation
   */
  PoseImpl addPose(const std::string& name, double x1, double y1, double x2, double y2)
  {
    return addPose(name, x1, y1, atan2(y2 - y1, x2 - x1));
  }

  /**
   * @brief Add a new region to the map
   */
  RegionImpl addRegion(const std::string& name, const std::vector<std::pair<double, double>>& points)
  {
    return this->ltmc.get().addRegion(*this, name, points);
  }

  /**
   * @brief Retrieve an existing point by its unique name
   */
  boost::optional<PointImpl> getPoint(const std::string& name)
  {
    return this->ltmc.get().getPoint(*this, name);
  }

  /**
 * @brief Retrieve an existing pose by its unique name
 */
  boost::optional<PoseImpl> getPose(const std::string& name)
  {
    return this->ltmc.get().getPose(*this, name);
  }

  /**
 * @brief Retrieve an existing region by its unique name
 */
  boost::optional<RegionImpl> getRegion(const std::string& name)
  {
    return this->ltmc.get().getRegion(*this, name);
  }

  /**
 * @brief Get all points that belong to this map
 */
  std::vector<PointImpl> getAllPoints()
  {
    return this->ltmc.get().getAllPoints(*this);
  }

  /**
 * @brief Get all poses that belong to this map
 */
  std::vector<PoseImpl> getAllPoses()
  {
    return this->ltmc.get().getAllPoses(*this);
  }

  /**
 * @brief Get all regions that belong to this map
 */
  std::vector<RegionImpl> getAllRegions()
  {
    return this->ltmc.get().getAllRegions(*this);
  }

  /**
   * @brief Creates a copy of all of the map's owned geometry
   * @param with_name the name to use for the new map
   * @return the copied map
   */
  LTMCMap deepCopy(const std::string& with_name)
  {
    // This operation doesn't need to be fast and it doesn't need to be
    // especially safe, so we'll do it in memory
    LTMCMap new_map = this->ltmc.get().getMap(with_name);
    const auto points = getAllPoints();
    const auto poses = getAllPoses();
    const auto regions = getAllRegions();
    for (const auto point : points)
    {
      new_map.addPoint(point.getName(), point.x, point.y);
    }
    for (const auto pose : poses)
    {
      new_map.addPose(pose.getName(), pose.x, pose.y, pose.theta);
    }
    for (const auto region : regions)
    {
      new_map.addRegion(region.getName(), region.points);
    }
    return new_map;
  }

  bool operator==(const LTMCMap& other) const
  {
    return this->entity_id == other.entity_id && this->map_id == other.map_id;
  }
  bool operator!=(const LTMCMap& other) const
  {
    return this->entity_id != other.entity_id || this->map_id != other.map_id;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCMap<LTMCImpl>& m)
{
  return strm << "Map(" << m.entity_id << " \"" << m.getName() << "\")";
}
}  // namespace knowledge_rep
