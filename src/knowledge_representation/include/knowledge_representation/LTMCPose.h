#pragma once

#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>

namespace knowledge_rep
{
template <typename LTMCImpl>
class LTMCRegion;

/// @brief An instance of the "pose" concept, representing a 2D point and orientation (x, y, theta) with respect to some
/// LTMCMap
template <typename LTMCImpl>
class LTMCPose : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;
  using RegionImpl = LTMCRegion<LTMCImpl>;

public:
  MapImpl parent_map;
  double x;
  double y;
  double theta;
  LTMCPose(uint entity_id, std::string name, double x, double y, double theta, MapImpl parent_map,
           LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), x(x), y(y), theta(theta), InstanceImpl(entity_id, name, ltmc)
  {
  }

  std::string getName() const
  {
    return this->name;
  }

  /**
   * @brief Gets the regions that belong to the same map that contain this pose
   * @return regions where this pose is inside or on the bounds
   */
  std::vector<RegionImpl> getContainingRegions()
  {
    return this->ltmc.get().getContainingRegions(parent_map, { x, y });
  }

  bool operator==(const LTMCPose& other) const
  {
    return this->entity_id == other.entity_id && this->name == other.name && this->parent_map == other.parent_map &&
           this->x == other.x && this->y == other.y && this->theta == other.theta;
  }

  bool operator!=(const LTMCPose& other) const
  {
    return this->entity_id != other.entity_id || this->name != other.name || this->parent_map != other.parent_map ||
           this->x != other.x || this->y != other.y || this->theta != other.theta;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCPose<LTMCImpl>& p)
{
  return strm << "Pose(" << p.entity_id << " \"" << p.getName() << "\" " << p.parent_map << " (" << p.x << ", " << p.y
              << ", " << p.theta << "))";
}

}  // namespace knowledge_rep
