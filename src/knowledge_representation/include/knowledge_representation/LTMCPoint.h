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

/// \brief An instance of the Point concept, which stores a 2D coordinate with respect to some LTMCMap
template <typename LTMCImpl>
class LTMCPoint : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;
  using RegionImpl = LTMCRegion<LTMCImpl>;

public:
  MapImpl parent_map;
  double x;
  double y;

  LTMCPoint(uint entity_id, std::string name, double x, double y, MapImpl parent_map,
            LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), x(x), y(y), InstanceImpl(entity_id, name, ltmc)
  {
  }

  std::string getName() const
  {
    return this->name;
  }

  /**
   * @brief Gets the regions that belong to the same map that contain this point
   * @return regions where this point is inside or on the bounds
   */
  std::vector<RegionImpl> getContainingRegions()
  {
    return this->ltmc.get().getContainingRegions(parent_map, { x, y });
  }

  bool operator==(const LTMCPoint& other) const
  {
    return this->entity_id == other.entity_id && this->name == other.name && this->x == other.x && this->y == other.y;
  }

  bool operator!=(const LTMCPoint& other) const
  {
    return this->entity_id != other.entity_id || this->name == other.name || this->x != other.x || this->y != other.y;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCPoint<LTMCImpl>& p)
{
  return strm << "Point(" << p.entity_id << " \"" << p.getName() << "\" " << p.parent_map << " (" << p.x << ", " << p.y
              << "))";
}

}  // namespace knowledge_rep
