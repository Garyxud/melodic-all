#pragma once

#include <iostream>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>
#include <typeindex>
#include <vector>
#include "EntityAttribute.h"

namespace knowledge_rep
{
template <typename EntLTMCImpl>
class LTMCEntity;

template <typename ConLTMCImpl>
class LTMCConcept;

template <typename InsLTMCImpl>
class LTMCInstance;

template <typename MapLTMCImpl>
class LTMCMap;

template <typename PointLTMCImpl>
class LTMCPoint;

template <typename PoseLTMCImpl>
class LTMCPose;

template <typename RegionLTMCImpl>
class LTMCRegion;

class EntityAttribute;

enum AttributeValueType;

/**
 * @brief Interface that defines the primary means of interacting with the knowledge base.
 *
 * This class implements the Curiously Recurring Template Pattern (CRTP) to enable multiple database backend
 * implementations while maintaining a static interface with no runtime dispatch. You can read more about this pattern
 * here: https://www.fluentcpp.com/2017/05/12/curiously-recurring-template-pattern/
 *
 * As a result, the implementation of the knowledgebase is completely defined in subclasses of this interface, one for
 * each backend implementation. These are expected to behave essentially identically, so documentation is presented
 * here.
 */

template <typename Impl>
class LongTermMemoryConduitInterface
{
  using LTMC = LongTermMemoryConduitInterface;

public:
  using EntityImpl = LTMCEntity<Impl>;
  using InstanceImpl = LTMCInstance<Impl>;
  using ConceptImpl = LTMCConcept<Impl>;
  using MapImpl = LTMCMap<Impl>;
  using PointImpl = LTMCPoint<Impl>;
  using PoseImpl = LTMCPose<Impl>;
  using RegionImpl = LTMCRegion<Impl>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;
  friend MapImpl;
  friend PointImpl;
  friend PoseImpl;
  friend RegionImpl;
  friend Impl;

  LongTermMemoryConduitInterface(LongTermMemoryConduitInterface&& that) noexcept = default;

  LongTermMemoryConduitInterface& operator=(LongTermMemoryConduitInterface&& that) noexcept = default;

  bool addAttribute(const std::string& name, const AttributeValueType type)
  {
    return static_cast<Impl*>(this)->addAttribute(name, type);
  };

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const uint other_entity_id)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, other_entity_id);
  };

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const int int_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, int_val);
  };

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const bool bool_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, bool_val);
  }

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const double float_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, float_val);
  }

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const char* string_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, std::string(string_val));
  }

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name,
                                                          const std::string& string_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, string_val);
  }

  /**
   * @brief Check whether an entity ID is currently tracked in the database
   * @param id the ID to check for
   * @return whether there is an entity with the given ID
   */
  bool entityExists(uint id) const
  {
    return static_cast<const Impl*>(this)->entityExists(id);
  };

  /**
 * @brief Inserts a new entity into the database.
 * @return the new entity
 */
  EntityImpl addEntity()
  {
    return static_cast<Impl*>(this)->addEntity();
  };

  /**
   * @brief Attempts to create an entity with a specific ID.
   * @param id
   * @return whether an entity with the given ID was created
   */
  bool addEntity(uint id)
  {
    return static_cast<Impl*>(this)->addEntity(id);
  };

  /**
* @brief Returns an entity with the given ID, if it exists
* @param entity_id the ID of the entity to fetch
* @return the entity requested, or an empty optional if no such entity exists
*/
  boost::optional<EntityImpl> getEntity(uint entity_id)
  {
    return static_cast<Impl*>(this)->getEntity(entity_id);
  };

  /**
 * @brief Returns an instance with the given ID, if it exists
 * @param entity_id the ID of the instance to fetch
 * @return the instance requested, or an empty optional if no such instance exists
 */
  boost::optional<InstanceImpl> getInstance(uint entity_id)
  {
    return static_cast<Impl*>(this)->getInstance(entity_id);
  };

  /**
  * @brief Returns an concept with the given ID, if it exists
  * @param entity_id the ID of the concept to fetch
  * @return the concept requested, or an empty optional if no such concept exists
  */
  boost::optional<ConceptImpl> getConcept(uint entity_id)
  {
    return static_cast<Impl*>(this)->getConcept(entity_id);
  };

  /**
  * @brief Returns an concept with the given ID, if it exists
  * @param entity_id the ID of the concept to fetch
  * @return the concept requested, or an empty optional if no such concept exists
  */
  boost::optional<MapImpl> getMap(uint entity_id)
  {
    return static_cast<Impl*>(this)->getMap(entity_id);
  };

  /**
 * @brief Returns an concept with the given ID, if it exists
 * @param entity_id the ID of the concept to fetch
 * @return the concept requested, or an empty optional if no such concept exists
 */
  boost::optional<PointImpl> getPoint(uint entity_id)
  {
    return static_cast<Impl*>(this)->getPoint(entity_id);
  };

  /**
* @brief Returns an concept with the given ID, if it exists
* @param entity_id the ID of the concept to fetch
* @return the concept requested, or an empty optional if no such concept exists
*/
  boost::optional<PoseImpl> getPose(uint entity_id)
  {
    return static_cast<Impl*>(this)->getPose(entity_id);
  };

  /**
* @brief Returns a region with the given ID, if it exists
   *
   * Useful for discovering whether an ID is a region
* @param entity_id the ID of the region to fetch
* @return the region requested, or an empty optional if no such region exists
*/
  boost::optional<RegionImpl> getRegion(uint entity_id)
  {
    return static_cast<Impl*>(this)->getRegion(entity_id);
  };

  // ATTRIBUTES

  /**
   * @brief Remove an attribute from the schema
   *
   * This will also remove all uses of this attribute.
   * @param name
   * @return whether the attribute was removed
   */
  bool deleteAttribute(const std::string& name)
  {
    return static_cast<Impl*>(this)->deleteAttribute(name);
  }

  /**
   * @brief Check whether an attribute exists in the current schema
   *
   * @param name
   * @return
   */
  bool attributeExists(const std::string& name) const
  {
    return static_cast<const Impl*>(this)->attributeExists(name);
  };

  // BULK OPERATIONS

  /**
   * @brief Get all current valid entities
   * @return list of all entities
   */
  std::vector<EntityImpl> getAllEntities()
  {
    return static_cast<Impl*>(this)->getAllEntities();
  }

  /**
   * @brief Queries for all entities that are marked as concepts
   * @return list of all concepts in the LTMC
   */
  std::vector<ConceptImpl> getAllConcepts()
  {
    return static_cast<Impl*>(this)->getAllConcepts();
  };

  /**
   * @brief Queries for all entities that are identified as instances
   * @return all instances in the LTMC
   */
  std::vector<InstanceImpl> getAllInstances()
  {
    return static_cast<Impl*>(this)->getAllInstances();
  }

  /**
 * @brief Queries for all entities that are identified as maps
   *
   * No operations are provided for bulk retrieving geometry as they can easily be accomplished
   * on a map by map basis.
 * @return all maps in the LTMC
 */
  std::vector<MapImpl> getAllMaps()
  {
    return static_cast<Impl*>(this)->getAllMaps();
  }

  /**
   * @brief Retrieves all attributes
   * @return a list of tuples. First element of each is the attribute name,
   * the second is the allowed type for the attribute
   */
  std::vector<std::pair<std::string, int>> getAllAttributes() const
  {
    return static_cast<const Impl*>(this)->getAllAttributes();
  }

  /**
   * @brief Retrieves all entity attributes
   * @return a list of entity attributes
   */
  std::vector<EntityAttribute> getAllEntityAttributes()
  {
    return static_cast<const Impl*>(this)->getAllEntityAttributes();
  }

  /**
 * @brief Remove all entities and all entity attributes except for the robot
 * @return The number of entities removed
 */
  uint deleteAllEntities()
  {
    return static_cast<Impl*>(this)->deleteAllEntities();
  }

  /**
   * @brief Remove all attributes except those defined in the schema as defaults
   * @return The number of attributes removed
   */
  uint deleteAllAttributes()
  {
    return static_cast<Impl*>(this)->deleteAllAttributes();
  }
  // RAW QUERIES

  bool selectQueryId(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryId(sql_query, result);
  }

  bool selectQueryBool(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryBool(sql_query, result);
  }

  bool selectQueryInt(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryInt(sql_query, result);
  }

  bool selectQueryFloat(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryFloat(sql_query, result);
  }

  bool selectQueryString(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryString(sql_query, result);
  }

  // CONVENIENCE
  /**
 * @brief Retrieves a concept of the given name, or creates one with the name if no such concept exists
 * @param name
 * @return the existing concept, or the newly created one. In either case, the concept will at least have the name
 *         passed as a parameter.
 */
  ConceptImpl getConcept(const std::string& name)
  {
    return static_cast<Impl*>(this)->getConcept(name);
  };

  /**
 * @brief Retrieves a map of the given name, or creates one with the name if no such map exists.
 * @param name
 * @return
 */
  MapImpl getMap(const std::string& name)
  {
    return static_cast<Impl*>(this)->getMap(name);
  }

  /**
   * @brief Gets the instance representing the robot
   * @return an instance representing the robot the LTMC is running on
   */
  InstanceImpl getRobot()
  {
    return static_cast<Impl*>(this)->getRobot();
  };

  // PROMOTERS
  /**
   * @brief A convenience for turning an existing entity ID into a concept
   * @param id The ID of an existing entity
   * @param name The desired name of the concept
   * @return whether the entity with the given ID was turned into a concept with the given name
   */
  bool makeConcept(uint id, std::string name)
  {
    return static_cast<Impl*>(this)->makeConcept(id, name);
  }

protected:
  // ENTITY BACKERS
  // These provide implementation for entity level operations. We want these to be centralized
  // with the rest of the database access code for ease of reimplementation in another backend,
  // but the users should see a nice API through the Entity class.
  bool deleteEntity(EntityImpl& entity)
  {
    return static_cast<Impl*>(this)->deleteEntity(entity);
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const uint other_entity_id)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, static_cast<const uint>(other_entity_id));
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const bool bool_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, bool_val);
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const int int_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, static_cast<const int>(int_val));
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const double float_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, float_val);
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const char* string_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, std::string(string_val));
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const std::string& string_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, string_val);
  }

  int removeAttribute(EntityImpl& entity, const std::string& attribute_name)
  {
    return static_cast<Impl*>(this)->removeAttribute(entity, attribute_name);
  }

  int removeAttributeOfValue(EntityImpl& entity, const std::string& attribute_name, const EntityImpl& other_entity)
  {
    return static_cast<Impl*>(this)->removeAttributeOfValue(entity, attribute_name, other_entity);
  }

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity) const
  {
    return static_cast<const Impl*>(this)->getAttributes(entity);
  }

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity, const std::string& attribute_name) const
  {
    return static_cast<const Impl*>(this)->getAttributes(entity, attribute_name);
  }

  bool isValid(const EntityImpl& entity) const
  {
    return static_cast<const Impl*>(this)->isValid(entity);
  }

  // INSTANCE BACKERS

  std::vector<ConceptImpl> getConcepts(const InstanceImpl& instance)
  {
    return static_cast<Impl*>(this)->getConcepts(instance);
  }

  std::vector<ConceptImpl> getConceptsRecursive(const InstanceImpl& instance)
  {
    return static_cast<Impl*>(this)->getConceptsRecursive(instance);
  }

  bool makeInstanceOf(InstanceImpl& instance, const ConceptImpl& concept)
  {
    return static_cast<Impl*>(this)->makeInstanceOf(instance, concept);
  }

  // CONCEPT BACKERS

  std::vector<ConceptImpl> getChildren(const ConceptImpl& concept)
  {
    return static_cast<Impl*>(this)->getChildren(concept);
  }

  std::vector<ConceptImpl> getChildrenRecursive(const ConceptImpl& concept)
  {
    return static_cast<Impl*>(this)->getChildrenRecursive(concept);
  }

  std::vector<InstanceImpl> getInstances(const ConceptImpl& concept)
  {
    return static_cast<Impl*>(this)->getInstances(concept);
  }

  boost::optional<InstanceImpl> getInstanceNamed(const ConceptImpl& concept, const std::string& name)
  {
    return static_cast<Impl*>(this)->getInstanceNamed(concept, name);
  };

  int removeInstances(const ConceptImpl& concept)
  {
    return static_cast<Impl*>(this)->removeInstances(concept);
  }

  int removeInstancesRecursive(const ConceptImpl& concept)
  {
    return static_cast<Impl*>(this)->removeInstancesRecursive(concept);
  }

  // MAP BACKERS
  PointImpl addPoint(MapImpl& map, const std::string& name, double x, double y)
  {
    return static_cast<Impl*>(this)->addPoint(map, name, x, y);
  }

  PoseImpl addPose(MapImpl& map, const std::string& name, double x, double y, double theta)
  {
    return static_cast<Impl*>(this)->addPose(map, name, x, y, theta);
  }

  RegionImpl addRegion(MapImpl& map, const std::string& name, const std::vector<std::pair<double, double>>& points)
  {
    return static_cast<Impl*>(this)->addRegion(map, name, points);
  }

  boost::optional<PointImpl> getPoint(MapImpl& map, const std::string& name)
  {
    return static_cast<Impl*>(this)->getPoint(map, name);
  }

  boost::optional<PoseImpl> getPose(MapImpl& map, const std::string& name)
  {
    return static_cast<Impl*>(this)->getPose(map, name);
  }

  boost::optional<RegionImpl> getRegion(MapImpl& map, const std::string& name)
  {
    return static_cast<Impl*>(this)->getRegion(map, name);
  }
  std::vector<PointImpl> getAllPoints(MapImpl& map)
  {
    return static_cast<Impl*>(this)->getAllPoints(map);
  }

  std::vector<PoseImpl> getAllPoses(MapImpl& map)
  {
    return static_cast<Impl*>(this)->getAllPoses(map);
  }

  std::vector<RegionImpl> getAllRegions(MapImpl& map)
  {
    return static_cast<Impl*>(this)->getAllRegions(map);
  }

  std::vector<RegionImpl> getContainingRegions(MapImpl& map, std::pair<double, double> point)
  {
    return static_cast<Impl*>(this)->getContainingRegions(map, point);
  }

  bool renameMap(MapImpl& map, const std::string& new_name)
  {
    return static_cast<Impl*>(this)->renameMap(map, new_name);
  }

  // REGION BACKERS

  std::vector<PointImpl> getContainedPoints(RegionImpl& region)
  {
    return static_cast<Impl*>(this)->getContainedPoints(region);
  }

  std::vector<PoseImpl> getContainedPoses(RegionImpl& region)
  {
    return static_cast<Impl*>(this)->getContainedPoses(region);
  }

  bool isPointContained(const RegionImpl& region, std::pair<double, double> point)
  {
    return static_cast<Impl*>(this)->isPointContained(region, point);
  }

private:
  // We make the constructor private to make sure people can't build this interface type directly
  LongTermMemoryConduitInterface() = default;
};

}  // namespace knowledge_rep
