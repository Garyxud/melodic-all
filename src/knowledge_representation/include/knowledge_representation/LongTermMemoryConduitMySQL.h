#pragma once

#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <iostream>
#include <knowledge_representation/EntityAttribute.h>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <list>
#include <map>
#include <mysqlx/xdevapi.h>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>
#include <memory>

namespace knowledge_rep
{
static const std::vector<std::string> table_names = { "entity_attributes_id", "entity_attributes_str",
                                                      "entity_attributes_float", "entity_attributes_bool" };

class LongTermMemoryConduitMySQL : public LongTermMemoryConduitInterface<LongTermMemoryConduitMySQL>
{
  using EntityImpl = LTMCEntity<LongTermMemoryConduitMySQL>;
  using InstanceImpl = LTMCInstance<LongTermMemoryConduitMySQL>;
  using ConceptImpl = LTMCConcept<LongTermMemoryConduitMySQL>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;
  friend class LongTermMemoryConduitInterface;

public:
  std::unique_ptr<mysqlx::Session> sess;
  std::unique_ptr<mysqlx::Schema> db;

  explicit LongTermMemoryConduitMySQL(const std::string& db_name);

  // Move constructor
  LongTermMemoryConduitMySQL(LongTermMemoryConduitMySQL&& that) = default;

  ~LongTermMemoryConduitMySQL();

  // Move assignment
  LongTermMemoryConduitMySQL& operator=(LongTermMemoryConduitMySQL&& that) noexcept = default;

  bool addNewAttribute(const std::string& name, const AttributeValueType type);

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, uint other_entity_id);

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const bool bool_val);

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name,
                                                          const std::string& string_val);

  bool entityExists(uint id) const;

  bool deleteAttribute(std::string& name);

  bool attributeExists(const std::string& name) const;

  void deleteAllEntities();

  std::vector<EntityImpl> getAllEntities();

  std::vector<ConceptImpl> getAllConcepts();

  std::vector<InstanceImpl> getAllInstances();

  std::vector<std::pair<std::string, int>> getAllAttributes() const;

  std::vector<EntityAttribute> getAllEntityAttributes();

  template <typename T, typename... Types>
  bool select_query_args(const std::string& sql_query, std::vector<EntityAttribute>& result, Types&&... vals) const
  {
    try
    {
      auto sql_result = sess->sql(sql_query).bind(vals...).execute();
      result = unwrapAttributeRows<T>(sql_result.fetchAll());
      return true;
    }
    catch (const mysqlx::Error& err)
    {
      std::cout << "ERROR: " << err << std::endl;
      return false;
    }
  }

  template <typename T>
  bool selectQuery(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    try
    {
      auto sql_result = sess->sql(sql_query).execute();
      result = unwrapAttributeRows<T>(sql_result.fetchAll());
      return true;
    }
    catch (const mysqlx::Error& err)
    {
      std::cout << "ERROR: " << err << std::endl;
      return false;
    }
  }

  bool selectQueryInt(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<int>(sql_query, result);
  }

  bool selectQueryFloat(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<float>(sql_query, result);
  }

  bool selectQueryString(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<std::string>(sql_query, result);
  }

  bool selectQueryBool(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<bool>(sql_query, result);
  }

  // CONVENIENCE
  LTMCConcept<LongTermMemoryConduitMySQL> getConcept(const std::string& name);

  InstanceImpl getInstanceNamed(const std::string& name);

  InstanceImpl getRobot();

  EntityImpl addEntity();

  bool addEntity(uint id);

  boost::optional<EntityImpl> getEntity(uint entity_id);

protected:
  template <typename T>
  std::vector<EntityAttribute> unwrapAttributeRows(std::list<mysqlx::Row> rows) const
  {
    std::vector<EntityAttribute> result_map;
    for (auto& row : rows)
    {
      int obj_id = row[0];
      std::string attribute_name = row[1];
      T unwrapped = row[2];
      AttributeValue col_value(unwrapped);
      result_map.emplace_back(EntityAttribute(obj_id, attribute_name, col_value));
    }
    return result_map;
  }

  std::vector<EntityAttribute> unwrapAttributeRows(const std::string& table_name,
                                                   const std::list<mysqlx::Row>& rows) const
  {
    if (table_name == "entity_attributes_str")
    {
      return unwrapAttributeRows<std::string>(rows);
    }
    else if (table_name == "entity_attributes_id")
    {
      return unwrapAttributeRows<int>(rows);
    }
    else if (table_name == "entity_attributes_bool")
    {
      return unwrapAttributeRows<bool>(rows);
    }
    else if (table_name == "entity_attributes_float")
    {
      return unwrapAttributeRows<float>(rows);
    }
  }

  bool deleteEntity(EntityImpl& entity);

  bool addAttribute(LTMCEntity<LongTermMemoryConduitMySQL>& entity, const std::string& attribute_name,
                    const float float_val);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const bool bool_val);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const uint other_entity_id);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const std::string& string_val);

  int removeAttribute(EntityImpl& entity, const std::string& attribute_name);

  int removeAttributeOfValue(EntityImpl& entity, const std::string& attribute_name, const EntityImpl& other_entity);

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity) const;

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity, const std::string& attribute_name) const;

  bool isValid(const EntityImpl& entity) const;

  std::vector<ConceptImpl> get_concepts(const InstanceImpl& instance);
};

typedef LTMCEntity<LongTermMemoryConduitMySQL> Entity;
typedef LTMCConcept<LongTermMemoryConduitMySQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitMySQL> Instance;

typedef LTMCMap<LongTermMemoryConduitMySQL> Map;
typedef LTMCRegion<LongTermMemoryConduitMySQL> Region;
typedef LTMCPose<LongTermMemoryConduitMySQL> Pose;
typedef LTMCPoint<LongTermMemoryConduitMySQL> Point;
typedef LongTermMemoryConduitMySQL LongTermMemoryConduit;

}  // namespace knowledge_rep
