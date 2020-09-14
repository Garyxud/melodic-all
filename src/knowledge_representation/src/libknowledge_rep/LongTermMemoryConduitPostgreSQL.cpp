#include <knowledge_representation/LongTermMemoryConduitPostgreSQL.h>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <iostream>
#include <string>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <knowledge_representation/LTMCPoint.h>
#include <knowledge_representation/LTMCPose.h>
#include <knowledge_representation/LTMCRegion.h>
#include <vector>
#include <utility>
#include <regex>

using std::string;
using std::vector;
namespace knowledge_rep
{
typedef LTMCEntity<LongTermMemoryConduitPostgreSQL> Entity;
typedef LTMCConcept<LongTermMemoryConduitPostgreSQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitPostgreSQL> Instance;
typedef LTMCPoint<LongTermMemoryConduitPostgreSQL> Point;
typedef LTMCPose<LongTermMemoryConduitPostgreSQL> Pose;
typedef LTMCRegion<LongTermMemoryConduitPostgreSQL> Region;
typedef LTMCMap<LongTermMemoryConduitPostgreSQL> Map;

template <typename Out>
void split(const string& s, char delim, Out result)
{
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim))
  {
    *result++ = item;
  }
}

std::vector<std::string> split(const string& s, char delim)
{
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}

std::vector<Region::Point2D> strToPoints(const string& s)
{
  std::vector<Region::Point2D> points;
  std::regex paren_regex("\\(|\\)");
  std::string result;
  // write the results to an output iterator
  std::regex_replace(back_inserter(result), s.begin(), s.end(), paren_regex, "");
  auto components = split(result, ',');
  auto i = components.begin();
  while (i < components.end())
  {
    auto f = *i++;
    auto s = *i++;
    points.emplace_back(std::stod(f), std::stod(s));
  }
  return points;
}

LongTermMemoryConduitPostgreSQL::LongTermMemoryConduitPostgreSQL(const string& db_name, const string& hostname)
  : LongTermMemoryConduitInterface<LongTermMemoryConduitPostgreSQL>()
{
  conn = std::unique_ptr<pqxx::connection>(new pqxx::connection("postgresql://postgres@" + hostname + "/" + db_name));
}

LongTermMemoryConduitPostgreSQL::~LongTermMemoryConduitPostgreSQL() = default;

bool LongTermMemoryConduitPostgreSQL::addEntity(uint id)
{
  pqxx::work txn{ *conn };
  pqxx::result result = txn.exec("INSERT INTO entities "
                                 "VALUES (" +
                                 txn.quote(id) + ") ON CONFLICT DO NOTHING RETURNING entity_id");
  txn.commit();
  return result.size() == 1;
}

/**
 * @brief Add a new attribute
 * @param name the name of the attribute
 * @param type the type of data for the attribute's values
 * @return whether the attribute was added. Note that addition will fail if the attribute already exists.
 */
bool LongTermMemoryConduitPostgreSQL::addNewAttribute(const string& name, const AttributeValueType type)
{
  try
  {
    pqxx::work txn{ *conn };
    pqxx::result result = txn.exec("INSERT INTO attributes VALUES (" + txn.quote(name) + ", " +
                                   txn.quote(attribute_value_type_to_string[type]) + ") ON CONFLICT DO NOTHING");
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool LongTermMemoryConduitPostgreSQL::entityExists(uint id) const
{
  pqxx::work txn{ *conn, "entityExists" };
  auto result = txn.exec("SELECT count(*) FROM entities WHERE entity_id=" + txn.quote(id));
  txn.commit();
  return result[0]["count"].as<uint>() == 1;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getEntitiesWithAttributeOfValue(const string& attribute_name,
                                                                                const uint other_entity_id)
{
  pqxx::work txn{ *conn, "getEntitiesWithAttributeOfValueId" };
  auto result = txn.exec("SELECT entity_id FROM entity_attributes_id "
                         "WHERE attribute_value=" +
                         txn.quote(other_entity_id) + " and attribute_name = " + txn.quote(attribute_name));
  txn.commit();

  vector<Entity> return_result;
  for (const auto& row : result)
  {
    return_result.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return return_result;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getEntitiesWithAttributeOfValue(const string& attribute_name,
                                                                                const bool bool_val)
{
  pqxx::work txn{ *conn, "getEntitiesWithAttributeOfValueBool" };
  auto result = txn.parameterized("SELECT entity_id FROM entity_attributes_bool "
                                  "WHERE attribute_value= $1  AND attribute_name = $2")(bool_val)(attribute_name)
                    .exec();
  txn.commit();

  vector<Entity> return_result;
  for (const auto& row : result)
  {
    return_result.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return return_result;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getEntitiesWithAttributeOfValue(const string& attribute_name,
                                                                                const int int_val)
{
  pqxx::work txn{ *conn, "getEntitiesWithAttributeOfValueInt" };
  auto result = txn.exec("SELECT entity_id FROM entity_attributes_int "
                         "WHERE attribute_value=" +
                         txn.quote(int_val) + " and attribute_name = " + txn.quote(attribute_name));
  txn.commit();

  vector<Entity> return_result;
  for (const auto& row : result)
  {
    return_result.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return return_result;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getEntitiesWithAttributeOfValue(const string& attribute_name,
                                                                                const double float_val)
{
  pqxx::work txn{ *conn, "getEntitiesWithAttributeOfValueFloat" };
  auto result = txn.exec("SELECT entity_id FROM entity_attributes_float "
                         "WHERE attribute_value=" +
                         txn.quote(float_val) + " and attribute_name = " + txn.quote(attribute_name));
  txn.commit();

  vector<Entity> return_result;
  for (const auto& row : result)
  {
    return_result.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return return_result;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getEntitiesWithAttributeOfValue(const string& attribute_name,
                                                                                const char* string_val)
{
  return this->getEntitiesWithAttributeOfValue(attribute_name, std::string(string_val));
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getEntitiesWithAttributeOfValue(const string& attribute_name,
                                                                                const string& string_val)
{
  pqxx::work txn{ *conn, "getEntitiesWithAttributeOfValueString" };
  auto result = txn.exec("SELECT entity_id FROM entity_attributes_str "
                         "WHERE attribute_value=" +
                         txn.quote(string_val) + " and attribute_name = " + txn.quote(attribute_name));
  txn.commit();

  vector<Entity> return_result;
  for (const auto& row : result)
  {
    return_result.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return return_result;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::getAllEntities()
{
  pqxx::work txn{ *conn, "getAllEntities" };

  auto result = txn.exec("TABLE entities");
  txn.commit();

  vector<Entity> entities;
  for (const auto& row : result)
  {
    entities.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return entities;
}

vector<Map> LongTermMemoryConduitPostgreSQL::getAllMaps()
{
  pqxx::work txn{ *conn, "getAllMaps" };

  auto result = txn.exec("TABLE maps");
  txn.commit();

  vector<Map> maps;
  for (const auto& row : result)
  {
    maps.emplace_back(row["entity_id"].as<uint>(), row["map_id"].as<uint>(), row["map_name"].as<string>(), *this);
  }
  return maps;
}

uint LongTermMemoryConduitPostgreSQL::deleteAllAttributes()
{
  pqxx::work txn{ *conn };

  // Remove all entities
  uint num_deleted = txn.exec("DELETE FROM attributes").affected_rows();
  // Use the baked in function to get the default configuration back
  txn.exec("SELECT * FROM add_default_attributes()");
  txn.commit();
  return num_deleted;
}

uint LongTermMemoryConduitPostgreSQL::deleteAllEntities()
{
  pqxx::work txn{ *conn };

  // Remove all entities
  uint num_deleted = txn.exec("DELETE FROM entities").affected_rows();
  // Use the baked in function to get the default configuration back
  txn.exec("SELECT * FROM add_default_entities()");
  txn.commit();
  assert(entityExists(1));
  return num_deleted;
}

bool LongTermMemoryConduitPostgreSQL::deleteAttribute(const string& name)
{
  pqxx::work txn{ *conn };
  uint num_deleted = txn.exec("DELETE FROM attributes WHERE attribute_name = " + txn.quote(name)).affected_rows();
  txn.commit();
  return num_deleted;
}

bool LongTermMemoryConduitPostgreSQL::attributeExists(const string& name) const
{
  pqxx::work txn{ *conn, "attributeExists" };
  auto result = txn.exec("SELECT count(*) FROM attributes WHERE attribute_name=" + txn.quote(name));
  txn.commit();
  return result[0]["count"].as<uint>() == 1;
}

Concept LongTermMemoryConduitPostgreSQL::getConcept(const string& name)
{
  pqxx::work txn{ *conn, "getConcept" };
  auto result = txn.exec("SELECT entity_id FROM concepts WHERE concept_name =" + txn.quote(name));
  txn.commit();

  if (result.empty())
  {
    Entity new_concept = addEntity();
    pqxx::work txn{ *conn, "getConcept" };
    auto result = txn.parameterized("INSERT INTO concepts VALUES ($1, $2)")(new_concept.entity_id)(name).exec();
    txn.commit();
    return { new_concept.entity_id, name, *this };
  }
  else
  {
    return { result[0]["entity_id"].as<uint>(), name, *this };
  }
}

boost::optional<Instance> LongTermMemoryConduitPostgreSQL::getInstanceNamed(const Concept& concept, const string& name)
{
  pqxx::work txn{ *conn, "getInstanceNamed" };
  auto result = txn.parameterized("SELECT entity_id FROM entity_attributes_str WHERE attribute_name = 'name' "
                                  "AND attribute_value = $1 AND entity_id IN (SELECT entity_id FROM instance_of WHERE "
                                  "concept_name = $2)")(name)(concept.getName())
                    .exec();
  txn.commit();
  if (result.empty())
  {
    return {};
  }
  else
  {
    // Can only be one instance with a given name
    assert(result.size() == 1);
    return Instance{ result[0]["entity_id"].as<uint>(), *this };
  }
}

boost::optional<Entity> LongTermMemoryConduitPostgreSQL::getEntity(uint entity_id)
{
  if (entityExists(entity_id))
  {
    return Entity{ entity_id, *this };
  }
  return {};
}

boost::optional<Instance> LongTermMemoryConduitPostgreSQL::getInstance(uint entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "getInstance" };
    auto result = txn.parameterized("SELECT count(*) FROM instance_of WHERE entity_id = $1")(entity_id).exec();
    txn.commit();
    if (result[0]["count"].as<uint>() == 1)
    {
      return Instance{ entity_id, *this };
    }
    return {};
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

boost::optional<Concept> LongTermMemoryConduitPostgreSQL::getConcept(uint entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "getConcept" };
    // A simple count won't do because we need the name
    auto result = txn.parameterized("SELECT concept_name FROM concepts WHERE entity_id = $1")(entity_id).exec();
    txn.commit();
    if (!result.empty())
    {
      return Concept{ entity_id, result[0]["concept_name"].as<string>(), *this };
    }
    return {};
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

boost::optional<Map> LongTermMemoryConduitPostgreSQL::getMap(uint entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "getMap" };
    // A simple count won't do because we need the name
    auto result = txn.parameterized("SELECT map_name, map_id FROM maps WHERE entity_id = $1")(entity_id).exec();
    txn.commit();
    if (!result.empty())
    {
      return Map{ entity_id, result[0]["map_id"].as<uint>(), result[0]["map_name"].as<string>(), *this };
    }
    return {};
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

boost::optional<Point> LongTermMemoryConduitPostgreSQL::getPoint(uint entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "getPoint" };
    // A simple count won't do because we need the name
    auto result = txn.parameterized("SELECT point_name, point[0] AS x, point[1] AS y, parent_map_id FROM points WHERE "
                                    "entity_id = $1")(entity_id)
                      .exec();
    txn.commit();
    if (!result.empty())
    {
      auto parent_map = *getMapForMapId(result[0]["parent_map_id"].as<uint>());
      return Point(entity_id, result[0]["point_name"].as<string>(), result[0]["x"].as<double>(),
                   result[0]["y"].as<double>(), parent_map, *this);
    }
    return {};
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

boost::optional<Pose> LongTermMemoryConduitPostgreSQL::getPose(uint entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "getPose" };
    // A simple count won't do because we need the name
    auto result = txn.parameterized("SELECT entity_id, pose_name, parent_map_id, x, y, theta FROM poses_point_angle "
                                    "WHERE entity_id = $1")(entity_id)
                      .exec();
    txn.commit();
    if (!result.empty())
    {
      auto parent_map = *getMapForMapId(result[0]["parent_map_id"].as<uint>());
      return Pose(entity_id, result[0]["pose_name"].as<string>(), result[0]["x"].as<double>(),
                  result[0]["y"].as<double>(), result[0]["theta"].as<double>(), parent_map, *this);
    }
    return {};
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

boost::optional<Region> LongTermMemoryConduitPostgreSQL::getRegion(uint entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "getRegion" };
    string query = "SELECT entity_id, region_name, region, parent_map_id "
                   "FROM regions WHERE entity_id"
                   "= $1";
    auto result = txn.parameterized(query)(entity_id).exec();
    txn.commit();
    if (!result.empty())
    {
      auto region = result[0];
      auto str = region["region"].as<string>();
      const auto points = strToPoints(str);
      auto parent_map = *getMapForMapId(result[0]["parent_map_id"].as<uint>());
      return Region{ region["entity_id"].as<uint>(), region["region_name"].as<string>(), points, parent_map, *this };
    }
    return {};
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

Instance LongTermMemoryConduitPostgreSQL::getRobot()
{
  Instance robot = Instance(1, *this);
  assert(robot.isValid());
  return robot;
}

Entity LongTermMemoryConduitPostgreSQL::addEntity()
{
  pqxx::work txn{ *conn, "addEntity" };

  auto result = txn.exec("INSERT INTO entities VALUES (DEFAULT) RETURNING entity_id");
  txn.commit();
  return { result[0]["entity_id"].as<uint>(), *this };
}

std::vector<Concept> LongTermMemoryConduitPostgreSQL::getAllConcepts()
{
  pqxx::work txn{ *conn, "getAllConcepts" };
  auto result = txn.exec("SELECT entity_id, concept_name FROM concepts");
  txn.commit();
  vector<Concept> concepts;
  for (const auto& row : result)
  {
    concepts.emplace_back(row["entity_id"].as<uint>(), row["concept_name"].as<string>(), *this);
  }
  return concepts;
}

std::vector<Instance> LongTermMemoryConduitPostgreSQL::getAllInstances()
{
  pqxx::work txn{ *conn, "getAllInstances" };
  auto result = txn.exec("SELECT entity_id FROM entities WHERE entity_id NOT IN ("
                         "SELECT entity_id FROM concepts)");
  txn.commit();
  vector<Instance> instances;
  for (const auto& row : result)
  {
    instances.emplace_back(row["entity_id"].as<uint>(), *this);
  }
  return instances;
}

vector<std::pair<string, AttributeValueType>> LongTermMemoryConduitPostgreSQL::getAllAttributes() const
{
  vector<std::pair<string, AttributeValueType>> attribute_names;
  pqxx::work txn{ *conn, "getAllAttributes" };
  auto result = txn.exec("TABLE attributes");
  txn.commit();
  for (const auto& row : result)
  {
    attribute_names.emplace_back(row["attribute_name"].as<string>(),
                                 string_to_attribute_value_type[row["type"].as<string>()]);
  }
  return attribute_names;
}

// MAP
Map LongTermMemoryConduitPostgreSQL::getMap(const std::string& name)
{
  pqxx::work txn{ *conn, "getMap" };
  auto result = txn.parameterized("SELECT entity_id, map_id FROM maps WHERE map_name = $1")(name).exec();
  txn.commit();

  if (result.empty())
  {
    Concept map_concept = getConcept("map");
    // This should succeed because we would've retrieved it above if such an instance existed
    Instance new_map = map_concept.createInstance(name).get();
    pqxx::work txn{ *conn, "getMap" };
    auto result =
        txn.parameterized("INSERT INTO maps VALUES ($1, DEFAULT, $2) RETURNING map_id")(new_map.entity_id)(name).exec();
    txn.commit();
    return { new_map.entity_id, result[0]["map_id"].as<uint>(), name, *this };
  }
  else
  {
    return { result[0]["entity_id"].as<uint>(), result[0]["map_id"].as<uint>(), name, *this };
  }
}

vector<EntityAttribute> LongTermMemoryConduitPostgreSQL::getAllEntityAttributes()
{
  std::vector<EntityAttribute> entity_attrs;
  for (const auto entity : this->getAllEntities())
  {
    auto attrs = entity.getAttributes();
    entity_attrs.insert(entity_attrs.end(), attrs.begin(), attrs.end());
  }
  return entity_attrs;
}

// PROMOTERS

bool LongTermMemoryConduitPostgreSQL::makeConcept(uint id, std::string name)
{
  try
  {
    pqxx::work txn{ *conn, "makeConcept" };
    auto result = txn.parameterized("INSERT INTO concepts VALUES ($1, $2)")(id)(name).exec();
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

// ENTITY BACKERS

bool LongTermMemoryConduitPostgreSQL::deleteEntity(Entity& entity)
{
  // TODO(nickswalker): Recursively remove entities that are members of directional relations
  if (!entity.isValid())
  {
    return false;
  }
  // Because we've all references to this entity have foreign key relationships with cascade set,
  // this should clear out any references to this entity in other tables as well
  try
  {
    pqxx::work txn{ *conn, "deleteEntity" };
    auto result = txn.exec("DELETE FROM entities WHERE entity_id = " + txn.quote(entity.entity_id));
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool LongTermMemoryConduitPostgreSQL::addAttribute(Entity& entity, const std::string& attribute_name,
                                                   const uint other_entity_id)
{
  try
  {
    pqxx::work txn{ *conn, "addAttribute (id)" };
    auto result = txn.exec("INSERT INTO entity_attributes_id VALUES (" + txn.quote(entity.entity_id) + ", " +
                           txn.quote(attribute_name) + ", " + txn.quote(other_entity_id) + ")");
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool LongTermMemoryConduitPostgreSQL::addAttribute(Entity& entity, const std::string& attribute_name,
                                                   const bool bool_val)
{
  try
  {
    pqxx::work txn{ *conn, "addAttribute (bool)" };
    auto result =
        txn.exec("INSERT INTO entity_attributes_bool "
                 "VALUES (" +
                 txn.quote(entity.entity_id) + ", " + txn.quote(attribute_name) + ", " + txn.quote(bool_val) + ")");
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool LongTermMemoryConduitPostgreSQL::addAttribute(Entity& entity, const std::string& attribute_name, const int int_val)
{
  try
  {
    pqxx::work txn{ *conn, "addAttribute (int)" };
    auto result = txn.parameterized("INSERT INTO entity_attributes_int "
                                    "VALUES ($1, $2, $3)")(entity.entity_id)(attribute_name)(int_val)
                      .exec();
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool LongTermMemoryConduitPostgreSQL::addAttribute(Entity& entity, const std::string& attribute_name,
                                                   const double float_val)
{
  try
  {
    pqxx::work txn{ *conn, "addAttribute (float)" };
    auto result = txn.parameterized("INSERT INTO entity_attributes_float "
                                    "VALUES ($1, $2, $3)")(entity.entity_id)(attribute_name)(float_val)
                      .exec();
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool LongTermMemoryConduitPostgreSQL::addAttribute(Entity& entity, const std::string& attribute_name,
                                                   const std::string& string_val)
{
  try
  {
    pqxx::work txn{ *conn, "addAttribute (str)" };
    auto result = txn.parameterized("INSERT INTO entity_attributes_str "
                                    "VALUES ($1, $2, $3)")(entity.entity_id)(attribute_name)(string_val)
                      .exec();
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

int LongTermMemoryConduitPostgreSQL::removeAttribute(Entity& entity, const std::string& attribute_name)
{
  string query;
  pqxx::work txn{ *conn, "removeAttribute" };
  try
  {
    auto result = txn.parameterized("SELECT * FROM remove_attribute"
                                    "($1, $2) AS count")(entity.entity_id)(attribute_name)
                      .exec();
    txn.commit();
    return result[0]["count"].as<int>();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 0;
  }
}

int LongTermMemoryConduitPostgreSQL::removeAttributeOfValue(Entity& entity, const std::string& attribute_name,
                                                            const Entity& other_entity)
{
  assert(false);
}

void unwrap_attribute_rows(const pqxx::result& rows, vector<EntityAttribute>& entity_attributes)
{
  pqxx::oid type = rows[0]["attribute_value"].type();
  if (type == 23)
  {
    for (const auto& row : rows)
    {
      entity_attributes.emplace_back(row["entity_id"].as<uint>(), row["attribute_name"].as<string>(),
                                     row["attribute_value"].as<int>());
    }
  }
  else if (type == 1043)
  {
    for (const auto& row : rows)
    {
      entity_attributes.emplace_back(row["entity_id"].as<uint>(), row["attribute_name"].as<string>(),
                                     row["attribute_value"].as<string>());
    }
  }
  else if (type == 701)
  {
    for (const auto& row : rows)
    {
      entity_attributes.emplace_back(row["entity_id"].as<uint>(), row["attribute_name"].as<string>(),
                                     row["attribute_value"].as<double>());
    }
  }
  else if (type == 16)
  {
    for (const auto& row : rows)
    {
      entity_attributes.emplace_back(row["entity_id"].as<uint>(), row["attribute_name"].as<string>(),
                                     row["attribute_value"].as<bool>());
    }
  }
  else
  {
    assert(false);
  }
}

vector<EntityAttribute> LongTermMemoryConduitPostgreSQL::getAttributes(const Entity& entity) const
{
  vector<EntityAttribute> attributes;
  for (const auto& name : TABLE_NAMES)
  {
    try
    {
      pqxx::work txn{ *conn, "getAttributes" };
      auto result =
          txn.parameterized("SELECT * FROM " + std::string(name) + " WHERE entity_id = $1")(entity.entity_id).exec();
      txn.commit();
      unwrap_attribute_rows(result, attributes);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return {};
    }
  }
  return attributes;
}

std::vector<EntityAttribute> LongTermMemoryConduitPostgreSQL::getAttributes(const Entity& entity,
                                                                            const std::string& attribute_name) const
{
  vector<EntityAttribute> attributes;
  for (const auto& name : TABLE_NAMES)
  {
    try
    {
      pqxx::work txn{ *conn, "getAttributes" };
      auto result = txn.parameterized("SELECT * FROM " + std::string(name) +
                                      " WHERE entity_id = $1 AND attribute_name = $2")(entity.entity_id)(attribute_name)
                        .exec();
      txn.commit();
      unwrap_attribute_rows(result, attributes);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return {};
    }
  }
  return attributes;
}

bool LongTermMemoryConduitPostgreSQL::isValid(const Entity& entity) const
{
  return entityExists(entity.entity_id);
}

// INSTANCE BACKERS

std::vector<Concept> LongTermMemoryConduitPostgreSQL::getConcepts(const Instance& instance)
{
  try
  {
    pqxx::work txn{ *conn, "getConcepts" };
    auto result = txn.parameterized("SELECT concepts.entity_id, concepts.concept_name FROM instance_of "
                                    "INNER JOIN concepts ON concepts.concept_name = instance_of.concept_name "
                                    "WHERE instance_of.entity_id = $1")(instance.entity_id)
                      .exec();
    txn.commit();
    std::vector<Concept> concepts{};
    for (const auto& row : result)
    {
      concepts.emplace_back(row["entity_id"].as<uint>(), row["concept_name"].as<string>(), *this);
    }
    return concepts;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

std::vector<Concept> LongTermMemoryConduitPostgreSQL::getConceptsRecursive(const Instance& instance)
{
  try
  {
    pqxx::work txn{ *conn, "getConceptsRecursive" };
    auto result = txn.parameterized("SELECT * FROM get_concepts_recursive($1)")(instance.entity_id).exec();
    txn.commit();
    std::vector<Concept> concepts{};
    for (const auto& row : result)
    {
      concepts.emplace_back(row["entity_id"].as<uint>(), row["concept_name"].as<string>(), *this);
    }
    return concepts;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

bool LongTermMemoryConduitPostgreSQL::makeInstanceOf(Instance& instance, const Concept& concept)
{
  try
  {
    pqxx::work txn{ *conn, "makeInstanceOf" };
    auto result =
        txn.parameterized("INSERT INTO instance_of VALUES ($1,$2) ")(instance.entity_id)(concept.getName()).exec();
    txn.commit();
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

// CONCEPT BACKERS

vector<Concept> LongTermMemoryConduitPostgreSQL::getChildren(const Concept& concept)
{
  try
  {
    pqxx::work txn{ *conn, "getChildren" };
    auto result = txn.parameterized("SELECT concepts.entity_id, concept_name FROM entity_attributes_id eai"
                                    " INNER JOIN concepts ON eai.entity_id = concepts.entity_id WHERE attribute_name = "
                                    "'is_a' AND attribute_value = $1")(concept.entity_id)
                      .exec();
    txn.commit();
    std::vector<Concept> concepts{};
    for (const auto& row : result)
    {
      concepts.emplace_back(row["entity_id"].as<uint>(), row["concept_name"].as<string>(), *this);
    }
    return concepts;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

vector<Concept> LongTermMemoryConduitPostgreSQL::getChildrenRecursive(const Concept& concept)
{
  try
  {
    pqxx::work txn{ *conn, "getChildrenRecursive" };
    auto result = txn.parameterized("SELECT * FROM get_all_concept_descendants($1)")(concept.entity_id).exec();
    txn.commit();
    std::vector<Concept> concepts{};
    for (const auto& row : result)
    {
      concepts.emplace_back(row["entity_id"].as<uint>(), row["concept_name"].as<string>(), *this);
    }
    return concepts;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

std::vector<Instance> LongTermMemoryConduitPostgreSQL::getInstances(const ConceptImpl& concept)
{
  try
  {
    pqxx::work txn{ *conn, "getInstances" };
    auto result =
        txn.parameterized("SELECT entity_id FROM instance_of WHERE concept_name = $1")(concept.getName()).exec();
    txn.commit();
    std::vector<Instance> instances{};
    for (const auto& row : result)
    {
      instances.emplace_back(row["entity_id"].as<uint>(), *this);
    }
    return instances;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return {};
  }
}

int LongTermMemoryConduitPostgreSQL::removeInstances(const Concept& concept)
{
  pqxx::work txn{ *conn, "removeInstances" };
  auto result = txn.parameterized("DELETE FROM entities WHERE entity_id IN "
                                  "(SELECT entity_id FROM instance_of WHERE concept_name = $1)")(concept.getName())
                    .exec();
  txn.commit();
  return result.affected_rows();
}

int LongTermMemoryConduitPostgreSQL::removeInstancesRecursive(const Concept& concept)
{
  pqxx::work txn{ *conn, "removeInstancesRecursive" };
  auto result =
      txn.parameterized("DELETE FROM entities WHERE entity_id IN "
                        "(SELECT entity_id FROM get_all_instances_of_concept_recursive($1))")(concept.entity_id)
          .exec();
  txn.commit();
  return result.affected_rows();
}

// MAP BACKERS
Point LongTermMemoryConduitPostgreSQL::addPoint(Map& map, const std::string& name, double x, double y)
{
  auto point_entity = addEntity();
  map.addAttribute("has", point_entity);
  pqxx::work txn{ *conn, "addPoint" };
  auto result = txn.parameterized("INSERT INTO points VALUES ($1, $2, $3, point($4 ,$5)) RETURNING entity_id")(
                       point_entity.entity_id)(name)(map.getId())(x)(y)
                    .exec();

  txn.parameterized("INSERT INTO instance_of VALUES ($1,$2)")(point_entity.entity_id)("point").exec();
  txn.commit();
  point_entity.addAttribute("name", name);
  return { point_entity.entity_id, name, x, y, map, *this };
}

Pose LongTermMemoryConduitPostgreSQL::addPose(Map& map, const string& name, double x, double y, double theta)
{
  auto pose_entity = addEntity();
  map.addAttribute("has", pose_entity);
  pqxx::work txn{ *conn, "addPose" };
  auto result =
      txn.parameterized("INSERT INTO poses VALUES ($1, $2, $3, lseg(point($4, $5), point($4+COS($6),$5+SIN($6))))")(
             pose_entity.entity_id)(name)(map.getId())(x)(y)(theta)
          .exec();
  txn.parameterized("INSERT INTO instance_of VALUES ($1,$2)")(pose_entity.entity_id)("pose").exec();
  txn.commit();
  pose_entity.addAttribute("name", name);
  return { pose_entity.entity_id, name, x, y, theta, map, *this };
}

Region LongTermMemoryConduitPostgreSQL::addRegion(Map& map, const string& name, const vector<Region::Point2D>& points)
{
  auto region_entity = addEntity();
  map.addAttribute("has", region_entity);
  std::ostringstream points_stream;
  points_stream << "(";
  for (const auto& point : points)
  {
    points_stream << "(" << point.first << "," << point.second << "),";
  }
  points_stream.seekp(-1, points_stream.cur) << ")";

  pqxx::work txn{ *conn, "addRegion" };
  auto result = txn.parameterized("INSERT INTO regions VALUES ($1, $2, $3, $4)")(region_entity.entity_id)(name)(
                       map.getId())(points_stream.str())
                    .exec();
  txn.parameterized("INSERT INTO instance_of VALUES ($1,$2)")(region_entity.entity_id)("region").exec();
  txn.commit();
  region_entity.addAttribute("name", name);
  return { region_entity.entity_id, name, points, map, *this };
}

boost::optional<Point> LongTermMemoryConduitPostgreSQL::getPoint(Map& map, const string& name)
{
  pqxx::work txn{ *conn, "getPoint" };

  auto q_result = txn.parameterized("SELECT entity_id, point[0] AS x, point[1] AS y FROM points WHERE parent_map_id "
                                    "= $1 AND point_name = $2")(map.getId())(name)
                      .exec();
  txn.commit();
  assert(q_result.size() <= 1);
  if (q_result.size() == 1)
  {
    auto point = q_result[0];
    return Point{ point["entity_id"].as<uint>(), name, point["x"].as<double>(), point["y"].as<double>(), map, *this };
  }
  return {};
}

boost::optional<Pose> LongTermMemoryConduitPostgreSQL::getPose(Map& map, const string& name)
{
  pqxx::work txn{ *conn, "getPose" };
  string query = "SELECT entity_id, x, y, theta FROM poses_point_angle WHERE parent_map_id "
                 "= $1 AND pose_name = $2";

  auto q_result = txn.parameterized(query)(map.getId())(name).exec();
  txn.commit();
  assert(q_result.size() <= 1);
  if (q_result.size() == 1)
  {
    auto pose = q_result[0];
    return Pose{ pose["entity_id"].as<uint>(),
                 name,
                 pose["x"].as<double>(),
                 pose["y"].as<double>(),
                 pose["theta"].as<double>(),
                 map,
                 *this };
  }
  return {};
}

boost::optional<Region> LongTermMemoryConduitPostgreSQL::getRegion(Map& map, const string& name)
{
  pqxx::work txn{ *conn, "getRegion" };
  string query = "SELECT entity_id, region, region_name "
                 "FROM regions WHERE parent_map_id "
                 "= $1 AND region_name = $2";
  auto q_result = txn.parameterized(query)(map.getId())(name).exec();
  txn.commit();
  assert(q_result.size() <= 1);
  if (q_result.size() == 1)
  {
    auto region = q_result[0];
    auto str = region["region"].as<string>();
    const auto points = strToPoints(str);
    return Region{ region["entity_id"].as<uint>(), name, points, map, *this };
  }
  return {};
}

vector<Point> LongTermMemoryConduitPostgreSQL::getAllPoints(Map& map)
{
  pqxx::work txn{ *conn, "getAllPoints" };
  auto q_result = txn.parameterized("SELECT entity_id, point[0] AS x, point[1] AS y, point_name FROM points WHERE "
                                    "parent_map_id = $1")(map.getId())
                      .exec();
  txn.commit();
  vector<Point> points;
  for (const auto& row : q_result)
  {
    points.emplace_back(row["entity_id"].as<uint>(), row["point_name"].as<string>(), row["x"].as<double>(),
                        row["y"].as<double>(), map, *this);
  }
  return points;
}

vector<Pose> LongTermMemoryConduitPostgreSQL::getAllPoses(Map& map)
{
  pqxx::work txn{ *conn, "getAllPoses" };

  auto q_result =
      txn.parameterized("SELECT entity_id, x, y, theta, pose_name FROM poses_point_angle WHERE parent_map_id = $1")(
             map.getId())
          .exec();
  txn.commit();
  vector<Pose> poses;
  for (const auto& row : q_result)
  {
    poses.emplace_back(row["entity_id"].as<uint>(), row["pose_name"].as<string>(), row["x"].as<double>(),
                       row["y"].as<double>(), row["theta"].as<double>(), map, *this);
  }
  return poses;
}

vector<Region> LongTermMemoryConduitPostgreSQL::getAllRegions(Map& map)
{
  pqxx::work txn{ *conn, "getAllPoses" };
  string query = "SELECT entity_id, region, region_name FROM regions WHERE parent_map_id = $1";

  auto q_result = txn.parameterized(query)(map.getId()).exec();
  txn.commit();
  vector<Region> regions;
  for (const auto& row : q_result)
  {
    auto str = row["region"].as<string>();
    const auto points = strToPoints(str);
    regions.emplace_back(row["entity_id"].as<uint>(), row["region_name"].as<string>(), points, map, *this);
  }
  return regions;
}

std::vector<Region> LongTermMemoryConduitPostgreSQL::getContainingRegions(Map& map, std::pair<double, double> point)
{
  pqxx::work txn{ *conn, "getContainingRegions" };
  auto result = txn.parameterized("SELECT entity_id, region, region_name FROM regions WHERE parent_map_id = $1 AND "
                                  "region <@ point($2,$3)")(map.map_id)(point.first)(point.second)
                    .exec();
  txn.commit();
  vector<Region> regions;
  for (const auto& row : result)
  {
    auto str = row["region"].as<string>();
    const auto points = strToPoints(str);
    regions.emplace_back(row["entity_id"].as<uint>(), row["region_name"].as<string>(), points, map, *this);
  }
  return regions;
}

bool LongTermMemoryConduitPostgreSQL::renameMap(Map& map, const std::string& new_name)
{
  try
  {
    pqxx::work txn{ *conn, "renameMap" };
    auto result =
        txn.parameterized("UPDATE maps SET map_name = $1 WHERE map_name = $2")(new_name)(map.getName()).exec();
    txn.commit();
    if (result.affected_rows() == 1)
    {
      map.removeAttribute("name");
      map.addAttribute("name", new_name);
    }
    return result.affected_rows() == 1;
  }
  catch (const std::exception& e)
  {
    // Likely a naming collision. In any case, we didn't rename, so return false
    return false;
  }
}

// REGION BACKERS

vector<Point> LongTermMemoryConduitPostgreSQL::getContainedPoints(Region& region)
{
  pqxx::work txn{ *conn, "getContainedPoints" };

  auto result =
      txn.parameterized("SELECT entity_id, point_name, point FROM points WHERE parent_map_id = $1 AND (SELECT region "
                        "FROM regions WHERE entity_id = $2) <@ point")(region.parent_map.map_id)(region.entity_id)
          .exec();
  txn.commit();
  vector<Point> points;
  for (const auto& row : result)
  {
    points.emplace_back(row["entity_id"].as<uint>(), row["point_name"].as<string>(), row["x"].as<double>(),
                        row["y"].as<double>(), region.parent_map, *this);
  }
  return points;
}

vector<Pose> LongTermMemoryConduitPostgreSQL::getContainedPoses(Region& region)
{
  pqxx::work txn{ *conn, "getContainedPoses" };

  string query = "SELECT entity_id, x, y, theta, pose_name FROM poses_point_angle"
                 "WHERE parent_map_id "
                 "= $1 AND (SELECT region FROM regions WHERE entity_id = $2) <@ pose[0]) AS dummy_sub_alias";
  auto result = txn.parameterized(query)(region.parent_map.map_id)(region.entity_id).exec();
  txn.commit();
  vector<Pose> poses;
  for (const auto& row : result)
  {
    poses.emplace_back(row["entity_id"].as<uint>(), row["pose_name"].as<string>(), row["x"].as<double>(),
                       row["y"].as<double>(), row["theta"].as<double>(), region.parent_map, *this);
  }
  return poses;
}

bool LongTermMemoryConduitPostgreSQL::isPointContained(const Region& region, std::pair<double, double> point)
{
  pqxx::work txn{ *conn, "isPointContained" };
  auto result = txn.parameterized("SELECT count(*) FROM regions WHERE entity_id = $1 AND region <@ point($2,$3)")(
                       region.entity_id)(point.first)(point.second)
                    .exec();
  txn.commit();
  return result[0]["count"].as<uint>() == 1;
}

boost::optional<Map> LongTermMemoryConduitPostgreSQL::getMapForMapId(uint map_id)
{
  pqxx::work txn{ *conn, "getMapForId" };
  auto result = txn.parameterized("SELECT entity_id, map_name FROM maps WHERE map_id= $1")(map_id).exec();
  txn.commit();
  if (result.size() == 1)
  {
    return Map(result[0]["entity_id"].as<uint>(), map_id, result[0]["map_name"].as<string>(), *this);
  }
  return {};
}

}  // namespace knowledge_rep
