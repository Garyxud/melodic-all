#pragma once
#include <string>
#include <boost/variant.hpp>
#include <boost/unordered_map.hpp>
#include <utility>
#include <map>

namespace knowledge_rep
{
enum AttributeValueType
{
  Id,
  Bool,
  Int,
  Float,
  Str
};

static std::map<std::string, AttributeValueType> string_to_attribute_value_type = { { "id", Id },
                                                                                    { "bool", Bool },
                                                                                    { "int", Int },
                                                                                    { "float", Float },
                                                                                    { "str", Str } };

static std::map<AttributeValueType, std::string> attribute_value_type_to_string = {
  { Id, "id" }, { Bool, "bool" }, { Int, "int" }, { Float, "float" }, { Str, "str" },
};
typedef boost::variant<uint, bool, int, double, std::string> AttributeValue;

static std::string toString(const AttributeValue& attribute_value)
{
  switch (attribute_value.which())
  {
    case 0:
      return std::to_string(boost::get<uint>(attribute_value));
    case 1:
      return std::to_string(boost::get<bool>(attribute_value));
    case 2:
      return std::to_string(boost::get<int>(attribute_value));
    case 3:
      return std::to_string(boost::get<double>(attribute_value));
    case 4:
      return boost::get<std::string>(attribute_value);
    default:
      assert(false);
  }
}

/**
 * @brief A tuple of an entity id, an attribute name, and a value.
 */
struct EntityAttribute
{
  uint entity_id;
  std::string attribute_name;
  AttributeValue value;

  EntityAttribute(int entity_id, std::string attribute_name, AttributeValue value)
    : entity_id(entity_id), attribute_name(std::move(attribute_name)), value(std::move(value))
  {
  }

  /**
   * @brief Get a copy of the value
   *
   * Primarily a helper for the Python API
   * @return
   */
  AttributeValue getValue()
  {
    return value;
  }

  /**
* @brief Extracts an ID (uint) from the value
*
* Throws an exception if the value isn't of the expected type.
* @return
*/
  uint getIdValue() const
  {
    return boost::get<uint>(value);
  }

  /**
 * @brief Extracts a bool from the value
 *
 * Throws an exception if the value isn't of the expected type.
 * @return
 */
  bool getBoolValue() const
  {
    return boost::get<bool>(value);
  }

  /**
   * @brief Extracts an int from the value
   *
   * Throws an exception if the value isn't of the expected type.
   * @return
   */
  int getIntValue() const
  {
    return boost::get<int>(value);
  }

  /**
   * @brief Extracts a float from the value
   *
   * Throws an exception if the value isn't of the expected type.
   * @return
   */
  double getFloatValue() const
  {
    return boost::get<double>(value);
  }

  /**
   * @brief Extracts a string from the value
   *
   * Throws an exception if the value isn't of the expected type.
   * @return
   */
  std::string getStringValue() const
  {
    return boost::get<std::string>(value);
  }

  bool operator==(const EntityAttribute& other)
  {
    return (this->entity_id == other.entity_id) && (this->attribute_name == other.attribute_name) &&
           (this->value == other.value);
  }
};
}  // namespace knowledge_rep
