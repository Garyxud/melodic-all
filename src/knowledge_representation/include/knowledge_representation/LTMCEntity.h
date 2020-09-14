#pragma once
#include <string>
#include <vector>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>

namespace knowledge_rep
{
template <typename LTMCImpl>

/// \brief Represents an Entity in the knowledgebase.
class LTMCEntity
{
public:
  using EntityId = uint;
  EntityId entity_id;

  LTMCEntity(EntityId entity_id, LongTermMemoryConduitInterface<LTMCImpl>& ltmc) : entity_id(entity_id), ltmc(ltmc)
  {
  }

  /**
   * @brief Set an attribute which points to some other valid entity
   * @param attribute_name
   * @param other_entity_id
   * @return whether the modification succeeded
   */
  bool addAttribute(const std::string& attribute_name, EntityId other_entity_id)
  {
    return ltmc.get().addAttribute(*this, attribute_name, other_entity_id);
  };

  /**
   * @brief Set an attribute which points to some other valid entity
   * @param attribute_name
   * @param other_entity
   * @return whether the modification succeeded
   */
  bool addAttribute(const std::string& attribute_name, const LTMCEntity& other_entity)
  {
    return ltmc.get().addAttribute(*this, attribute_name, other_entity.entity_id);
  };

  /**
   * @brief Set a bool-valued attribute on this entity
   * @param attribute_name
   * @param bool_val
   * @return whether the modification succeeded
   */
  bool addAttribute(const std::string& attribute_name, bool bool_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, bool_val);
  };

  /**
 * @brief Set a int-valued attribute on this entity
 * @param attribute_name
 * @param int_val
 * @return whether the modification succeeded
 */
  bool addAttribute(const std::string& attribute_name, int int_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, static_cast<int>(int_val));
  };

  /**
   * @brief Set a float-valued attribute on this entity
   * @param attribute_name
   * @param float_val
   * @return whether the modification succeeded
   */
  bool addAttribute(const std::string& attribute_name, double float_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, float_val);
  };

  /**
 * @brief Set a string-valued attribute on this entity
 * @param attribute_name
 * @param string_val
 * @return whether the modification succeeded
 */
  bool addAttribute(const std::string& attribute_name, const std::string& string_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, string_val);
  };

  /**
   * @brief Set a string-valued attribute on this entity
   * @param attribute_name
   * @param string_val
   * @return whether the modification succeeded
   */
  bool addAttribute(const std::string& attribute_name, const char* string_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, std::string(string_val));
  };

  /**
   * @brief Unsets all values of a given attribute on this entity
   * @param attribute_name
   * @return the number of attribute values removed
   */
  int removeAttribute(const std::string& attribute_name)
  {
    return ltmc.get().removeAttribute(*this, attribute_name);
  };

  // TODO(nickswalker): Decide whether to remove this
  int removeAttributeOfValue(const std::string& attribute_name, const LTMCEntity& other_entity)
  {
    return ltmc.get().removeAttributeOfValue(*this, attribute_name, other_entity);
  };

  /**
   * @brief Get the name and value of all attributes set for this entity
   * @return a list of attributes and their values
   */
  std::vector<EntityAttribute> getAttributes() const
  {
    return ltmc.get().getAttributes(*this);
  };

  /**
   * @brief Get all values for an attribute of a given name
   * @param attribute_name
   * @return a list of attributes and their values
   */
  std::vector<EntityAttribute> getAttributes(const std::string& attribute_name) const
  {
    return ltmc.get().getAttributes(*this, attribute_name);
  };

  /**
   * @brief Get the name of an entity, if it has one
   * Not all entities have names, but some specific types of entities (e.g. LTMCConcept, LTMCMap, and other geometry
   * types)
   * _must_ have names). These subclasses will override this method to return a non-optional string.
   * @return the name, if it exists
   */
  boost::optional<std::string> getName()
  {
    // There should only be one
    auto name_attrs = getAttributes("name");
    if (!name_attrs.empty())
    {
      return boost::get<std::string>(name_attrs[0].value);
    }
    else
    {
      return {};
    }
  };

  /**
   * @brief Deletes an entity and any other entities and relations that rely on it.
   * @return true if the entity was deleted. False if it could not be, or already was
   */
  bool deleteEntity()
  {
    return ltmc.get().deleteEntity(*this);
  };

  /**
   * @brief Verifies that the entity still exists in the LTMC
   * An entity can be invalidated if it is explicitly deleted from the LTMC, or if
   * it is removed as a result of other helpers that delete entities.
   * @return whether the entity is valid
   */
  bool isValid() const
  {
    return ltmc.get().isValid(*this);
  };

  bool operator==(const LTMCEntity& other) const
  {
    return this->entity_id == other.entity_id;
  }

  bool operator!=(const LTMCEntity& other) const
  {
    return this->entity_id != other.entity_id;
  }

  std::vector<EntityAttribute> operator[](const std::string& attr_name) const
  {
    return getAttributes(attr_name);
  };

protected:
  std::reference_wrapper<LongTermMemoryConduitInterface<LTMCImpl>> ltmc;
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCEntity<LTMCImpl>& e)
{
  return strm << "Entity(" << e.entity_id << ")";
}

}  // namespace knowledge_rep
