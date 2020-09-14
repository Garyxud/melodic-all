#pragma once

#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>

namespace knowledge_rep
{
/// @brief A LTMCEntity representing a Concept; the abstract idea of something
template <typename LTMCImpl>
class LTMCConcept : public LTMCEntity<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  std::string name;

public:
  LTMCConcept(uint entity_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : name(std::move(name)), EntityImpl(entity_id, ltmc)
  {
  }

  /**
   * @name Gets the name of the concept
   *
   * All concepts must have a name, as we don't expect to deal with ideas that don't at least have a name to ground them
   * @return
   */
  std::string getName() const
  {
    return name;
  }
  /**
   * @brief Get entities that directly instantiate this concept
   * @return a list of all entities that are instances of this concept
   */
  std::vector<InstanceImpl> getInstances() const
  {
    return this->ltmc.get().getInstances(*this);
  }

  /**
   * @brief Remove all entities that are directly an instance of this concept
   *
   * @return the number of instances removed
   */
  int removeInstances() const
  {
    return this->ltmc.get().removeInstances(*this);
  }

  /**
   * @brief Recursively remove instances of the concept
   *
   * An instance of a given concept "C" is transitively an instance of any concept that that defines an "is_a" relation
   * with "C". For example, if the concept of apple "is_a" concept of fruit, then removing instances of fruit will
   * remove all instances of apple.
   * @return the number of instances removed
   */
  int removeInstancesRecursive() const
  {
    return this->ltmc.get().removeInstancesRecursive(*this);
  }

  /**
   * @brief Create a fresh instance of this concept
   * @return a new entity that is an instance of this concept
   */
  InstanceImpl createInstance() const
  {
    auto fresh_entity = this->ltmc.get().addEntity();
    InstanceImpl as_instance = { fresh_entity.entity_id, this->ltmc.get() };
    this->ltmc.get().makeInstanceOf(as_instance, *this);
    return as_instance;
  }

  /**
   * @brief Create an instance of the concept and give it a particular name
   * @param name the name to give the instance
   * @return the created instance, or empty if the instance cannot be created (as when the name is taken)
   */
  boost::optional<InstanceImpl> createInstance(const std::string& name) const
  {
    auto instance = createInstance();
    bool added_name = instance.addAttribute("name", name);
    if (!added_name)
    {
      instance.deleteEntity();
      return {};
    }
    return instance;
  }

  /**
   * @brief Get all concepts that are direct children of this concept
   *
   * A child is a concept that has an `is_a` attribute pointing at this concept.
   * @return all direct childern
   */
  std::vector<LTMCConcept> getChildren() const
  {
    return this->ltmc.get().getChildren(*this);
  }

  /**
   * @brief Get all concepts that are descended from this concept
   *
   * A child is a concept that has an `is_a` attribute pointing at this concept.
   * @return
   */
  std::vector<LTMCConcept> getChildrenRecursive() const
  {
    return this->ltmc.get().getChildrenRecursive(*this);
  }

  /**
   * @brief Get the instance of a particular name
   * @param name the name of the instance
   * @return the instance, or empty if the instance does not exist
   */
  boost::optional<InstanceImpl> getInstanceNamed(const std::string& name) const
  {
    return this->ltmc.get().getInstanceNamed(*this, name);
  }

  /**
   * @brief Removes all references to a concept
   *
   * References include all entity-attributes that refer to this concept. But note that this method doesn't
   * delete the entities that are the subjects of the references. This means that all instances of this
   * concept will continue to exist, but they will no longer be identified as instances of this concept.
   * @return
   */
  bool removeReferences()
  {
    // Rely on the schema to clear out the childern via cascading delete
    this->deleteEntity();
    {
      // Recreate it with the same ID
      auto new_entity = this->ltmc.get().getEntity(this->entity_id);
      if (!new_entity)
      {
        return false;
      }
    }

    // The current entity should still be valid (because we recreated it), so make it a concept again
    this->ltmc.get().makeConcept(this->entity_id, name);
    return true;
  }

  bool operator==(const LTMCConcept& other) const
  {
    return this->entity_id == other.entity_id && this->name == other.name;
  }

  bool operator!=(const LTMCConcept& other) const
  {
    return this->entity_id != other.entity_id || this->name != other.name;
  }

  bool isValid()
  {
    // Check whether LTMC still knows that this ID belongs to some concept
    auto retrieved = this->ltmc.get().getConcept(this->entity_id);
    if (retrieved)
    {
      // Name could've changed if ID was reassigned to different concept. Check sameness
      return *this == *retrieved;
    }
    return false;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCConcept<LTMCImpl>& c)
{
  return strm << "Concept(" << c.entity_id << " \"" << c.getName() << "\")";
}

}  // namespace knowledge_rep
