#pragma once
#include <utility>
#include <vector>
#include <string>

#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCEntity.h>

namespace knowledge_rep
{
/// \brief Represents an Entity that is an Instance of some Concept.

/// Instances are like regular entities, but they have
/// at least one Concept which they are directly an instance of.
template <typename LTMCImpl>
class LTMCInstance : public LTMCEntity<LTMCImpl>
{
protected:
  std::string name;

public:
  /**
   * @brief Build an instance with a given name
   *
   * Note that no validation is performed on construction; the name and entity id are presumed
   * to exist, and the behavior is undefined if they do not.
   * @param entity_id
   * @param name
   * @param ltmc
   */
  LTMCInstance(uint entity_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : name(std::move(name)), LTMCEntity<LTMCImpl>(entity_id, ltmc)
  {
  }

  LTMCInstance(uint entity_id, LongTermMemoryConduitInterface<LTMCImpl>& ltmc) : LTMCEntity<LTMCImpl>(entity_id, ltmc)
  {
  }

  /**
   * @brief Convenience for getting name attribute
   *
   * Caching is provided for name since it's so commonly used. Note that behavior
   * is undefined if more than one name attribute is set on the instance.
   * @return name, if it is set
   */
  boost::optional<std::string> getName()
  {
    if (!name.empty())
    {
      return name;
    }
    // There should only be one
    auto name_attrs = this->ltmc.get().getAttributes(*this, "name");
    if (!name_attrs.empty())
    {
      name = boost::get<std::string>(name_attrs[0].value);
      return name;
    }
    return {};
  };

  /**
   * @brief Attempts to make this instance also an instance of the given concept
   *
   * Fails if this instance is already an instance of the concept.
   * @param concept The concept to make this entity an instance of
   * @return whether this instance was newly made an instance of the given concept
   */
  bool makeInstanceOf(const LTMCConcept<LTMCImpl>& concept)
  {
    return this->ltmc.get().makeInstanceOf(*this, concept);
  }

  /**
   * @brief Get all concepts that this instance is transitively an instance of
   *
   * For example, if an entity A "instance_of" concept named apple, and apple "is_a" concept of fruit,
   * then getConcepts will return the concepts of both apple and fruit.
   * @return
   */
  std::vector<LTMCConcept<LTMCImpl>> getConcepts() const
  {
    return this->ltmc.get().getConcepts(*this);
  }

  /**
   * @brief Get all concepts that this instance is transitively an instance of
   *
   * For example, if an entity A "instance_of" concept named apple, and apple "is_a" concept of fruit,
   * then getConcepts will return the concepts of both apple and fruit.
   * @return
   */
  std::vector<LTMCConcept<LTMCImpl>> getConceptsRecursive() const
  {
    return this->ltmc.get().getConceptsRecursive(*this);
  }
  /**
 * @brief whether instance descends from the concept recursively
 * @return
 */
  bool hasConcept(const LTMCConcept<LTMCImpl>& concept) const
  {
    auto concepts = this->getConcepts();
    return std::find(concepts.begin(), concepts.end(), concept) != concepts.end();
  }
  /**
   * @brief whether instance descends from the concept recursively
   * @return
   */
  bool hasConceptRecursively(const LTMCConcept<LTMCImpl>& concept) const
  {
    auto concepts = this->getConceptsRecursive();
    return std::find(concepts.begin(), concepts.end(), concept) != concepts.end();
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCInstance<LTMCImpl>& c)
{
  strm << "Instance(" << c.entity_id;
  // Get the name if it exists. Work around for non-const getName method
  const auto name = c.getAttributes("name");
  if (!name.empty())
  {
    strm << " \"" << name[0].value << "\": ";
  }
  else
  {
    strm << ": ";
  }
  for (const auto& parent_concept : c.getConcepts())
  {
    strm << "\"" << parent_concept.getName() << "\" ";
  }
  return strm << ")";
}

}  // namespace knowledge_rep
