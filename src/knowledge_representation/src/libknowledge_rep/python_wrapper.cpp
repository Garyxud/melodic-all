#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/noncopyable.hpp>
#include <boost/mpl/if.hpp>
#include <boost/optional.hpp>
#include <boost/type_traits/integral_constant.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <knowledge_representation/LTMCPose.h>
#include <knowledge_representation/LTMCPoint.h>
#include <knowledge_representation/LTMCRegion.h>
#include <knowledge_representation/convenience.h>
#include <vector>
#include <string>
#include <utility>

namespace python = boost::python;
using boost::optional;
using knowledge_rep::AttributeValue;
using knowledge_rep::AttributeValueType;
using knowledge_rep::Concept;
using knowledge_rep::Entity;
using knowledge_rep::EntityAttribute;
using knowledge_rep::Instance;
using knowledge_rep::LongTermMemoryConduit;
using knowledge_rep::Map;
using knowledge_rep::Point;
using knowledge_rep::Pose;
using knowledge_rep::Region;
using python::bases;
using python::class_;
using python::enum_;
using python::init;
using python::tuple;
using python::vector_indexing_suite;
using std::string;
using std::vector;

namespace detail
{
/// @brief Type trait that determines if the provided type is
///        a boost::optional.
template <typename T>
struct IsOptional : boost::false_type
{
};

template <typename T>
struct IsOptional<boost::optional<T>> : boost::true_type
{
};

/// @brief Type used to provide meaningful compiler errors.
template <typename>
struct ReturnOptionalRequiresAOptionalReturnType
{
};

/// @brief ResultConverter model that converts a boost::optional object to
///        Python None if the object is empty (i.e. boost::none) or defers
///        to Boost.Python to convert object to a Python object.
template <typename T>
struct ToPythonOptional
{
  /// @brief Only supports converting Boost.Optional types.
  /// @note This is checked at runtime.
  bool convertible() const
  {
    return detail::IsOptional<T>::value;
  }

  /// @brief Convert boost::optional object to Python None or a
  ///        Boost.Python object.
  PyObject* operator()(const T& obj) const
  {
    namespace python = boost::python;
    python::object result = obj  // If boost::optional has a value, then
                                ?
                                python::object(*obj)  // defer to Boost.Python converter.
                                :
                                python::object();  // Otherwise, return Python None.

    // The python::object contains a handle which functions as
    // smart-pointer to the underlying PyObject.  As it will go
    // out of scope, explicitly increment the PyObject's reference
    // count, as the caller expects a non-borrowed (i.e. owned) reference.
    return python::incref(result.ptr());
  }

  /// @brief Used for documentation.
  const PyTypeObject* get_pytype() const  // NOLINT
  {
    return nullptr;
  }
};

}  // namespace detail

/// @brief Converts a boost::optional to Python None if the object is
///        equal to boost::none.  Otherwise, defers to the registered
///        type converter to returs a Boost.Python object.
struct ReturnOptional
{
  template <class T>
  struct apply  //  NOLINT
  {
    // The to_python_optional ResultConverter only checks if T is convertible
    // at runtime.  However, the following MPL branch cause a compile time
    // error if T is not a boost::optional by providing a type that is not a
    // ResultConverter model.
    typedef typename boost::mpl::if_<detail::IsOptional<T>, detail::ToPythonOptional<T>,
                                     detail::ReturnOptionalRequiresAOptionalReturnType<T>>::type type;
  };  // apply
};    // return_optional

template <typename T1, typename T2>
struct PairToPythonConverter
{
  static PyObject* convert(const std::pair<T1, T2>& pair)
  {
    return boost::python::incref(boost::python::make_tuple(pair.first, pair.second).ptr());
  }
};

template <typename T1, typename T2>
struct PythonToPairConverter
{
  using pair_type = std::pair<T1, T2>;
  PythonToPairConverter()
  {
    boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<pair_type>());
  }
  static void* convertible(PyObject* obj)
  {
    if (!PyTuple_CheckExact(obj))
      return 0;
    if (PyTuple_Size(obj) != 2)
      return 0;
    return obj;
  }
  static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    tuple tuple(boost::python::borrowed(obj));
    void* storage = ((boost::python::converter::rvalue_from_python_storage<pair_type>*)data)->storage.bytes;
    new (storage) pair_type(boost::python::extract<T1>(tuple[0]), boost::python::extract<T2>(tuple[1]));
    data->convertible = storage;
  }
};

template <typename T1, typename T2>
struct py_pair
{
  boost::python::to_python_converter<std::pair<T1, T2>, PairToPythonConverter<T1, T2>> toPy;
  PythonToPairConverter<T1, T2> fromPy;
};

/// @brief Type that allows for registration of conversions from
///        python iterable types.
struct iterable_converter
{
  /// @note Registers converter from a python interable type to the
  ///       provided type.
  template <typename Container>
  iterable_converter& from_python()
  {
    boost::python::converter::registry::push_back(&iterable_converter::convertible,
                                                  &iterable_converter::construct<Container>,
                                                  boost::python::type_id<Container>());

    // Support chaining.
    return *this;
  }

  /// @brief Check if PyObject is iterable.
  static void* convertible(PyObject* object)
  {
    return PyObject_GetIter(object) ? object : NULL;
  }

  /// @brief Convert iterable PyObject to C++ container type.
  ///
  /// Container Concept requirements:
  ///
  ///   * Container::value_type is CopyConstructable.
  ///   * Container can be constructed and populated with two iterators.
  ///     I.e. Container(begin, end)
  template <typename Container>
  static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    namespace python = boost::python;
    // Object is a borrowed reference, so create a handle indicting it is
    // borrowed for proper reference counting.
    python::handle<> handle(python::borrowed(object));

    // Obtain a handle to the memory block that the converter has allocated
    // for the C++ type.
    typedef python::converter::rvalue_from_python_storage<Container> storage_type;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    typedef python::stl_input_iterator<typename Container::value_type> iterator;

    // Allocate the C++ type into the converter's memory block, and assign
    // its handle to the converter's convertible variable.  The C++
    // container is populated by passing the begin and end iterators of
    // the python object to the container's constructor.
    new (storage) Container(iterator(python::object(handle)),  // begin
                            iterator());                       // end
    data->convertible = storage;
  }
};

template <typename Variant>
struct variant_adaptor
{
  variant_adaptor()
  {
    boost::python::to_python_converter<Variant, variant_adaptor>{};
    register_convertibles<Variant>::apply();
  }

  template <typename T>
  struct register_convertibles;
  template <typename T>
  struct register_convertibles<boost::variant<T>>
  {
    static void apply()
    {
      boost::python::implicitly_convertible<T, Variant>();
    }
  };
  template <typename T, typename... Ts>
  struct register_convertibles<boost::variant<T, Ts...>>
  {
    static void apply()
    {
      boost::python::implicitly_convertible<T, Variant>();
      register_convertibles<boost::variant<Ts...>>::apply();
    }
  };

  static PyObject* convert(Variant const& v)
  {
    return apply_visitor(to_python{}, v);
  }
  struct to_python : boost::static_visitor<PyObject*>
  {
    template <typename T>
    result_type operator()(T const& t) const
    {
      return boost::python::incref(boost::python::object(t).ptr());
    }
  };
};

/**
 * @brief wraps a stream output overload into a to-string function for __str__ implementations
 * @tparam T
 * @param c
 * @return
 */
template <typename T>
std::string to_str_wrap(const T& c)
{
  std::ostringstream oss;
  oss << c;
  return oss.str();
}

std::ostream& operator<<(std::ostream& strm, const EntityAttribute& m)
{
  return strm << "EntityAttribute(" << m.entity_id << " " << m.attribute_name << " " << m.value << ")";
}

BOOST_PYTHON_MODULE(_libknowledge_rep_wrapper_cpp)
{
  typedef LongTermMemoryConduit LTMC;

  // Register bidirectional conversion of Python tuples <--> std::pairs
  py_pair<double, double>();
  py_pair<string, AttributeValueType>();

  // Automatically convert Python lists into vectors
  iterable_converter().from_python<vector<Region::Point2D>>();

  // Expose C++ vectors of certain types as special Python classes via vector indexing suite.
  // No proxy must be set to true for the contained elements to be converted to tuples on demand
  class_<vector<Entity>>("PyEntityList").def(vector_indexing_suite<vector<Entity>, true>());

  class_<vector<Concept>>("PyConceptList").def(vector_indexing_suite<vector<Concept>, true>());

  class_<vector<Instance>>("PyInstanceList").def(vector_indexing_suite<vector<Instance>, true>());

  class_<vector<Point>>("PyPointList").def(vector_indexing_suite<vector<Point>, true>());

  class_<vector<Pose>>("PyPoseList").def(vector_indexing_suite<vector<Pose>, true>());

  class_<vector<Region>>("PyRegionList").def(vector_indexing_suite<vector<Region>, true>());

  class_<vector<Region::Point2D>>("PyDoublePairList").def(vector_indexing_suite<vector<Region::Point2D>, true>());

  class_<vector<std::pair<string, AttributeValueType>>>("PyStrAttributeValueTypeTupleList")
      .def(vector_indexing_suite<vector<std::pair<string, AttributeValueType>>, true>());

  enum_<AttributeValueType>("AttributeValueType")
      .value("id", AttributeValueType::Id)
      .value("int", AttributeValueType::Int)
      .value("str", AttributeValueType::Str)
      .value("bool", AttributeValueType::Bool)
      .value("float", AttributeValueType::Float);

  class_<Entity>("Entity", init<uint, LTMC&>())
      .def_readonly("entity_id", &Entity::entity_id)
      .def<bool (Entity::*)(const string&, const Entity&)>("add_attribute", &Entity::addAttribute)
      .def<bool (Entity::*)(const string&, uint)>("add_attribute", &Entity::addAttribute)
      .def<bool (Entity::*)(const string&, int)>("add_attribute", &Entity::addAttribute)
      .def<bool (Entity::*)(const string&, bool)>("add_attribute", &Entity::addAttribute)
      .def<bool (Entity::*)(const string&, double)>("add_attribute", &Entity::addAttribute)

      .def<bool (Entity::*)(const string&, const string&)>("add_attribute", &Entity::addAttribute)
      .def("remove_attribute", &Entity::removeAttribute)
      .def<vector<EntityAttribute> (Entity::*)(const string&) const>("get_attributes", &Entity::getAttributes)
      .def<vector<EntityAttribute> (Entity::*)() const>("get_attributes", &Entity::getAttributes)
      .def("delete", &Entity::deleteEntity)
      .def("is_valid", &Entity::isValid)
      .def("__getitem__", &Entity::operator[])
      .def("__eq__", &Entity::operator==)
      .def("__ne__", &Entity::operator!=)
      .def("__str__", to_str_wrap<Entity>);

  class_<Concept, bases<Entity>>("Concept", init<uint, string, LTMC&>())
      .def("remove_instances", &Concept::removeInstances)
      .def("remove_instances_recursive", &Concept::removeInstancesRecursive)
      .def("remove_references", &Concept::removeReferences)
      .def("get_instances", &Concept::getInstances)
      .def<optional<Instance> (Concept::*)(const string&) const>("get_instance_named", &Concept::getInstanceNamed,
                                                                 python::return_value_policy<ReturnOptional>())
      .def("get_name", &Concept::getName)
      .def("get_children", &Concept::getChildren)
      .def("get_children_recursive", &Concept::getChildrenRecursive)
      .def<Instance (Concept::*)() const>("create_instance", &Concept::createInstance)
      .def<optional<Instance> (Concept::*)(const string&) const>("create_instance", &Concept::createInstance,
                                                                 python::return_value_policy<ReturnOptional>())
      .def("__str__", to_str_wrap<Concept>);

  class_<Instance, bases<Entity>>("Instance", init<uint, LTMC&>())
      .def("make_instance_of", &Instance::makeInstanceOf)
      .def("get_name", &Instance::getName, python::return_value_policy<ReturnOptional>())
      .def("get_concepts", &Instance::getConcepts)
      .def("get_concepts_recursive", &Instance::getConceptsRecursive)
      .def("has_concept", &Instance::hasConcept)
      .def("has_concept_recursively", &Instance::hasConceptRecursively)
      .def("__str__", to_str_wrap<Instance>);

  variant_adaptor<AttributeValue>();
  class_<EntityAttribute>("EntityAttribute", init<uint, string, AttributeValue>())
      .def_readonly("entity_id", &EntityAttribute::entity_id)
      .def_readonly("attribute_name", &EntityAttribute::attribute_name)
      .def_readonly("value", &EntityAttribute::getValue)
      .def("get_id_value", &EntityAttribute::getIdValue)
      .def("get_bool_value", &EntityAttribute::getBoolValue)
      .def("get_int_value", &EntityAttribute::getIntValue)
      .def("get_float_value", &EntityAttribute::getFloatValue)
      .def("get_string_value", &EntityAttribute::getStringValue)

      .def("__str__", to_str_wrap<EntityAttribute>);

  class_<vector<EntityAttribute>>("PyAttributeList").def(vector_indexing_suite<vector<EntityAttribute>>());

  class_<Map, bases<Instance>>("Map", init<uint, uint, string, LTMC&>())
      .def("add_point", &Map::addPoint)
      .def<Pose (Map::*)(const string&, double, double, double)>("add_pose", &Map::addPose)
      .def<Pose (Map::*)(const string&, double, double, double, double)>("add_pose", &Map::addPose)
      .def("add_region", &Map::addRegion)
      .def("get_point", &Map::getPoint, python::return_value_policy<ReturnOptional>())
      .def("get_pose", &Map::getPose, python::return_value_policy<ReturnOptional>())
      .def("get_region", &Map::getRegion, python::return_value_policy<ReturnOptional>())
      .def("get_all_points", &Map::getAllPoints)
      .def("get_all_poses", &Map::getAllPoses)
      .def("get_all_regions", &Map::getAllRegions)
      .def("deep_copy", &Map::deepCopy)
      .def("rename", &Map::rename)
      .def("__str__", to_str_wrap<Map>);

  class_<Point, bases<Instance>>("Point", init<uint, string, double, double, Map, LTMC&>())
      .def_readonly("x", &Point::x)
      .def_readonly("y", &Point::y)
      .def_readonly("parent_map", &Point::parent_map)
      .def("get_containing_regions", &Point::getContainingRegions)
      .def("__str__", to_str_wrap<Point>);

  class_<Pose, bases<Instance>>("Pose", init<uint, string, double, double, double, Map, LTMC&>())
      .def_readonly("x", &Pose::x)
      .def_readonly("y", &Pose::y)
      .def_readonly("theta", &Pose::theta)
      .def_readonly("parent_map", &Pose::parent_map)
      .def("get_containing_regions", &Pose::getContainingRegions)
      .def("__str__", to_str_wrap<Pose>);

  class_<Region, bases<Instance>>("Region", init<uint, string, const vector<Region::Point2D>, Map, LTMC&>())
      .def_readonly("points", &Region::points)
      .def_readonly("parent_map", &Region::parent_map)
      .def("get_contained_points", &Region::getContainedPoints)
      .def("get_contained_poses", &Region::getContainedPoses)
      .def("is_point_contained", &Region::isPointContained)
      .def("__str__", to_str_wrap<Region>);

  class_<LongTermMemoryConduit, boost::noncopyable>("LongTermMemoryConduit",
                                                    init<const string&, python::optional<const string&>>())
      .def<Entity (LTMC::*)()>("add_entity", &LTMC::addEntity)
      .def("add_new_attribute", &LTMC::addNewAttribute)
      .def("entity_exists", &LTMC::entityExists)
      .def("attribute_exists", &LTMC::attributeExists)
      .def("delete_all_entities", &LTMC::deleteAllEntities)
      .def("delete_all_attributes", &LTMC::deleteAllAttributes)
      .def<vector<Entity> (LTMC::*)(const string&, const uint)>("get_entities_with_attribute_of_value",
                                                                &LTMC::getEntitiesWithAttributeOfValue)
      .def<vector<Entity> (LTMC::*)(const string&, const int)>("get_entities_with_attribute_of_value",
                                                               &LTMC::getEntitiesWithAttributeOfValue)
      .def<vector<Entity> (LTMC::*)(const string&, const bool)>("get_entities_with_attribute_of_value",
                                                                &LTMC::getEntitiesWithAttributeOfValue)
      .def<vector<Entity> (LTMC::*)(const string&, const double)>("get_entities_with_attribute_of_value",
                                                                  &LTMC::getEntitiesWithAttributeOfValue)
      .def<vector<Entity> (LTMC::*)(const string&, const string&)>("get_entities_with_attribute_of_value",
                                                                   &LTMC::getEntitiesWithAttributeOfValue)

      .def<bool (LTMC::*)(const string&, vector<EntityAttribute>&) const>("select_query_id", &LTMC::selectQueryId)
      .def<bool (LTMC::*)(const string&, vector<EntityAttribute>&) const>("select_query_bool", &LTMC::selectQueryBool)
      .def<bool (LTMC::*)(const string&, vector<EntityAttribute>&) const>("select_query_int", &LTMC::selectQueryInt)

      .def<bool (LTMC::*)(const string&, vector<EntityAttribute>&) const>("select_query_float", &LTMC::selectQueryFloat)
      .def<bool (LTMC::*)(const string&, vector<EntityAttribute>&) const>("select_query_string",
                                                                          &LTMC::selectQueryString)
      .def<Concept (LTMC::*)(const string&)>("get_concept", &LTMC::getConcept)
      .def<Map (LTMC::*)(const std::string&)>("get_map", &LTMC::getMap)
      .def("get_robot", &LTMC::getRobot)
      .def("get_all_entities", &LTMC::getAllEntities)
      .def("get_all_concepts", &LTMC::getAllConcepts)
      .def("get_all_instances", &LTMC::getAllInstances)
      .def("get_all_maps", &LTMC::getAllMaps)
      .def("get_all_attributes", &LTMC::getAllAttributes)
      .def("get_entity", &LTMC::getEntity, python::return_value_policy<ReturnOptional>())
      .def("get_instance", &LTMC::getInstance, python::return_value_policy<ReturnOptional>())
      .def<optional<Concept> (LTMC::*)(uint)>("get_concept", &LTMC::getConcept,
                                              python::return_value_policy<ReturnOptional>())
      .def<optional<Map> (LTMC::*)(uint)>("get_map", &LTMC::getMap, python::return_value_policy<ReturnOptional>())
      .def<optional<Point> (LTMC::*)(uint)>("get_point", &LTMC::getPoint, python::return_value_policy<ReturnOptional>())
      .def<optional<Pose> (LTMC::*)(uint)>("get_pose", &LTMC::getPose, python::return_value_policy<ReturnOptional>())
      .def<optional<Region> (LTMC::*)(uint)>("get_region", &LTMC::getRegion,
                                             python::return_value_policy<ReturnOptional>());
}
