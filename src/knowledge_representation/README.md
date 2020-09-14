# knowledge_representation [![CI](https://github.com/utexas-bwi/knowledge_representation/workflows/CI/badge.svg)](https://github.com/utexas-bwi/knowledge_representation/actions?query=workflow%3ACI)

Mechanisms for storing and querying information about the world. A nicely packaged [database schema](https://en.wikipedia.org/wiki/Database_schema) to structure the kinds of knowledge that a robot might need and tools for [CRUD operations](https://en.wikipedia.org/wiki/Create,_read,_update_and_delete) within that structure. Integrate it into your robot's perception so you can query where objects were seen, or into your task planning system so you can easily manage all of your facts.

* Persistent storage of facts and map geometry backed by a local PostgreSQL database
* Convenient APIs for common queries, like getting all instances of a type of object
* Link directly against your C++ code, or import a Python binding
* Not dependent on ROS, but easy to integrate into ROS projects

## Representation

Knowledge is broken up into _concepts_, _instances_ and _attributes_.

**Concepts** are abstract ideas. They represent the _idea_ of something. For example, the general idea of an apple but not any one apple in particular.

**Instances** are concrete, usually physical manifestations of concepts. You could have a dozen instances of a concept like apple.

Both concepts and instances are **entities** in the knowledgebase. Each entity is granted a unique identifying number.

**Attributes** tie entities together and store data about them. Each attribute has a name and a type. For example, an apple instance might be _on_ some other instance. The attribute is named `is_on` and its type is `entity_id`, because it relates some entity to another. Attributes with a value type of `entity_id` are also called **relations** because they relate entities to each other. Attributes with types other than `entity_id` function more like properties; for example, the relation `is_graspable` with a type of `bool` is just storing a simple fact about an entity.

### Map Types

Mobile robots often have to answer questions like "what room am I in?" or "which is the nearest X?". knowledge_representation includes special affordances for representing 2D geometric information that can be queried to answer these kinds of questions:

* **Points** are a special kind of instance which store an x and y coordinate.
* **Poses** are a special kind of instance which store an x and y coordinate as well as a direction.
* **Regions** are a special kind of instance which store a list of x and y coordinates defining a closed region.

All of these types are uniquely tied to a single **map**, a special kind of instance.

Like concepts, all of these geometric types _must_ have names configured. Without names attached, there isn't enough semantic information to support any kind of useful operation. What would it mean for a point to be in a region if neither the point nor the region had names?

## Installation

Make sure you've pulled down the package dependencies using rosdep

    rosdep install --from-paths . -ry

Then run one of `scripts/configure_{mysql,postgresql}.sh` to install the default database configuration and schema. **PostgreSQL is the preferred backend.**

## Usage

Integrate knowledge_representation by using the API wherever you need to store facts and observations that the robot might need later. Use the same API to retrieve facts en masse so you can plan over them, inspect them, or do whatever else you need for your application.

### C++/Python API

To create applications that store and query, link your C++ against the `knowledge_rep` library or just import the `knowledge_representation` Python module. See the [documentation for the latest version of the C++ API](https://utexas-bwi.github.io/knowledge_representation/) and example usage in `test/*.cpp`. The Python API is a generated wrapper, so must classes and methods are the same but with snake case conventions. See scripts `test_ltmc` or `scripts/show_me` for example usage, and try out the `ikr` script to interactively explore the API.

### Loading Knowledge

Scripts are provided for bulk loading knowledge. This is helpful if say, you want to load in map annotations, or you have an ontology that you want to use to initialize the robot's knowledge.

`populate_with_map` loads in SVG map annotations (marked over an image, ROS-style map). See `test/resources` for some example SVG files. In the future, we'll provide a simple browser-based annotator as well.

`populate_with_[knowledge|owl|xml]` support loading in different kinds of ontologies. Documentation and example files will come in a later release.

### Exploration

Once your robot has accumulated knowledge, you'll want to poke around. Use the `show_me` script to quickly see a summary of the current knowledge, then pass it an ID or a name to see details about entities and their relations.

### As Part of a ROS System

Once you add `knowledge_representation` to your package's dependencies, your code will link against the library that provides the APIs.

The `populate_` scripts are geared towards use in a launch files. For instance, to load a map on launch, just add a line like this to your launch file:

    <node name="load_map_annotations" pkg="knowledge_representation" type="populate_with_map" args="$(find my_map_package)/maps/map_name.yaml"/>

Once the package is built in a catkin workspace, you can access additional shell shortcuts:

* `ikr`: run the iPython environment
* `kr-query`: run a SQL query against the knowledgebase
* `kr-save`: save a SQL-dump of the current knowledgebase
* `kr-show`: run `show_me`


## Development

If you're working on the MySQL interface, we access the backing store via the xdev API. See the [documentation](https://dev.mysql.com/doc/dev/connector-cpp/8.0/) for the official MySQL xdev API C++ library.