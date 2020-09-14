import os

import yaml
from warnings import warn


def get_instance(ltmc, instance_name, concept_name):
    concept = ltmc.get_concept(concept_name)
    instance = concept.get_instance_named(instance_name)
    if not instance:
        instance = concept.create_instance(instance_name)
    return instance


def add_attributes(entity, attributes):
    for attribute in attributes:
        name = attribute["name"]
        value = attribute["value"]
        entity.add_attribute(name, value)


def evaluate_attribute_values(ltmc, attributes):
    for attribute in attributes:
        value = attribute["value"]
        if isinstance(value, dict) and "instance_name" in value.keys():
            attribute["value"] = get_instance(ltmc, value["instance_name"], value["concept_name"])
        elif isinstance(value, dict) and "concept" in value.keys():
            attribute["value"] = ltmc.get_concept(value["concept"])


def read_yaml_from_file(file_path):
    if not os.path.isfile(file_path):
        warn("File not found at " + file_path + ".")
        return {}
    with open(file_path, 'r') as stream:
        try:
            contents = yaml.load(stream, Loader=yaml.FullLoader)
            return contents
        except yaml.YAMLError:
            warn("File found at " + file_path + ", but cannot be parsed by YAML parser.")


def validate_attributes(attributes):
    for attribute in attributes:
        if "name" not in attribute:
            warn("Attribute missing name")
            continue
        if "value" not in attribute:
            warn("Attribute missing value")
            continue
        value = attribute["value"]
        if isinstance(value, dict) and "instance" in value.keys():
            if len(value["instance"]) != 2:
                warn("Expected two entries (instance name, concept name). Saw: {}".format(value["instance"]))
                continue
            instance_name, concept_name = value["instance"]
            del value["instance"]
            value["instance_name"] = instance_name
            value["concept_name"] = concept_name
        elif "concept" in attribute.keys():
            pass
    return attributes


def load_knowledge_from_yaml(file_path):
    knowledge = read_yaml_from_file(file_path)
    version = knowledge.get("version")
    if version != 1:
        warn("Unrecognized knowledge format '{}' for {}. This file may not load correctly.".format(version, file_path))
    if not knowledge.get("entities"):
        knowledge["entities"] = {}
    concepts = []
    instances = []
    for entry in knowledge["entities"]:
        if "concept" in entry.keys():
            if not entry.get("attributes"):
                entry["attributes"] = []
            entry["attributes"] = validate_attributes(entry["attributes"])
            entry["name"] = entry["concept"]
            del entry["concept"]
            concepts.append(entry)
        elif "instance" in entry.keys():
            if len(entry["instance"]) != 2:
                warn("Expected two entries (instance name, concept name). Saw: {}".format(entry["instance"]))
                continue
            instance_name, concept_name = entry["instance"]
            del entry["instance"]
            entry["name"] = instance_name
            entry["concept_name"] = concept_name

            if not entry.get("attributes"):
                entry["attributes"] = []
            entry["attributes"] = validate_attributes(entry["attributes"])
            instances.append(entry)
        else:
            warn("Unrecognized entry: {}".format(entry))
    return concepts, instances


def populate_with_knowledge(ltmc, all_data):
    concept_count, instance_count = 0, 0
    for concepts, instances in all_data:
        for concept_data in concepts:
            concept = ltmc.get_concept(concept_data["name"])
            evaluate_attribute_values(ltmc, concept_data["attributes"])
            add_attributes(concept, concept_data["attributes"])
            concept_count += 1
        for instance_data in instances:
            instance_name, concept_name = instance_data["name"], instance_data["concept_name"]
            instance = get_instance(ltmc, instance_name, concept_name)
            evaluate_attribute_values(ltmc, instance_data["attributes"])
            add_attributes(instance, instance_data["attributes"])
            instance_count += 1

    return concept_count, instance_count
