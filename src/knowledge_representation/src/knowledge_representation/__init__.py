from knowledge_representation._libknowledge_rep_wrapper_cpp import LongTermMemoryConduit, PyAttributeList, Entity, \
    EntityAttribute, Concept, Instance, AttributeValueType, Map, Point, Pose, Region


def get_default_ltmc():
    """
    Gets a handle for the knowledgebase with the default parameters.
    :return: a LongTermMemoryConduit object
    """
    return LongTermMemoryConduit("knowledge_base")


def id_to_typed_wrapper(ltmc, entity_id):
    """
    Turns an entity ID into the most specific kind of wrapper object available for that entity.
    :param entity_id: the ID to get the wrapper for
    :return: a specific, valid type wrapper, or None of no such entity exists
    """
    as_entity = Entity(entity_id, ltmc)
    if not as_entity.is_valid():
        return None
    as_concept = ltmc.get_concept(entity_id)
    if as_concept:
        # No specializations of concept
        return as_concept
    as_instance = ltmc.get_instance(entity_id)
    if not as_instance:
        return as_entity
    concepts = set(map(lambda x: x.get_name(), as_instance.get_concepts()))
    if "map" in concepts:
        return ltmc.get_map(entity_id)
    if "point" in concepts:
        return ltmc.get_point(entity_id)
    if "pose" in concepts:
        return ltmc.get_pose(entity_id)
    if "region" in concepts:
        return ltmc.get_region(entity_id)

    return as_instance
