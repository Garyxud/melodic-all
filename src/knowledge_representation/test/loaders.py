#!/usr/bin/env python
import sys
import unittest
from knowledge_representation.map_loader import load_map_from_yaml
from knowledge_representation.knowledge_loader import load_knowledge_from_yaml
import os

resource_path = os.path.dirname(__file__) + "/resources"


class TestLoaders(unittest.TestCase):

    def test_load_annotator_tool_map_works(self):
        name, annotations = load_map_from_yaml(resource_path + "/map/map.yaml")
        points, poses, regions = annotations
        self.assertEqual(2, len(points))
        self.assertEqual(8, len(poses))
        self.assertEqual(6, len(regions))

    def test_load_inkscape_map_works(self):
        name, annotations = load_map_from_yaml(resource_path + "/map/map_inkscape.yaml")
        points, poses, regions = annotations
        self.assertEqual(6, len(points))
        self.assertEqual(2, len(poses))
        self.assertEqual(2, len(regions))

    def test_load_knowledge_works(self):
        concepts, instances = load_knowledge_from_yaml(resource_path + "/knowledge.yaml")
        self.assertEqual(1, len(concepts))
        self.assertEqual(2, len(instances))


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun("knowledge_representation", 'test_bare_bones', TestLoaders)
