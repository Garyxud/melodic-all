import unittest

from dynamic_graph.tutorial.simu import build_graph


class DynamicGraphTutorialTest(unittest.TestCase):
    def test_simu(self):
        s, f, a = build_graph()
        self.assertEqual(a.getCartMass(), 1)


if __name__ == '__main__':
    unittest.main()
