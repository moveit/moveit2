import os
import unittest

import numpy as np

from test_moveit.core.collision_detection import (
    BodyType,
    CollisionRequest,
    CollisionResult,
    Contact,
    CostSource,
)
from test_moveit.core.planning_scene import PlanningScene
from test_moveit.core.robot_model import RobotModel

dir_path = os.path.dirname(os.path.realpath(__file__))
URDF_FILE = '{}/fixtures/panda.urdf'.format(dir_path)
SRDF_FILE = '{}/fixtures/panda.srdf'.format(dir_path)


def get_robot_model():
    """Return a RobotModel instance."""
    return RobotModel(urdf_xml_path=URDF_FILE, srdf_xml_path=SRDF_FILE)


class TestBodyType(unittest.TestCase):

    def test_enum_members(self):
        """Test that BodyType contains the expected enumeration members."""
        self.assertTrue(hasattr(BodyType, 'ROBOT_LINK'))
        self.assertTrue(hasattr(BodyType, 'ROBOT_ATTACHED'))
        self.assertTrue(hasattr(BodyType, 'WORLD_OBJECT'))


class TestContact(unittest.TestCase):

    def test_contact_properties(self):
        """Test initialization and property access on Contact."""
        contact = Contact()
        contact.body_name_1 = 'panda_link1'
        contact.body_name_2 = 'obstacle'
        contact.body_type_1 = BodyType.ROBOT_LINK
        contact.body_type_2 = BodyType.WORLD_OBJECT
        contact.depth = 0.025
        contact.pos = np.array([0.1, 0.2, 0.3])
        contact.normal = np.array([0.0, 0.0, 1.0])
        contact.percent_interpolation = 0.5
        pt1 = np.array([0.1, 0.2, 0.3])
        pt2 = np.array([0.1, 0.2, 0.325])
        contact.nearest_points = (pt1, pt2)

        self.assertEqual(contact.body_name_1, 'panda_link1')
        self.assertEqual(contact.body_name_2, 'obstacle')
        self.assertEqual(contact.body_type_1, BodyType.ROBOT_LINK)
        self.assertEqual(contact.body_type_2, BodyType.WORLD_OBJECT)
        self.assertAlmostEqual(contact.depth, 0.025)
        np.testing.assert_allclose(contact.pos, [0.1, 0.2, 0.3])
        np.testing.assert_allclose(contact.normal, [0.0, 0.0, 1.0])
        self.assertAlmostEqual(contact.percent_interpolation, 0.5)

        nearest = contact.nearest_points
        self.assertEqual(len(nearest), 2)
        np.testing.assert_allclose(nearest[0], pt1)
        np.testing.assert_allclose(nearest[1], pt2)

        self.assertIn('panda_link1', repr(contact))
        self.assertIn('obstacle', repr(contact))

    def test_nearest_points_invalid_sequence(self):
        """Test that assigning invalid sequence to nearest_points raises ValueError."""
        contact = Contact()
        with self.assertRaises(ValueError):
            contact.nearest_points = (np.array([1.0, 2.0, 3.0]),)

    def test_contact_equality(self):
        """Test Contact equality operator."""
        c1 = Contact()
        c1.body_name_1 = 'a'
        c1.body_name_2 = 'b'
        c1.depth = 0.01

        c2 = Contact()
        c2.body_name_1 = 'a'
        c2.body_name_2 = 'b'
        c2.depth = 0.01

        c3 = Contact()
        c3.body_name_1 = 'a'
        c3.body_name_2 = 'b'
        c3.depth = 0.05

        self.assertEqual(c1, c2)
        self.assertNotEqual(c1, c3)


class TestCostSource(unittest.TestCase):

    def test_cost_source_properties(self):
        """Test CostSource initialization and properties."""
        cs = CostSource()
        cs.aabb_min = [0.0, 0.0, 0.0]
        cs.aabb_max = [1.0, 2.0, 3.0]
        cs.cost = 0.75

        self.assertEqual(cs.aabb_min, [0.0, 0.0, 0.0])
        self.assertEqual(cs.aabb_max, [1.0, 2.0, 3.0])
        self.assertAlmostEqual(cs.cost, 0.75)
        self.assertAlmostEqual(cs.get_volume(), 6.0)
        self.assertAlmostEqual(cs.volume, 6.0)
        self.assertIn('0.75', repr(cs))

    def test_cost_source_ordering_and_hash(self):
        """Test CostSource ordering, hashing, and set insertion."""
        cs1 = CostSource()
        cs1.aabb_min = [0.0, 0.0, 0.0]
        cs1.aabb_max = [1.0, 1.0, 1.0]
        cs1.cost = 1.0

        cs2 = CostSource()
        cs2.aabb_min = [0.0, 0.0, 0.0]
        cs2.aabb_max = [2.0, 2.0, 2.0]
        cs2.cost = 0.5

        self.assertTrue(cs2 < cs1)

        s = {cs1, cs2}
        self.assertIn(cs1, s)
        self.assertIn(cs2, s)


class TestCollisionResult(unittest.TestCase):

    def test_collision_result_defaults(self):
        """Test default values of CollisionResult."""
        res = CollisionResult()
        self.assertFalse(res.collision)
        self.assertEqual(res.contact_count, 0)
        self.assertIsInstance(res.contacts, dict)
        self.assertEqual(len(res.contacts), 0)
        self.assertIsInstance(res.cost_sources, list)
        self.assertEqual(len(res.cost_sources), 0)

    def test_collision_result_contacts_roundtrip(self):
        """Test assigning and reading contacts on CollisionResult."""
        res = CollisionResult()
        c = Contact()
        c.body_name_1 = 'panda_link1'
        c.body_name_2 = 'panda_link2'
        c.depth = 0.01

        res.contacts = {('panda_link1', 'panda_link2'): [c]}
        self.assertEqual(len(res.contacts), 1)
        self.assertIn(('panda_link1', 'panda_link2'), res.contacts)
        retrieved_contacts = res.contacts[('panda_link1', 'panda_link2')]
        self.assertEqual(len(retrieved_contacts), 1)
        self.assertEqual(retrieved_contacts[0].body_name_1, 'panda_link1')
        self.assertAlmostEqual(retrieved_contacts[0].depth, 0.01)

    def test_collision_result_cost_sources_roundtrip(self):
        """Test assigning and reading cost_sources on CollisionResult."""
        res = CollisionResult()
        cs = CostSource()
        cs.aabb_min = [0.0, 0.0, 0.0]
        cs.aabb_max = [1.0, 1.0, 1.0]
        cs.cost = 0.5

        res.cost_sources = [cs]
        self.assertEqual(len(res.cost_sources), 1)
        self.assertAlmostEqual(res.cost_sources[0].cost, 0.5)

    def test_collision_result_clear(self):
        """Test that clear() resets all results."""
        res = CollisionResult()
        res.collision = True
        res.contact_count = 5
        res.distance = 0.0

        c = Contact()
        c.body_name_1 = 'link1'
        c.body_name_2 = 'link2'
        res.contacts = {('link1', 'link2'): [c]}

        cs = CostSource()
        cs.cost = 1.0
        res.cost_sources = [cs]

        res.clear()
        self.assertFalse(res.collision)
        self.assertEqual(res.contact_count, 0)
        self.assertEqual(len(res.contacts), 0)
        self.assertEqual(len(res.cost_sources), 0)

    def test_planning_scene_check_collision(self):
        """Test running check_collision and reading contacts and cost_sources."""
        robot_model = get_robot_model()
        scene = PlanningScene(robot_model)

        req = CollisionRequest()
        req.contacts = True
        req.cost = True
        req.max_contacts = 10
        req.max_cost_sources = 10

        res = CollisionResult()
        scene.check_collision(req, res)

        self.assertIsInstance(res.contacts, dict)
        self.assertIsInstance(res.cost_sources, list)


if __name__ == '__main__':
    unittest.main()
