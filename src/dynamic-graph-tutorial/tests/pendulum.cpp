#include <boost/test/unit_test.hpp>

#include "dynamic-graph/tutorial/inverted-pendulum.hh"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_pendulum) {
  dynamicgraph::tutorial::InvertedPendulum pendulum("test pendulum");
  pendulum.setCartMass(1.2);
  BOOST_CHECK(pendulum.getCartMass() == 1.2);
  pendulum.setPendulumMass(0.6);
  BOOST_CHECK(pendulum.getPendulumMass() == 0.6);
  pendulum.setPendulumLength(1.8);
  BOOST_CHECK(pendulum.getPendulumLength() == 1.8);
}

BOOST_AUTO_TEST_SUITE_END()
