#include <boost/test/unit_test.hpp>

#include "dynamic-graph/tutorial/feedback-controller.hh"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_controller) { dynamicgraph::tutorial::FeedbackController controller("test controller"); }

BOOST_AUTO_TEST_SUITE_END()
