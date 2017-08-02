#include <boost/test/unit_test.hpp>
#include <drivers-advanced_navigation_anpp/Dummy.hpp>

using namespace drivers-advanced_navigation_anpp;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    drivers-advanced_navigation_anpp::DummyClass dummy;
    dummy.welcome();
}
