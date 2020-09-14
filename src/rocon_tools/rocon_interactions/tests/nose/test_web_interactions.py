#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises, assert_false, assert_is_not_none, assert_is_none, assert_true
import rocon_interactions.web_interactions as web_interactions
import rocon_console.console as console

##############################################################################
# Utility functions
##############################################################################

example_web_url = 'web_url(http://wiki.ros.org/chatter_concert)'
example_web_url_with_quotes = 'web_url("http://wiki.ros.org/chatter_concert")'
example_web_app = 'web_app(http://robotics-in-concert.github.io/rocon_tools/js/current/listener.html)'
example_web_app_with_quotes = 'web_app("http://robotics-in-concert.github.io/rocon_tools/js/current/listener.html")'
undecorated_web_url = 'http://wiki.ros.org/chatter_concert'
not_a_web_interaction = 'com.github.robotics_in_concert.rocon_android.SolutionManager'

##############################################################################
# Tests
##############################################################################

def test_web_urls():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Valid Web URL's" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    
    for example_url in [example_web_url, example_web_url_with_quotes]:
        web_interaction = web_interactions.parse(example_url)
        assert_is_not_none(web_interaction)
        assert_true(web_interaction.is_web_url(), "A web url should be a web url")
        assert_false(web_interaction.is_web_app(), "A web url is not a web app")

def test_web_app():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Valid Web Apps" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
 
    for example_app in [example_web_app, example_web_app_with_quotes]:
        web_interaction = web_interactions.parse(example_app)
        assert_is_not_none(web_interaction)
        assert_false(web_interaction.is_web_url())
        assert_true(web_interaction.is_web_app())
 
def test_bad_web_interactions():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Invalid Web Interactions" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
 
    web_interaction = web_interactions.parse(undecorated_web_url)
    assert_is_none(web_interaction)
    web_interaction = web_interactions.parse(not_a_web_interaction)
    assert_is_none(web_interaction)

