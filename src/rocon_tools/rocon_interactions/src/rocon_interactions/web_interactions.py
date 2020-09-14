#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: web_interactions
   :platform: Unix
   :synopsis: Parser for web url/web app interaction strings.


Module for parsing web interaction strings provided to the interactions
manager.

----

"""
##############################################################################
# Imports
##############################################################################

import re

##############################################################################
# Methods
##############################################################################


def parse(interaction):
    """
    Tries to parse the specified string to see if it is a valid web interaction
    (web app or web url). If it is, it passes back a web interaction object,
    or None if it is not valid.

    :param interaction str: the string to parse.

    :returns: the web interaction object if parsed
    :rtype: WebInteraction or None
    """
    # handle quotes or non quotes
    web_app_with_quotes = (WebInteraction.WEB_APP, re.compile(r"web_app\(\"(.+)\"\)"))
    web_app_without_quotes = (WebInteraction.WEB_APP, re.compile(r"web_app\((.+)\)"))
    web_url_with_quotes = (WebInteraction.WEB_URL, re.compile(r"web_url\(\"(.+)\"\)"))
    web_url_without_quotes = (WebInteraction.WEB_URL, re.compile(r"web_url\((.+)\)"))
    for (web_interaction_type, compiled_regular_expression) in [
            web_app_with_quotes, web_app_without_quotes, web_url_with_quotes, web_url_without_quotes]:
        result = compiled_regular_expression.match(interaction)
        if result:
            return WebInteraction(web_interaction_type, result.group(1))
    return None


class WebInteraction(object):
    """
    Generic web interaction object that stores the type (web url or app) and
    the actual url to be used. This is important because the resulting
    url that will get formed for this interaction will have many extra arguments
    (name, remappings, parameters) appended to the url here if it is a web app.
    """

    WEB_APP = "web_app"
    """Prefix used to define web apps (e.g. web_app(https://github.com/...)"""
    WEB_URL = "web_url"
    """Prefix used to define web urls (e.g. web_url(http://wiki.ros.org/rocon_interactions)"""

    def __init__(self, interaction_type, interaction_url):
        """
        Don't instantiate this directly, use the :func:`.parse` method instead.

        :param str interaction_type: either :data:`.WEB_APP` or :data:`.WEB_URL`.
        :param str interaction_url: the parsed url
        """
        self._type = interaction_type
        self._url = interaction_url

    def is_web_app(self):
        """
        Is a web app or not.

        :returns: result of the query
        :rtype: bool
        """
        return True if self._type == WebInteraction.WEB_APP else False

    def is_web_url(self):
        """
        Is a web url or not.

        :returns: result of the query
        :rtype: bool
        """
        return True if self._type == WebInteraction.WEB_URL else False

    @property
    def url(self):
        """
        The interaction url (e.g. http://wiki.ros.org/rocon_interactions).
        """
        return self._url
