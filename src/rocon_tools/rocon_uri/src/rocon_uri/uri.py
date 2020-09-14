#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: uri
   :platform: Unix
   :synopsis: The public api for parsing and comparing rocon uri's.

This module contains the public api and classes used to parse and
compare rocon uri strings.
"""

##############################################################################
# Imports
##############################################################################

# this needs updating to urllib.parse for python3
import urlparse
# register properties of the rocon scheme, otherwise fragments don't get allocated properly, refer to:
#   http://stackoverflow.com/questions/1417958/parse-custom-uris-with-urlparse-python
getattr(urlparse, 'uses_fragment').append('rocon')
getattr(urlparse, 'uses_netloc').append('rocon')
#getattr(urlparse, 'uses_params').append('rocon')
#getattr(urlparse, 'uses_query').append('rocon')
#getattr(urlparse, 'uses_relative').append('rocon')
import rocon_ebnf.rule_parser as rule_parser
import re
from collections import namedtuple

# Local imports
from .exceptions import RoconURIValueError
import rules

##############################################################################
# Public Methods
##############################################################################


def parse(rocon_uri_string):
    """
    Convenience method for creating RoconURI objects.

    :param str rocon_uri_string: a rocon uri in string format.

    :returns: a validated rocon uri object
    :rtype: :class:`.RoconURI`.

    :raises: :exc:`rocon_uri.exceptions.RoconURIValueError`
    """
    return RoconURI(rocon_uri_string)


def is_compatible(rocon_uri_a, rocon_uri_b):
    '''
    Checks if two rocon uri's are compatible.

    :param rocon_uri_a: a rocon uri in either string or RoconURI object format.
    :type rocon_uri_a: str or :class:`.RoconURI`

    :param rocon_uri_b: a rocon uri in either string or RoconURI object format.
    :type rocon_uri_b: str or :class:`.RoconURI`

    :returns: true if compatible, i.e. wildcards or intersections of fields are nonempty
    :rtype: bool

    :raises: :exc:`rocon_uri.exceptions.RoconURIValueError` if either rocon_uri string is not valid

    .. todo:: tighten up the name pattern matching
    '''
    completely_compatible_rocon_uri_list = ["rocon:/", "rocon://", "rocon:/*"]
    try:  # python 2.7 (use basestring to get unicode objects)
        requires_conversion_a = isinstance(rocon_uri_a, basestring)
        requires_conversion_b = isinstance(rocon_uri_b, basestring)
    except NameError:  # python3
        requires_conversion_a = isinstance(rocon_uri_a, str)
        requires_conversion_b = isinstance(rocon_uri_b, str)
    # do some preliminary checks now before we parse for official rocon uris
    # i.e. allow compatibility true if one is rocon:/ regardless of the other
    if requires_conversion_a and rocon_uri_a in completely_compatible_rocon_uri_list:
        return True
    if requires_conversion_b and rocon_uri_b in completely_compatible_rocon_uri_list:
        return True
    # Parse - this makes sure it is an official rocon uri being parsed in
    a = parse(rocon_uri_a) if requires_conversion_a else rocon_uri_a
    b = parse(rocon_uri_b) if requires_conversion_b else rocon_uri_b
    no_wildcards = lambda l1, l2: '*' not in l1 and '*' not in l2
    intersection = lambda l1, l2: [x for x in l1 if x in l2]
    for field in ["hardware_platform", "application_framework", "operating_system"]:
        if no_wildcards(getattr(a, field).list, getattr(b, field).list):
            if not intersection(getattr(a, field).list, getattr(b, field).list):
                return False
    # do some subset of regex pattern matching for names.
    if no_wildcards(a.name.list, b.name.list):
        if not intersection(a.name.list, b.name.list):
            # check for our regex subset (i.e. '*' at the end of a string)
            matches = False
            for a_name in a.name.list:
                if matches is True:
                    break
                for b_name in b.name.list:
                    match_result_a = re.match(r"(.*)[*]+$", a_name)
                    match_result_b = re.match(r"(.*)[*]+$", b_name)
                    a_name_substring = match_result_a.group(1) if match_result_a is not None else a_name
                    b_name_substring = match_result_b.group(1) if match_result_b is not None else b_name
                    if match_result_a is not None:
                        if re.match(a_name_substring, b_name_substring):
                            matches = True
                            break
                    if match_result_b is not None:
                        if re.match(b_name_substring, a_name_substring):
                            matches = True
                            break
            if not matches:
                return False
    return True

##############################################################################
# RoconURI Class
##############################################################################

# Descriptors are defined as class variables - use weak reference dictionaries
# to store data from each instance of the classes. Refer to:
#   http://nbviewer.ipython.org/urls/gist.github.com/ChrisBeaumont/5758381/raw/descriptor_writeup.ipynb
from weakref import WeakKeyDictionary


class RoconURIField(object):
    # A python descriptor that does rule parsing on rocon uri field strings

    Value = namedtuple('RoconURIField', 'string list')

    def __init__(self, name, rules):
        self.rules = rules
        self.field_name = name
        self.field = WeakKeyDictionary()
        self.field_list = WeakKeyDictionary()

    def __get__(self, instance, unused_owner):
        return RoconURIField.Value(self.field.get(instance, '*'),
                                   self.field_list.get(instance, ['*']))

    def __set__(self, instance, new_field):
        try:
            match_result = rule_parser.match(self.rules, new_field)
            #for rule in self.rules:
            #    print("  %s" % rule)
            self.field[instance] = new_field
            self.field_list[instance] = getattr(match_result, self.field_name + "_list")
        except AttributeError:  # result of getattr when it fails, in this case, typically when the match is None
            raise RoconURIValueError("%s specification is invalid or not officially registered (see https://github.com/robotics-in-concert/rocon_tools/blob/indigo/rocon_uri/src/rocon_uri/rules/rules.yaml)[%s]" % (self.field_name, new_field))


class RoconURI(object):
    """
    Initialises from a rocon uri string parsing the relevant information into fields internally.
    This class uses python decorators to establish the parsed fields into the following variables:

    * hardware_platform
    * name
    * application_framework
    * operating_system

    Since each field can contain one or more values, each of these variables can return either a
    string representation of the original field, or a list of the parsed field. e.g.

    .. code-block:: python

       >>> rocon_uri_object = rocon_uri.parse('rocon:/turtlebot2|pr2/dude/hydro/precise#rocon_apps/chirp')
       >>> print rocon_uri_object.hardware_platform.list
       ['turtlebot2', 'pr2']
       >>> print rocon_uri_object.hardware_platform.string
       turtlebot2|pr2
    """
    # Can't use slots here if we wish to have weak references to this object (see the descriptor).

    ####################
    # Class Fields
    ####################
    ebnf_rules = rules.load_ebnf_rules()

    # These are descriptors - required to be defined in the class area and instantiated separately in __init__
    hardware_platform     = RoconURIField("hardware_platform", ebnf_rules["hardware_platform"])          # noqa @IgnorePep8
    name                  = RoconURIField("name", ebnf_rules["name"])                                    # noqa @IgnorePep8
    application_framework = RoconURIField("application_framework", ebnf_rules["application_framework"])  # noqa @IgnorePep8
    operating_system      = RoconURIField("operating_system", ebnf_rules["operating_system"])            # noqa @IgnorePep8

    ####################
    # Initialisation
    ####################
    def __init__(self, rocon_uri_string="rocon:/"):
        """
        Initialise the rocon uri object from a rocon uri string.

        :param str rocon_uri_string: a rocon uri in string format.
        :raises: :exc:`rocon_uri.exceptions.RoconURIValueError` if either rocon_uri string is not valid
        """
        parsed_url = urlparse.urlparse(rocon_uri_string)
        if parsed_url.scheme != 'rocon':
            raise RoconURIValueError("uri scheme '%s' != 'rocon'" % parsed_url.scheme)
        self.concert_name = parsed_url.netloc
        uri_path_elements = [element for element in parsed_url.path.split('/') if element]
        if len(uri_path_elements) > 4:
            raise RoconURIValueError("uri path element invalid, need at most platform/os/system/name fields [%s]" % parsed_url.path)
        try:
            self.hardware_platform     = uri_path_elements[0]  #@IgnorePep8
            self.name                  = uri_path_elements[1]  #@IgnorePep8
            self.application_framework = uri_path_elements[2]  #@IgnorePep8
            self.operating_system      = uri_path_elements[3]  #@IgnorePep8
        except IndexError:  # if optional fields are left off, we end up here
            pass  # ok, since defaults are suitably set by the descriptor
        except RoconURIValueError as e:
            #  print("Raised %s %s" % type(e), str(e))
            raise e
        self.rapp = parsed_url.fragment

    def __str__(self):
        # according to uri specifications, must be preceded by '//'
        concert_name = '//' + self.concert_name if self.concert_name else ''
        return "rocon:%s/%s/%s/%s/%s%s" % (concert_name, self.hardware_platform.string, self.name.string, self.application_framework.string, self.operating_system.string, '#' + self.rapp if self.rapp else "")
