#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
# This is more just an example and a check to ensure that the parts we
# use stay working.

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import rocon_ebnf.rule_parser as rule_parser

##############################################################################
# Tests
##############################################################################

def try_using_windoze_attribute(parser_result):
    '''
      Used to check if one of the parser's variables is set or not. If not, it raises
      an AttributeError
    '''
    print("%s" % parser_result.windoze)

def print_linux_parsing(input_string, parser_result):
    print("Input: %s" % input_string)
    if parser_result is not None:
        try:
            print("  OS list: %s" % parser_result.operating_systems_list)
        except AttributeError:
            pass
        try:
            print("  Ubuntu : %s" % parser_result.ubuntu)
        except AttributeError:
            pass
        try:
            print("  Linux  : %s" % parser_result.linux)
        except AttributeError:
            pass
        try:
            print("  OS : %s" % parser_result.os)
        except AttributeError:
            pass
        #print("  Windows: %s" % result.windows) throws an AttributeError
    else:
        print("Error in parsing")

def test_message_to_string():
    # about this rule:
    #   - accomodate a trailing slash (sep? at the end)
    operating_systems_rule = [
             'init operating_systems_list=[] ',
             'pattern           ::= os_zero operating_systems*',
             'os_zero           ::= os                          @operating_systems_list.append("$os")', 
             'operating_systems ::= "|" os                      @operating_systems_list.append("$os")', 
             'os                ::= "*" | windoze | linux | "osx" | "freebsd"',
               'windoze         ::= "winxp" | "windows7"',
               'linux           ::= "arch" | "debian" | "fedora" | "gentoo" | "opensuse" | ubuntu | "linux"',
                 'ubuntu        ::= "precise" | "quantal" | "raring" | "ubuntu"' 
              ]
    input_string = "precise|quantal"
    result = rule_parser.match(operating_systems_rule, input_string)
    assert result is not None
    assert "precise" in result.operating_systems_list
    assert "quantal" in result.operating_systems_list
    assert result.ubuntu == "quantal"
    assert result.linux == "quantal"
    assert result.os == "quantal"
    assert_raises(AttributeError, try_using_windoze_attribute, result)
    print_linux_parsing(input_string, result)

    input_string = "*"
    result = rule_parser.match(operating_systems_rule, input_string)
    assert result is not None
    assert result.os == "*"
    print_linux_parsing(input_string, result)

    input_string = "ubuntu"
    result = rule_parser.match(operating_systems_rule, input_string)
    assert result is not None
    assert result.os == "ubuntu"
    print_linux_parsing(input_string, result)
    
    rule = [ 'uri      ::= sep os* sep system* sep platform* sep name* sep?',
             'sep      ::= r"/"',
             'os       ::= "windows" | "linux" | "precise"',
             'system   ::= "opros" | ros',
               'ros      ::= "groovy" | "hydro" | "ros"', 
             'platform ::= r"." ^sep',
             'name     ::= r"." ^sep',
             ]
    rocon_uri = "/precise/ros/turtlebot/dude"
    print("Input: %s" % rocon_uri)
    result = rule_parser.match(rule, rocon_uri)
    if result is not None:
        print("  os      : %s" % result.os)
        print("  system  : %s" % result.system)
        print("  platform: %s" % result.platform)
        print("  name    : %s" % result.name)
    else:
        print("Error in parsing")
    
#     rule = [ 'rosdistro ::= "groovy" | "hydro"' ]
#     text = 'groovy'
#     result = rule_parser.rp.match(rule, text)
#     print("\nResult: %s" % result)
#     print("rosdistro: %s" % result.rosdistro)

#     assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, 'http:/precise/hydro/turtlebot/dude#rocon_apps/chirp')
#     assert_raises(rocon_uri.RoconURIInvalidException, rocon_uri.parse, 'rocon:/precise//turtlebot/dude#rocon_apps/chirp')
#     assert(str(rocon_uri_object), rocon_uri_string)
        
# #
# # we define the rule as a list of (sub)rules
# #
# # r"\S"*   means regular expression specifying 
# #          any character except blank 
# rule=['sqs  ::=  parms  fileid ', 
#       'parms::=  r"\S"* ',        
#       'fileid::= r"\S"* ']        
# #
# # we concatenate arguments ... as words
# parms=' '.join(sys.argv[1:])
# #
# # we make the parsing 
# cmp=rule_parser.rp.match(rule,parms)
# # 
# #as re module, if the result object is None, 
# # the parsing is unsuccessful
# if cmp==None:
#     print "Error in parsing:"   
# else:
#     #
#     # now, to get values from parsing, 
#     # we use rule names as parser arguments.
#     # cmp.sqs    will contain input parameters
#     # cmp.parms  will contain string to locate
#     # cmp.fileid will contain fileid to search in
#     try:
#         id=open(cmp.fileid)
#         for l in id.readlines():
#             if l.find(cmp.parms)>-1: 
#                 print l[:-1]
#     except Exception,e:
#         print e
#     else:
#         id.close()

