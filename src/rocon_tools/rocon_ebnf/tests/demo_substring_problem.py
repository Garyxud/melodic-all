#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
# Substrings in the original module caused problems:
#
# Matching against "foo" | "foo2" below would originally throw these errors:
#
# Parsing rule: "pattern" for string:"foo2"
# .Parsing terminal:"FOO" for string:"foo2"
# <Parsed terminal :"FOO" - value="foo"
# Parsed rule :"pattern" - value="foo"
# foo2
#
# This was originally caused by incorrectly getting the length of the term
# to match inbetween operators.
#
# 
##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import rocon_ebnf.rule_parser as rule_parser

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    print("==================================================")
    foo_rules = [
             'option verbose',
             'pattern           ::= "foo" | "foo2"',
              ]
    input_string = "foo2"
    result = rule_parser.match(foo_rules, input_string)
    print("Result %s" % result)
    print("==================================================")
    foo_rules = [
             'option verbose',
             'pattern           ::= r"foo" | r"foo2"',
              ]
    input_string = "foo2"
    result = rule_parser.match(foo_rules, input_string)
    print("Result %s" % result)
    print("==================================================")
