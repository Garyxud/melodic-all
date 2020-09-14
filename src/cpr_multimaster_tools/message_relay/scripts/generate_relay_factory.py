#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Paul Bovbel <pbovbel@clearpath.ai>
# @copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from Cheetah.Template import Template
import argparse
import pprint

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Generate topic relay factory source.')
    parser.add_argument('--msg-srv-names', metavar='msg_srv_names', nargs='*', help='Message/Service Names')
    parser.add_argument('--cpp-tmpl', metavar='*.cpp.tmpl', help='Input template', required=True)
    parser.add_argument('--cpp-out', metavar='*.cpp', help='Output source file', required=True)

    args = parser.parse_args()

    template_namespace = {}

    template_namespace['msg_srv_names'] = args.msg_srv_names
    template_namespace['pkg_names'] = set(s.rsplit('/', 1)[0] for s in args.msg_srv_names)

    # For debug, print definitions to console
    # pp = pprint.PrettyPrinter(indent=1)
    # pp.pprint(template_namespace)

    with open(args.cpp_tmpl, 'r') as f:
        source_template = Template(f.read(), searchList=[template_namespace])

    # print(template_namespace)
    with open(args.cpp_out, 'w') as f:
        f.write(str(source_template))
