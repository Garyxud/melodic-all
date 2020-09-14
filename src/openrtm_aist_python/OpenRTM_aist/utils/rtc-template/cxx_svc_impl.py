#!/usr/bin/env python
# -*- python -*-
#
#  @file cxx_svc_impl.py
#  @brief rtc-template C++ service source code generator class
#  @date $Date: 2007/02/07 02:51:49 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2005-2007
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id: cxx_svc_impl.py,v 1.4 2007/02/07 02:51:49 n-ando Exp $
# 

#
# $Log: cxx_svc_impl.py,v $
# Revision 1.4  2007/02/07 02:51:49  n-ando
# RCS Id tag was removed from template source code.
#
# Revision 1.3  2007/01/11 07:44:39  n-ando
# Now service implementation class does not inherit RtcServiceBase.
# The implementation template was changed.
#
# Revision 1.2  2005/09/06 14:37:18  n-ando
# rtc-template's command options and data structure for ezt (Easy Template)
# are changed for RTComponent's service features.
# Now rtc-template can generate services' skeletons, stubs and
# implementation files.
# The implementation code generation uses omniidl's IDL parser.
#
# Revision 1.1  2005/08/29 17:50:57  n-ando
# The first version.
#
#

import string
import os

# omniidl modules
import _omniidl
from omniidl import idlast, idlvisitor
from omniidl_be.cxx import ast, util, id, types, output

# import myself
import cxx_svc_impl
self = cxx_svc_impl


#------------------------------------------------------------
# Example code
#------------------------------------------------------------
interface_def = """\
/*
 * Example class implementing IDL interface @fq_name@
 */
class @impl_fqname@
 : public virtual @fq_POA_name@,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~@impl_name@();

 public:
   // standard constructor
   @impl_name@();
   virtual ~@impl_name@();

   // attributes and operations
   @operations@
};
"""

interface_code = """\
/*
 * Example implementational code for IDL interface @fqname@
 */
@impl_fqname@::@impl_name@()
{
  // Please add extra constructor code here.
}


@impl_fqname@::~@impl_name@()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
@operations@

// End of example implementational code
"""

class_h = """\
// -*-C++-*-
/*!
 * @atmark@file  @impl_h@
 * @atmark@brief Service implementation header of @file@
 *
 */

#include "@skel_h@"


#ifndef @include_guard@
#define @include_guard@
 
@interfaces@

#endif // @include_guard@

"""

class_cpp = """\
// -*-C++-*-
/*!
 * @atmark@file  @impl_cpp@
 * @atmark@brief Service implementation code of @file@
 *
 */

#include "@impl_h@"

@interfaces@
"""

def generate(idl_file, preproc_args, impl_suffix, skel_suffix = "Skel", fd_h=None, fd_cpp=None):
	basename = idl_file.replace(".idl","")
	preprocessor_cmd = "omnicpp"
	preprocessor_opt = "-I" + string.join(preproc_args, " -I")

	preproc_cmd = '%s %s %s' % (preprocessor_cmd,\
				  preprocessor_opt, idl_file)

	file = os.popen(preproc_cmd, "r")

	skel_filename = basename + skel_suffix + ".h"
	idl_filename = idl_file

	# ignore the following operations
	ignore_operations = ["profile"]
	
	tree = _omniidl.compile(file)
	ast.__init__(tree)

	cxx_svc_impl.__init__(idl_filename, \
					  basename, \
					  skel_filename, \
					  impl_suffix, \
					  ignore_operations, \
					  fd_h, \
					  fd_cpp)
	ifs = cxx_svc_impl.run(tree)
	file.close()
	_omniidl.clear()
	return ifs


#============================================================
# This module's __init__()
#============================================================
def __init__(idl_filename, impl_basename, skel_filename, \
		suffix = "_impl", ignore_op = [], fd_h = None, fd_cpp = None):
	self.idl_filename = idl_filename
	self.suffix = suffix
	self.impl_h_filename = impl_basename + self.suffix + ".h"
	self.impl_cpp_filename = impl_basename + self.suffix + ".cpp"
	self.skel_filename = skel_filename
	self.ignore_op = ignore_op

	self.include_guard = self.impl_h_filename.upper().replace(".","_")

	if fd_h == None:
		self.stream_h = \
		    output.Stream(output.createFile(self.impl_h_filename), 2)
	else:
		self.stream_h = output.Stream(fd_h, 2)
		
	if fd_cpp == None:
		self.stream_cpp = \
		    output.Stream(output.createFile(self.impl_cpp_filename), 2)
	else:
		self.stream_cpp = output.Stream(fd_cpp, 2)
		
	

# Given an IDL name convert it into the fully qualified name of the
# implementation class
def impl_fullname(name):
    bits = name.suffix(self.suffix).fullName()
    return string.join(bits, "_")

# Convert an IDL name into the simple name of the implementation class
def impl_simplename(name):
    return impl_fullname(name)

#
# Main code entrypoint
#
def run(tree):
	# The implementation template code stream
	impl_cpp = output.StringStream()
	# The implementation template header stream
	impl_h   = output.StringStream()
	bii = BuildInterfaceImplementations(impl_h, impl_cpp, self.ignore_op)
	tree.accept(bii)

	# Generate mplementation class template header
	stream_h.out(class_h,
				 atmark = "@",
				 impl_h = self.impl_h_filename,
				 include_guard = self.include_guard,
				 skel_h = self.skel_filename,
				 file = self.idl_filename,
				 interfaces = str(impl_h))
	
	# Generate implementation class template code
	stream_cpp.out(class_cpp,
				   atmark = "@",
				   impl_cpp = self.impl_cpp_filename,
				   impl_h = self.impl_h_filename,
				   file = self.idl_filename,
				   interfaces = str(impl_cpp))

	self.ifs = []
	for n in bii.allInterfaces():
		self.ifs.append(string.join(n.scopedName(), "_") + self.suffix)
	return self.ifs
	
	


#============================================================
# Build the interface implementations
#============================================================
class BuildInterfaceImplementations(idlvisitor.AstVisitor):

	def __init__(self, stream_h, stream_cpp, ignore_operations):
		self.stream_h = stream_h
		self.stream_cpp = stream_cpp
		self.ignore_operations = ignore_operations
		# keep track of all interfaces for later use
		self.__allInterfaces = []

	# Returns the list of all present interfaces (each one will be
	# implemented)
	def allInterfaces(self):
		return self.__allInterfaces[:]

	# Tree walking code
	def visitAST(self, node):
		for n in node.declarations():
			if ast.shouldGenerateCodeForDecl(n):
				n.accept(self)

	# modules can contain interfaces
	def visitModule(self, node):
		for n in node.definitions():
			n.accept(self)

	# interfaces cannot be further nested
	def visitInterface(self, node):
		self.__allInterfaces.append(node)
	
		scopedName = id.Name(node.scopedName())
		
		cxx_fqname = scopedName.fullyQualify()
		impl_flat_name = impl_fullname(scopedName)

		fqname = scopedName.fullyQualify(cxx = 0)

		
		# build methods corresponding to attributes, operations etc.
		# attributes[] and operations[] will contain lists of function
		# signatures eg
		#   [ char *echoString(const char *mesg) ]
		attributes = []
		operations = []
		virtual_operations = []

		# we need to consider all callables, including inherited ones
		# since this implementation class is not inheriting from anywhere
		# other than the IDL skeleton
		allInterfaces = [node] + ast.allInherits(node)
		allCallables = util.fold( map(lambda x:x.callables(), allInterfaces),
								  [], lambda x, y: x + y )


		# declarations[] contains a list of in-class decl signatures
		# implementations[] contains a list of out of line impl signatures
		# (typically differ by classname::)
		declarations = []
		implementations = []
		
		for c in allCallables:

			if isinstance(c, idlast.Attribute) :
				attrType = types.Type(c.attrType())
				d_attrType = attrType.deref()

				for i in c.identifiers():
					attribname = id.mapID(i)
					returnType = attrType.op(types.RET)
					inType = attrType.op(types.IN)
					attributes.append(returnType + " " + attribname + "()")
					# need a set method if not a readonly attribute
					if not c.readonly():
						args = attribname + "(" + inType + ")"
						declarations.append("void " + args)
						implementations.append("void " + impl_flat_name +\
											   "::" + args)
					if not attribname in self.ignore_operations:
						declarations.append(returnType + " " + attribname + "()")
						implementations.append(returnType + " " + impl_flat_name+\
										   "::" + attribname + "()")
			elif isinstance(c, idlast.Operation):
				params = []
				for p in c.parameters():
					paramType = types.Type(p.paramType())
					cxx_type = paramType.op(types.direction(p), use_out = 0)
					
					argname = id.mapID(p.identifier())
					params.append(cxx_type + " " + argname)

				# deal with possible "context"
				if c.contexts() != []:
					params.append("CORBA::Context_ptr _ctxt")

				return_type = types.Type(c.returnType()).op(types.RET)

				opname = id.mapID(c.identifier())
				if not opname in self.ignore_operations:
					arguments = string.join(params, ", ")
					args = opname + "(" + arguments + ")"
					declarations.append(return_type + " " + args)
					implementations.append(return_type + " " + \
										   impl_flat_name + \
										   "::" + args)
			else:
				util.fatalError("Internal error generating interface member")
				raise "No code for interface member: " + repr(c)

		# the class definition has no actual code...
		defs = string.join(map(lambda x:x + ";\n", declarations), "")

		# Output the class definition of the implementation
		self.stream_h.out(interface_def,
						  impl_fqname = impl_flat_name,
						  impl_name = impl_flat_name,
						  fq_name = fqname,
						  fq_POA_name = "POA_" + cxx_fqname,
						  operations = defs)

		# Output the class methods implementations
		impls = string.join(map(lambda x: x + """\

{
  // Please insert your code here and remove the following warning pragma
  #warning "Code missing in function <""" + x + """>"
}

""",
								implementations), "")
		
		self.stream_cpp.out(interface_code,
							fqname = fqname,
							impl_name = impl_flat_name,
							impl_fqname = impl_flat_name,
							operations = impls)


if __name__ == "__main__":
	import cxx_svc_impl
	import sys
	print "Interfaces:"
	print cxx_svc_impl.generate(sys.argv[1], "SVC_impl")
