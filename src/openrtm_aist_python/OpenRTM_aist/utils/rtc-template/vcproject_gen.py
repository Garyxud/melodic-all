#!/usr/bin/env python
# -*- Python -*-
#

#  @file vcproject_gen.py
#  @brief rtc-template VC++ project file generator class
#  @date $Date: 2007/09/26 07:43:29 $
#  @author Noriaki Ando <n-ando@aist.go.jp> and S.Kurihara

import re
import os
import time
import StringIO
import ezt
import gen_base
import uuid

def description():
	return "VC++ project-file generator"

def usage_short():
	"""
	VC++ project-file generator specific usage (short version)
	"""
	return """
Options for VC++ backend:

"""

def usage():
	"""
	VC++ project-file generator specific usage
	"""
	return """
-------------------------------
  Help for VC++ project-file geenrator
-------------------------------
"""

def get_opt_fmt():
	opt_args_fmt = []
	return opt_args_fmt


proj_file = """<?xml version="1.0" encoding="shift_jis"?>
<VisualStudioProject
	ProjectType="Visual C++"
	Version="[vc_version]"
	Name="[fname]"
	ProjectGUID="[guid]"
	RootNamespace="[fname]"
	Keyword="Win32Proj"
	>
	<Platforms>
		<Platform
			Name="Win32"
		/>
	</Platforms>
	<ToolFiles>
	</ToolFiles>
	<Configurations>
		<Configuration
			Name="Debug|Win32"
			OutputDirectory=".\\Debug"
			IntermediateDirectory=".\\Debug"
			ConfigurationType="1"
			InheritedPropertySheets=".\\OpenRTM-aist.vsprops"
			CharacterSet="2"
			>
			<Tool
				Name="VCPreBuildEventTool"
			/>
			<Tool
				Name="VCCustomBuildTool"
			/>
			<Tool
				Name="VCXMLDataGeneratorTool"
			/>
			<Tool
				Name="VCWebServiceProxyGeneratorTool"
			/>
			<Tool
				Name="VCMIDLTool"
			/>
			<Tool
				Name="VCCLCompilerTool"
				Optimization="0"
				PreprocessorDefinitions="USE_stub_in_nt_dll;WIN32;_DEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0400;__NT__;__OSVERSION__=4;_CRT_SECURE_NO_DEPRECATE"
				MinimalRebuild="true"
				BasicRuntimeChecks="3"
				RuntimeLibrary="3"
				UsePrecompiledHeader="0"
				WarningLevel="3"
				Detect64BitPortabilityProblems="false"
				DebugInformationFormat="4"
			/>
			<Tool
				Name="VCManagedResourceCompilerTool"
			/>
			<Tool
				Name="VCResourceCompilerTool"
			/>
			<Tool
				Name="VCPreLinkEventTool"
			/>
			<Tool
				Name="VCLinkerTool"
				AdditionalDependencies="ACEd.lib RTC041d.lib omniORB407_rtd.lib omniDynamic407_rtd.lib omnithread32_rtd.lib advapi32.lib ws2_32.lib mswsock.lib"
				OutputFile="$(OutDir)\\[fname].exe"
				LinkIncremental="2"
				IgnoreDefaultLibraryNames=""
				GenerateDebugInformation="true"
				SubSystem="1"
				TargetMachine="1"
			/>
			<Tool
				Name="VCALinkTool"
			/>
			<Tool
				Name="VCManifestTool"
			/>
			<Tool
				Name="VCXDCMakeTool"
			/>
			<Tool
				Name="VCBscMakeTool"
			/>
			<Tool
				Name="VCFxCopTool"
			/>
			<Tool
				Name="VCAppVerifierTool"
			/>
			<Tool
				Name="VCWebDeploymentTool"
			/>
			<Tool
				Name="VCPostBuildEventTool"
			/>
		</Configuration>
		<Configuration
			Name="Release|Win32"
			OutputDirectory=".\\Release"
			IntermediateDirectory=".\\Release"
			ConfigurationType="1"
			InheritedPropertySheets=".\\OpenRTM-aist.vsprops"
			CharacterSet="0"
			WholeProgramOptimization="0"
			>
			<Tool
				Name="VCPreBuildEventTool"
			/>
			<Tool
				Name="VCCustomBuildTool"
			/>
			<Tool
				Name="VCXMLDataGeneratorTool"
			/>
			<Tool
				Name="VCWebServiceProxyGeneratorTool"
			/>
			<Tool
				Name="VCMIDLTool"
			/>
			<Tool
				Name="VCCLCompilerTool"
				PreprocessorDefinitions="USE_stub_in_nt_dll;WIN32;NDEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0400;__NT__;__OSVERSION__=4"
				RuntimeLibrary="2"
				UsePrecompiledHeader="0"
				WarningLevel="3"
				Detect64BitPortabilityProblems="false"
				DebugInformationFormat="3"
			/>
			<Tool
				Name="VCManagedResourceCompilerTool"
			/>
			<Tool
				Name="VCResourceCompilerTool"
			/>
			<Tool
				Name="VCPreLinkEventTool"
			/>
			<Tool
				Name="VCLinkerTool"
				AdditionalDependencies="ACE.lib RTC041.lib omniORB407_rt.lib omniDynamic407_rt.lib omnithread32_rt.lib advapi32.lib ws2_32.lib mswsock.lib"
				OutputFile="$(OutDir)/$(ProjectName).exe"
				LinkIncremental="1"
				GenerateDebugInformation="false"
				SubSystem="1"
				OptimizeReferences="2"
				EnableCOMDATFolding="2"
				LinkTimeCodeGeneration="0"
				TargetMachine="1"
			/>
			<Tool
				Name="VCALinkTool"
			/>
			<Tool
				Name="VCManifestTool"
			/>
			<Tool
				Name="VCXDCMakeTool"
			/>
			<Tool
				Name="VCBscMakeTool"
			/>
			<Tool
				Name="VCFxCopTool"
			/>
			<Tool
				Name="VCAppVerifierTool"
			/>
			<Tool
				Name="VCWebDeploymentTool"
			/>
			<Tool
				Name="VCPostBuildEventTool"
			/>
		</Configuration>
	</Configurations>
	<References>
	</References>
	<Files>
		<Filter
			Name="source file"
			Filter="cpp;c;cxx;def;odl;idl;hpj;bat;asm;asmx"
			UniqueIdentifier="[SourceGUID]"
			>
			<File
				RelativePath=".\\[fname_cpp]"
				>
			</File>
			<File
				RelativePath=".\\[fname_comp]"
				>
			</File>
		</Filter>
		<Filter
			Name="header file"
			Filter="h;hpp;hxx;hm;inl;inc;xsd"
			UniqueIdentifier="[HeaderGUID]"
			>
			<File
				RelativePath=".\\[fname_h]"
				>
			</File>
		</Filter>
		<Filter
			Name="resource file"
			Filter="rc;ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe;resx;tiff;tif;png;wav"
			UniqueIdentifier="[ResourceGUID]"
			>
		</Filter>
	</Files>
	<Globals>
	</Globals>
</VisualStudioProject>
""" 

props_file = """<?xml version="1.0" encoding="shift_jis"?>
<VisualStudioPropertySheet
	ProjectType="Visual C++"
	Version="8.00"
	Name="OpenRTM-aist-0.4"
	>
	<Tool
		Name="VCCLCompilerTool"
		AdditionalIncludeDirectories="&quot;$(rtm_root)&quot;;&quot;$(rtm_root)\\rtm\\idl&quot;;&quot;$(ace_include)&quot;;&quot;$(omni_include)&quot;;."
	/>
	<Tool
		Name="VCLinkerTool"
		AdditionalLibraryDirectories="&quot;$(rtm_root)\\bin&quot;;&quot;$(ace_libdir)&quot;;&quot;$(omni_libdir)&quot;"
	/>
	<UserMacro
		Name="ace_root"
		Value="%ACE_ROOT%"
	/>
	<UserMacro
		Name="omni_root"
		Value="%OMNI_ROOT%"
	/>
	<UserMacro
		Name="rtm_root"
		Value="%RTM_ROOT%"
	/>
	<UserMacro
		Name="ace_include"
		Value="$(ace_root)"
	/>
	<UserMacro
		Name="ace_libdir"
		Value="$(ace_root)\\lib"
	/>
	<UserMacro
		Name="omni_include"
		Value="$(omni_root)\\include"
	/>
	<UserMacro
		Name="omni_libdir"
		Value="$(omni_root)\\lib\\x86_win32"
	/>
	<UserMacro
		Name="omni_bin"
		Value="$(omni_root)\\bin\\x86_win32"
	/>
	<UserMacro
		Name="rtm_includes"
		Value="&quot;$(rtm_root)&quot;;&quot;$(rtm_root)\\rtm\\idl&quot;;&quot;$(ace_include)&quot;;&quot;$(omni_include)&quot;;."
	/>
	<UserMacro
		Name="rtm_libdir"
		Value="&quot;$(rtm_root)\\bin&quot;;&quot;$(ace_libdir)&quot;;&quot;$(omni_libdir)&quot;;."
	/>
	<UserMacro
		Name="rtm_path"
		Value="&quot;$(rtm_root)\\bin&quot;;&quot;$(rtm_root)\\build&quot;;&quot;$(omni_bin)&quot;"
	/>
</VisualStudioPropertySheet>
"""


class vcproject_gen(gen_base.gen_base):
	"""
	VC++ project-file generator
	"""
	_fname_space = 16
	def __init__(self, data, opts):
		self.data = data.copy()

		self.data["begin_brace"] = "["
		self.data["end_brace"] = "]"
		self.data["vcproject_file"] = self.data["fname"] + ".vcproj"
		self.data["props_file"] = "OpenRTM-aist.vsprops"
		self.data["l_name"] = self.data["fname"].lower()
		self.data["guid"] = uuid.uuid1()
		self.data["SourceGUID"] = uuid.uuid1()
		self.data["HeaderGUID"] = uuid.uuid1()
		self.data["ResourceGUID"] = uuid.uuid1()
		self.data["vc_version"] = "8.00"
		self.data["fname_h"] = self.data["fname"] + ".h"
		self.data["fname_cpp"] = self.data["fname"] + ".cpp"
		self.data["fname_comp"] = self.data["fname"] + "Comp.cpp"

		self.tags = {}
		self.gen_tags(self.tags)

		return

	def print_vcproject(self):
		"""
		Generate VC++ project-file
		"""
		self.gen(self.data["props_file"],
			 props_file, self.data, self.tags)

		self.gen(self.data["vcproject_file"],
			 proj_file, self.data, self.tags)
		return


	def print_all(self):
		self.print_vcproject()
