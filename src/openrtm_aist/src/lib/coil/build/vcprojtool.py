#!/usr/bin/env python
#
# @brief VCProject file generator
# @date $Date: 2008-02-29 04:52:14 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: vcprojtool.py 775 2008-07-28 16:14:45Z n-ando $
#

#------------------------------------------------------------
# Generic vcproj template
#------------------------------------------------------------
vcproj_template = """<?xml version="1.0" encoding="shift_jis"?>
<VisualStudioProject
	ProjectType="[ProjectType]"
	Version="[Version]"
	Name="[RootNamespace]"
	ProjectGUID="{[ProjectGUID]}"
	RootNamespace="[RootNamespace]"
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
[for conf in Configurations]
		<Configuration
			Name="[conf.Name]"
			OutputDirectory="[conf.OutputDirectory]"
			IntermediateDirectory="[conf.IntermediateDirectory]"
			ConfigurationType="%d"
			CharacterSet="0"
[if-any conf.InheritedPropertySheets]
			InheritedPropertySheets="[conf.InheritedPropertySheets]"
[endif]
			>
%s
		</Configuration>
[endfor]
	</Configurations>
	<References>
	</References>
	<Files>
[if-any Source]
		<Filter
			Name="[Source.Name]"
			Filter="[Source.Filter]"
			UniqueIdentifier="{[Source.GUID]}"
			>
[if-any Source.Files][for file in Source.Files]
			<File
				RelativePath="[file.Path]"
				>
			</File>
[endfor][endif]
		</Filter>
[endif]
[if-any Header]
		<Filter
			Name="[Header.Name]"
			Filter="[Header.Filter]"
			UniqueIdentifier="{[Header.GUID]}"
			>
[if-any Header.Files][for file in Header.Files]
			<File
				RelativePath="[file.Path]"
				>
			</File>
[endfor][endif]
		</Filter>
[endif]
[if-any Resource]
		<Filter
			Name="[Resource.Name]"
			Filter="[Resource.Filter]"
			UniqueIdentifier="{[Resource.GUID]}"
			>
[if-any Resoruce.Files][for file in Resoruce.Files]
			<File
				RelativePath="[file.Path]"
				>
			</File>
[endfor][endif]
		</Filter>
[endif]
	</Files>
	<Globals>
	</Globals>
</VisualStudioProject>
"""

#------------------------------------------------------------
# ConfigurationType
#------------------------------------------------------------
conf_type = {"EXE": 1, "DLL": 2, "NMAKE": 3, "LIB": 4,
             "RTCEXE": 1, "RTCDLL": 2}

#------------------------------------------------------------
# Tool set for configuration
#------------------------------------------------------------
tools = {"EXE":
             ["VCPreBuildEventTool",
              "VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VCCLCompilerTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool",
              "VCPreLinkEventTool",
              "VCLinkerTool",
              "VCALinkTool",
              "VCManifestTool",
              "VCXDCMakeTool",
              "VCBscMakeTool",
              "VCFxCopTool",
              "VCAppVerifierTool",
              "VCWebDeploymentTool",
              "VCPostBuildEventTool"],
         "DLL":
             ["VCPreBuildEventTool",
              "VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VCCLCompilerTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool",
              "VCPreLinkEventTool",
              "VCLinkerTool",
              "VCALinkTool",
              "VCManifestTool",
              "VCXDCMakeTool",
              "VCBscMakeTool",
              "VCFxCopTool",
              "VCAppVerifierTool",
              "VCWebDeploymentTool",
              "VCPostBuildEventTool"],
         "LIB":
             ["VCPreBuildEventTool",
              "VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VCCLCompilerTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool",
              "VCPreLinkEventTool",
              "VCLibrarianTool",
              "VCALinkTool",
              "VCXDCMakeTool",
              "VCBscMakeTool",
              "VCFxCopTool",
              "VCPostBuildEventTool"]
         }
tools["RTCEXE"] = tools["EXE"]
tools["RTCDLL"] = tools["DLL"]


#------------------------------------------------------------
# Tool element
#------------------------------------------------------------
tool_elem = """			<Tool
				Name="%s"
[if-any conf.%s][for tool in conf.%s]
[if-any tool.Key]
				[tool.Key]="[tool.Value]"
[endif]
[endfor][endif]
			/>
"""


exeproj_yaml = """
ProjectType: Visual C++
Version: 8.00
Name: # Your Project Name
ProjectGUID: __GUID__
RootNamespace: 
Keyword: Win32Proj
Platforms:
  Platform:
    Name: Win32
Configurations:
  - Name: Debug
    OutputDirectory: $(ProjectDir)$(ConfigurationName)
    IntermediateDirectory: $(ConfigurationName)
    InheritedPropertySheets: # Set vsprops file if you need
"""


#------------------------------------------------------------
# Yaml template
#------------------------------------------------------------
exe_yaml = """ProjectType: "Visual C++"
Version: "8.00"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
#------------------------------------------------------------
# Debug Configuration
#------------------------------------------------------------
  - Name: "Debug|Win32"
    OutputDirectory: $(ProjectDir)$(ConfigurationName)"
    IntermediateDirectory: "$(ConfiguratioName)"
    ConfigurationType: "1"
#    InheritedPropertySheets:
    CharacterSet: "0"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VCCLCompilerTool:
      - Key: Optimization
        Value: 0
      - Key: PreprocessorDefinitions
        Value: "WIN32;_DEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0500;__NT__;__OSVERSION__=4"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "3"
      - Key: RuntimeLibrary
        Value: "3"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "4"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
#    VCPreLinkEventTool:
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: ""
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "2"
      - Key: IgnoreDefaultLibraryNames
        Value: ""
      - Key: GenerateDebugInformation
        Value: "true"
      - Key: SubSystem
        Value: "1"
      - Key: TargetMachine
        Value: "1"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
    VCPostBuildEventTool:
#------------------------------------------------------------
# Release Configuration
#------------------------------------------------------------
  - Name: "Release|Win32"
    OutputDirectory: $(ProjectDir)$(ConfigurationName)"
    IntermediateDirectory: "$(ConfiguratioName)"
    ConfigurationType: "1"
    InheritedPropertySheets: ""
    CharacterSet: "0"
    WholeProgramOptimization: "0"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "WIN32;NDEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0500;__NT__;__OSVERSION__=4"
      - Key: RuntimeLibrary
        Value: "2"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "3"
#    VCManagedResourceCompilerTool"
#    VCResourceCompilerTool"
#    VCPreLinkEventTool"
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: ""
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "1"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "1"
      - Key: OptimizeReferences
        Value: "2"
      - Key: EnableCOMDATFolding
        Value: "2"
      - Key: LinkTimeCodeGeneration
        Value: "0"
      - Key: TargetMachine
        Value: "1"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
#    VCPostBuildEventTool:
"""

dll_yaml = """ProjectType: "Visual C++"
Version: "8.00"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
  - Name: "Debug|Win32"
    OutputDirectory: "$(ProjectDir)$(ConfigurationName)"
    IntermediateDirectory: "$(ConfigurationName)"
    ConfigurationType: "2"
#    InheritedPropertySheets: ""
    CharacterSet: "0"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VCCLCompilerTool:
      - Key: Optimization
        Value: "0"
      - Key: PreprocessorDefinitions
        Value: "WIN32;_DEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0500;_CRT_SECURE_NO_DEPRECATE"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "3"
      - Key: RuntimeLibrary
        Value: "3"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "4"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
    VCPreLinkEventTool:
      - Key: CommandLine
        Value: |
          lib -out:"$(TargetDir)RTC_static.lib" "$(TargetDir)*.obj" "$(SolutionDir)\\\\rtm\\\\idl\\\\$(ConfigurationName)\\\\*.obj"
          set PATH=%PATH%;$(rtm_path)
          cd $(OutDir)
          start /wait cmd /c makedeffile.py RTC_static.lib RTC042d 0.4.1 RTC042d.def
          move RTC042d.def ..\\\\
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: ""
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.dll"
      - Key: Version
        Value: __VERSION__
      - Key: LinkIncremental
        Value: "2"
      - Key: ModuleDefinitionFile
        Value: "$(TargetName).def"
      - Key: GenerateDebugInformation
        Value: "true"
      - Key: SubSystem
        Value: "2"
      - Key: TargetMachine
        Value: "1"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
    VCPostBuildEventTool:
      - Key: CommandLine
        Value: |
          copy "$(OutDir)\\\\$(TargetName).lib" "$(SolutionDir)bin\\\\"
          copy "$(OutDir)\\\\$(TargetName).dll" "$(SolutionDir)bin\\\\"
  - Name: "Release|Win32"
    OutputDirectory: "$(ProjectDir)$(ConfigurationName)"
    IntermediateDirectory: "$(ConfigurationName)"
    ConfigurationType: "2"
    InheritedPropertySheets: ""
    CharacterSet: "0"
    WholeProgramOptimization: "0"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "WIN32;NDEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0500;_CRT_SECURE_NO_DEPRECATE"
      - Key:         RuntimeLibrary
        Value: "2"
      - Key:         UsePrecompiledHeader
        Value: "0"
      - Key:         WarningLevel
        Value: "3"
      - Key:         Detect64BitPortabilityProblems
        Value: "false"
      - Key:         DebugInformationFormat
        Value: "3"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
    VCPreLinkEventTool:
      - Key: CommandLine
        Value: |
          lib -out:"$(TargetDir)RTC_static.lib" "$(TargetDir)*.obj" "$(SolutionDir)\\\\rtm\\\\idl\\\\$(ConfigurationName)\\\\*.obj"
          set PATH=%PATH%;$(rtm_path)
          cd "$(OutDir)"
          start /wait cmd /c makedeffile.py RTC_static.lib RTC042 0.4.1 RTC042.def
          move RTC042.def ..\\\\
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: ""
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.dll"
      - Key: LinkIncremental
        Value: "1"
      - Key: ModuleDefinitionFile
        Value: "$(TargetName).def"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "2"
      - Key: OptimizeReferences
        Value: "2"
      - Key: EnableCOMDATFolding
        Value: "2"
      - Key: TargetMachine
        Value: "1"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
    VCPostBuildEventTool:
      - Key: CommandLine
        Value: |
          copy "$(OutDir)\\\\$(TargetName).lib" "$(SolutionDir)bin\\\\"
          copy "$(OutDir)\\\\$(TargetName).dll" "$(SolutionDir)bin\\\\"
"""
#------------------------------------------------------------
lib_yaml = """ProjectType: "Visual C++"
Version: "8.00"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
  - Name: "Debug|Win32"
    OutputDirectory: "$(ProjectDir)$(ConfigurationName)"
    IntermediateDirectory: "$(ConfigurationName)"
    ConfigurationType: "4"
#    InheritedPropertySheets: "..\\\\..\\\\OpenRTM-aist.vsprops"
    CharacterSet: "0"
    DeleteExtensionsOnClean: ""
    PreBuildEventTool:
      - Key: CommandLine
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do makewrapper.py %%x
          for %%x in (*.idl) do omniidl -bcxx -Wba -nf %%x
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VCCLCompilerTool:
      - Key: Optimization
        Value: "0"
      - Key: PreprocessorDefinitions
        Value: "WIN32;_DEBUG;_LIB;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0500;_CRT_SECURE_NO_DEPRECATE"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "3"
      - Key: RuntimeLibrary
        Value: "3"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "4"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
#    VCPreLinkEventTool:
    VCLibrarianTool:
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.lib"
#    VCALinkTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
    VCPostBuildEventTool:
      - Key: Description
        Value: "make .def file"
      - Key: CommandLine
        Value: |
          copy "$(OutDir)\\\\libRTCSkeld.lib" "$(SolutionDir)\\\\bin"
  - Name: "Release|Win32"
    OutputDirectory: "$(ProjectDir)$(ConfigurationName)"
    IntermediateDirectory: "$(ConfigurationName)"
    ConfigurationType: "4"
#    InheritedPropertySheets: "..\\\\..\\\\OpenRTM-aist.vsprops"
    CharacterSet: "0"
    WholeProgramOptimization: "0"
    VCPreBuildEventTool:
      - Key: CommandLine
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do makewrapper.py %%x
          for %%x in (*.idl) do omniidl -bcxx -Wba -nf %%x
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "WIN32;NDEBUG;_LIB;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0500;_CRT_SECURE_NO_DEPRECATE"
      - Key: RuntimeLibrary
        Value: "2"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "3"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
#    VCPreLinkEventTool:
    VCLibrarianTool:
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.lib"
#    VCALinkTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
    VCPostBuildEventTool:
      - Key: CommandLine
        Value: |
          copy "$(OutDir)\\\\libRTCSkel.lib" "$(SolutionDir)\\\\bin"
"""


rtcexe_yaml="""ProjectType: "Visual C++"
Version: "__VCVERSION__"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
#------------------------------------------------------------
# Debug Configuration
#------------------------------------------------------------
  - Name: "Debug|Win32"
    OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(ConfigurationName)"
    IntermediateDirectory: "__PROJECT_NAME__\\\\$(ConfigurationName)"
    ConfigurationType: "1"
    InheritedPropertySheets: "$(SolutionDir)rtm_config.vsprops;$(SolutionDir)user_config.vsprops"
    CharacterSet: "0"
    VCPreBuildEventTool:
      - Key: CommandLine
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VCCLCompilerTool:
      - Key: Optimization
        Value: 0
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;_DEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0500;__NT__;__OSVERSION__=4;_CRT_SECURE_NO_DEPRECATE"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "3"
      - Key: RuntimeLibrary
        Value: "3"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "true"
      - Key: DebugInformationFormat
        Value: "4"
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_libd)"
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "2"
      - Key: GenerateDebugInformation
        Value: "true"
      - Key: SubSystem
        Value: "1"
      - Key: TargetMachine
        Value: "1"
#------------------------------------------------------------
# Release Configuration
#------------------------------------------------------------
  - Name: "Release|Win32"
    OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(ConfigurationName)"
    IntermediateDirectory: "__PROJECT_NAME__\\\\$(ConfigurationName)"
    ConfigurationType: "1"
    InheritedPropertySheets: "$(SolutionDir)rtm_config.vsprops;$(SolutionDir)user_config.vsprops"
    CharacterSet: "0"
    WholeProgramOptimization: "0"
    VCPreBuildEventTool:
      - Key: CommandLine
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VCPostBuildEventTool:
      - Key: CommandLine
        Value: |
          if NOT EXIST "$(SolutionDir)\\\\components" mkdir "$(SolutionDir)\\\\components"
          copy "$(OutDir)\\\\__PROJECT_NAME__.exe" "$(SolutionDir)\\\\components"
    VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;NDEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0500;__NT__;__OSVERSION__=4;_CRT_SECURE_NO_DEPRECATE"
      - Key: RuntimeLibrary
        Value: "2"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "true"
      - Key: DebugInformationFormat
        Value: "3"
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_lib)"
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "1"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "1"
      - Key: OptimizeReferences
        Value: "2"
      - Key: EnableCOMDATFolding
        Value: "2"
      - Key: LinkTimeCodeGeneration
        Value: "0"
      - Key: TargetMachine
        Value: "1"
"""

rtcdll_yaml="""ProjectType: "Visual C++"
Version: "__VCVERSION__"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
#------------------------------------------------------------
# Debug Configuration
#------------------------------------------------------------
  - Name: "Debug|Win32"
    OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(ConfigurationName)"
    IntermediateDirectory: "__PROJECT_NAME__\\\\$(ConfigurationName)"
    ConfigurationType: "2"
    InheritedPropertySheets: "$(SolutionDir)rtm_config.vsprops;$(SolutionDir)user_config.vsprops"
    CharacterSet: "0"
    VCPreBuildEventTool:
      - Key: CommandLine
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VCCLCompilerTool:
      - Key: Optimization
        Value: "0"
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;_DEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0500;_CRT_SECURE_NO_DEPRECATE"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "3"
      - Key: RuntimeLibrary
        Value: "3"
      - Key: UsePrecompiledHeader
        Value: "0"
      - Key: WarningLevel
        Value: "3"
      - Key: Detect64BitPortabilityProblems
        Value: "true"
      - Key: DebugInformationFormat
        Value: "4"
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_libd)"
#      - Key: OutputFile
#        Value: "$(OutDir)\\\\__PROJECT_NAME__.dll"
#      - Key: Version
#        Value: __VERSION__
      - Key: LinkIncremental
        Value: "2"
#      - Key: ModuleDefinitionFile
#        Value: "$(TargetName).def"
      - Key: GenerateDebugInformation
        Value: "true"
      - Key: SubSystem
        Value: "2"
      - Key: TargetMachine
        Value: "1"
#------------------------------------------------------------
# Release Configuration
#------------------------------------------------------------
  - Name: "Release|Win32"
    OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(ConfigurationName)"
    IntermediateDirectory: "__PROJECT_NAME__\\\\$(ConfigurationName)"
    ConfigurationType: "2"
    InheritedPropertySheets: "$(SolutionDir)rtm_config.vsprops;$(SolutionDir)user_config.vsprops"
    CharacterSet: "0"
    WholeProgramOptimization: "0"
    VCPreBuildEventTool:
      - Key: CommandLine
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VCPostBuildEventTool:
      - Key: CommandLine
        Value: |
          if NOT EXIST "$(SolutionDir)\\\\components" mkdir "$(SolutionDir)\\\\components"
          copy "$(OutDir)\\\\__PROJECT_NAME__.dll" "$(SolutionDir)\\\\components"
    VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;NDEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0500;_CRT_SECURE_NO_DEPRECATE"
      - Key:         RuntimeLibrary
        Value: "2"
      - Key:         UsePrecompiledHeader
        Value: "0"
      - Key:         WarningLevel
        Value: "3"
      - Key:         Detect64BitPortabilityProblems
        Value: "true"
      - Key:         DebugInformationFormat
        Value: "3"
    VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_lib)"
#      - Key: OutputFile
#        Value: "$(OutDir)\\\\__PROJECT_NAME__.dll"
      - Key: LinkIncremental
        Value: "1"
#      - Key: ModuleDefinitionFile
#        Value: "$(TargetName).def"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "2"
      - Key: OptimizeReferences
        Value: "2"
      - Key: EnableCOMDATFolding
        Value: "2"
      - Key: TargetMachine
        Value: "1"
"""



def usage():
    print """Usage:
  vcprojtool.py cmd options
commands:
  vcproj: Generate vcproj
  yaml  : Generate example yaml file
  flist : Generate file list as yaml
examples:
  vcprojtool.py vcproj --type [exe|dll|nmake|lib]
                     --output out_fname
                     --yaml *.yaml
                     --source *.cpp
                     --header *.h
                     --resource *.txt
  vcprojtool.py yaml --type [exe|dll|nmake|lib] --output
  vcprojtool.py flist --out --source|--header|--resource *
"""

import sys

#------------------------------------------------------------
# Exceptions
#------------------------------------------------------------
class VCProjException:
    pass

class InvalidOption(VCProjException):
    def __init__(self, msg):
        self.msg = "Error: InvalidOption:\n    "
        self.msg += msg

class InvalidCommand(VCProjException):
    def __init__(self, msg):
        self.msg = "Error: InvalidCommand:\n    "
        self.msg += msg

#------------------------------------------------------------
# VCProject generator class
#------------------------------------------------------------
class VCProject:
    def __init__(self, type, yaml_text):
        import yaml
        self.type = type
        self.dict = yaml.load(yaml_text)
        self.escape_cmdline(self.dict)

    def generate(self):
        import yat
        self.template = yat.Template(self.get_template(self.type))
        return self.template.generate(self.dict).replace("\r\n", "\n").replace("\n", "\r\n")

    def tool_element(self, type):
        text = ""
        for tool in tools[type]:
            t = tool_elem % (tool, tool, tool)
            text += t
        return text

    def get_template(self, type):
        return vcproj_template % (conf_type[type], self.tool_element(type))

    def escape_cmdline(self, dict):
        if not dict.has_key("Configurations"): return
    
        def escape_cmd(text):
            text = text.replace("\"", "&quot;")
            text = text.replace("\r\n", "\n")
            text = text.replace("\n", "&#x0D;&#x0A;")
            return text
        from types import DictType, ListType
        for conf in dict["Configurations"]:
            for tool in conf.keys(): # Tool
                if isinstance(conf[tool], ListType):
                    for keyval in conf[tool]:
                        if isinstance(keyval, DictType) \
                                and keyval.has_key("Key") \
                                and keyval.has_key("Value") \
                                and keyval["Key"] == "CommandLine":
                            keyval["Value"] = escape_cmd(keyval["Value"])

#------------------------------------------------------------
# YAML configuration file generator
#------------------------------------------------------------
class YamlConfig:
    def __init__(self, type, vcversion, projectname, version, flist):
        self.type = type
        self.vcversion = vcversion
        self.projectname = projectname
        self.version = version
        self.flist = flist

        self.yaml_template = {"EXE": exe_yaml, "DLL": dll_yaml, "LIB": lib_yaml,
                              "RTCEXE": rtcexe_yaml, "RTCDLL": rtcdll_yaml}

    def load_yamls(self, yfiles):
        text = ""
        for f in yfiles:
            fd = open(f, "r")
            text += fd.read()
            fd.close()
        return text

    def replace_uuid(self, text):
        import uuid
        token0 = text.split("__GUID__")
        text0 = token0[0]
        for i in range(1, len(token0)):
            u = str(uuid.uuid1()).upper()
            text0 += u + token0[i]
    
        token1 = text0.split("__UUID")
        text1 = token1[0]
        for i in range(1, len(token1)):
            u = "_" + str(uuid.uuid1()).replace("-", "")
            text1 += u + token1[i]
        return text1

    def generate(self):
        text = ""
        loaded = ""
        if self.flist.has_key("yaml") and len(self.flist["yaml"]) > 0:
            loaded = self.load_yamls(self.flist["yaml"])

        if loaded.find("ProjectType:") < 0: # No toplevel config
            if self.yaml_template.has_key(self.type):
                text = self.yaml_template[self.type]
                text += loaded
            else:
                print "type should be specified."
                usage()
        else:
            text = loaded

        text += FileList(self.flist).generate()

        text = self.replace_uuid(text)
        if self.projectname:
            text = text.replace("__PROJECT_NAME__", self.projectname)
        if self.version:
            text = text.replace("__VERSION__", self.version)
        if self.vcversion:
            text = text.replace("__VCVERSION__", self.vcversion)
        return text

#------------------------------------------------------------
# File list yaml file generator
#------------------------------------------------------------
class FileList:
    def __init__(self, flist):
        self.flist = flist
        self.filter = {"source":
                      {"Id": "Source",
                       "name": "Source Files",
                       "filter": "cpp;c;cc;cxx;def;odl;idl;hpj;bat;asm;asmx",
                       },
                  "header":
                      {"Id": "Header",
                       "name": "Header Files",
                       "filter": "h;hpp;hxx;hm;inl;inc;xsd",
                       },
                  "resource":
                      {"Id": "Resoruce",
                       "name": "Resource Files",
                       "filter": "rc;ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe;resx;tiff;tif;png;wav",
                       }
                  }
        self.temp = """%s:
  Name: %s
  Filter: %s
  GUID: __GUID__
  Files:
"""
        return

    def generate(self):
        text = ""
        for f in ["source", "header", "resource"]:
            if len(self.flist[f]) > 0:
                text += self.temp % \
                    (self.filter[f]["Id"], self.filter[f]["name"],
                     self.filter[f]["filter"])
                for file in self.flist[f]:
                    # vcproj's path delimiter should be "\"
                    file = file.replace("/","\\")
                    text += "    - Path: " + file + "\n"
        return text



           

#def generate_vcproj(type, projectname, version, flist):
#    yaml_text = ""
#    for f in flist["yaml"]:
#        fd = open(f, "r")
#        yaml_text += fd.read()
#        fd.close()
#    yaml_text += generate_flist(flist)
#    yaml_text = replace_uuid(yaml_text)
#    if projectname:
#        yaml_text = yaml_text.replace("__PROJECT_NAME__", projectname)
#    if version:
#        yaml_text = yaml_text.replace("__VERSION__", version)
#    vcproj = VCProject(type, yaml_text)
#    return vcproj.generate()
#
# 
#
#
#def generate_yaml(type, projectname, version, flist):
#    yaml_template = {"EXE": exe_yaml, "DLL": dll_yaml, "LIB": lib_yaml}
#    text = yaml_template[type]
#    text += generate_flist(flist)
#    if projectname:
#        text = text.replace("__PROJECT_NAME__", projectname)
#    if version:
#        text = text.replace("__VERSION__", version)
#    return text

#------------------------------------------------------------
# command option
#------------------------------------------------------------
def parse_args(argv):
    cmd = argv[0]
    if not (cmd == "vcproj" or cmd == "flist" or cmd == "yaml"):
        raise InvalidCommand("no such command: " + cmd)
        
    outfname = None
    type = None
    vcversion = None
    projectname = None
    version = None
    flist = {"yaml": [], "source": [], "header": [], "resource": []}
    i = 1
    argc = len(argv)

    while i < argc:
        opt = argv[i]
        if opt == "--projectname":
            i += 1
            if i < argc: projectname = argv[i]
            else: raise InvalidOption(opt + " needs value")
        elif opt == "--version":
            i += 1
            if i < argc: version = argv[i]
            else: raise InvalidOption(opt + " needs value")
        elif opt == "--vcversion":
            i += 1
            if i < argc: vcversion = argv[i]
            else: raise InvalidOption(opt + " needs value")
        elif opt == "--output" or opt == "--out" or opt == "-o":
            i += 1
            if i < argc: outfname = argv[i]
            else: raise InvalidOption(opt + " needs value")
        elif opt == "--type" or opt == "-t":
            i += 1
            if i < argc: type = argv[i]
            else: raise InvalidOption(opt + " needs value")
            type = type.upper()
            if not conf_type.has_key(type):
                raise InvalidOption("unknown type: "
                                    + type + "\n" +
                                    "    --type should be [exe|dll|nmake|lib]")
        elif opt[:2] == "--" and flist.has_key(opt[2:]):
            lname = opt[2:]
            i += 1
            if not i < argc: raise InvalidOption(opt + " need value") 
            while i < argc and argv[i][:2] != "--":
                flist[lname].append(argv[i])
                i += 1
            if len(flist[lname]) == 0:
                raise InvalidOption(opt + " needs value")
            i -= 1
        else:
            raise InvalidOption("unknown option: " + opt)
        i += 1
    return (cmd, vcversion, projectname, version, outfname, type, flist)

#------------------------------------------------------------
# main function
#------------------------------------------------------------
def main(argv):
    if len(argv) == 0:
        usage()
        sys.exit(-1)

    try:
        res = parse_args(argv)
    except VCProjException, e:
        print "\n" + e.msg + "\n"
        usage()
        sys.exit(-1)

    cmd = res[0]
    vcversion = res[1]
    projectname = res[2]
    version = res[3]
    outfile = res[4]
    type = res[5]
    flist = res[6]

    if cmd == "vcproj":
        t = VCProject(type,
                      YamlConfig(type, vcversion,
                                 projectname, version, flist).generate()
                      ).generate()
    elif cmd == "flist":
        t = FileList(flist).generate()
    elif cmd == "yaml":
        t = YamlConfig(type, vcversion, projectname, version, flist).generate()

    if outfile == None:
        fd = sys.stdout
    else:
        fd = open(outfile, "wb")

    fd.write(t)
        
#------------------------------------------------------------
# tests
#------------------------------------------------------------
def test_filelist():
    print FileList({"source": ["hoge.cpp", "hage.cpp", "fuga.cpp"],
                    "header": ["hoge.h", "hage.h", "fuga.h"],
                    "resource": []}).generate()

def test_yamlconfig():
    print YamlConfig("EXE", "8.00", "Test", "0.9.1",
                     {"source":
                          ["hoge.cpp",
                           "hage.cpp",
                           "fuga.cpp"],
                      "header":
                          ["hoge.h", "hage.h", "fuga.h"],
                      "resource":
                          []}).generate()

def test_vcproj():
    print VCProject("EXE", YamlConfig("EXE", "8.00", "Test", "1.0.0",
                                      {"source":
                                           ["hoge.cpp",
                                            "hage.cpp",
                                            "fuga.cpp"],
                                       "header":
                                           ["hoge.h", "hage.h", "fuga.h"],
                                       "resource":
                                          [],
                                       "yaml":
                                           []}).generate()).generate()

#------------------------------------------------------------
# entry point
#------------------------------------------------------------
if __name__ == "__main__":
#    test_filelist()
#    test_yamlconfig()
#    test_vcproj()
    main(sys.argv[1:])
    
