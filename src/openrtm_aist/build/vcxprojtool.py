#!/usr/bin/env python
#
# @brief VCXProject file generator
# @date $Date: 2008-02-29 04:52:14 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Tsuyoto Katami, Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: vcxprojtool.py 1668 2010-01-16 17:13:48Z n-ando $
#

#------------------------------------------------------------
# Generic vcxproj template
#------------------------------------------------------------
vcxproj_template = """<?xml version="1.0" encoding="utf-8"?>
<Project DefaultTargets="Build" ToolsVersion="[Version]" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <ItemGroup Label="ProjectConfigurations">
    <ProjectConfiguration Include="Debug|Win32">
      <Configuration>Debug</Configuration>
      <Platform>Win32</Platform>
    </ProjectConfiguration>
    <ProjectConfiguration Include="Debug|x64">
      <Configuration>Debug</Configuration>
      <Platform>x64</Platform>
    </ProjectConfiguration>
    <ProjectConfiguration Include="Release|Win32">
      <Configuration>Release</Configuration>
      <Platform>Win32</Platform>
    </ProjectConfiguration>
    <ProjectConfiguration Include="Release|x64">
      <Configuration>Release</Configuration>
      <Platform>x64</Platform>
    </ProjectConfiguration>
  </ItemGroup>
  <PropertyGroup Label="Globals">
    <ProjectName>[RootNamespace]</ProjectName>
    <ProjectGuid>{[ProjectGUID]}</ProjectGuid>
    <RootNamespace>[RootNamespace]</RootNamespace>
    <Keyword>Win32Proj</Keyword>
  </PropertyGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.Default.props" />
[for conf in Configurations]
  <PropertyGroup Condition="'$(Configuration)|$(Platform)'=='[conf.Name]'" Label="Configuration">
    <ConfigurationType>%s</ConfigurationType>
    <CharacterSet>NotSet</CharacterSet>
  </PropertyGroup>
[endfor]
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.props" />
  <ImportGroup Label="ExtensionSettings">
  </ImportGroup>
[for conf in Configurations]
  <ImportGroup Condition="'$(Configuration)|$(Platform)'=='[conf.Name]'" Label="PropertySheets">
    <Import Project="$(UserRootDir)\Microsoft.Cpp.$(Platform).user.props" Condition="exists('$(UserRootDir)\Microsoft.Cpp.$(Platform).user.props')" Label="LocalAppDataPlatform" />
[for inher in conf.VC10_InheritedPropertySheets]
[if-any inher]
    <Import Project="[inher]" />
[endif]
[endfor]
  </ImportGroup>
[endfor]
  <PropertyGroup Label="UserMacros" />
  <PropertyGroup>
    <_ProjectFileVersion>10.0.30319.1</_ProjectFileVersion>
[for conf in Configurations]
    <OutDir Condition="'$(Configuration)|$(Platform)'=='[conf.Name]'">[conf.VC10_OutputDirectory]\</OutDir>
    <IntDir Condition="'$(Configuration)|$(Platform)'=='[conf.Name]'">[conf.VC10_IntermediateDirectory]\</IntDir>
    <LinkIncremental Condition="'$(Configuration)|$(Platform)'=='[conf.Name]'">[conf.VC10_LinkIncrementalCondition]</LinkIncremental>
[endfor]
  </PropertyGroup>

[for conf in Configurations]
  <ItemDefinitionGroup Condition="'$(Configuration)|$(Platform)'=='[conf.Name]'">
    <!-- PreBuildEvent -->
    <PreBuildEvent>
%s
    </PreBuildEvent>
    <!-- ClCompile -->
    <ClCompile>
%s
    </ClCompile>
    <!-- Lib -->
    <Lib>
%s
    </Lib>
    <!-- PreLinkEvent -->
    <PreLinkEvent>
%s
    </PreLinkEvent>
    <!-- Link -->
    <Link>
%s
    </Link>
    <!-- PostBuildEvent -->
    <PostBuildEvent>
%s
    </PostBuildEvent>
  </ItemDefinitionGroup>
[endfor]
  <ItemGroup>
[if-any Source]
    <Filter Include="[Source.Name]">
      <UniqueIdentifier>{[Source.GUID]}</UniqueIdentifier>
      <Extensions>[Source.Filter]</Extensions>
    </Filter>
[endif]
[if-any Header]
    <Filter Include="[Header.Name]">
      <UniqueIdentifier>{[Header.GUID]}</UniqueIdentifier>
      <Extensions>[Header.Filter]</Extensions>
    </Filter>
[endif]
  </ItemGroup>
  <ItemGroup>
[if-any Source.Files][for file in Source.Files]
    <ClCompile Include="[file.Path]">
      <Filter>[Source.Name]</Filter>
    </ClCompile>
[endfor][endif]
  </ItemGroup>
  <ItemGroup>
[if-any Header.Files][for file in Header.Files]
    <ClInclude Include="[file.Path]">
      <Filter>[Header.Name]</Filter>
    </ClInclude>
[endfor][endif]
  </ItemGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.targets" />
  <ImportGroup Label="ExtensionTargets">
  </ImportGroup>
</Project>

"""

#------------------------------------------------------------
# ConfigurationType
#------------------------------------------------------------
conf_type = {"EXE": "Application", "DLL": "DynamicLibrary", 
             "NMAKE": "Makefile", 
             "LIB": "StaticLibrary",
             "RTCEXE": "Application", "RTCDLL": "DynamicLibrary"}


#------------------------------------------------------------
# Tool set for configuration
#------------------------------------------------------------
PreBuildEventtools = {"EXE":
             ["VC10_VCPreBuildEventTool",
              "VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool",
              "VCManifestTool"],
         "DLL":
             ["VC10_VCPreBuildEventTool"],
         "LIB":
             ["VC10_VCPreBuildEventTool"]
         }
PreBuildEventtools["RTCEXE"] = PreBuildEventtools["EXE"]
PreBuildEventtools["RTCDLL"] = PreBuildEventtools["DLL"]
Cltools = {"EXE":
             ["VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VC10_VCCLCompilerTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool",
              "VCManifestTool"],
         "DLL":
             ["VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VC10_VCCLCompilerTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool",
              "VCManifestTool"],
         "LIB":
             ["VCCustomBuildTool",
              "VCXMLDataGeneratorTool",
              "VCWebServiceProxyGeneratorTool",
              "VCMIDLTool",
              "VC10_VCCLCompilerTool",
              "VCManagedResourceCompilerTool",
              "VCResourceCompilerTool"]
         }
Cltools["RTCEXE"] = Cltools["EXE"]
Cltools["RTCDLL"] = Cltools["DLL"]
Libtools = {"EXE":
             ["VCLibrarianTool"],
         "DLL":
             ["VCLibrarianTool"],
         "LIB":
             ["VCLibrarianTool"]
         }
Libtools["RTCEXE"] = Libtools["EXE"]
Libtools["RTCDLL"] = Libtools["DLL"]
PreLinkEventtools = {"EXE":
             ["VC10_VCPreLinkEventTool"],
         "DLL":
             ["VC10_VCPreLinkEventTool"],
         "LIB":
             [""]
         }
PreLinkEventtools["RTCEXE"] = PreLinkEventtools["EXE"]
PreLinkEventtools["RTCDLL"] = PreLinkEventtools["DLL"]
Linktools = {"EXE":
             ["VC10_VCPreLinkEventTool",
              "VC10_VCLinkerTool",
              "VCALinkTool",
              "VCManifestTool",
              "VCXDCMakeTool",
              "VCBscMakeTool",
              "VCFxCopTool",
              "VCAppVerifierTool",
              "VCWebDeploymentTool"],
         "DLL":
             ["VC10_VCLinkerTool",
              "VCALinkTool",
              "VCManifestTool",
              "VCXDCMakeTool",
              "VCBscMakeTool",
              "VCFxCopTool",
              "VCAppVerifierTool",
              "VCWebDeploymentTool"],
         "LIB":
             [""]
         }
Linktools["RTCEXE"] = Linktools["EXE"]
Linktools["RTCDLL"] = Linktools["DLL"]
PostBuildEventtools = {"EXE":
             ["VC10_VCPostBuildEventTool"],
         "DLL":
             ["VC10_VCPostBuildEventTool"],
         "LIB":
             ["VC10_VCPostBuildEventTool"]
         }
PostBuildEventtools["RTCEXE"] = PostBuildEventtools["EXE"]
PostBuildEventtools["RTCDLL"] = PostBuildEventtools["DLL"]


#------------------------------------------------------------
# Tool element
#------------------------------------------------------------
tool_elem = """[if-any conf.%s][for tool in conf.%s]
[if-any tool.Key]
      <[tool.Key]>[tool.Value]</[tool.Key]>
[endif]
[endfor][endif]"""


exeproj_yaml = """
ProjectType: Visual C++
Version: "__VCVERSION__"
Name: # Your Project Name
ProjectGUID: __GUID__
RootNamespace: 
Keyword: Win32Proj
Platforms:
  Platform:
    Name: Win32
Configurations:
  - Name: Debug
    VC10_OutputDirectory: $(ProjectDir)$(Configuration)
    VC10_IntermediateDirectory: $(Configuration)
    VC10_InheritedPropertySheets: # Set vsprops file if you need
"""

def get_after_config(text):
  import re
  ret = ""
  flag = False
  for l in text.splitlines():
    m = re.match("^Configurations:", l)
    if m:
      flag = True
      continue
    if flag:
      ret += l.replace("Win32", "x64") + "\n"
  return ret

#------------------------------------------------------------
# Yaml template
#------------------------------------------------------------
exe_yaml = """ProjectType: "Visual C++"
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
    VC10_OutputDirectory: $(ProjectDir)$(Configuration)"
    VC10_IntermediateDirectory: "$(Configuratio)"
    ConfigurationType: "1"
#    VC10_InheritedPropertySheets:
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "true"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VC10_VCCLCompilerTool:
      - Key: Optimization
        Value: Disabled 
      - Key: PreprocessorDefinitions
        Value: "WIN32;_DEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0400;__NT__;__OSVERSION__=4;%(PreprocessorDefinitions)"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "EnableFastChecks"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDebugDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "EditAndContinue"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
#    VC10_VCPreLinkEventTool:
    VC10_VCLinkerTool:
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
        Value: "Console"
      - Key: TargetMachine
        Value: "MachineX86"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
    VC_10VCPostBuildEventTool:
    VCPreLinkEvent: 'lib -out:"$(TargetDir)coil_static.lib" "$(TargetDir)*.obj"
set PATH=%PATH%%3b$(coil_path)
cd "$(TargetDir)"
start /wait cmd /c makedeffile.py coil_static.lib coil$(coil_dllver)d $(coil_version) coil$(coil_dllver)d.def
move coil$(coil_dllver)d.def ..\'
    VC10_VCPostBuildEventTool: 'copy "$(OutDir)\$(TargetName).lib" "$(SolutionDir)bin\\coil$(coil_dllver)d.lib"
copy "$(OutDir)\coil$(coil_dllver)d.dll" "$(SolutionDir)bin\\"'
    VCPreLinkEvent: 'lib -out:"$(TargetDir)coil_static.lib" "$(TargetDir)*.obj"
set PATH=%PATH%%3b$(coil_path)
cd "$(OutDir)"
start /wait cmd /c makedeffile.py coil_static.lib coil$(coil_dllver) $(coil_version) coil$(coil_dllver).def
move coil$(coil_dllver).def ..\\'
    VC10_VCPostBuildEventTool: 'copy "$(OutDir)\$(TargetName).lib" "$(SolutionDir)bin\\coil$(coil_dllver).lib"
copy "$(OutDir)\coil$(coil_dllver).dll" "$(SolutionDir)bin\\"'
#------------------------------------------------------------
# Release Configuration
#------------------------------------------------------------
  - Name: "Release|Win32"
    VC10_OutputDirectory: $(ProjectDir)$(Configuration)"
    VC10_IntermediateDirectory: "$(Configuratio)"
    ConfigurationType: "1"
#    VC10_InheritedPropertySheets: ""
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "false"
    WholeProgramOptimization: "0"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VC10_VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "WIN32;NDEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0400;__NT__;__OSVERSION__=4;%(PreprocessorDefinitions)"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "ProgramDatabase"
#    VCManagedResourceCompilerTool"
#    VCResourceCompilerTool"
#    VC10_VCPreLinkEventTool"
    VC10_VCLinkerTool:
      - Key: AdditionalDependencies
        Value: ""
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "1"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "Console"
      - Key: OptimizeReferences
        Value: "true"
      - Key: EnableCOMDATFolding
        Value: "true"
      - Key: LinkTimeCodeGeneration
        Value: ""
      - Key: TargetMachine
        Value: "MachineX86"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
#    VC10_VCPostBuildEventTool:
"""
exe_yaml = exe_yaml + get_after_config(exe_yaml)

dll_yaml = """ProjectType: "Visual C++"
Version: "__VCVERSION__"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
  - Name: "Debug|Win32"
    VC10_OutputDirectory: "$(ProjectDir)$(Configuration)"
    VC10_IntermediateDirectory: "$(Configuration)"
    ConfigurationType: "2"
#    VC10_InheritedPropertySheets: ""
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "true"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VC10_VCCLCompilerTool:
      - Key: Optimization
        Value: "Disabled"
      - Key: PreprocessorDefinitions
        Value: "WIN32;_DEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0400;_CRT_SECURE_NO_DEPRECATE"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "EnableFastChecks"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDebugDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "EditAndContinue"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
    VC10_VCPreLinkEventTool:
      - Key: Command
        Value: |
          lib -out:"$(TargetDir)RTC_static.lib" "$(TargetDir)*.obj" "$(SolutionDir)\\\\rtm\\\\idl\\\\$(ConfigurationName)\\\\*.obj"
          set PATH=%PATH%;$(rtm_path)
          cd $(OutDir)
          start /wait cmd /c makedeffile.py RTC_static.lib RTC042d 0.4.1 RTC042d.def
          move RTC042d.def ..\\\\
    VC10_VCLinkerTool:
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
        Value: "Windows"
      - Key: TargetMachine
        Value: "MachineX86"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
    VC10_VCPostBuildEventTool:
      - Key: Command
        Value: |
          copy "$(OutDir)\\\\$(TargetName).lib" "$(SolutionDir)bin\\\\"
          copy "$(OutDir)\\\\$(TargetName).dll" "$(SolutionDir)bin\\\\"
  - Name: "Release|Win32"
    VC10_OutputDirectory: "$(ProjectDir)$(Configuration)"
    VC10_IntermediateDirectory: "$(Configuration)"
    ConfigurationType: "2"
    VC10_InheritedPropertySheets: ""
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "false"
    WholeProgramOptimization: "0"
#    VCPreBuildEventTool:
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VC10_VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "WIN32;NDEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0400;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key:         RuntimeLibrary
        Value: "MultiThreadedDLL"
      - Key:         UsePrecompiledHeader
        Value: "NotUsing"
      - Key:         WarningLevel
        Value: "Level3"
      - Key:         Detect64BitPortabilityProblems
        Value: "false"
      - Key:         DebugInformationFormat
        Value: "ProgramDatabase"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
    VC10_VCPreLinkEventTool:
      - Key: Command
        Value: |
          lib -out:"$(TargetDir)RTC_static.lib" "$(TargetDir)*.obj" "$(SolutionDir)\\\\rtm\\\\idl\\\\$(ConfigurationName)\\\\*.obj"
          set PATH=%PATH%;$(rtm_path)
          cd "$(OutDir)"
          start /wait cmd /c makedeffile.py RTC_static.lib RTC042 0.4.1 RTC042.def
          move RTC042.def ..\\\\
    VC10_VCLinkerTool:
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
        Value: "Windows"
      - Key: OptimizeReferences
        Value: "true"
      - Key: EnableCOMDATFolding
        Value: "true"
      - Key: TargetMachine
        Value: "MachineX86"
#    VCALinkTool:
#    VCManifestTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
#    VCAppVerifierTool:
#    VCWebDeploymentTool:
    VC10_VCPostBuildEventTool:
      - Key: Command
        Value: |
          copy "$(OutDir)\\\\$(TargetName).lib" "$(SolutionDir)bin\\\\"
          copy "$(OutDir)\\\\$(TargetName).dll" "$(SolutionDir)bin\\\\"
"""
dll_yaml = dll_yaml + get_after_config(dll_yaml)

#------------------------------------------------------------
lib_yaml = """ProjectType: "Visual C++"
Version: "__VCVERSION__"
Name: __PROJECT_NAME__
ProjectGUID: __GUID__
RootNamespace: __PROJECT_NAME__
Keyword: "Win32Proj"
Configurations:
  - Name: "Debug|Win32"
    VC10_OutputDirectory: "$(ProjectDir)$(Configuration)"
    VC10_IntermediateDirectory: "$(Configuration)"
    ConfigurationType: "4"
#    VC10_InheritedPropertySheets: "..\\\\..\\\\OpenRTM-aist.vsprops"
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "true"
    DeleteExtensionsOnClean: ""
    PreBuildEvent:
      - Key: Command
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do makewrapper.py %%x
          for %%x in (*.idl) do omniidl -bcxx -Wba -nf %%x
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VC10_VCCLCompilerTool:
      - Key: Optimization
        Value: "Disabled"
      - Key: PreprocessorDefinitions
        Value: "WIN32;_DEBUG;_LIB;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0400;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "EnableFastChecks"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDebugDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "EditAndContinue"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
#    VC10_VCPreLinkEventTool:
    VCLibrarianTool:
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.lib"
#    VCALinkTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
    VC10_VCPostBuildEventTool:
      - Key: Description
        Value: "make .def file"
      - Key: Command
        Value: |
          copy "$(OutDir)\\\\libRTCSkeld.lib" "$(SolutionDir)\\\\bin"
  - Name: "Release|Win32"
    VC10_OutputDirectory: "$(ProjectDir)$(Configuration)"
    VC10_IntermediateDirectory: "$(Configuration)"
    ConfigurationType: "4"
#    VC10_InheritedPropertySheets: "..\\\\..\\\\OpenRTM-aist.vsprops"
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "false"
    WholeProgramOptimization: "0"
    PreBuildEvent:
      - Key: Command
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do makewrapper.py %%x
          for %%x in (*.idl) do omniidl -bcxx -Wba -nf %%x
#    VCCustomBuildTool:
#    VCXMLDataGeneratorTool:
#    VCWebServiceProxyGeneratorTool:
#    VCMIDLTool:
    VC10_VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "WIN32;NDEBUG;_LIB;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0400;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "ProgramDatabase"
#    VCManagedResourceCompilerTool:
#    VCResourceCompilerTool:
#    VC10_VCPreLinkEventTool:
    VCLibrarianTool:
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.lib"
#    VCALinkTool:
#    VCXDCMakeTool:
#    VCBscMakeTool:
#    VCFxCopTool:
    VC10_VCPostBuildEventTool:
      - Key: Command
        Value: |
          copy "$(OutDir)\\\\libRTCSkel.lib" "$(SolutionDir)\\\\bin"
"""
lib_yaml = lib_yaml + get_after_config(lib_yaml)

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
    VC10_OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(Configuration)"
    VC10_IntermediateDirectory: "__PROJECT_NAME__\\\\$(Configuration)"
    ConfigurationType: "1"
    VC10_InheritedPropertySheets: 
      - "$(SolutionDir)user_config.props"
      - "$(SolutionDir)rtm_config.props"
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "true"
    PreBuildEvent:
      - Key: Command
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VC10_VCCLCompilerTool:
      - Key: Optimization
        Value: Disabled
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;_DEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0400;__NT__;__OSVERSION__=4;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "EnableFastChecks"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDebugDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "EditAndContinue"
    VC10_VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_libd);%(AdditionalDependencies)"
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "2"
      - Key: GenerateDebugInformation
        Value: "true"
      - Key: SubSystem
        Value: "Console"
      - Key: TargetMachine
        Value: "MachineX86"
#------------------------------------------------------------
# Release Configuration
#------------------------------------------------------------
  - Name: "Release|Win32"
    VC10_OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(Configuration)"
    VC10_IntermediateDirectory: "__PROJECT_NAME__\\\\$(Configuration)"
    ConfigurationType: "1"
    VC10_InheritedPropertySheets: 
      - "$(SolutionDir)user_config.props"
      - "$(SolutionDir)rtm_config.props"
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "false"
    WholeProgramOptimization: "0"
    PreBuildEvent:
      - Key: Command
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VC10_VCPostBuildEventTool:
      - Key: Command
        Value: |
          if NOT EXIST "$(SolutionDir)\\\\components" mkdir "$(SolutionDir)\\\\components"
          copy "$(OutDir)\\\\__PROJECT_NAME__.exe" "$(SolutionDir)\\\\components"
    VC10_VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;NDEBUG;_CONSOLE;__WIN32__;__x86__;_WIN32_WINNT=0x0400;__NT__;__OSVERSION__=4;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "ProgramDatabase"
    VC10_VCLinkerTool:

      - Key: AdditionalDependencies
        Value: "$(rtm_lib);%(AdditionalDependencies)"
      - Key: OutputFile
        Value: "$(OutDir)\\\\__PROJECT_NAME__.exe"
      - Key: LinkIncremental
        Value: "1"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "Console"
      - Key: OptimizeReferences
        Value: "true"
      - Key: EnableCOMDATFolding
        Value: "true"
      - Key: LinkTimeCodeGeneration
        Value: ""
      - Key: TargetMachine
        Value: "MachineX86"
"""
rtcexe_yaml = rtcexe_yaml + get_after_config(rtcexe_yaml)

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
    VC10_OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(Configuration)"
    VC10_IntermediateDirectory: "__PROJECT_NAME__\\\\$(Configuration)"
    ConfigurationType: "2"
    VC10_InheritedPropertySheets: 
      - "$(SolutionDir)user_config.props"
      - "$(SolutionDir)rtm_config.props"
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "true"
    PreBuildEvent:
      - Key: Command
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VC10_VCCLCompilerTool:
      - Key: Optimization
        Value: "Disabled"
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;_DEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0400;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key: MinimalRebuild
        Value: "true"
      - Key: BasicRuntimeChecks
        Value: "EnableFastChecks"
      - Key: RuntimeLibrary
        Value: "MultiThreadedDebugDLL"
      - Key: PrecompiledHeader
        Value: "NotUsing"
      - Key: WarningLevel
        Value: "Level3"
      - Key: Detect64BitPortabilityProblems
        Value: "false"
      - Key: DebugInformationFormat
        Value: "EditAndContinue"
    VC10_VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_libd);%(AdditionalDependencies)"
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
        Value: "Windows"
      - Key: TargetMachine
        Value: "MachineX86"
#------------------------------------------------------------
# Release Configuration
#------------------------------------------------------------
  - Name: "Release|Win32"
    VC10_OutputDirectory: "$(ProjectDir)__PROJECT_NAME__\\\\$(Configuration)"
    VC10_IntermediateDirectory: "__PROJECT_NAME__\\\\$(Configuration)"
    ConfigurationType: "2"
    VC10_InheritedPropertySheets: 
      - "$(SolutionDir)user_config.props"
      - "$(SolutionDir)rtm_config.props"
    CharacterSet: "0"
    VC10_LinkIncrementalCondition: "false"
    WholeProgramOptimization: "0"
    PreBuildEvent:
      - Key: Command
        Value: |
          set PATH=$(rtm_path);%PYTHON_ROOT%\\\\;%PATH%
          for %%x in (*.idl) do rtm-skelwrapper.py --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=%%x
          for %%x in (*.idl) do $(rtm_idlc) $(rtm_idlflags) %%x
    VC10_VCPostBuildEventTool:
      - Key: Command
        Value: |
          if NOT EXIST "$(SolutionDir)\\\\components" mkdir "$(SolutionDir)\\\\components"
          copy "$(OutDir)\\\\__PROJECT_NAME__.dll" "$(SolutionDir)\\\\components"
    VC10_VCCLCompilerTool:
      - Key: PreprocessorDefinitions
        Value: "USE_stub_in_nt_dll;WIN32;NDEBUG;_WINDOWS;_USRDLL;__WIN32__;__NT__;__OSVERSION__=4;__x86__;_WIN32_WINNT=0x0400;_CRT_SECURE_NO_DEPRECATE;%(PreprocessorDefinitions)"
      - Key:         RuntimeLibrary
        Value: "MultiThreadedDLL"
      - Key:         PrecompiledHeader
        Value: "NotUsing"
      - Key:         WarningLevel
        Value: "Level3"
      - Key:         Detect64BitPortabilityProblems
        Value: "false"
      - Key:         DebugInformationFormat
        Value: "ProgramDatabase"
    VC10_VCLinkerTool:
      - Key: AdditionalDependencies
        Value: "$(rtm_lib);%(AdditionalDependencies)"
#      - Key: OutputFile
#        Value: "$(OutDir)\\\\__PROJECT_NAME__.dll"
      - Key: LinkIncremental
        Value: "1"
#      - Key: ModuleDefinitionFile
#        Value: "$(TargetName).def"
      - Key: GenerateDebugInformation
        Value: "false"
      - Key: SubSystem
        Value: "Windows"
      - Key: OptimizeReferences
        Value: "true"
      - Key: EnableCOMDATFolding
        Value: "true"
      - Key: TargetMachine
        Value: "MachineX86"
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
rtcdll_yaml = rtcdll_yaml + get_after_config(rtcdll_yaml)

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

    def PreBuildEventtool_element(self, type):
        text = ""
        for tool in PreBuildEventtools[type]:
            t = tool_elem % (tool, tool)
            text += t
        return text

    def CLtool_element(self, type):
        text = ""
        for tool in Cltools[type]:
            t = tool_elem % (tool, tool)
            text += t
        return text

    def Libtool_element(self, type):
        text = ""
        for tool in Libtools[type]:
            t = tool_elem % (tool, tool)
            text += t
        return text

    def PreLinkEventtool_element(self, type):
        text = ""
        for tool in PreLinkEventtools[type]:
            t = tool_elem % (tool, tool)
            text += t
        return text

    def Linktool_element(self, type):
        text = ""
        for tool in Linktools[type]:
            t = tool_elem % (tool, tool)
            text += t
        return text

    def PostBuildEventtool_element(self, type):
        text = ""
        for tool in PostBuildEventtools[type]:
            t = tool_elem % (tool, tool)
            text += t
        return text


    def get_template(self, type):
        #return vcxproj_template % (conf_type[type], conf_type[type], self.tool_element(type))
        return vcxproj_template % (conf_type[type], self.PreBuildEventtool_element(type), self.CLtool_element(type), self.Libtool_element(type), self.PreLinkEventtool_element(type), self.Linktool_element(type), self.PostBuildEventtool_element(type))

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
                                and keyval["Key"] == "Command":
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

        print self.flist

        text += FileList(self.flist).generate()

        text = self.replace_uuid(text)
        if self.projectname:
            text = text.replace("__PROJECT_NAME__", self.projectname)
        if self.version:
            text = text.replace("__VERSION__", self.version)
        if self.vcversion:
            text = text.replace("__VCVERSION__", self.vcversion)
            text = text.replace("__VCSHORTVER__",
                                self.vcversion.replace(".",""))
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
    if not (cmd == "vcxproj" or cmd == "flist" or cmd == "yaml"):
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

    if cmd == "vcxproj":
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
    print YamlConfig("EXE", "10.00", "Test", "0.9.1",
                     {"source":
                          ["hoge.cpp",
                           "hage.cpp",
                           "fuga.cpp"],
                      "header":
                          ["hoge.h", "hage.h", "fuga.h"],
                      "resource":
                          []}).generate()

def test_vcproj():
    print VCProject("EXE", YamlConfig("EXE", "10.00", "Test", "1.0.0",
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
    
