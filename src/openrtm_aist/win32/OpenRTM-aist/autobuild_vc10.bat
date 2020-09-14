@rem
@rem @brief Visual Studio automatic build script
@rem @date $Date: 2008-03-06 06:55:42 $
@rem @author Norkai Ando <n-ando@aist.go.jp>
@rem
@rem Copyright (C) 2008-2010
@rem     Noriaki Ando
@rem     Task-intelligence Research Group,
@rem     Intelligent Systems Research Institute,
@rem     National Institute of
@rem         Advanced Industrial Science and Technology (AIST), Japan
@rem     All rights reserved.
@rem
@rem $Id: autobuild_vc10.bat 726 2008-05-14 03:05:42Z n-ando $
@rem

@rem ------------------------------------------------------------
@rem Notice:
@rem   omniORB should be under the following OMNI_ROOT directory.
@rem   RTSE should be under the following OMNI_ROOT directory.
@rem ------------------------------------------------------------
@set RTM_ROOT=%~dp0
@set PATH="C:\Program Files\Microsoft Visual Studio 10.0\Common7\IDE\";"C:\Program Files\Microsoft Visual Studio 10.0\VC\BIN";"c:\WINDOWS\Microsoft.NET\Framework\v4.0.30319";"C:\Program Files\Microsoft Visual Studio 10.0\VC\vcpackages";%PATH%
@set OMNI_ROOT=C:\distribution\omniORB-4.1.4_vc10
@set RTSE_ROOT=C:\distribution\OpenRTP\RTSystemEditor
@set VC_VERSION=Visual C++ 2010
@set OPENCV_ROOT=C:\distribution\OpenCV2.2
@set OPENCV_RTC_ROOT=C:\distribution\ImageProcessing\opencv

@rem ============================================================
@rem copy property sheet
@rem ============================================================
copy   etc\rtm_config_omni414.vsprops rtm_config.vsprops
copy   etc\rtm_config_omni414.props rtm_config.props
copy   etc\rtm_config_omni414.props examples\USBCamera\rtm_config.props
copy   coil_config.props examples\USBCamera\coil_config.props

@rem ============================================================
@rem convert property sheet to cmake
@rem ============================================================
set TMP_PYTHONPATH=%PYTHONPATH%
set PYTHONPATH=./bin;%PYTHONPATH%

echo Generating rtm_config.cmake file
build\vsprops2cmake.py rtm_config.vsprops
move rtm_config.cmake cmake

echo Generating OpenRTMConfig.cmake file
build\cmakeconfgen.py rtm_config.vsprops
move OpenRTMConfig.cmake cmake

set PYTHONPATH=%TMP_PYTHONPATH%

@rem ============================================================
@rem build OpenRTM-aist
@rem ============================================================

msbuild /M:2 /t:rebuild /p:configuration=debug OpenRTM-aist_vc10.sln
msbuild /M:2 /t:rebuild /p:configuration=release OpenRTM-aist_vc10.sln

@rem ============================================================
@rem build USBCamera examples
@rem ============================================================
msbuild /M:2 /t:rebuild /p:configuration=debug examples\USBCamera\USBCamera_vc10.sln
msbuild /M:2 /t:rebuild /p:configuration=release examples\USBCamera\USBCamera_vc10.sln

@rem ============================================================
@rem build OpenCV-RTC
@rem ============================================================

cd %OPENCV_RTC_ROOT%
call copyprops.bat
msbuild /M:2 /t:rebuild ImageProcessing_opencv.sln
cd %RTM_ROOT%

cd installer
call autowix.cmd
cd ..

