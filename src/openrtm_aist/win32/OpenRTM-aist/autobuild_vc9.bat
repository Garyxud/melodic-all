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
@rem $Id: autobuild_vc8.bat 726 2008-05-14 03:05:42Z n-ando $
@rem

@rem ------------------------------------------------------------
@rem Notice:
@rem   omniORB should be under the following OMNI_ROOT directory.
@rem   RTSE should be under the following OMNI_ROOT directory.
@rem ------------------------------------------------------------
@set RTM_ROOT=%~dp0
@set PATH="C:\Program Files\Microsoft Visual Studio 9.0\VC\vcpackages";%PATH%
@set OMNI_ROOT=C:\distribution\omniORB-4.1.4
@set RTSE_ROOT=C:\distribution\OpenRTP\RTSystemEditor
@set VC_VERSION=Visual C++ 2008
@set OPENCV_ROOT=C:\distribution\OpenCV2.1
@set OPENCV_RTC_ROOT=C:\distribution\ImageProcessing\opencv

@rem ============================================================
@rem copy property sheet
@rem ============================================================
copy   etc\rtm_config_omni414.vsprops rtm_config.vsprops

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

vcbuild /M2 /rebuild OpenRTM-aist_vc9.sln

@rem ============================================================
@rem build OpenCV-RTC
@rem ============================================================

cd %OPENCV_RTC_ROOT%
call copyprops.bat
vcbuild /M2 /rebuild OpenCV-RTC_vc9.sln
cd %RTM_ROOT%

cd installer
call autowix.cmd
cd ..

