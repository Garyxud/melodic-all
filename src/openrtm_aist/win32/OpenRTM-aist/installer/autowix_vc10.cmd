@rem
@rem @brief WiX automatic build script
@rem @date $Date: 2008-02-09 20:04:03 $
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
@rem $Id: autowix.cmd.in 2083 2011-05-05 14:53:50Z kurihara $
@rem

@rem ------------------------------------------------------------
@rem Variable Settings
@rem   usually only %TARGET% might be changed
@rem ------------------------------------------------------------
@set PATH=%WIX%\bin;%PATH%
@set VERSION=1.1.0
@set TARGET=OpenRTM-aist
@set TARGET_WXS=%TARGET%.wxs
@set TARGET_WIXOBJ=%TARGET%.wixobj
@set TARGET_FULL=%TARGET%-%VERSION%

@rem ------------------------------------------------------------
@rem WixUI Customization Settings
@rem   usually only %WIXUI_RTM% might be changed
@rem ------------------------------------------------------------
@set WIXUI_RTM=WixUI_Mondo_rtm
@set WIXUI_RTM_WXS=%WIXUI_RTM%.wxs
@set WIXUI_RTM_WIXOBJ=%WIXUI_RTM%.wixobj

@rem ------------------------------------------------------------
@rem Supported languages
@rem   supported languages have to be specified
@rem ------------------------------------------------------------
set LANGUAGES=(ja-jp de-de es-es fr-fr hu-hu it-it ko-kr zh-tw)

echo off
@rem ------------------------------------------------------------
@rem Checking WiX
@rem ------------------------------------------------------------
if "x%WIX%" == "x" (
   echo "Windows Installer XML (WiX) is not installed"
   echo "Please download WiX 3.5 or later from http://wix.sourceforge.net/"
   goto END
)

@rem ------------------------------------------------------------
@rem Generate RTSystemEditor wxs file
@rem
@rem RTSystemEditorRCP.exe should be under %RTSE_ROOT%
@rem
@rem ------------------------------------------------------------
if "x%RTSE_ROOT%" == "x" (
   echo Envrionment variable "RTSE_ROOT" is not set. Abort.
   goto END
)
if not exist "%RTSE_ROOT%\RTSystemEditorRCP.exe" (
   echo RTSystemEditorRCP.exe does not found. Abort
   goto END
)
set INCLUDE_RTSE=YES
set INCLUDE_OPENRTP=YES

if not exist OpenRTP_inc.wxs (
   cd OpenRTP
rem set TMP_PYTHONPATH=%PYTHONPATH%
rem set PYTHONPATH=../../bin;%PYTHONPATH%
rem echo Generating OpenRTP_inc.wxs......
rem openrtpwxs.py
rem set PYTHONPATH=%TMP_PYTHONPATH%
   copy OpenRTP_inc.wxs ..
   cd ..
)


@rem ------------------------------------------------------------
@rem Generate omniORB's wxs file
@rem ------------------------------------------------------------
if "x%OMNI_ROOT%" == "x" (
   echo Environment Variable "OMNI_EOOR" is not set. Abort.
   goto END
)
set INCLUDE_OMNIORB=YES

if not exist omniORB_inc.wxs (
   cd omniORB
   set TMP_PYTHONPATH=%PYTHONPATH%
   set PYTHONPATH=../../bin;%PYTHONPATH%
   echo Generating omniORB_inc.wxs......
   omniwxs.py 'THIS_IS_OMNIORB_4_1_5'
   set PYTHONPATH=%TMP_PYTHONPATH%
   copy omniORB_inc.wxs ..
   cd ..
)


@rem ------------------------------------------------------------
@rem Generate OpenCV's wxs file
@rem ------------------------------------------------------------
if "x%OPENCV_ROOT%" == "x" (
   echo Environment Variable "OPENCV_ROOT" is not set. Abort.
   goto END
)
set INCLUDE_OPENCV=YES

if not exist OpenCV_inc.wxs (
   cd OpenCV2.3
   set TMP_PYTHONPATH=%PYTHONPATH%
   set PYTHONPATH=../../bin;%PYTHONPATH%
   echo Generating OpenCV_inc.wxs......
   opencvwxs.py
   set PYTHONPATH=%TMP_PYTHONPATH%
   copy OpenCV_inc.wxs ..
   cd ..
)


@rem ------------------------------------------------------------
@rem Generate OpenCV-RTC wxs file
@rem ------------------------------------------------------------
if "x%OPENCV_RTC_ROOT%" == "x" (
   echo Environment Variable "OPENCV_RTC_ROOT" is not set. Abort.
   goto END
)
set INCLUDE_OPENCV_RTC=YES

if not exist OpenCV-RTC_inc.wxs (
   cd OpenCV-RTC
   set TMP_PYTHONPATH=%PYTHONPATH%
   set PYTHONPATH=../../bin;%PYTHONPATH%
   echo Generating OpenCV-RTC_inc.wxs......
   opencvrtcwxs.py
   set PYTHONPATH=%TMP_PYTHONPATH%
   copy OpenCV-RTC_inc.wxs ..
   cd ..
)


@rem ------------------------------------------------------------
@rem Import Language-Country, Language codes, Codepages
@rem from langs.txt
@rem http://www.tramontana.co.hu/wix/lesson2.php#2.4
@rem ------------------------------------------------------------
for /F "tokens=1,2,3,4 delims=, " %%i in (langs.txt) do (
    set LC[%%j]=%%j
    set LANG[%%j]=%%k
    set CODE[%%j]=%%l
)

@rem ============================================================
@rem compile wxs file and link msi
@rem ============================================================
candle.exe %TARGET_WXS% %WIXUI_RTM_WXS% -dlanguage=1033 -dcodepage=1252
light.exe -ext WixUIExtension -loc WixUI_en-us.wxl ^
      	       -out %TARGET_FULL%.msi %TARGET_WIXOBJ% %WIXUI_RTM_WIXOBJ%

set IDS=1033
setlocal ENABLEDELAYEDEXPANSION

for %%i in %LANGUAGES% do (

    @rem ------------------------------------------------------------
    @rem language ID list
    @rem
    set IDS=!IDS!,!LANG[%%i]!

    @rem ------------------------------------------------------------
    @rem compile wxs file and link msi
    @rem
    candle.exe %TARGET_WXS% %WIXUI_RTM_WXS% -dlanguage=!LANG[%%i]! -dcodepage=!CODE[%%i]!

    if exist WixUI_!LC[%%i]!.wxl (
       light.exe -ext WixUIExtension -loc WixUI_!LC[%%i]!.wxl ^
            -out %TARGET_FULL%_!LC[%%i]!.msi %TARGET_WIXOBJ% %WIXUI_RTM_WIXOBJ%
    )
    if not exist WixUI_!LC[%%i]!.wxl (
        light.exe -ext WixUIExtension -cultures:!LC[%%i]! ^
            -out %TARGET_FULL%_!LC[%%i]!.msi %TARGET_WIXOBJ% %WIXUI_RTM_WIXOBJ%
    )
    @rem ------------------------------------------------------------
    @rem creating transformation files
    @rem
    torch.exe -p -t language %TARGET_FULL%.msi %TARGET_FULL%_!LC[%%i]!.msi ^
    	      -out !LC[%%i]!.mst

    @rem ------------------------------------------------------------
    @rem embed transformation files
    @rem
    cscript wisubstg.vbs %TARGET_FULL%.msi !LC[%%i]!.mst !LANG[%%i]!

)

@rem ------------------------------------------------------------
@rem here mst embedded msi can be selected languages by 
@rem > msiexec /i SampleMulti.msi TRANSFORMS=":fr-fr.mst"
@rem

@rem ------------------------------------------------------------
@rem Update the summary information stream to list all
@rem supported languages of this package
@rem ------------------------------------------------------------
cscript WiLangId.vbs %TARGET_FULL%.msi Package %IDS%

:END





@rem ------------------------------------------------------------
@rem References
@rem
@rem WiX Tutorial Lesson 9 "Transforms"
@rem http://www.tramontana.co.hu/wix/lesson9.php
@rem
@rem WiX Tutorial Lesson 2 "User Interface"
@rem http://www.tramontana.co.hu/wix/lesson2.php#2.4
@rem
@rem Multi-Language MSI Packages without Setup.exe Launcher
@rem http://www.installsite.org/pages/en/msi/articles/embeddedlang/index.htm
@rem
@rem vb scripts
@rem http://www.myitforum.com/articles/6/view.asp?id=1070
@rem
@rem ------------------------------------------------------------

