
@rem ------------------------------------------------------------
@rem Variable Settings
@rem   usually only %TARGET% might be changed
@rem ------------------------------------------------------------
@set PATH=%WIX%\bin;%PATH%
@set VERSION=1.1.0
@set TARGET=OpenRTM-aist-Python
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
@rem default distribution package folder
@rem ------------------------------------------------------------
@set DISTRIBUTION=C:\distribution
@set OPENRTM_PY=%DISTRIBUTION%\OpenRTM-aist-Python-1.1.0
@set OMNIORB_PY24=%DISTRIBUTION%\omniORBpy-3.0-Python2.4
@set OMNIORB_PY25=%DISTRIBUTION%\omniORBpy-3.4-Python2.5
@set OMNIORB_PY26=%DISTRIBUTION%\omniORBpy-3.4-Python2.6
@set RTSE_ROOT=C:\distribution\OpenRTP\RTSystemEditor

@rem ------------------------------------------------------------
@rem Supported languages
@rem   supported languages have to be specified
@rem ------------------------------------------------------------
set LANGUAGES=(ja-jp de-de es-es fr-fr hu-hu it-it ko-kr zh-tw)
copy OpenRTM-aist-Python.wxs.yaml.in OpenRTM-aist-Python.wxs.yaml
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
@rem Make OpenRTM-aist-Python file list
@rem ============================================================
python omniORBpy24wxs.py
python omniORBpy25wxs.py
python omniORBpy26wxs.py
python OpenRTMpywxs.py

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
   copy ..\makewxs.py .
   copy ..\yat.py .
   echo Generating OpenRTP_inc.wxs......
@rem   openrtpwxs.py
@rem   set PYTHONPATH=%TMP_PYTHONPATH%
   copy OpenRTP_inc.wxs ..
   del makewxs.py yat.py
   del *.yaml
   cd ..
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
       light.exe -ext WixUIExtension -ext WixUtilExtension -loc WixUI_!LC[%%i]!.wxl ^
            -out %TARGET_FULL%_!LC[%%i]!.msi %TARGET_WIXOBJ% %WIXUI_RTM_WIXOBJ%
    )
    if not exist WixUI_!LC[%%i]!.wxl (
        light.exe -ext WixUIExtension -ext WixUtilExtension -cultures:!LC[%%i]! ^
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
del *.yaml

pause;

