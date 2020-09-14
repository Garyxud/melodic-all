@rem ------------------------------------------------------------
@rem IDL Compile bat
@rem   argument %1: Python script name
@rem   argument %2: Python version
@rem   argument %3: examples Path
@rem ------------------------------------------------------------
@rem @echo off
echo --- IDL Compile Python %2 Start ---
set PATH_TMP=%PATH%
set PATH=%CD%\;%PATH_TMP%
set PYTHONPATH=%CD%\Lib\site-packages

echo --- IDL Compile Python %1 %2 %3 ---
python %1 %2 %3

echo --- IDL Compile Python %2 Complete ---
