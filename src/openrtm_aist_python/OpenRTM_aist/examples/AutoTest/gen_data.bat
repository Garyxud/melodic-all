for /L %%i in (0,1,15) do (
	echo %%i>>original-data
	echo %RANDOM% %RANDOM% %RANDOM% %RANDOM% %RANDOM%>>original-data
	echo message%%i>>original-data
	echo.>>original-data
)
