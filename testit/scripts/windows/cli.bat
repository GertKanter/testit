@ECHO OFF
set argC=0
for %%x in (%*) do Set /A argC+=1
if %argC% ==0 goto USAGE
:: -v \"%1/output\":\"/output\" 
::shift
set CMD=docker exec -it testit_daemon /bin/bash -c "source /catkin_ws/devel/setup.bash && rosrun testit testit_command.py -v %*"
echo %CMD%
%CMD%
goto EOF
:USAGE
echo "Usage: %0 command"
echo "command - Testit CLI command"
echo "E.g., %0 status"
:EOF
