@ECHO OFF
set argC=0
for %%x in (%*) do Set /A argC+=1
if %argC%==2 goto EXECUTE
echo "Usage: %0 host_project_directory launch_file_name"
echo "host_project_directory - The directory outside Docker environment where the TestIt project is located."
echo "E.g., %0 C:\Users\user\testit_project testit.launch"
goto EOF
:EXECUTE
:: -v \"%1/output\":\"/output\" 
set CMD=docker run --rm -v //var//run//docker.sock:/var/run/docker.sock -v "%1":"/catkin_ws/src/testit_project" --name testit_daemon -d testitros/testit:latest /bin/bash -c "source /catkin_ws/devel/setup.bash && cd /catkin_ws/ && catkin_make && roslaunch /catkin_ws/src/testit_project/launch/%2"
echo %CMD%
%CMD%
:EOF
