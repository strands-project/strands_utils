source /localhome/gestom/catkin/devel/setup.bash
if [ $(rostopic echo -n 1 /battery_state|grep powerSupplyPresent|grep -c True) == 0 ];then 
if [ $(($(rostopic echo -n 1 /battery_state |grep lifePercent|cut -f 2 -d ' ') < 100)) == 1 ];
then
a=$(rostopic echo -n 1 /battery_state |grep lifePercent|cut -f 2 -d ' ')
b=$(($a*))
b="had an accident or got lost"
cat battery.txt|sed s/XXX/$a/ |sed s/YYY/$b/|mail -r linda@lincoln.ac.uk -s "Battery low" cdondrup@gmail.com,pulidofentanes@gmail.com,tomkrajnik@hotmail.com
else
t update "Battery at $a %%"
fi
fi
