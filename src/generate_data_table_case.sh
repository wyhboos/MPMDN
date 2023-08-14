#!/bin/bash

# for i in {0..9}
# do
# if [ "$i" == "3" ] || [ "$i" == "4" ] || [ "$i" == "6" ] || [ "$i" == "7" ] || [ "$i" == "8" ] || [ "$i" == "9" ]; then
#   gnome-terminal --tab --title="Terminal $i" --command="bash -c 'python3 src/generate_data.py --part $i --type 1; $SHELL'"
# fi
# done


# launch different move_group in different namespace
for i in {0..4}
do
  gnome-terminal --tab --title="move_group $i" --command="bash -c 'roslaunch panda_moveit_config group_demo.launch __ns:=/robot$i; $SHELL'"
  sleep 3
done

for i in {0..9}
do
# if [ "$i" == "1" ] || [ "$i" == "4" ] || [ "$i" == "7" ] || [ "$i" == "9" ] || [ "$i" == "9" ] || [ "$i" == "9" ]; then
  move_group_index=$((i/2))
  gnome-terminal --tab --title="Terminal $i" --command="bash -c 'rosrun moveit_tutorials mytest.py __ns:=/robot$move_group_index --thread_index $i; $SHELL'"
  sleep 0.5
# fi
done



# for i in {0..0}
# do
# # if [ "$i" == "1" ] || [ "$i" == "4" ] || [ "$i" == "7" ] || [ "$i" == "9" ] || [ "$i" == "9" ] || [ "$i" == "9" ]; then
#   move_group_index=$((i/2))
#   gnome-terminal --tab --title="Terminal $i" --command="bash -c 'rosrun moveit_tutorials mytest.py __ns:=/robot$move_group_index --thread_index $i; $SHELL'"
#   sleep 0.5
# # fi
# done

