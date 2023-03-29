#!/bin/bash

# for i in {0..9}
# do
# if [ "$i" == "3" ] || [ "$i" == "4" ] || [ "$i" == "6" ] || [ "$i" == "7" ] || [ "$i" == "8" ] || [ "$i" == "9" ]; then
#   gnome-terminal --tab --title="Terminal $i" --command="bash -c 'python3 src/planning.py --part $i --type 1; $SHELL'"
# fi
# done

for i in {0..9}
do
if [ "$i" == "1" ] || [ "$i" == "4" ] || [ "$i" == "7" ] || [ "$i" == "9" ] || [ "$i" == "9" ] || [ "$i" == "9" ]; then
  gnome-terminal --tab --title="Terminal $i" --command="bash -c 'python3 src/planning.py --part $i --type 0; $SHELL'"
fi
done

