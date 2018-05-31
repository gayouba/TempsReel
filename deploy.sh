#!/bin/sh
scp -r superviseur_robot/destijl_init/* pi@10.105.1.9:~/superviseur_robot/destijl_init && ssh -t pi@10.105.1.9 "cd ~/superviseur_robot/destijl_init/ && make"
