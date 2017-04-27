#!/bin/bash 
source ~/.bashrc
kuka_lwr_bringup_path=$(rospack find kuka_lwr_bringup)         
rqt --perspective-file $kuka_lwr_bringup_path/rqt_config/hand_ft.perspective
echo "set"