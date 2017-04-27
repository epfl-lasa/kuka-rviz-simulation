#!/bin/bash 
source ~/.bashrc
kuka_lwr_bringup_path=$(rospack find kuka_lwr_bringup)         
rqt --perspective-file $kuka_lwr_bringup_path/rqt_config/lwr2_all.perspective
echo "set"