#!/bin/bash

sudo scp nvidia@192.168.90.231:/home/nvidia/path.csv /home/$USER/IROS_f1tenth/trajectory_generator/

source ~/miniconda3/etc/profile.d/conda.sh
conda activate py37

cd ~/Downloads/trajectory_generator/
echo What is the track width?
read track_width
python prep_file.py $track_width 
python main_globaltraj.py
python prep_final.py

sudo scp /home/$USER/IROS_f1tenth/trajectory_generator/final_raceline.csv nvidia@192.168.90.231:/home/nvidia/f1tenth_ws/src/pure_pursuit/checkpoints/

conda deactivate
