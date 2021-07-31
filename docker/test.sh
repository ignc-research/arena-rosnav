#! /bin/sh
#source /usr/local/bin/virtualenvwrapper.sh
source /opt/ros/melodic/setup.sh
#source /root/.bashrc
#source /root/catkin_ws/devel/setup.bash


#export WORKON_HOME=/root/.python_env
#export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
#export VIRTUALENVWRAPPER_PYTHON=/root/.python_env/rosnav/bin/python3
#export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
#export PYTHONPATH=/root/catkin_ws/src/arena-rosnav:${PYTHONPATH}
#source /usr/local/bin/virtualenvwrapper.sh
source /root/catkin_ws/devel/setup.bash
#export PYTHONPATH=//geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
source /root/.python_env/rosnav/bin/activate
#mkvirtualenv --python=python3.6 testann2
workon testann2
#python3


# roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true         use_viz:=true  task_mode:=random
INPUT=train_params.csv
OLDIFS=$IFS
IFS=,
[ ! -f $INPUT ] && { echo "$INPUT file not found"; }
init=false

# python scripts/training/train_agent.py --agent MLP_ARENA2D

while read agent
do

	    if [ "$init" = false ] ;
		            then
				                init=true
						        continue;
							    fi

							        echo $agent
echo $1
								  done < $INPUT
								    # bash
