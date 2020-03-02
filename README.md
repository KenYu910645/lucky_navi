# A* algorithm 
Navigation and localization stack for AMR(Automumous mobile robot)

![a_star_room](pic/test.png|width=100)

![dijkstra_room](pic/dijkstra_room.gif)
![a_star_near](pic/a_star_near.gif)
![dijkstra_near](pic/dijkstra_near.gif)
## Set up 
This project is build on Ubuntu 16.04, ROS-kinetic, Gazebo9

## Running 
Run A* algorithm 
```
$roslaunch lucky_navi global_planner.launch
```

# Argument 

Run map split algorithm
```
$roslaunch lucky_navi map_spliter.launch
```

Invoke a willowgarage map 
global_cartographer.py genarate global costmap 
global_planner.py genarate 
Using simple goal on the map of rviz to assign a goal for A* 
TODO : 
Using argument to swtich debug flag 
Using costmap-like mark to show marking


Run simulation at Gazebo
```
$gazebo_amr_willowgarage.launch
```


Run simulation at Gazebo
```
$roslaunch lucky_navi lucky_navi.launch
```

## How to change map 


## Node and topic relationship
