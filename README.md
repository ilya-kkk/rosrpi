# ITMO omni robot project rep

## LAUNCH  (пока-что так потом пофиксим))

### First terminal: 
```
git clone https://github.com/ilya-kkk/rosrpi.git
```
```
cd rosrpi
```
  Or 
```
cd rosrpi
```
```
git pull
```
Then
```
./pc_run/docker_run.bash 
```

 #### In container:
 ```
 ./workspace/CONTAINER_START.bash 
```
```
roslaunch ...
```

### Second terminal on pc:
```
ssh rpi@rpi
```
  12345 (password)

```
git clone https://github.com/ilya-kkk/rosrpi.git
```
```
cd rosrpi/rpi_run/ && docker compose up -d && docker attach ros_rpi
```

  #### In container:
(мб уже пофиксил)
```
        ./workspace/CONTAINER_START.bash 
```
WITHOUT arduino launch 
```
        roslaunch core start.launch disable_ll:=true
```
normal launch
```
        roslaunch core start.launch 
```
## LAUNCH on pc (if you want to launch roscore to get information from rpi on your pc)
### Change directory to pc_run and run one of this two options
### best way
```
        docker compose up -d && docker attach ros_pc
```
### In this case entery point don`t execute 
```
        docker compose up -d && docker exec -it ros_pc bash
```

## LAUNCH on pc in simulator (if you want to launch project in simulator)

### Change directory to pc_run and run one of this two options
Run a container
```
        docker compose up -d && docker exec -it ros_pc bash
```