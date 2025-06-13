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

## LAUNCH on pc (if you want to lauch progect in sumulator)

###If you the problem with Docker (does not have access to /tmp/.X11-unix) This needs to be fixed in the Docker driver.
You need to:
1 Open Docker Desktop
2 Go to Settings (Settings)
3 Select Resources -> File Sharing
4 Add the /tmp path to the list of allowed sharing methods
5 Click "Apply and restart"

### Run container

if not work, try this
```
        docker run -it ilya9kkk/ros_itmo:base 
```
