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
