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
    cd rosrpi
```
   or 
   ```
    cd rosrpi
```
```
    git pull
```
```
    bash rpi_run/rpi_docker_run.bash
```

  #### In container:
  ```
        bash workspace/rpi_depend_KOSTYL.bash
```
```
        ./workspace/CONTAINER_START.bash
```
```
        roslaunch ...
```
