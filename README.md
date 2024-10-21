# Checkpoint 20 Tortoisebot and Navigation

command for checkpoint 20

To find tortoisebot IP address

```
nmap -sP 192.168.3.1/24
```

how to ssh

```
ssh tortoisebot@192.168.3.21
```

or if you need to transfer X windows to the working computer

```
ssh -X tortoisebot@192.168.3.21
```


```
username: tortoisebot
password:raspberry
```

All terminal must call

```
noetic
```

before any commands

# To go straight forward in x direction

```
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

# To stop

```
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```


To Visualize the Rviz

```
cd Documents/
rviz -d peerajak_rviz.rviz

```

To save flash drive to file .img

```
 sudo dd if=/dev/sdd of=$PWD/system_boot_2.img status=progress
```


To mount the flash drive save file .img 

- system-boot partition


```
sudo mount -o loop,offset=269484032 system_boot_2.img /media/peerajak/
```

- writable partition


To Unmount the above commands

```
sudo umount /media/peerajak/
```

To overlay the catkin_ws/carto_ws over ros1_ws

```
cd  ~/ros1_ws
source ~/ros1_ws/devel/setup.bash
catkin_make_isolated

```




