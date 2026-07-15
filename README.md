# THIS FORK INFORMATION

This fork is used just for connecting to the gripper and sending open and grab commands. nothing else.

## Usage: 
```bash
ros2 launch lfd_apples lfd_gripper.launch.py ssid:=alejos password:=harvesting
```

# LFD Apples - Gripper Control

## Things You'll Need

- **Robot:** Franka Research
- **Gripper:** micro-ROS enabled
- **ROS2**
- [Moveit2](https://moveit.ai/install-moveit2/source/)

---

## GRIPPER

### lfd_gripper.launch.py 
This launch file performs the following actions: 

1) Turns your laptop into a **Wi-Fi hotspot**.
2) Runs **microROS agent** to handle the communication with the ESP32 on the gripper's side.
3) Runs a ROS2 node to control the air valve and fingers manually.


```bash
ros2 launch lfd_apples lfd_gripper.launch.py ssid:=alejos password:=harvesting
```


Notes: 
* Replace `wlo1` in the launch file with your wireless interface name. You can check this with `nmcli device status` or `ip link`.
* Use the **ssid / password** that were previously uploaded to the ESP32 board. By default these values are `alejos` / `harvesting`.

