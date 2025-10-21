# X650 Setup


## Hardware and Connection




- **Holybro Pixhawk 6X** - Advanced flight controller
- **Jetson Orin NX 16GB** - Edge AI computing unit
- **Holybro Jetson Baseboard** - motherboard for Jetson
  - 1 T ssd
  - one wifi card
  - one switch between pixhawk and jetson
  - etc.
- **Holybro X650** - Carbon fiber quadcopter frame
- **Livox MID360** - 3D LiDAR sensor for SLAM or data collection
- **Holybro RTK F9P Bundle** - High-precision GPS with RTK capabilities (optional)
- **realsense D435** - Stereo camera for stereo vision or data collection


First, make sure you have read official documents carefully, especially this page:[Holybro Pixhawk Jetson Baseboard](https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard)

### Important Notes


#### 1. **Power Wiring** [in the official document](https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard#hardware-setup).

And we highly recommend to have a proper power adapter that is interchangable with the battery, so you can easily switch to the power adapter when you need to, and only need to plug the power adapter before the power module, just like the battery, so we can take our time to do the setup and debugging, no need to worry about the battery. It should like the image below, but you are using one power adapter instead of the battery before the power module.
![alt text](images/power1_one_battery.jpg)


#### 2. **Understand this diagarm**:

It shows the connection between the pixhawk, jetson, and the peripherals. Remember that this baseboard has a **switch** between pixhawk and jetson, and also one **UART** and **CAN** are connected to each other between pixhawk and jetson. In most cases, we just use the **Ethernet switch** or **UART** communication. The **UART** communication is connected to the pixhawk's **TELEM2** port and jetson's **UART1** port. 

![alt text](images/peripherals_block_diagram.png)



### Pitfalls

#### 1. **Flashing couldn't work**

The Jetson companion computer can be flashed from a development computer using the Nvidia SDK Manager when the board is in recovery mode. When flashing the Jetson Board, sometimes it just couldn't work. If you tried to debug, but couldn't find the issue, then most likely you need to try different computers and different cable to flash the Jetson Board.

#### 2. **Familiar with the connection**

Remember the internal Ethernet switch on the board is powered by the pixhawk power supply port, not the Jetson power supply port. So if your switch doesn't work, you need to check the pixhawk power supply port.



Also notice that the Jetson baseboard has a wireless network card, and my development computer (Ubuntu 20.04) is connected to the same Wi-Fi as the Jetson wifi card. My computer is then connected to the Jetson via **SSH** to operate the Jetson.
  
#### 3. **Configuring Ethernet for Same Subnet (Jetson & PX4)**

To ensure the Jetson and PX4 are able to communicate over Ethernet, both devices need to be on the same subnet. We configure the network using **Netplan**.

> **Note:**  
> The official documentation states:  
> *"PX4 has a default IP address of 10.41.10.2 (for PX4 v1.15 and earlier, 192.168.0.3)"*  
> However, this IP address may be different in practice. **Always manually verify the actual IP address of your PX4.**  
> In our setup, we use the `192.168.0.x` subnet:  
> - Jetson IP: `192.168.0.1`  
> - Pixhawk (PX4) IP: `192.168.0.3`


The following example, taken from the official documentation, may cause your Jetson’s Wi-Fi to become unavailable:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 10.41.10.1/24
      routes:
        - to: 0.0.0.0/0
          via: 10.41.10.254
      nameservers:
        addresses:
          - 10.41.10.254
```

Instead, we use a simpler configuration as shown below. **Remember to adapt the settings to your own network environment.** Make sure your Jetson and PX4 are on the same subnet and there are no conflicting IP addresses.

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.0.1/24
```

**Summary:**  
- Use a unique and consistent subnet for both Jetson and PX4, such as `192.168.0.x`
- Double-check actual device IPs before proceeding
- Avoid configurations that conflict with other network interfaces (e.g., Wi-Fi)



## MAVLink Setup


### Pitfalls



#### 1. **Patience During MAVSDK Installation**

- When installing `mavsdk` via pip, the process may take a significant amount of time.  
  This is normal—please be patient and allow the installation to complete.  
  ```bash
  pip3 install mavsdk
  ```

#### 2. **Choosing the Right Connection Type**

There are two main ways to connect your Jetson to the Pixhawk:

- **Ethernet (recommended):** More stable and faster.
- **UART (Serial):** Traditional connection via TELEM ports.

You must choose between Serial Connection or Ethernet Connection, and configure accordingly.

---

##### **A. Serial (UART) Connection**

- Ensure a MAVLink instance is running on the *TELEM2* port and that it is set to "onboard mode."  
  This is typically the default method for many setups.
- Device paths may vary. Double-check which `/dev/tty*` device your Jetson uses.

**Example (from documentation):**
```python
await drone.connect(system_address="serial:///dev/ttyTHS1:921600")
```
**However, on my hardware, the correct device is `/dev/ttyTHS0`:**
```python
await drone.connect(system_address="serial:///dev/ttyTHS0:921600")
```

---

##### **B. Ethernet Connection**

- Ensure a MAVLink instance is configured for the Ethernet port and set to "onboard mode."
- In many cases, you need to explicitly specify both IP address and port for a successful connection.

**Example (from documentation):** does not work in my case
```python
await drone.connect(system_address="udp://:14550")
```
**In practice, for active connection over Ethernet and explicitly specify the ip and port, we use:**
```python 
await drone.connect(system_address="udp://192.168.0.3:14540")
```

---

##### **Example of MAVLink Instances (Ethernet Use Case)**

In our setup, we choose the Ethernet connection, the MAVLink instances are assigned as follows:

| Instance | Port           | Function                                  |
|----------|----------------|-------------------------------------------|
| mav_0    | TELEM1         | Connected to telemetry radio              |
| mav_1    | TELEM3         | Connected to another telemetry radio      |
| mav_2    | Ethernet port  | Connected to onboard switch, so also jetson |

Make sure your MAVLink configuration matches your intended hardware connections and network environment.



## **MAVROS Setup**

After the MAVLink setup, we need to setup the MAVROS. MAVROS is just a ROS package that wraps the MAVLink protocol and provides a ROS interface to the Pixhawk.

Follow the instructions in the [official documentation](https://docs.px4.io/main/en/ros/mavros_installation) to setup the MAVROS.

### Pitfalls

#### 1. **Patience During Installation**

There is a high probability that the installation will fail, you need to be patient and ready to use Source Installation rather than binary installation.


#### 2. **Explicitly specify the ip and port**

just like the python example in **MAVLink Setup**, you need to explicitly specify the ip and port for the MAVROS.
```bash
roslaunch mavros px4.launch fcu_url:=udp://192.168.0.1:14540@192.168.0.3:14540
```

