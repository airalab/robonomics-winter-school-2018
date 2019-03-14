Package for AIRA to control turtlesim through Robonomics
========================================================

For the period of the contract conclusion, the creditor may order services of movement along the circle (clockwise or against) from turtilesim.

Service installation and launch
-------------------------------

The system consists of three parts: a DApp web-interface, AIRA with the `turtlesim_aira` package, and a robot, in our case, a [turtlesim](http://wiki.ros.org/turtlesim) simulation.

1. Download the image of the virtual machine with `turtlesim` using [this link](https://drive.google.com/file/d/1q-ANERcsjIOHZQSFJSOD6mbqSQmIMdT3), then we will call this machine `hive`.

User         | Password
-------------|---------
engi         | engi

> GNU / Linux users can use `turtlesim` on their local machine by installing `ROS` according to [the instruction](http://wiki.ros.org/melodic/Installation) and [the turtlesim package](http://wiki.ros.org/turtlesim). To install ROS on NixOS, use [the airapkgs channel](https://github.com/airalab/airapkgs).

2. To connect `hive` to AIRA, create a virtual network between machines in VirtualBox. To do this, in the VirtualBox Manager window: `File -> Host Network Manager`. In the window that appears, click `Create`. The created network will appear in the list, enable the `DHCP server` in it by checking the checkbox at the end of the line.

3. Import the `hive` image downloaded before.
4. Connect AIRA to the virtual adapter. To do this, go to the `Network` section in the machine settings and in the `Adapter 2` tab enable the network adapter with the `Virtual Host Adapter` connection type.
5. Launch both virtual machines.

<details>
  <summary>Does it brake?</summary>
  
  Enable video acceleration in the virtual machine settings.
  ![image](https://user-images.githubusercontent.com/833019/53819437-5e7c9780-3f83-11e9-8cd1-088dc8d93dea.png)
</details>

6. Now you need to tell the machines the names and IP-addresses of each other. To do this, in the consoles of each of the machines type:
```console
ip a | awk '{print $2}' | egrep -o '192.168.56.[0-9]{,}'
```
And write down the received IP-addresses.
7. Now in the AIRA configuration, add the `hive` address. Open the configuration file
```console
nano /etc/nixos/configuration.nix
```
Under the `imports = [ ... ];` string, from the new string add:
```console
networking.extraHosts = "<IP Адрес hive> hive";
networking.hostName = "aira";
networking.firewall.enable = false;
```
Close the text editor with saving the changes `Ctrl-x, y` and run the new configuration:
```console
nixos-rebuild switch --fast
```
8. Now add AIRA to `hive`. To do this, in the `hive` console:
```console
sudo nano /etc/hosts
```
In the file, find a string like `<IP address> aira` and replace the IP-address with the address of your `hive`.

9. Check for communication between the machines. To do this, in `hive` run:
```console
ping aira
```
If everything is done correctly, every second there will be a message with the packet transit time from `hive` to AIRA. The same from AIRA to `hive`:
```console
ping hive
```
In case of problems, contact the engineers in [the school chat](https://t.me/joinchat/GKXpc0v8rerK9h67TDW9kA).

10. AIRA must have `erc20` and `liability` system services running in order to connect your image with Robonomics. Run these services with the following command:
```console
systemctl restart liability erc20
```

11. Instal and launch `turtlesim_aira` in AIRA. To do this, in the AIRA console:
```console
cd robonomics-winter-school-2018
git pull
nix build -f ./release.nix turtlesim_aira
source ./result/setup.zsh
roslaunch turtlesim_aira trader.launch
```
And in the other window:
```console
source ./result/setup.zsh
roslaunch turtlesim_aira worker.launch
```

12. Connect `hive` to `rosmaster` in AIRA and run `turtlesim`. To do this, in the `hive` console:
```console
export ROS_MASTER_URI=http://aira:11311
rosrun turtlesim turtlesim_node
```

Ready! Now AIRA is connected to the robot simulation in `turtlesim`.
The interface for managing is available at the following link:
> https://airalab.github.io/robonomics-winter-school-2018/#/robot

Parameters
---------

IPFS hash                                      | File name
-----------------------------------------------|-------------------------------------------------------
QmPVr7k4N2jNiCYjbvQWPcmxzm5jwY3ZHEuJMgbQLmPKvY | turtlesim_aira_order_allow.model
QmPtwRTjPmvBweSmG4zVGtUc9KWxLsPp76xERvjUXFJWEz | turtlesim_aira_order_duration_1h.objective
QmbYXWWhNtnjhhBTvs2UfHiFLTUsXSZsoLoKSufiZHPxvR | turtlesim_aira_order_duration_24h.objective
QmYijVc27M27WyS1UiAB72GmeDBKVo2Nyvh1EYXUBZUNJb | turtlesim_aira_order_duration_60s.objective
QmNeMoBUiYjk4VzLtsBe9XAXfpyFawsUd9wEYTQy4tZpEj | turtlesim_aira_order_move.model
QmRmj9VnRBbgmQwZMVU3oCinaYG8oh1UAvQJbtPUmEWSq1 | turtlesim_aira_order_circle_clockwise.objective
Qmd1YREP5MMLzoxT2kmvEocPxFMGFiCrLK6zQRmp5ebBqU | turtlesim_aira_order_circle_counterclockwise.objective

Parameter     | Meaning
--------------|--------------------------------------------------------------------------------
lighthouse    | airalab.lighthouse.4.robonomics.sid, 0xE85764E29583224C1D063639d2AeeeD7c389DF4d
token         | xrt.4.robonomics.sid, 0x093ac06910f23570292fd5027a4fA558ed4Cd010

Report content
------------------

### Service contract

Field / Topic| Type          | Meaning
-------------|---------------|--------------------------------------
success	     | std_msgs/Bool | False - not executed, True - executed

### Order

Field / Topic| Type            | Meaning
-------------|-----------------|-------------------------------------
success	     | std_msgs/Bool   | False - not executed, True - executed
/id          | std_msgs/String | IPFS ID of the executor
/objective   | std_msgs/String | IPFS hash objective of the order
