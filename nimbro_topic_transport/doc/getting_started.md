
Getting started
===============

We assume that we have two machines, machineA and machineB. Both will be running
roscores, and we will transport some topics from machineA to machineB.

On machineA:

```bash
roslaunch nimbro_topic_transport udp_sender.launch target:=machineB
rostopic pub -r 1.0 /my_first_topic std_msgs/String "Hello World"
```

Instead of using the host name `machineB`, you can also use an IP address.

On machineB:

```bash
roslaunch nimbro_topic_transport udp_receiver.launch
rostopic echo /my_first_topic
```

You should see the `Hello World` messages arriving.

For customization, you should copy the launch files into your own package.
