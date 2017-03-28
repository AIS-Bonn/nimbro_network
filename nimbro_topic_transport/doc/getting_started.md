
Getting started
===============

We assume that we have two machines, machineA and machineB. Both will be running
roscores, and we will transport some topics from machineA to machineB.

On machineA:

```bash
roslaunch nimbro_topic_transport sender.launch target:=machineB
rostopic pub -r 1.0 /my_first_topic std_msgs/String "Hello World"
```

Instead of using the host name `machineB`, you can also use an IP address.

On machineB:

```bash
roslaunch nimbro_topic_transport udp_receiver.launch
rostopic echo /recv/my_first_topic
```

You should see the `Hello World` messages arriving. Note that the topics
are remapped to a /recv/ prefix, since the topics would conflict if you were
to run both sender and receiver on the same roscore. In an usual application,
this would not be necessary.

For customization, you should copy the launch files into your own package.
