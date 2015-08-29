
nimbro_log_transport
--------------------

The `rosconsole` logging system (the ROS_INFO/WARN/ERROR/... macros) is very
useful for aggregating all logging output of ROS nodes in a single place
(the `/rosout` topic).

However, the topic usually has high bandwidth consisting of very small messages,
which makes it hard to transmit over the network.

`nimbro_log_transport` contains a sender node, which sends aggregates log
messages at fixed intervals, and a receiver node, which publishes the
information on `/rosout` again.

Note that the sender node uses a circular buffer for aggregating messages, and
thus will drop messages if the bandwidth is too high.

Sender
------

Publications:
 - `/rosout_transport`: Aggregated log messages ready for transport over the
   network (see nimbro_topic_transport)

Parameters:
 - `min_level` (int): Minimum log level (default: rosgraph_msgs::Log::INFO)
 - `buffer_size` (int): Size of the circular buffer (default: 10)
 - `rate` (float): Rate at which the circular buffer is emptied and published
   (default: 2.0)

Receiver
--------

Subscriptions:
 - `/rosout_transport`: Output from `log_sender`

Publications:
 - `/rosout`: Unpacked log messages
