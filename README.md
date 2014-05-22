
nimbro_network - ROS transport for high-latency, low-quality networks
=====================================================================

`nimbro_network` is a set of [ROS][1] packages for transporting ROS topics
and services over network. It was developed for the
[DLR SpaceBotCup competition][2] and performed very well during the competition.
Our team was one of the few teams which did not have communication problems.

The SpaceBotCup network had a few special aspects which forced us to design
our own network solution, in particular a two-second delay in each direction.

[1]: http://www.ros.org
[2]: http://www.dlr.de/rd/desktopdefault.aspx/tabid-8101/13875_read-35268/

Why?
----

ROS has a network transparency layer. But it has issues, namely:

* For subscription, a lengthy TCP handshake is required, even if you want to
  use the UDP transport. If you lose the connection, you have to re-do the
  handshake, possibly taking a long time
* No compression
* ROS service calls need several handshakes for each call
* Messages are transmitted at the rate at which they are published

Features
--------

* Topic transport:
    * TCP protocol for reliable connections (still with communication timeouts!)
    * UDP protocol for unreliable connections (data which has no meaning if it
      arrives late)
    * Optional transparent BZip2 compression using libbz2
    * Automatic topic discovery on the receiver side. The transmitter defines
      which topics get transferred
    * Optional rate-limiting for each topic
* Service transport:
    * TCP protocol with minimal latency (support for TCP Fast-Open is included)
* Additional nodes:
    * Special nimbro_log_transport node for transporting the ROS log over a
      lossy connection
    * Special tf_throttle node for creating & transferring TF snapshots at pre-
      defined intervals.

Getting started
---------------

See nimbro_topic_transport/README.md and nimbro_service_transport/README.md
for documentation on the topic and service transport.

State
-----

`nimbro_network` is mature in the sense that it has been used extensively in
the preparation and during the competition of the SpaceBotCup.

License
-------

`nimbro_network` is licensed under the GPLv2, available at
http://www.gnu.org/licenses/gpl-2.0.txt
