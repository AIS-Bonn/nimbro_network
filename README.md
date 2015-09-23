
nimbro_network - ROS transport for high-latency, low-quality networks
=====================================================================

`nimbro_network` is a set of [ROS][1] packages for transporting ROS topics
and services over network. It was developed for the
[DLR SpaceBotCup competition][2] and performed very well during the competition.
Our team was one of the few teams which did not have communication problems.

The SpaceBotCup network had a few special aspects which forced us to design
our own network solution, in particular a two-second delay in each direction.

The system was also used extensively in the [DARPA Robotics Challenge][3], in
which our team NimbRo Rescue achieved the fourth place.

[1]: http://www.ros.org
[2]: http://www.dlr.de/rd/desktopdefault.aspx/tabid-8101/13875_read-35268/
[3]: http://www.theroboticschallenge.org/

Why?
----

ROS has a network transparency layer. But it has issues, namely:

* For subscription, a lengthy TCP handshake is required, even if you want to
  use the UDP transport. If you lose the connection, you have to re-do the
  handshake, possibly taking a long time
* No compression
* ROS service calls need several handshakes for each call
* Messages are transmitted at the rate at which they are published

Our network stack offers the same functions as the ROS network transparency,
but addresses each of the above issues.

Alternatives
------------

`nimbro_network` offers robust transport of ROS topics and services over
unreliable networks. For high-level features like auto-discovery, job scheduling
etc. take a look at alternatives like [rocon][rocon] or
[multimaster_fkie][multimaster_fkie].

[rocon]: http://www.robotconcert.org
[multimaster_fkie]: https://fkie.github.io/multimaster_fkie/

Features
--------

* Topic transport:
    * TCP protocol for transmission guarantee
      (still with communication timeouts!)
    * UDP protocol for streaming data (data which has no meaning if it
      arrives late)
    * Optional transparent BZip2 compression using libbz2
    * Automatic topic discovery on the receiver side. The transmitter defines
      which topics get transferred
    * Optional rate-limiting for each topic
    * Experimental Forward Error Correction (FEC) for the UDP transport
* Service transport:
    * TCP protocol with minimal latency (support for TCP Fast-Open is included)
    * UDP protocol
* Additional nodes:
    * Special nimbro_log_transport node for transporting the ROS log over a
      lossy connection
    * Special tf_throttle node for creating & transferring TF snapshots at pre-
      defined intervals.
    * Special nimbro_cam_transport package for encoding/decoding camera images
      to/from H.264

Getting started
---------------

See nimbro_topic_transport/README.md and nimbro_service_transport/README.md
for documentation on the topic and service transport.

State
-----

`nimbro_network` is mature in the sense that it has been used extensively in
the preparation and during the competition of the SpaceBotCup and in the
DARPA Robotics Challenge.

In the DRC, the software was used for the high-bandwidth link. Communication
over the low-bandwidth link was handled by custom, highly specific code, which
is not released at this point.

License
-------

`nimbro_network` is licensed under the BSD 3-clause license.
This repository includes the [QCustomPlot][4] library, which is licensed under
the GPLv3 license.

[4]: http://www.qcustomplot.com

Authors & Contact
-----------------

```
Max Schwarz <max.schwarz@uni-bonn.de>
Institute of Computer Science VI
Rheinische Friedrich-Wilhelms-Universität Bonn
Friedrich Ebert-Allee 144
53113 Bonn
```

