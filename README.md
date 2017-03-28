
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

The built-in network transparency of ROS creates a strong dependency on
a good connection to the ROS master. Multi-master solutions strive to
solve this issue by synchronizing individual ROS masters on the networked
hosts.

Here is a high-level comparison of `nimbro_network` with the ROS network
transparency and the [multimaster_fkie][multimaster_fkie] stack:

| Feature                                 | ROS 1 | `multimaster_fkie` | `nimbro_network` |
| --------------------------------------- | ----- | ------------------ | ---------------- |
| Reliable topic/service transport        | ✔     | ✔                  | ✔                |
| Unreliable topic/service transport      | (✔)   | (✔)                | ✔                |
| Multi-master operation                  | ✘     | ✔                  | ✔                |
| Works without ROS master handshake      | ✘     | ✘                  | ✔                |
| Automatic remote host/topic discovery   | ✘     | ✔                  | ✘                |
| Transparent compression                 | ✘     | ✘                  | ✔                |
| Forward Error Correction (FEC)          | ✘     | ✘                  | ✔                |
| Built-in rate limiting                  | ✘     | ✘                  | ✔                |
| Rate-limiting for tf messages           | ✘     | ✘                  | ✔                |
| Low-latency H.264 video transport       | ✘     | ✘                  | ✔                |

`nimbro_network` especially targets robust operations with autonomous or
semi-autonomous robots. Everything is statically configured, there is no
auto-discovery which might fail at critical points during a mission.

`nimbro_network` excels in the presence of communication restrictions
(artificial or natural) like unidirectional communication, latency or
packet drops.

Forward error correction (FEC) is realized using the [OpenFEC][OpenFEC]
software framework. This is a powerful tool to increase the robustness
of the unreliable transport --- without using bidirectional communication.

[OpenFEC]: http://www.openfec.org/
[multimaster_fkie]: https://fkie.github.io/multimaster_fkie/

Features
--------

* Topic transport:
    * TCP protocol for transmission guarantee
      (still with communication timeouts!)
    * UDP protocol for streaming data (data which has no meaning if it
      arrives late)
    * Optional transparent [zstd][zstd] compression
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

[zstd]: https://github.com/facebook/zstd

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

