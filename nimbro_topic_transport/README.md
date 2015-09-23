
nimbro_topic_transport
======================

This package includes nodes for transmitting ROS topic messages over a network
connection. For an overview over the available parameters, see
`doc/configuration.md`.

For a quick getting-started guide see `doc/getting_started.md`.

The remainder of this document introduces the features of the topic transport
and explains the choices you have in detail.

The fundamental choice you have is whether you want to use the TCP or the UDP
protocol.

UDP
---

The UDP protocol is the right choice for most ROS topics. It is especially
suited for lossy and/or delayed connections, where TCP has problems. Note
however, that with our UDP protocol there is no guarantee that a message
arrives.

Typical message types transmitted using the UDP protocol are camera images,
joystick commands, TF snapshots (see tf_throttle). In short, everything where
retransmitting missed packets does not make sense, because you have newer data
available already.

For example launch files, see the launch directory.

TCP
---

The TCP protocol is more useful for topics where you need a transmission
guarantee. For example, it makes sense to transmit 3D laser scans via this
method, because re-transmission of dropped packets is still faster than
waiting for the next scan.

For example launch files, see the launch directory.

Visualization & diagnosis
-------------------------

The nimbro_topic_transport package provides GUI plugins for the `rqt` GUI:

 - "nimbro_network/Topic Transport" shows a dot graph of all network connections
 - "nimbro_network/Topic Bandwidth" shows detailed bandwidth usage for a
   single connection.

Note that it may be necessary to transmit the topics `/network/sender_stats` and
`/network/receiver_stats` over the network to get the full amount of
information.

Relay mode
----------

The UDP transport supports a special "relay" mode that was used in the DARPA
Robotics Challenge. Basically, it saturates a given target bitrate, repeating
messages if necessary. In the DRC, the high bandwidth link was switched on for
very short times (1 second). By saturating the available bandwidth, we made
sure that we got the maximum amount of data across in that window.

Forward error correction (FEC)
------------------------------

The UDP transport can do forward error correction if [OpenFEC][1] is available.
A short guide on how to install and configure the system is available
in `doc/FEC.md`.

[1]: http://openfec.org/
