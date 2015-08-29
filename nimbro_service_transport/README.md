
nimbro_service_transport
------------------------

`nimbro_service_transport` provides nodes which allow you to make service
calls over the network.

For an overview over the available parameters, see `doc/configuration.md`.

As with the topic transport, you have the choice of using TCP or UDP as the
underlying protocol.

TCP
---

The TCP protocol is very simple and straightforward. The nodes maintain a single
TCP connection to do all service calls. For a call, one chunk of data is
transmitted containing the request, and the other side answers with a chunk
containing the response.

Compare this to several rounds of handshaking that the native ROS protocol has
to do before it even gets around to transmitting the request payload...

If using TCP is not problematic for you, we recommend that you use the TCP
transport for service calls.

For historical reasons, the TCP nodes are named `service_client` and
`service_server`.

UDP
---

The UDP transport was developed for situations where TCP is not usable because
of the handshakes built into the protocol. It sends a single UDP packet for
the request, and receives a single UDP packet as response.

If the UDP client does not receive a response in a user-defined timeout, it
considers the service call as failed.

The UDP transport thus achieves the theoretical minimum in call latency if no
packets are lost.
