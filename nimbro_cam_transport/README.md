
nimbro_cam_transport
====================

This package contains nodes for encoding/decoding camera images to/from H.264.
At this stage, the package is still experimental and should be used with care.

Dependencies
------------

The `sender` node uses the `x264` library for encoding the video stream. In most
distributions, this library can be installed from the repositories
(Ubuntu: package libx264-dev).

The `receiver` node uses the `ffmpeg` library for decoding. Note that the
`avconv` framework is *not* supported. Also, most distribution packages are
outdated, so it is best to compile ffmpeg from source.

Here is a minimal ffmpeg `configure` line that enables just the modules we need
and installs `ffmpeg` into `/opt/ffmpeg`:

    ./configure --enable-shared --prefix=/opt/ffmpeg --disable-everything \
      --enable-decoder=h264

sender node
-----------

Subscriptions:
 - `~image` (sensor_msgs/Image): Image input

Publications:
 - `~encoded` (sensor_msgs/CompressedImage): H264 packet output

Parameters:
 - `rate` (float): Input image rate limit (default: 60.0)
 - `width` (int): Input images are scaled to this width before encoding
   (default: 640)

receiver node
-------------

Subscriptions:
 - `~encoded` (sensor_msgs/CompressedImage): H264 packet input

Publications:
 - `~image` (sensor_msgs/Image): Decoded images
