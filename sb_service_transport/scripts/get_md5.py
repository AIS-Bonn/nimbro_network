#!/usr/bin/python

import roslib.message
import sys

sys.stdout.write(roslib.message.get_service_class(sys.argv[1])._md5sum)
