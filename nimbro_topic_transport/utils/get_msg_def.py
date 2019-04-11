#!/usr/bin/env python

"""
Retrieves the message definition from the generated python files for a message
type

Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>
"""

import roslib.message

def message_definition_for_type(msg_type):
	cls = roslib.message.get_message_class(msg_type)
	return cls._full_text

def message_md5_for_type(msg_type):
	cls = roslib.message.get_message_class(msg_type)
	return cls._md5sum

if __name__ == "__main__":
	import sys

	if sys.argv[1] == 'def':
		sys.stdout.write(message_definition_for_type(sys.argv[2]))
	elif sys.argv[1] == 'md5':
		sys.stdout.write(message_md5_for_type(sys.argv[2]))
	else:
		sys.stderr.write('Usage: get_msg_def <def|md5> <type>\n')
		exit(1)
