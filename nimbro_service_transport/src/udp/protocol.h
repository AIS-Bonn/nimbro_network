// UDP protocol definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#include "../le_value.h"

namespace nimbro_service_transport
{

struct ServiceCallRequest
{
	LEValue<8> timestamp;
	uint8_t counter;

	LEValue<2> name_length;
	LEValue<4> request_length;
} __attribute__((packed));

struct ServiceCallResponse
{
	LEValue<8> timestamp;
	uint8_t counter;

	LEValue<4> response_length;
};

}

#endif
