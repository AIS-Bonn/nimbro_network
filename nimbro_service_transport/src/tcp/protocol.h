// Protocol definition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "../le_value.h"

namespace nimbro_service_transport
{

namespace protocol
{

#define NIMBRO_SERVICE_TRANSPORT_FASTOPEN 0

struct ServiceDescription
{
	LEValue<4> md5[4];
	LEValue<2> name_length;
	LEValue<2> type_length;
};

struct ServiceCallRequest
{
	LEValue<2> name_length;
	LEValue<4> request_length;
};

struct ServiceCallResponse
{
	LEValue<4> response_length;
};

}

}

#endif
