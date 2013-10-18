// Provides information about topic types
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPIC_INFO_H
#define TOPIC_INFO_H

#include <string>
#include "le_value.h"

namespace sb_topic_transport
{

namespace topic_info
{

std::string getMsgDef(const std::string& type);
std::string getMd5Sum(const std::string& type);

void packMD5(const std::string& str, LEValue<4>* dest);
void unpackMD5(const LEValue<4>* src, std::string* dest);

}

}

#endif
