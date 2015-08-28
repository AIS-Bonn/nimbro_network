// Provides information about topic types
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPIC_INFO_H
#define TOPIC_INFO_H

#include "le_value.h"

#include <string>

namespace nimbro_topic_transport
{

namespace topic_info
{

/**
 * Get the message definition string (from the .msg file)
 *
 * @return msg definition, or empty on failure (unknown type, etc)
 **/
std::string getMsgDef(const std::string& type);

/**
 * Get the message md5 string
 *
 * @return md5 sum (32 chars), or empty on failure (unknown type, etc)
 **/
std::string getMd5Sum(const std::string& type);

/**
 * md5 string -> LEValue<4>
 **/
void packMD5(const std::string& str, LEValue<4>* dest);

/**
 * LEValue<4> -> md5 string
 **/
void unpackMD5(const LEValue<4>* src, std::string* dest);

}

}

#endif
