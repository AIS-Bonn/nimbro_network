// Interface to gadget
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef TT_GADGET_INTERFACE_H
#define TT_GADGET_INTERFACE_H

#include <vector>
#include <cstdint>
#include <string>

typedef std::vector<uint8_t> (*MorphPtr)(const std::vector<uint8_t>& msg, const std::string& prefix);

#endif
