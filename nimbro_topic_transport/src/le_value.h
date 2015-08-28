// Little-Endian value
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LE_VALUE_H
#define LE_VALUE_H

#include <stdint.h>
#include <endian.h>

namespace nimbro_topic_transport
{

/**
 * The LEValue template provides automatic endianness correction. The bytes
 * in memory are always ordered little-endian.
 **/
template<int Width>
class LEValue
{
};

template<>
class LEValue<1>
{
public:
	typedef uint8_t Type;

	inline operator uint8_t() const
	{ return m_value; }

	inline uint8_t operator()() const
	{ return m_value; }

	inline uint8_t operator=(uint8_t value)
	{ return m_value = value; }
private:
	uint8_t m_value;
} __attribute__((packed));

template<>
class LEValue<2>
{
public:
	typedef uint16_t Type;

	inline operator uint16_t() const
	{ return le16toh(m_value); }

	inline uint16_t operator()() const
	{ return le16toh(m_value); }

	inline uint16_t operator=(uint16_t value)
	{
		m_value = htole16(value);
		return value;
	}
private:
	uint16_t m_value;
} __attribute__((packed));

template<>
class LEValue<4>
{
public:
	typedef uint32_t Type;

	inline operator uint32_t() const
	{ return le32toh(m_value); }

	inline uint32_t operator()() const
	{ return le32toh(m_value); }

	inline uint32_t operator=(uint32_t value)
	{
		m_value = htole32(value);
		return value;
	}
private:
	uint32_t m_value;
} __attribute__((packed));

template<>
class LEValue<8>
{
public:
	typedef uint64_t Type;

	inline operator uint64_t() const
	{ return le64toh(m_value); }

	inline uint64_t operator()() const
	{ return le64toh(m_value); }

	inline uint64_t operator=(uint64_t value)
	{
		m_value = htole64(value);
		return value;
	}
private:
	uint64_t m_value;
} __attribute__((packed));

}

#endif
