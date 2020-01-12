// Rewriting gadget (this is compiled for every encountered message type)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include MSG_INCLUDE

#include <topic_tools/shape_shifter.h>

#include <type_traits>

#include "interface.h"

template <typename>
constexpr std::false_type isCompositeH (long);

template <typename M>
constexpr auto isCompositeH (int)
   -> decltype( ros::serialization::Serializer<M>::allInOne(std::declval<ros::serialization::OStream&>(), std::declval<M>()), std::true_type{} );

template <typename T>
using isComposite = decltype( isCompositeH<T>(0) );


using FinalMessageType = MSG_PACKAGE::MSG_TYPE;

template<class T> void handle(T& instance, const std::string& prefix);

class Remapper
{
public:
	explicit Remapper(const std::string& prefix)
	 : m_prefix{prefix}
	{}

	// This method is called recursively for every field instance of the message.
	template<class T>
	void next(T& instance)
	{
		handle<T>(instance, m_prefix);
	}

private:
	const std::string& m_prefix;
};

template<class T> void handle(T& instance, const std::string& prefix)
{
	// Do we have a std_msgs::Header instance?
	if constexpr(std::is_same_v<std::decay_t<T>, std_msgs::Header>)
	{
		// Leave empty frame_ids untouched.
		if(!instance.frame_id.empty())
			instance.frame_id = prefix + instance.frame_id;
	}
	else if constexpr(isComposite<std::decay_t<T>>{})
	{
		// Recurse
		Remapper remapper(prefix);
		ros::serialization::Serializer<T>::template allInOne<Remapper, T&>(remapper, instance);
	}
}

template<typename MessageType>
std::vector<uint8_t> morph(const std::vector<uint8_t>& msg, const std::string& prefix)
{
	ros::serialization::IStream istream(const_cast<uint8_t*>(msg.data()), msg.size());

	MessageType instance;
	ros::serialization::deserialize(istream, instance);

	handle(instance, prefix);

	uint32_t serial_size = ros::serialization::serializationLength(instance);
	std::vector<uint8_t> ret(serial_size);
	ros::serialization::OStream stream(ret.data(), ret.size());
	ros::serialization::serialize(stream, instance);

	return ret;
}

template std::vector<uint8_t> morph<FinalMessageType>(const std::vector<uint8_t>&, const std::string& prefix);

extern "C"
{
	MorphPtr registerMorph()
	{
		return &morph<FinalMessageType>;
	}
}
