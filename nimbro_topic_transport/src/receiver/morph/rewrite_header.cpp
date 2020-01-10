
#ifndef MSG_PACKAGE
#define MSG_PACKAGE geometry_msgs
#define MSG_TYPE PointStamped
#define MSG_INCLUDE <geometry_msgs/PointStamped.h>
#endif

#include MSG_INCLUDE

#include <topic_tools/shape_shifter.h>

#include <type_traits>

template <typename>
constexpr std::false_type isCompositeH (long);

template <typename M>
constexpr auto isCompositeH (int)
   -> decltype( ros::serialization::Serializer<M>::allInOne(std::declval<ros::serialization::OStream&>(), std::declval<M>()), std::true_type{} );

template <typename T>
using isComposite = decltype( isCompositeH<T>(0) );


using ShapeShifterPtr = boost::shared_ptr<topic_tools::ShapeShifter>;

using FinalMessageType = MSG_PACKAGE::MSG_TYPE;



class Remapper
{
public:
	explicit Remapper(const std::string& prefix)
	 : m_prefix{prefix}
	{}

	template<class T>
	void next(T& instance)
	{
		if constexpr(std::is_same_v<std::decay_t<T>, std_msgs::Header>)
		{
			instance.frame_id = m_prefix + instance.frame_id;
		}
		else if constexpr(isComposite<std::decay_t<T>>{})
		{
			ros::serialization::Serializer<T>::allInOne(*this, instance);
		}
	}

private:
	std::string m_prefix;
};

template<typename MessageType>
ShapeShifterPtr morph(const ShapeShifterPtr& msg, const std::string& prefix)
{
	if constexpr(ros::message_traits::HasHeader<MessageType>::value)
	{
		auto instance = msg->instantiate<MessageType>();
		if(!instance)
			throw std::runtime_error(std::string{"Could not create instance"} + ros::message_traits::DataType<MessageType>::value());

		Remapper remapper(prefix);
		ros::serialization::Serializer<MessageType>::allInOne(remapper, *instance);

		uint32_t serial_size = ros::serialization::serializationLength(*instance);
		boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
		ros::serialization::OStream stream(buffer.get(), serial_size);
		ros::serialization::serialize(stream, *instance);

		auto out = boost::make_shared<topic_tools::ShapeShifter>();
		ros::serialization::IStream istream(buffer.get(), serial_size);
		out->read(istream);

		return out;
	}
	else
		return msg;
}

template ShapeShifterPtr morph<FinalMessageType>(const ShapeShifterPtr&, const std::string& prefix);

typedef ShapeShifterPtr (*MorphPtr)(const ShapeShifterPtr& msg, const std::string& prefix);

extern "C"
{
	MorphPtr registerMorph()
	{
		return &morph<FinalMessageType>;
	}
}
