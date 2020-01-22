// Rewriting gadget (this is compiled for every encountered message type)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include MSG_INCLUDE

#include <type_traits>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "interface.h"

namespace
{
	template<template <typename...> class Template, typename T>
	struct is_specialization_of : std::false_type {};

	template<template<typename...> class Template, typename... Args>
	struct is_specialization_of<Template, Template<Args...>> : std::true_type {};

	template<typename Msg>
	struct is_boost_array : std::false_type {};

	template<typename T, std::size_t N>
	struct is_boost_array<boost::array<T, N>> : std::true_type {};
}


using FinalMessageType = MSG_PACKAGE::MSG_TYPE;
using MessageMD5 = ros::message_traits::MD5Sum<FinalMessageType>;

static_assert(
	MessageMD5::static_value1 == MSG_MD5_HIGH &&
	MessageMD5::static_value2 == MSG_MD5_LOW,
	"C++ message MD5 does not match the requested MD5. This should not happen!"
);

namespace visitor
{
	template<class T, class M, class F>
	void visit(T& instance, F&& f);

	template<class M, class F>
	class Caller
	{
	public:
		explicit Caller(F& f)
		 : m_f{f}
		{}

		// This method is called recursively for every field instance of the message.
		template<class T>
		void next(T&& instance)
		{
			visit<T,M>(instance, m_f);
		}

	private:
		F& m_f;
	};

	template<class T, class M, class F>
	void visit(T& instance, F&& f)
	{
		using DecT = std::decay_t<T>;

		// Do we have an instance of our seached type M?
		if constexpr(std::is_same_v<DecT, M>)
		{
			f(const_cast<DecT&>(instance));
		}

		// Figure out how to recurse.

		// Skip primitive types, they are not interesting
		else if constexpr(std::is_arithmetic_v<DecT> || std::is_same_v<DecT, std::string>)
			;

		// Do we have a vector or array?
		else if constexpr(is_specialization_of<std::vector, DecT>() || is_boost_array<DecT>())
		{
			// Visit each item.
			for(auto& item : const_cast<DecT&>(instance))
				visit<std::decay_t<decltype(item)>, M, F>(item, f);
		}

		// For other cases (structs) use the ros serializer to recurse into
		// members.
		else if constexpr(!is_boost_array<DecT>())
		{
			// Recurse
			Caller<M, F> caller(f);
			ros::serialization::serialize(caller, instance);
		}
	}
}

namespace
{
	void rewriteFrameID(std::string& field, const std::string& prefix)
	{
		// Leave empty frame_ids untouched.
		if(!field.empty())
			field = prefix + field;
	}
}

template<typename MessageType>
std::vector<uint8_t> morph(const std::vector<uint8_t>& msg, const std::string& prefix)
{
	ros::serialization::IStream istream(const_cast<uint8_t*>(msg.data()), msg.size());

	MessageType instance;
	ros::serialization::deserialize(istream, instance);

	// Begin transformations

	// Rewrite std_msgs::Header
	visitor::visit<MessageType, std_msgs::Header>(instance, [&prefix](std_msgs::Header& header) {
		rewriteFrameID(header.frame_id, prefix);
	});

	// Rewrite geometry_msgs::TransformStamped
	visitor::visit<MessageType, geometry_msgs::TransformStamped>(instance, [&prefix](geometry_msgs::TransformStamped& msg) {
		rewriteFrameID(msg.child_frame_id, prefix);
	});

	// Rewrite nav_msgs::Odometry
	visitor::visit<MessageType, nav_msgs::Odometry>(instance, [&prefix](nav_msgs::Odometry& odom) {
		rewriteFrameID(odom.child_frame_id, prefix);
	});

	// End transformations

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
