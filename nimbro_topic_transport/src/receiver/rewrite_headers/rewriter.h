// Rewrites all TF frame IDs
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef TT_REWRITE_HEADERS_REWRITER_H
#define TT_REWRITE_HEADERS_REWRITER_H

#include <boost/shared_array.hpp>

namespace nimbro_topic_transport
{

class Rewriter
{
public:
	explicit Rewriter(const std::string& prefix);
	~Rewriter();

	void prepare(const std::string& topicType, const std::string& md5);
	boost::shared_array<uint8_t> rewrite(const std::string& topicType, const std::string& md5, const boost::shared_array<uint8_t>& msg);
private:
	class Private;

	std::unique_ptr<Private> m_d;
};

}

#endif
