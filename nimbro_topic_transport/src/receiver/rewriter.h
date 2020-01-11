// Rewrites all TF frame IDs
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef TT_REWRITE_HEADERS_REWRITER_H
#define TT_REWRITE_HEADERS_REWRITER_H

#include <future>
#include <optional>
#include <vector>

namespace nimbro_topic_transport
{

class Rewriter
{
private:
	class Private;

public:
	class TopicRewriter
	{
	public:
		TopicRewriter(const TopicRewriter&) = delete;
		TopicRewriter(TopicRewriter&&) = default;
		~TopicRewriter();

		TopicRewriter& operator=(const TopicRewriter&) = delete;
		TopicRewriter& operator=(TopicRewriter&&) = default;

		/**
		 * Rewrite a message payload. May return an empty vector to indicate
		 * that the data should be used unchanged.
		 */
		std::vector<uint8_t> rewrite(const std::vector<uint8_t>& data) const;
	private:
		friend class Rewriter;
		friend class Rewriter::Private;

		TopicRewriter();
		explicit TopicRewriter(const std::string& library, const std::string& prefix);

		class Private;
		std::unique_ptr<Private> m_d;
	};

	using TopicRewriterFuture = std::shared_future<TopicRewriter>;

	explicit Rewriter(const std::string& prefix);
	~Rewriter();

	std::string rewriteTopicName(const std::string& name);

	bool active() const;

	TopicRewriterFuture open(const std::string& topicType, const std::string& md5);

private:
	std::unique_ptr<Private> m_d;
};

}

#endif
