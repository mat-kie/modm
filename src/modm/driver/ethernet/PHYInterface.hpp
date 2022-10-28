#ifndef MODM_PHY_INTERFACE_HPP
#define MODM_PHY_INTERFACE_HPP

namespace modm
{
	struct PHYInterface
	{
		enum class LinkStatus
		{
			Down = 0,
			Up,
		};
		enum class DuplexMode
		{
			Half = 0,
			Full = 1,
		};

		enum class
			Speed
		{
			Speed10M = 0,
			Speed100M = 1,
		};
		struct ANResult
		{
			bool successful = false;
			DuplexMode mode = DuplexMode::Full;
			Speed speed = Speed::Speed100M;
		};
		virtual LinkStatus readLinkStatus() = 0;
		virtual LinkStatus getLinkStatus() = 0;
		virtual bool initialize() = 0;
		virtual ANResult startAutoNegotiation() = 0;
	};
};
#endif