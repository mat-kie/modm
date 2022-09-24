#ifndef LAN8742A_HPP
#define LAN8742A_HPP
#include <modm/platform/eth/eth.hpp>
#include <modm/platform/eth/PAL.hpp>

namespace modm
{
	template <class MIIMI>
	class LAN8742a : PAL<LAN8742a>, PHY
	{
	private:
		static platform::eth::LinkStatus linkStatus;

	public:
		static constexpr uint32_t Address = 0x00;
		static bool initialize()
		{
			// Initialize PHY
			/// Step 1.: reset phy. this is mandatory for config (see 802.3-22.2.4)
			uint32_t phy_register = uint32_t(PHY::CR::RESET);
			if (!MIIMI::writePhyRegister(0, PHY::Register::CR, phy_register))
			{
				return false;
			}
			// wait for reset done
			modm::delay_us(255);
			uint16_t timeout = 0'500;
			do
			{
				(void)MIIMI::readPhyRegister(0, PHY::Register::CR, phy_register);
				if ((phy_register & PHY::CR::RESET) == 0)
					break;
				modm::delay_ms(1);
			} while (timeout-- > 0);
			if (timeout <= 0)
			{
				return false;
			}

			// FIXME: Delegate these values to the PHY
			// configure auto negotiation
			phy_register = 0x100  // 100 FD
						   | 0x80 // 100
						   | 0x40 // 10 FD
						   | 0x20 // 10
						   | 0x01 // 802.3
				;
			(void)MIIMI::writePhyRegister(0, PHY::Register::ANA, phy_register);
		};

		static platform::ANResult startAutoNegotiation()
		{
			uint32_t phy_register{0};
			platform::ANResult cfg
			{
				.successful = false,
				.mode = platform::eth::DuplexMode::Full,
				.speed = platform::eth::Speed::Speed100M;
			}
			// enable auto-negotiation
			(void)MIIMI::readPhyRegister(0, PHY::Register::CR, phy_register);
			phy_register |= PHY::CR::RAN;
			if (!MIIMI::writePhyRegister(0, PHY::Register::CR, phy_register))
			{
				return cfg;
			}
			// wait for auto-negotiation complete (5s)
			uint16_t timeout = 5'000;
			do
			{
				(void)MIIMI::readPhyRegister(0, PHY::Register::SR, phy_register);
				if ((phy_register & PHY::SR::AN_COMP) == PHY::SR::AN_COMP)
				{
					break;
				}
				modm::delay_ms(1);
			} while (timeout-- > 0);
			if (timeout == 0)
			{
				return cfg;
			}
			// read auto-negotiation result
			if (!MIIMI::readPhyRegister(0, PHY::Register::SR, phy_register))
			{
				return cfg;
			}

			if ((phy_register & (PHY::SR::SPD100T2FD | PHY::SR::SPD100XFD | PHY::SR::SPD10FD)) == 0)
			{
				cfg.mode = platform::eth::DuplexMode::Half;
			}
			if ((phy_register & (PHY::SR::SPD100T4 | PHY::SR::SPD100T2FD | PHY::SR::SPD100T2HD | PHY::SR::SPD100T4 | PHY::SR::SPD100XFD | PHY::SR::SPD100XHD)) == 0)
			{
				cfg.speed = platform::eth::Speed::Speed10M;
			}
			return cfg;
		};

		static platform::eth::LinkStatus readLinkStatus()
		{
			uint32_t phy_register{0};
			MIIMI::readPhyRegister(0, PHY::Register::SR, phy_register);
			if ((phy_register & PHY::SR::LINK_STATUS) == PHY::SR::LINK_STATUS)
				linkStatus = platform::eth::LinkStatus::Up;
			else
				linkStatus = platform::eth::LinkStatus::Down;

			return linkStatus;
		};
		static platform::eth::LinkStatus getLinkStatus()
		{
			return linkStatus;
		};
	};
};
#endif