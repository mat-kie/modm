#ifndef LAN8742A_HPP
#define LAN8742A_HPP
#include <modm/platform/eth/eth.hpp>

namespace modm
{
template<class MIIMI>
class LAN8742a : platform::PHY
{

	enum class ANA : uint16_t
	{
		SEL0 = Bit0,
		SEL1 = Bit1,
		SEL2 = Bit2,
		SEL3 = Bit3,
		S10BASE_T = Bit5,
		S10BASE_T_FD = Bit6,
		S100BASE_TX = Bit7,
		S100BASE_TX_FD = Bit8,
		// S100BASE_T4=Bit9,
		PS1 = Bit10,
		PS2 = Bit11,
		RF2 = Bit13,
		ACK = Bit14,
		NP = Bit15
	};
	MODM_FLAGS16(ANA)
private:
	static inline platform::eth::LinkStatus linkStatus;

public:
	static bool
	initialize()
	{
		// Initialize PHY
		volatile uint32_t phy_register{0};
		/* Reset */
		(void)MIIMI::readPhyRegister(0, Register::CR, phy_register);
		CR_t phycr = static_cast<CR_t>(phy_register);
		phycr.set(CR::RESET);
		if (not MIIMI::writePhyRegister(0, Register::CR, phycr.value)) { return false; }

		// wait for reset done
		modm::delay_us(255);
		uint32_t timeout = 1'000;

		do {
			phy_register = 0;
			if (!MIIMI::readPhyRegister(0, Register::CR, phy_register)) break;
			phycr = static_cast<CR_t>(phy_register);
			if (!(phycr & CR::RESET)) { break; }
			modm::delay_ms(1);
		} while (timeout-- > 0);
		if (timeout <= 0) { return false; }

		phycr = CR::SPD_SEL_LSB | CR::AN_EN | CR::DUP_MOD;
		MIIMI::writePhyRegister(0, Register::CR, phycr.value);

		// configure auto negotiation
		ANA_t phyana =
			ANA::SEL0 | ANA::S10BASE_T | ANA::S10BASE_T_FD | ANA::S100BASE_TX | ANA::S100BASE_TX_FD;
		return MIIMI::writePhyRegister(0, Register::ANA, phyana.value);
	};

	static platform::ANResult
	startAutoNegotiation()
	{
		uint32_t phy_register{0};
		platform::ANResult cfg{.successful = false,
							   .mode = platform::eth::DuplexMode::Full,
							   .speed = platform::eth::Speed::Speed100M};
		// enable auto-negotiation
		(void)MIIMI::readPhyRegister(0, Register::CR, phy_register);
		CR_t phycr = static_cast<CR_t>(phy_register);
		phycr.set(CR::RAN);
		if (!MIIMI::writePhyRegister(0, Register::CR, phycr.value)) { return cfg; }
		// wait for auto-negotiation complete (5s)
		uint16_t timeout = 5'000;
		do {
			(void)MIIMI::readPhyRegister(0, Register::SR, phy_register);
			if (static_cast<SR_t>(phy_register) & SR::AN_COMP) { break; }
			modm::delay_ms(1);
		} while (timeout-- > 0);
		if (timeout == 0) { return cfg; }
		// read auto-negotiation result
		if (!MIIMI::readPhyRegister(0, PHY::Register::SR, phy_register)) { return cfg; }

		if (!(static_cast<SR_t>(phy_register) & (SR::SPD100T2FD | SR::SPD100XFD | SR::SPD10FD)))
		{
			cfg.mode = platform::eth::DuplexMode::Half;
		}
		if (!(static_cast<SR_t>(phy_register) &
			  (PHY::SR::SPD100T4 | PHY::SR::SPD100T2FD | PHY::SR::SPD100T2HD | PHY::SR::SPD100T4 |
			   PHY::SR::SPD100XFD | PHY::SR::SPD100XHD)))
		{
			cfg.speed = platform::eth::Speed::Speed10M;
		}
		return cfg;
	};

	static platform::eth::LinkStatus
	readLinkStatus();
	static platform::eth::LinkStatus
	getLinkStatus();
};
};  // namespace modm
#include "LAN8742a_impl.hpp"
#endif