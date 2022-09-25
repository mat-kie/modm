/**
 * @file LAN8742a.hpp
 * @author Mattis Kieffer 
 * @brief Header file for LAN8742a Phy management.
 * @version 0.1
 * @date 2022-09-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef LAN8742A_HPP
#define LAN8742A_HPP
#include <modm/platform/eth/eth.hpp>

namespace modm
{
/**
 * @brief PHY Class for the LAN8742a phy found on the NUCLEO-F746ZG.
 * 
 * @tparam MIIMI static class containing the read and write functions for the SMI / MIIMI.
 */
template<class MIIMI>
class LAN8742a : platform::PHY
{
	/**
	 * @brief Autonegotiation advertisement register.
	 * 
	 */
	enum class ANA : uint16_t
	{
		SEL0 = Bit0,			/// Mode Selector Bit 0 (if set:= IEEE802.3)
		SEL1 = Bit1,			/// Mode Selector Bit 1 (reserved)
		SEL2 = Bit2,			/// Mode Selector Bit 2 (reserved)
		SEL3 = Bit3,			/// Mode Selector Bit 3 (reserved)
		S10BASE_T = Bit5,		/// Advertise 10BASE-T capability (Half Duplex)
		S10BASE_T_FD = Bit6,	/// Advertise 10BASE-T capability (Full Duplex)
		S100BASE_TX = Bit7,		/// Advertise 100BASE-T capability (Half Duplex)
		S100BASE_TX_FD = Bit8,	/// Advertise 100BASE-T capability (Full Duplex)
		// S100BASE_T4=Bit9,	/// 100BASE-T4 not supported by this chip
		PS1 = Bit10,			/// Advertise Symmetric pause capability
		PS2 = Bit11,			/// Advertise Asymmetric pause capability
		RF2 = Bit13,			/// Remote Fault detection
		ACK = Bit14,			/// Acknowlege
		NP = Bit15				/// Next page supported
	};
	MODM_FLAGS16(ANA)
private:
	static inline platform::eth::LinkStatus linkStatus;		/// the phys last linkstatus

public:

	/**
	 * @brief Initialize the PHY.
	 * 
	 * @return true if sucessful
	 * @return false if an error occurred
	 */
	static bool
	initialize()
	{ 
		/// Step 1: Reset PHY. Writing registers might not be allowed prior to a Reset.
		volatile uint32_t phy_register{0};
		(void)MIIMI::readPhyRegister(0, Register::CR, phy_register);
		CR_t phycr = static_cast<CR_t>(phy_register);
		phycr.set(CR::RESET);
		if (not MIIMI::writePhyRegister(0, Register::CR, phycr.value)) { return false; }

		// wait for reset done
		modm::delay_us(255);
		uint32_t timeout = 1'000;
		/// Step 2: Wait for the Reset bit to clear. indicating successful reset.
		do {
			phy_register = 0;
			if (!MIIMI::readPhyRegister(0, Register::CR, phy_register)) break;
			phycr = static_cast<CR_t>(phy_register);
			if (!(phycr & CR::RESET)) { break; }
			modm::delay_ms(1);
		} while (timeout-- > 0);
		if (timeout <= 0) { return false; }
		// Configure the Autonegotiation advertisement register.
		ANA_t phyana =
			ANA::SEL0 | ANA::S10BASE_T | ANA::S10BASE_T_FD | ANA::S100BASE_TX | ANA::S100BASE_TX_FD;
		if (!MIIMI::writePhyRegister(0, Register::ANA, phyana.value)){ return false; }
		// Step 3: configure the phy for 100M Full duplex for a start and enable Autonegotiation
		phycr = CR::SPD_SEL_LSB | CR::AN_EN | CR::DUP_MOD;
		return  MIIMI::writePhyRegister(0, Register::CR, phycr.value);

	};


	/**
	 * @brief Perform Autonegotiation
	 * 
	 * @return platform::ANResult The negotiated link speed and duplex mode.
	 */
	static platform::ANResult
	startAutoNegotiation()
	{
		uint32_t phy_register{0};
		// if Autonegotiation fails, assume 100M Full duplex
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