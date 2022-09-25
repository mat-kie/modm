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
#include <modm/platform/eth/PHYBase.hpp>

namespace modm
{
/**
 * @brief PHY Class for the LAN8742a phy found on the NUCLEO-F746ZG.
 * 
 * @tparam MIIMI static class containing the read and write functions for the SMI / MIIMI.
 */
template<class MIIMI>
class LAN8742a : platform::PHYBase
{

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
		ControlRegister_t phy_cr;
		(void)MIIMI::readPhyRegister(0, Register::CR, phy_cr);
		phy_cr.set(ControlRegister::Reset);
		if (not MIIMI::writePhyRegister(0, Register::CR, phy_cr)) { return false; }
		// wait for reset done
		modm::delay_us(255);
		uint32_t timeout = 1'000;
		/// Step 2: Wait for the Reset bit to clear. indicating successful reset.
		do {
			if (!MIIMI::readPhyRegister(0, Register::CR, phy_cr)) break;
			if (!(phy_cr & ControlRegister::Reset)) { break; }
			modm::delay_ms(1);
		} while (timeout-- > 0);
		if (timeout <= 0) { return false; }
		// Configure the Autonegotiation advertisement register.
		ANAbilityMII_t phy_an_adv = Selector_t(Selector::IEEE802_3)
									| ANAbilityMII::TECH_10BASE_T
									| ANAbilityMII::TECH_10BASE_T_FD
									| ANAbilityMII::TECH_100BASE_TX
									| ANAbilityMII::TECH_100BASE_TX_FD;
		if (!MIIMI::writePhyRegister(0, Register::AN_ADV, phy_an_adv)){ return false; }
		// Step 3: configure the phy for 100M Full duplex for a start and enable Autonegotiation
		return  true;//MIIMI::writePhyRegister(0, Register::CR, ControlRegister_t(ControlRegister::AN_Enable | ControlRegister::DuplexMode | ControlRegister::SpeedSelection_LSB));

	};


	/**
	 * @brief Perform Autonegotiation
	 * 
	 * @return platform::ANResult The negotiated link speed and duplex mode.
	 */
	static platform::ANResult
	startAutoNegotiation()
	{
		ControlRegister_t phy_cr;
		// if Autonegotiation fails, assume 100M Full duplex
		platform::ANResult cfg{.successful = false,
							   .mode = platform::eth::DuplexMode::Full,
							   .speed = platform::eth::Speed::Speed100M};
		// enable auto-negotiation
		(void)MIIMI::readPhyRegister(0, Register::CR, phy_cr);
		phy_cr.set(ControlRegister::AN_Restart);
		if (!MIIMI::writePhyRegister(0, Register::CR, phy_cr)) { return cfg; }
		// wait for auto-negotiation complete (5s)
		uint16_t timeout = 5'000;
		StatusRegister_t phy_sr;
		do {
			
			(void)MIIMI::readPhyRegister(0, Register::SR, phy_sr);
			if (phy_sr & StatusRegister::AN_Complete) { break; }
			modm::delay_ms(1);
		} while (timeout-- > 0);
		if (timeout == 0) { return cfg; }
		// convert to ANResult
		if (!(phy_sr & DuplexStatus_t(DuplexStatus::FullDuplex)))
		{
			cfg.mode = platform::eth::DuplexMode::Half;
		}
		if (!(phy_sr & SpeedStatus_t(SpeedStatus::Speed100)))
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