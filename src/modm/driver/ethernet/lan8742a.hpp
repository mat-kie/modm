/*
 * Copyright (c) 2020, Mike Wolfram, modfied by Mattis Kieffer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MODM_LAN8742A_HPP
#define MODM_LAN8742A_HPP
#include <modm/platform/eth/PHYBase.hpp>
#include <modm/driver/ethernet/PHYInterface.hpp>
namespace modm
{
	/**
	 * @brief PHY Class for the LAN8742a phy found on the NUCLEO-F746ZG.
	 *
	 * @tparam MIIMI static class containing the read and write functions for the SMI / MIIMI.
	 */
	template <class MIIMI>
	class LAN8742a : public platform::PHYBase, public PHYInterface
	{

	private:
		static inline PHYInterface::LinkStatus linkStatus; /// the phys last linkstatus
		static constexpr uint32_t phy_id{0};

	public:
		/**
		 * @brief Initialize the PHY.
		 *
		 * @return true if sucessful
		 * @return false if an error occurred
		 */
		bool
		initialize() override
		{
			/// Step 1: Reset PHY. Writing registers might not be allowed prior to a Reset.
			ControlRegister_t phy_cr;
			(void)MIIMI::readPhyRegister(phy_id, Register::CR, phy_cr);
			phy_cr.set(ControlRegister::Reset);
			if (not MIIMI::writePhyRegister(phy_id, Register::CR, phy_cr))
			{
				return false;
			}
			// wait for reset done
			modm::delay_us(255);
			uint32_t timeout = 1'000;
			/// Step 2: Wait for the Reset bit to clear. indicating successful reset.
			do
			{

				if (!MIIMI::readPhyRegister(phy_id, Register::CR, phy_cr))
					break;
				if (!(phy_cr & ControlRegister::Reset))
				{
					break;
				}
				modm::delay_ms(1);
			} while (timeout-- > 0);
			if (timeout <= 0)
			{
				return false;
			}
			// Configure the Autonegotiation advertisement register.
			ANAbilityMII_t phy_an_adv = Selector_t(Selector::IEEE802_3) | ANAbilityMII::TECH_10BASE_T | ANAbilityMII::TECH_10BASE_T_FD | ANAbilityMII::TECH_100BASE_TX | ANAbilityMII::TECH_100BASE_TX_FD;
			if (!MIIMI::writePhyRegister(phy_id, Register::AN_ADV, phy_an_adv))
			{
				return false;
			}
			// Step 3: configure the phy for 100M Full duplex for a start and enable Autonegotiation
			return true; // MIIMI::writePhyRegister(phy_id, Register::CR, ControlRegister_t(ControlRegister::AN_Enable | ControlRegister::DuplexMode | ControlRegister::SpeedSelection_LSB));
		};

		/**
		 * @brief Perform Autonegotiation
		 *
		 * @return platform::ANResult The negotiated link speed and duplex mode.
		 */
		ANResult
		startAutoNegotiation() override
		{
			using namespace modm::platform;
			ControlRegister_t phy_cr;
			// if Autonegotiation fails, assume 100M Full duplex
			ANResult cfg{.successful = false,
							  .mode = DuplexMode::Full,
							  .speed = Speed::Speed100M};
			// enable auto-negotiation

			(void)MIIMI::readPhyRegister(phy_id, Register::CR, phy_cr);

			phy_cr.set(ControlRegister::AN_Restart);
			if (!MIIMI::writePhyRegister(phy_id, Register::CR, phy_cr))
			{
				return cfg;
			}
			// wait for auto-negotiation complete (5s)
			uint16_t timeout = 5'000;
			StatusRegister_t phy_sr;
			do
			{

				(void)MIIMI::readPhyRegister(phy_id, Register::SR, phy_sr);

				if (phy_sr & StatusRegister::AN_Complete)
				{
					break;
				}
				modm::delay_ms(1);
			} while (timeout-- > 0);
			if (timeout == 0)
			{
				return cfg;
			}
			// convert to ANResult
			if (!(phy_sr & DuplexStatus_t(DuplexStatus::FullDuplex)))
			{
				cfg.mode = DuplexMode::Half;
			}
			if (!(phy_sr & SpeedStatus_t(SpeedStatus::Speed100)))
			{
				cfg.speed = Speed::Speed10M;
			}
			return cfg;
		};

		LinkStatus
		readLinkStatus() override
		{
			StatusRegister_t phy_sr;

			MIIMI::readPhyRegister(phy_id, Register::SR, phy_sr);

			if (phy_sr & StatusRegister::LinkStatus)
				linkStatus = LinkStatus::Up;
			else
				linkStatus = LinkStatus::Down;

			return linkStatus;
		};

		LinkStatus
		getLinkStatus() override
		{
			return linkStatus;
		};
	};
}; // namespace modm

#endif