/**
 * @file eth_impl.hpp
 * @author Mattis Kieffer
 * @brief Adapted from Mike Wolframs original implementation for FreeRTOS+TCP for modm
 * @version 0.1
 * @date 2022-09-27
 *
 * @copyright Copyright (c) 2022
 *
 */

// just for ide, will actually never be included due to include guard
#include "eth.hpp"
namespace modm::platform
{
	template <class T>
	bool
	Eth::writePhyRegister(uint32_t Address, PHYBase::Register reg, T value)
	{
		// get only CR bits from MACMIIAR
		uint32_t tmp = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
		tmp |= (Address << 11) & ETH_MACMIIAR_PA;
		tmp |= (uint32_t(reg) << 6) & ETH_MACMIIAR_MR;
		tmp |= ETH_MACMIIAR_MW;
		tmp |= ETH_MACMIIAR_MB;

		ETH->MACMIIDR = value.value;
		ETH->MACMIIAR = tmp;

		int timeout = 0xffff;
		while (timeout-- > 0)
		{
			if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0)
				return true;
		}

		return false;
	};

	template <class T>
	bool
	Eth::readPhyRegister(uint32_t Address, PHYBase::Register reg, T &value)
	{
		// get only CR bits from MACMIIAR
		uint32_t tmp = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
		tmp |= (Address << 11) & ETH_MACMIIAR_PA;
		tmp |= (uint32_t(reg) << 6) & ETH_MACMIIAR_MR;
		tmp &= ~ETH_MACMIIAR_MW;
		tmp |= ETH_MACMIIAR_MB;

		ETH->MACMIIAR = tmp;

		int timeout = 0xffff;
		while (timeout-- > 0)
		{
			if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0)
			{
				// busy flag cleared, read data
				value.value = ETH->MACMIIDR;
				return true;
			}
		}

		return false;
	};
};