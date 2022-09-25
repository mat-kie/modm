
/**
 * @brief implementation of the template specific methods.
 * 
 */
#ifndef LAN8742A_HPP
# error "do not include directly, include LAN8742a.hpp"
#endif
namespace modm
{
template<class MIIMI>
platform::eth::LinkStatus
LAN8742a<MIIMI>::readLinkStatus()
{
	uint32_t phy_register{0};
	MIIMI::readPhyRegister(0, PHY::Register::SR, phy_register);
	if (static_cast<SR_t>(phy_register) & PHY::SR::LINK_STATUS)
		linkStatus = platform::eth::LinkStatus::Up;
	else
		linkStatus = platform::eth::LinkStatus::Down;

	return linkStatus;
};

template<class MIIMI>
platform::eth::LinkStatus
LAN8742a<MIIMI>::getLinkStatus()
{
	return linkStatus;
};
}  // namespace modm
