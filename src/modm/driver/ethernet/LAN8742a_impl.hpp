
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
	StatusRegister_t phy_sr;
	MIIMI::readPhyRegister(0, Register::SR, phy_sr);
	if (phy_sr & StatusRegister::LinkStatus)
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
