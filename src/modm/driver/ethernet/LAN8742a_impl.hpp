/*
 * Copyright (c) 2020, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

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
