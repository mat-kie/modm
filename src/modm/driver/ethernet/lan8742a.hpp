#ifndef LAN8742A_DRIVER_HPP
#define LAN8742A_DRIVER_HPP

#include <modm/platform/eth/phy_registers.hpp>
#include <modm/processing/timer.hpp>
#include <modm/platform.hpp>

namespace modm::driver
{

    class Lan8742a
    {
        static inline modm::ethernet::api::LinkStatus MyLinkStatus{};

    public:
        static const constexpr uint32_t Address{0};
        static const constexpr std::chrono::milliseconds ReadTimeout{1s};
        static const constexpr std::chrono::milliseconds WriteTimeout{1s};

        static bool initialize() { return true; }
        static modm::ethernet::api::LinkStatus startAutonegotiation()
        {
            using namespace modm::platform;

            PHY::SR_t statusRegister;
            PHY::CR_t controlRegister;

            // enable auto-negotiation
            Eth::readPhyRegister<Lan8742a>(controlRegister);
            controlRegister |= PHY::CR::RAN;
            if (not Eth::writePhyRegister<Lan8742a>(controlRegister))
                return {};

            // wait for auto-negotiation complete (5s)
            modm::Timeout timeout(5s);
            while (not timeout.isExpired())
            {

                (void)Eth::readPhyRegister<Lan8742a>(statusRegister);
                if (statusRegister.any(PHY::SR::AN_COMP))
                {
                    return readLinkStatus();
                }
            }
            return {};
        }
        static modm::ethernet::api::LinkStatus readLinkStatus()
        {
            using namespace modm::platform;

            PHY::SR_t statusRegister;
            if (not Eth::readPhyRegister<Lan8742a>(statusRegister))
                return {};

            if (!statusRegister.any(PHY::SR::LINK_STATUS))
            {
                return {};
            }
            MyLinkStatus = modm::ethernet::api::LinkStatus{
                .mode = statusRegister.any(PHY::SR::SPD100T2FD | PHY::SR::SPD100XFD | PHY::SR::SPD10FD) ? modm::ethernet::api::LinkStatus::DuplexMode::Full : modm::ethernet::api::LinkStatus::DuplexMode::Half,
                .speed = statusRegister.any(PHY::SR::SPD100T2FD | PHY::SR::SPD100T2HD | PHY::SR::SPD100T4 | PHY::SR::SPD100XFD | PHY::SR::SPD100XHD) ? modm::ethernet::api::LinkStatus::Speed::S100MBit : modm::ethernet::api::LinkStatus::Speed::S10MBit};
            return MyLinkStatus;
        }
        static modm::ethernet::api::LinkStatus getLinkStatus()
        {
            return MyLinkStatus;
        }
    };
};

#endif