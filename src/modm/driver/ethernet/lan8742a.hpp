#ifndef LAN8742A_DRIVER_HPP
#define LAN8742A_DRIVER_HPP

#include <modm/processing/timer.hpp>
#include <modm/platform.hpp>
#include <modm/platform/eth/phy_registers.hpp>
#include <modm/platform/eth/api_concepts.hpp>

namespace modm::driver
{

    class Lan8742a : public modm::platform::PHYBase
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

            StatusRegister_t statusRegister;
            ControlRegister_t controlRegister;

            // enable auto-negotiation
            Eth::readPhyRegister<Lan8742a>(Register::CR, controlRegister);
            controlRegister |= ControlRegister::AN_Restart;
            if (not Eth::writePhyRegister<Lan8742a>(Register::CR, controlRegister))
                return {};

            // wait for auto-negotiation complete (5s)
            modm::Timeout timeout(5s);
            while (not timeout.isExpired())
            {

                (void)Eth::readPhyRegister<Lan8742a>(Register::SR, statusRegister);
                if (statusRegister.any(StatusRegister::AN_Complete))
                {
                    return readLinkStatus();
                }
            }
            return {};
        }
        static modm::ethernet::api::LinkStatus readLinkStatus()
        {
            using namespace modm::platform;

            StatusRegister_t statusRegister;
            if (not Eth::readPhyRegister<Lan8742a>(Register::SR, statusRegister))
                return {};

            if (!statusRegister.any(StatusRegister::LinkStatus))
            {
                return {};
            }
            MyLinkStatus = modm::ethernet::api::LinkStatus{
                .mode = statusRegister.any(
                    StatusRegister::SPD100T2FD
                    | StatusRegister::SPD100XFD
                    | StatusRegister::SPD10FD
                ) ? modm::ethernet::api::LinkStatus::DuplexMode::Full : modm::ethernet::api::LinkStatus::DuplexMode::Half,
                .speed = statusRegister.any(
                    StatusRegister::SPD100T2FD
                    | StatusRegister::SPD100T2HD
                    | StatusRegister::SPD100T4
                    | StatusRegister::SPD100XFD
                    | StatusRegister::SPD100XHD
                ) ? modm::ethernet::api::LinkStatus::Speed::S100MBit : modm::ethernet::api::LinkStatus::Speed::S10MBit
            };
            return MyLinkStatus;
        }
        static modm::ethernet::api::LinkStatus getLinkStatus()
        {
            return MyLinkStatus;
        }
    };
};

#endif