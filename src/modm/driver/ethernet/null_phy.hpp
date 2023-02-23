#include <modm/communication/ethernet/api_concepts.hpp>

namespace modm::driver{

    template<
    modm::ethernet::api::LinkStatus::DuplexMode Mode,
    modm::ethernet::api::LinkStatus::Speed Speed
    >
    class NullPHY{
        public:
        static const constexpr uint32_t Address{0};
        static const constexpr std::chrono::milliseconds ReadTimeout{1s};
        static const constexpr std::chrono::milliseconds WriteTimeout{1s};

        static constexpr bool initialize(){return true;}
        static constexpr modm::ethernet::api::LinkStatus startAutonegotiation()
        {
            return {Mode, Speed};
        }
        static constexpr modm::ethernet::api::LinkStatus readLinkStatus()
        {
            return {Mode, Speed};
        }
        static constexpr modm::ethernet::api::LinkStatus getLinkStatus()
        {
            return {Mode, Speed};
        }

    };
};