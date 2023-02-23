#ifndef MODM_ETH_API_CONCEPTS_HPP
#define MODM_ETH_API_CONCEPTS_HPP
#include <concepts>
#include <chrono>
#include <stdint.h>

namespace modm::ethernet::api
{
    struct LinkStatus
    {
        enum class DuplexMode
        {
            None,
            Half,
            Full
        };
        enum class Speed
        {
            None,
            S10MBit,
            S100MBit
        };

        DuplexMode mode{DuplexMode::None};
        Speed speed{Speed::None};
    };

    template <class PHY>
    concept PhyAbstractionLayer = requires() {
        {PHY::Address} -> std::convertible_to<uint32_t>;
        {PHY::ReadTimeout} -> std::convertible_to<std::chrono::milliseconds>;
        {PHY::WriteTimeout} -> std::convertible_to<std::chrono::milliseconds>;

        ///@brief initialize PHY (reset phy and config its regs).
        ///@return true if sucessfull.
        {PHY::initialize()} -> std::same_as<bool>;

        /// @brief start the autonegociation process of the phy.
        ///@return true if sucessfull
        {PHY::startAutonegotiation()} -> std::same_as<LinkStatus>;

        ///@brief read the Link bit from the status register.
        {PHY::readLinkStatus()} -> std::same_as<LinkStatus>;

        /// @brief return the last linkStatus that was read from the status register.
        {PHY::getLinkStatus()} -> std::same_as<LinkStatus>;
    };

    template<class Implementation>
    concept BufferHandle = requires(Implementation obj){
            {obj.detach()}->std::same_as<typename Implementation::pointer_type>;
            {obj.data()}->std::same_as<typename Implementation::container_type>;
            {obj.size()}->std::same_as<size_t>;
            obj.reset();
            {obj.resize(size_t{})}->std::same_as<bool>;
            {obj.hasData()}->std::same_as<bool>;
            {Implementation::getPayloadAddress(typename Implementation::pointer_type{})}->std::same_as<uint8_t*>;
            {Implementation::getPayloadSizeBytes(typename Implementation::pointer_type{})}->std::same_as<size_t>;

    };

};
#endif