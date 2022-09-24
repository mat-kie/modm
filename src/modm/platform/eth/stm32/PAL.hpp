/**
 * @brief Phy abstraction layer (PAL).
 * The Basic Register Set for a PHY is defined in IEEE 802.3-2018 Section 22.2.4.
 * in Some applications, the phy might need to be emulated or is accessed by another bus
 * then MIIMI. To separate MAC, PHY and Management Bus functionality a phy abstraction layer
 * is introduced. The MAC layer code interacts with the PHY via the PAL. the PAL only assumes
 * the minimal required functionality of the PHY.
 * By doing it this way PHY emulation for direct-to-switch applications 
 * can be easily implemented without touching other code.
 * 
 * @author Mattis Kieffer
 * 
 */
#include <modm/architecture/interface/register.hpp>

namespace modm{
    struct PHYRegister{
// TODO: Standard Phy Register mappings as per 802.3-22.2.4
    };
    /**
     * @brief Interface for PHY abstraction layer (PAL).
     * Implementations shall inhert PAL<Derived>. static const function pointers
     * enforce existence of static methods needed by the MAC-Layer.
     * An Implementing class shall 
     * 
     * @tparam T : Implementing class of the interface.
     */
    template<class T>
    struct PAL
    {
        /**
         * @brief initialize PHY (reset phy and config its regs).
         * 
         * @return true if sucessfull.
         */
        static constexpr bool (*initialize)(void)= T::initialize;

        /**
         * @brief start the autonegociation process of the phy.
         * 
         * @return true if sucessfull
         */
        static constexpr bool (*startAutoNegociation)(void) = T::startAutoNegociation;

        /**
         * @brief read the Link bit from the status register.
         * 
         */
        static constexpr bool (*readLinkStatus)(void) = T::readLinkStatus;

        /**
         * @brief return the last linkStatus that was read from the status register.
         * 
         */
        static constexpr bool (*getLinkStatus)(void) = T::getLinkStatus;
    };

};
