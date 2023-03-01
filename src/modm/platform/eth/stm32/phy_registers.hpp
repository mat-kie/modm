/**
 * @file base_PHY.hpp
 * @author Mattis Kieffer
 * @brief Basic Phy MII Management interface Register mapping definition (IEEE802.3 22.2.4).
 * @version 0.1
 * @date 2022-09-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef BASE_PHY_HPP
#define BASE_PHY_HPP

#include <modm/architecture/interface.hpp>
namespace modm::platform
{
struct PHYBase
{
	static constexpr
	uint32_t resetTimeout{500}; ///< maximum reset time in miliseconds (IEEE802.3-22.2.4.1.1)

	enum class Register : uint8_t
	{
		CR = 0,                 ///< Control Register
		SR = 1,                 ///< Status Register
		PHYID1 = 2,             ///< PHi ID high register
		PHYID2 = 3,             ///< PHY ID low register
		AN_ADV = 4,       ///< Auto-Negotiation Advertisement
		AN_LP_BASE = 5,   ///< Auto-Negotiation Link Partner Base Page Ability
		AN_EXP = 6,       ///< Auto-Negotiation Expansion
		AN_NP_TX = 7,     ///< Auto-Negotiation Next Page Transmit
		AN_LP_RX_NP = 8,  ///< Auto-Negotiation Link Partner Received Next Page
		MASTER_SLAVE_CR = 9,    ///< MASTER-SLAVE Control Register
		MASTER_SLAVE_SR = 10,   ///< MASTER-SLAVE Status Register
		PSE_CR = 11,            ///< PSE Control register
		PSE_SR = 12,            ///< PSE Status register
		MMDA_CR = 13,           ///< MMD Access Control Register
		MMDA_ADR = 14,          ///< MMD Access Address Data Register
		ESR = 15                ///< Extended Status
	};

	enum class ControlRegister : uint16_t
	{
		UnidirectionalEnable = Bit5,        ///< Enable tx from MII regardless of link status.
		SpeedSelection_MSB = Bit6,          ///< Speed selection lsb @see SpeedSelection_t
		CollisionTest = Bit7,               ///< Enable COL signal test
		DuplexMode = Bit8,                  ///< Enable full duplex mode
		AN_Restart = Bit9,                  ///< Restart autonegotiation process
		Isolate = Bit10,                    ///< Isolate PHY from MII
		PowerDown = Bit11,                  ///< Power down PHY
		AN_Enable = Bit12,                  ///< Enable autonegotiation
		SpeedSelection_LSB = Bit13,         ///< Speed selection MSB @see SpeedSelection_t
		Loopback = Bit14,                   ///< Enable loopback mode
		Reset = Bit15                       ///< PHY reset
	};
	MODM_FLAGS16(ControlRegister);

	enum class SpeedSelection: uint16_t{
		Speed10M = 0,
		Speed100M = Bit13,
		Speed1000M = Bit6
	};
	typedef Configuration<ControlRegister_t, SpeedSelection, (Bit6|Bit13)> SpeedSelection_t;


	enum class StatusRegister : uint16_t
	{
		ExtendedCapability = Bit0,      ///< Extended Capability flag
		JabberDetect = Bit1,            /// Jabber detection flag
		LinkStatus = Bit2,              /// link status flag
		AN_Ability = Bit3,              /// Auto-negotiation ability flag
		RemoteFault = Bit4,             /// remote fault flag
		AN_Complete = Bit5,             /// auto-negotiation clomplete flag
		MF_PreambleSuppression = Bit6,  /// management frame preamble supression flag
		UnidirectionalAbility = Bit7,   /// unidirectional ability flag
		ExtendedStatus = Bit8,          /// Extended status information in Register 15
		SPD100T2HD = Bit9,              /// PHY able to perform half duplex 100BASE-T2
		SPD100T2FD = Bit10,             /// PHY able to perform full duplex 100BASE-T2
		SPD10HD = Bit11,                /// PHY able to operate at 10 Mb/s in half duplex mode
		SPD10FD = Bit12,                /// PHY able to operate at 10 Mb/s in full duplex mode
		SPD100XHD = Bit13,              /// PHY able to perform half duplex 100BASE-X
		SPD100XFD = Bit14,              /// PHY able to perform full duplex 100BASE-X
		SPD100T4 = Bit15                /// 100BASE-T4 capable
	};
	MODM_FLAGS16(StatusRegister);

	enum class SpeedStatus : uint16_t
	{
		Speed10 = Bit11 | Bit12,
		Speed100 = Bit9 | Bit10 | Bit13 | Bit14 | Bit15
	};
	typedef Configuration<StatusRegister_t, SpeedStatus, (Bit9|Bit10|Bit11|Bit12|Bit13|Bit14|Bit15)> SpeedStatus_t;

	enum class DuplexStatus : uint16_t
	{
		FullDuplex = Bit12 | Bit10 | Bit14,
		HalfDuplex = Bit9 | Bit11 | Bit13 | Bit15
	};
	typedef Configuration<StatusRegister_t, DuplexStatus, (Bit9|Bit10|Bit11|Bit12|Bit13|Bit14|Bit15)> DuplexStatus_t;
    /**
     * @brief Autonegotiation ability flag definition used by AN_ADV and AN_LP_BASE in 10/100MBit.
     *
     */
    enum class ANAbilityMII : uint16_t{
        S0 = Bit0,                  ///< Selector field S0
        S1 = Bit1,                  ///< Selector field S1
        S2 = Bit2,                  ///< Selector field S2
        S3 = Bit3,                  ///< Selector field S3 (reserved)
        S4 = Bit4,                  ///< Selector field S4 (reserved)
        TECH_10BASE_T = Bit5,       ///< Technology ability field A0: 10BASE-T
        TECH_10BASE_T_FD = Bit6,    ///< Technology ability field A1: 10BASE-T full duplex
        TECH_100BASE_TX = Bit7,     ///< Technology ability field A2: 100BASE-TX
        TECH_100BASE_TX_FD = Bit8,  ///< Technology ability field A3: 100BASE-TX full duplex
        TECH_100BASE_T4 = Bit9,     ///< Technology ability field A4: 100BASE-T4
        SYM_PAUSE = Bit10,          ///< Technology ability field A5: symmetric pause on fd links
        ASYM_PAUSE = Bit11,         ///< Technology ability field A6: asymmetric pause on fd links
        XNP = Bit12,                ///< Extended Next Page ability
        RF = Bit13,                 ///< Remote fault
        Ack= Bit14,                 ///< Acknowlege
        NP = Bit15                  ///< Next page

    };
    MODM_FLAGS16(ANAbilityMII);

    enum class Selector: uint16_t{
        IEEE802_3 = Bit0,
        IEEE802_9a = Bit1,
        IEEE802_9v = Bit0 | Bit1,
        IEEE1394 = Bit2,
        INCITS = Bit0 | Bit2
    };
    typedef Configuration<ANAbilityMII_t, Selector, (Bit0|Bit1|Bit2|Bit3)> Selector_t;


    // TODO: Implement Registers 6 to 15 flag definitions
};
}  // namespace modm

#endif