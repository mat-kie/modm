#ifndef MODM_PHY_REGISTERS_HPP
#define MODM_PHY_REGISTERS_HPP
#include <modm/architecture/interface/register.hpp>

namespace modm::platform
{
/**
 * @brief Standard PHY Management register description.
 * This adheres to IEEE802.3 22.2.4.
 *
 */
struct PHY
{
	enum class Register : uint8_t
	{
		CR = 0,        /// Control Register
		SR = 1,        /// Status Register
		PHYID1 = 2,    /// PHi ID high register
		PHYID2 = 3,    /// PHY ID low register
		ANA = 4,       /// Auto-Negotiation Advertisement
		ANLPBPA = 5,   /// Auto-Negotiation Link Partner Base Page Ability
		ANE = 6,       /// Auto-Negotiation Expansion
		ANNPT = 7,     /// Auto-Negotiation Next Page Transmit
		ANLPRNP = 8,   /// Auto-Negotiation Link Partner Received Next Page
		MSCR = 9,      /// MASTER-SLAVE Control Register
		MSSR = 10,     /// MASTER-SLAVE Status Register
		PSECR = 11,    /// PSE Control register
		PSESR = 12,    /// PSE Status register
		MMDACR = 13,   /// MMD Access Control Register
		MMDAADR = 14,  /// MMD Access Address Data Register
		ESR = 15       /// Extended Status
	};

	enum class CR : uint16_t
	{
		UNID = Bit5,
		SPD_SEL_MSB = Bit6,
		COL_TEST = Bit7,
		DUP_MOD = Bit8,
		RAN = Bit9,
		ISOL = Bit10,
		PWR_DN = Bit11,
		AN_EN = Bit12,
		SPD_SEL_LSB = Bit13,
		LOOPBACK = Bit14,
		RESET = Bit15
	};
	MODM_FLAGS16(CR);

	enum class SR : uint16_t
	{
		EXT_CAP = Bit0,      /// Extended Capability flag
		JAB_DET = Bit1,      /// Jabber detection flag
		LINK_STATUS = Bit2,  /// link status flag
		AN_AB = Bit3,        /// Auto-negotiation ability flag
		REM_FAULT = Bit4,    /// remote fault flag
		AN_COMP = Bit5,      /// auto-negotiation clomplete flag
		MF_PS = Bit6,        /// management frame preamble supression flag
		UNI_AB = Bit7,       /// unidirectional ability flag
		EXT_STAT = Bit8,     /// Extended status information in Register 15
		SPD100T2HD = Bit9,   /// PHY able to perform half duplex 100BASE-T2
		SPD100T2FD = Bit10,  /// PHY able to perform full duplex 100BASE-T2
		SPD10HD = Bit11,     /// PHY able to operate at 10 Mb/s in half duplex mode
		SPD10FD = Bit12,     /// PHY able to operate at 10 Mb/s in full duplex mode
		SPD100XHD = Bit13,   /// PHY able to perform half duplex 100BASE-X
		SPD100XFD = Bit14,   /// PHY able to perform full duplex 100BASE-X
		SPD100T4 = Bit15     /// 100BASE-T4 capable
	};
	MODM_FLAGS16(SR);
};
};  // namespace modm::drivers

#endif