#include <modm/platform/eth/stm32/eth.hpp>


namespace modm{

    class LAN8742a: PAL<LAN8742a>, PHY{
		private:
			static bool linkStatus{false};
        public:
            static bool initialize(){
                // Initialize PHY
		        uint32_t phy_register(0);
		        /// Step 1.: reset phy. this is mandatory for config (see 802.3-22.2.4)
		        phy_register |= PHY::CR::RESET;
				if (!writePhyRegister(PHY::Register::CR, phy_register)) {
					return false;
				}
				// wait for reset done
				modm::delay_us(255);
				uint16_t timeout = 0'500;
				do {
					(void) readPhyRegister(PHY::Register::CR, phy_register);
					if ((phy_register & PHY::CR::RESET) == 0)
						break;
					modm::delay_ms(1);
				} while (timeout-- > 0);
				if (timeout <= 0) {
					return false;
				}

				// FIXME: Delegate these values to the PHY
				// configure auto negotiation
				phy_register = 0x100 // 100 FD
						| 0x80 // 100
						| 0x40 // 10 FD
						| 0x20 // 10
						| 0x01 // 802.3
						;
				(void) writePhyRegister(PHY::Register::ANA, phy_register);

    		};

            static ANResult startAutoNegotiation(){
				uint32_t phy_register { 0 };
				// enable auto-negotiation
				(void) readPhyRegister(PHY::Register::CR, phy_register);
				phy_register |= PHY::CR::RAN;
				if (!writePhyRegister(PHY::Register::CR, phy_register))
				{
					return ANResult();
				}
				// wait for auto-negotiation complete (5s)
				uint16_t timeout = 5'000;
				do {
					(void) readPhyRegister(PHY::Register::SR, phy_register);
					if ((phy_register & PHY::SR::AN_COMP) == PHY::SR::AN_COMP)
					{
						break;
					}
					modm::delay_ms(1);
				} while (timeout-- > 0);
				if (timeout == 0)
				{
					return ANResult();
				}
				// read auto-negotiation result
				if (!readPhyRegister(PHY::Register::SR, phy_register))
				{
					return ANResultU();
				}
				ANResult cfg{
					.isFullDuplex = (phy_register & (PHY::SR::SPD100T2FD | PHY::SR::SPD100XFD | PHY::SR::SPD10FD)) != 0,
					.speed = (phy_register & (PHY::SR::SPD100T4|PHY::SR::SPD100T2FD|PHY::SR::SPD100T2HD |PHY::SR::SPD100T4 | PHY::SR::SPD100XFD | PHY::SR::SPD100XHD) !=0 ? 100 : 10
				};

				return cfg;
            };
           
           static bool readLinkStatus(){
                uint32_t phy_register { 0 };
				(void) readPhyRegister(PHY::Register::SR, phy_register);
				if ((phy_register & PHY::SR::LINK_STATUS) == PHY::SR::LINK_STATUS)
					linkStatus = true;
				else
					linkStatus false;;

				return linkStatus;
            };
            static bool getLinkStatus(){
                return linkStatus;
            };
           
    };

};