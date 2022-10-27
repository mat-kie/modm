# Interface concept
The Ethernet periphery to IP Stack connection in now as follows:
- The Input layer to the IP Stack Accesses a PHY and a EMAC class
- the PHY class provides the Link Status information
- the EMAC class provides generalized configuration methods and the interface to the dma-managed buffers

# Transmit / Recieve procedure
THe DMA Descriptors are now managed by the EMAC class.
when an Interrupt is recieved (ISR is still in the NetworkInterface Implementation),
the Networkinterface checks if there is indeed a Descriptor currently not owned by the dma
with HasRxDescriptor / HasTxDescriptor.
if this is the case, a Handle to the descriptor can be obtained with GetDescriptor.
The buffer pointers can be exchanged and the descriptor is handed back to the dma.

# why?
Ideally the implementations for the FreeRTOS+TCP Networkinterface are now independent
from the Hardware as long as the interface is provided. only the isr is still HW specific.

xNetworkInterfaceOutput
xGetLinkStatus
and the deferred interrupt handler task are no longer hw specific.

this might be helpful for code readability and future implementations for other hw?
this enables easier integration with obscure phys that do not use the SMI / MIIMI
or direct-to-switch usecases.
