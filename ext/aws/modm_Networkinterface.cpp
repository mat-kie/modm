/// the modm specific implementation of the freertos + tcp Networkinterface goes here

#if __has_include("user_network_config.hpp")
#include <user_network_config.hpp>
#else
#error "you have to provide a user_network_config.hpp providing the Phy, Transport and BufferHandle"
// using BufferHandle = modm::ext::tcp::BufferHandle;
// using EthernetPhy = modm::driver::NullPHY<modm::ethernet::api::LinkStatus::DuplexMode::Full, modm::ethernet::api::LinkStatus::Speed::S100MBit>;
// using Transport = modm::ethernet::EthernetTransport<BufferHandle, 4, 4, 4, EthernetPhy>;
#endif

#include <modm/processing/rtos.hpp>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// FreeRTOS+TCP includes
#include "FreeRTOS_IP.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

using namespace modm::ethernet;

Transport::MacFilterHandle OwnMacHandle{};

#if (ipconfigUSE_LLMNR == 1)
Transport::MacFilterHandle LLMNRMacHandle{};
static const constexpr uint8_t LLMNR_MACAddress[] = {0x01, 0x00, 0x5E, 0x00, 0x00, 0xFC};
#endif

class EMACThread : public modm::rtos::Thread
{
    static TaskHandle_t EMACHandle;
    enum ThreadEvents
    {
        TxComplete = 1,
        RxComplete = 1 << 1,
        LinkStatusEvt = 1 << 2,
        Error = 1 << 3
    };

    bool m_linkUp{false};
public:
    EMACThread() : Thread(configMAX_PRIORITIES - 1, 1024, "emac_task") {}
    static void NotifyRxCompleteEvent()
    {
        if (EMACHandle == nullptr)
            return;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(EMACHandle, RxComplete, eSetBits, &(xHigherPriorityTaskWoken));
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
    static void NotifyTxCompleteEvent()
    {
        if (EMACHandle == nullptr)
            return;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(EMACHandle, TxComplete, eSetBits, &(xHigherPriorityTaskWoken));
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };

    static void NotifyLinkStatusEvent()
    {
        if (EMACHandle == nullptr)
            return;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(EMACHandle, LinkStatusEvt, eSetBits, &(xHigherPriorityTaskWoken));
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
    // static void ErrorCB();


    void run()
    {
        if (EMACHandle == nullptr)
            EMACHandle = xTaskGetCurrentTaskHandle();
        else
        {
            // error
            return;
        }

        while (true)
        {
            uint32_t notificationValue{0};
            if (xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, pdMS_TO_TICKS(500)) != pdTRUE)
            {
                auto linkStatus =m_linkUp ?  EthernetPhy::readLinkStatus(): EthernetPhy::startAutonegotiation();
                if (m_linkUp && (linkStatus.mode == api::LinkStatus::DuplexMode::None || linkStatus.speed == api::LinkStatus::Speed::None))
                {
                    FreeRTOS_NetworkDown();
                    m_linkUp = false;
                }else if(!m_linkUp)
                {
                    m_linkUp = true;
                }
                continue;
            }

            if (notificationValue & TxComplete)
            {
                // the transmission has finished
                Transport::clearFinishedTransmissions();
            }
            if (notificationValue & RxComplete)
            {
                // we received a new Frame send to ip task
                if (OwnMacHandle.hasRXFrame())
                {
                    NetworkBufferDescriptor_t* buf = OwnMacHandle.getRXFrame().detach();
                    IPStackEvent_t rxEvent{
                        .eEventType = eNetworkRxEvent,
                        .pvData = buf
                    };
                    auto result = xSendEventStructToIPTask(&rxEvent, portMAX_DELAY);
                    if (result == pdPASS)
                    {
                        iptraceNETWORK_INTERFACE_RECEIVE();
                    }
                    else
                    {
                        vReleaseNetworkBufferAndDescriptor((NetworkBufferDescriptor_t *)rxEvent.pvData);
                        iptraceETHERNET_RX_EVENT_LOST();
                    }
                }
            }
            if (notificationValue & LinkStatusEvt)
            {

            }
        }
    }
};
TaskHandle_t EMACThread::EMACHandle{nullptr};
EMACThread prvEMACHandler;

extern "C" BaseType_t xNetworkInterfaceInitialise(void)
{
    using namespace modm::ethernet;
    BaseType_t xReturn;

    /*
     * Perform the hardware specific network initialisation here.  Typically
     * that will involve using the Ethernet driver library to initialise the
     * Ethernet (or other network) hardware, initialise DMA descriptors, and
     * perform a PHY auto-negotiation to obtain a network link.
     *
     * This example assumes InitialiseNetwork() is an Ethernet peripheral driver
     * library function that returns 0 if the initialisation fails.
     */
    const uint8_t *macptr = FreeRTOS_GetMACAddress();
    Configuration cfg{};
    memcpy(cfg.macAddress, macptr, sizeof(modm::platform::eth::MacAddr_t));

    if (not Transport::initialize(cfg))
    {
        xReturn = pdFAIL;
    }
    else
    {
        OwnMacHandle = Transport::subscribe(EMACThread::NotifyRxCompleteEvent);
        xReturn = pdPASS;
    }

    return xReturn;
}

extern "C" BaseType_t xNetworkInterfaceOutput(
    NetworkBufferDescriptor_t *const pxDescriptor,
    BaseType_t xReleaseAfterSend)
{

    if (xReleaseAfterSend == pdTRUE)
    {
        Transport::transmitFrame(BufferHandle(pxDescriptor), EMACThread::NotifyTxCompleteEvent);
        return pdTRUE;
    }
    else{
        Transport::transmitFrameUnsafe(pxDescriptor, EMACThread::NotifyTxCompleteEvent);
        return pdTRUE;
    }
    return pdFALSE;
}
