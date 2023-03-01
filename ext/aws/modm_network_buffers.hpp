#ifndef MODM_NETWORK_BUFFERS_HPP
#define MODM_NETWORK_BUFFERS_HPP
#include <concepts>
#include <cstddef>
#include <etl/array.h>
#include <etl/span.h>
#include <etl/optional.h>
#include <etl/atomic.h>

#include <FreeRTOS_IP.h>

namespace modm::ext::tcp
{
    /// @brief Unique pointer like handle for a network buffer descriptor structure
    class BufferHandle
    {
    public:
        typedef NetworkBufferDescriptor_t *pointer_type;
        typedef etl::span<uint8_t> container_type;

    private:
        pointer_type m_data{nullptr};

    public:
        BufferHandle() = default;
        BufferHandle(const BufferHandle &) = delete;
        BufferHandle &operator=(const BufferHandle &) = delete;

        /// @brief get the Address of the buffer for the dma
        /// @param ptr ptr to the descriptor object which is managed by this Handle class
        /// @return
        static uint8_t *getPayloadAddress(pointer_type ptr)
        {
            return ptr ? ptr->pucEthernetBuffer : nullptr;
        }
        /// @brief get the payload size of the buffer in bytes
        /// @param ptr ptr to the descriptor object which is managed by this Handle class
        /// @return
        static size_t getPayloadSizeBytes(pointer_type ptr)
        {
            return ptr ? ptr->xDataLength : 0;
        }

        explicit BufferHandle(pointer_type data) : m_data(data){};
        explicit BufferHandle(size_t size) : m_data(pxGetNetworkBufferWithDescriptor(size, 0)){

                                             };


        BufferHandle(BufferHandle &&rhs) : m_data(rhs.m_data)
        {
            rhs.m_data = nullptr;
        };

        ~BufferHandle()
        {
            reset();
        };

        BufferHandle &operator=(BufferHandle &&rhs)
        {
            reset();
            m_data = rhs.m_data;
            rhs.m_data = nullptr;
            return *this;
        };


        pointer_type detach()
        {
            pointer_type tmp = m_data;
            m_data = nullptr;
            return tmp;
        };

        container_type data()
        {
            if (m_data != nullptr)
            {
                return etl::span<uint8_t>{m_data->pucEthernetBuffer, m_data->xDataLength};
            }
            return {};
        };

        // etl::span data();
        size_t size()
        {
            if (m_data)
            {
                return m_data->xDataLength;
            }
            return 0;
        };
        void reset()
        {
            if (m_data != nullptr)
            {
                vReleaseNetworkBufferAndDescriptor(m_data);
            };
        }
        bool resize(size_t size)
        {
            if (m_data && m_data->xDataLength >= size)
            {
                m_data->xDataLength = size;
                return true;
            }
            return false;
        };

        bool hasData() { return m_data != nullptr; }
    };

};

#endif