// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2009, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Roboterclub Aachen e.V. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ROBOTERCLUB AACHEN E.V. ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOTERCLUB AACHEN E.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_I2C__EEPROM_HPP
	#error	"Don't include this file directly, use 'eeprom.hpp' instead!"
#endif

// ----------------------------------------------------------------------------
template <typename I2C>
I2C xpcc::i2c::Eeprom<I2C>::i2c;

// ----------------------------------------------------------------------------
template <typename I2C>
xpcc::i2c::Eeprom<I2C>::Eeprom(uint8_t address) :
	deviceAddress(address)
{
}

// ----------------------------------------------------------------------------
template <typename I2C>
bool
xpcc::i2c::Eeprom<I2C>::isAvailable() const
{
	bool ack = i2c.start(deviceAddress | READ);
	i2c.stop();
	
	return ack;
}

// ----------------------------------------------------------------------------
template <typename I2C>
bool
xpcc::i2c::Eeprom<I2C>::write(uint16_t address, uint8_t data) const
{
	bool ack = false;
	
	if (i2c.start(deviceAddress | WRITE))
	{
		ack = true;
		ack &= i2c.write(address >> 8);
		ack &= i2c.write(address & 0xff);
		ack &= i2c.write(data);
	}
	i2c.stop();
	
	return ack;
}

template <typename I2C>
bool
xpcc::i2c::Eeprom<I2C>::write(uint16_t address, const uint8_t *data, uint8_t bytes) const
{
	bool ack = false;
	
	if (i2c.start(deviceAddress | WRITE))
	{
		ack = true;
		ack &= i2c.write(address >> 8);
		ack &= i2c.write(address & 0xff);
		
		for (uint8_t i = 0; i < bytes; ++i) {
			ack &= i2c.write(*data++);
		}
	}
	i2c.stop();
	
	return ack;
}

// ----------------------------------------------------------------------------
template <typename I2C>
bool
xpcc::i2c::Eeprom<I2C>::read(uint16_t address, uint8_t &data) const
{
	bool ack = false;
	if (i2c.start(deviceAddress | WRITE))
	{
		ack = true;
		ack &= i2c.write(address >> 8);
		ack &= i2c.write(address & 0xff);
		
		if (ack && i2c.repeatedStart(deviceAddress | READ))
		{
			data = i2c.read(NACK);
		}
	}
	i2c.stop();
	
	return ack;
}

template <typename I2C>
bool
xpcc::i2c::Eeprom<I2C>::read(uint16_t address, uint8_t *data, uint8_t bytes) const
{
	bool ack = false;
	if (i2c.start(deviceAddress | WRITE))
	{
		ack = true;
		ack &= i2c.write(address >> 8);
		ack &= i2c.write(address & 0xff);
		
		if (ack && i2c.repeatedStart(deviceAddress | READ))
		{
			for (uint8_t i = 0; i < bytes; ++i) {
				*data++ = i2c.read(ACK);
			}
		}
	}
	i2c.stop();
	
	return ack;
}
