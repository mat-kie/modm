/*
 * Copyright (c) 2011-2012, Fabian Greif
 * Copyright (c) 2012-2013, Niklas Hauser
 * Copyright (c) 2013, Kevin Laeufer
 * Copyright (c) 2013, 2015, Sascha Schade
 * Copyright (c) 2014, Daniel Krebs
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

// created with FontCreator 3.0

#include <modm/architecture/driver/accessor.hpp>

namespace xpcc
{
	namespace font
	{
		FLASH_STORAGE(uint8_t AllCaps3x5[]) =
		{
			0x72, 0x01, // total size of this array
			3,	// width (may vary)
			5,	// height
			1,	// hspace
			1, 	// vspace
			32,	// first char
			96,	// char count
			
			// char widths
			// for each character the separate width in pixels
			 3,  1,  2,  4,  3,  3,  4,  1,  2,  2, 
			 3,  3,  2,  3,  1,  3,  3,  3,  3,  3, 
			 3,  3,  3,  3,  3,  3,  1,  2,  3,  3, 
			 3,  3,  3,  3,  3,  3,  3,  3,  3,  3, 
			 3,  1,  2,  3,  3,  3,  3,  3,  3,  3, 
			 3,  3,  3,  3,  3,  3,  3,  3,  3,  2, 
			 3,  2,  3,  3,  2,  3,  3,  3,  3,  3, 
			 3,  3,  3,  1,  2,  3,  3,  3,  3,  3, 
			 3,  3,  3,  3,  3,  3,  3,  3,  3,  3, 
			 3,  3,  1,  3,  3,  3, 
			
			// font data
			// bit field of all characters
			0x00, 0x00, 0x00, // 32
			0x17, // 33
			0x03, 0x03, // 34
			0x1F, 0x0A, 0x1F, 0x0A, // 35
			0x0A, 0x1F, 0x0A, // 36
			0x0A, 0x04, 0x0A, // 37
			0x1A, 0x15, 0x0D, 0x12, // 38
			0x03, // 39
			0x0E, 0x11, // 40
			0x11, 0x0E, // 41
			0x0A, 0x04, 0x0A, // 42
			0x04, 0x0E, 0x04, // 43
			0x10, 0x08, // 44
			0x04, 0x04, 0x04, // 45
			0x10, // 46
			0x10, 0x0E, 0x01, // 47
			0x1F, 0x11, 0x1F, // 48
			0x12, 0x1F, 0x10, // 49
			0x1D, 0x15, 0x17, // 50
			0x11, 0x15, 0x1F, // 51
			0x07, 0x04, 0x1E, // 52
			0x17, 0x15, 0x1D, // 53
			0x1F, 0x15, 0x1D, // 54
			0x01, 0x1D, 0x03, // 55
			0x1F, 0x15, 0x1F, // 56
			0x17, 0x15, 0x1F, // 57
			0x0A, // 58
			0x10, 0x0A, // 59
			0x04, 0x0A, 0x11, // 60
			0x0A, 0x0A, 0x0A, // 61
			0x11, 0x0A, 0x04, // 62
			0x01, 0x15, 0x03, // 63
			0x1C, 0x12, 0x1A, // 64
			0x1E, 0x05, 0x1E, // 65
			0x1F, 0x15, 0x0A, // 66
			0x0E, 0x11, 0x11, // 67
			0x1F, 0x11, 0x0E, // 68
			0x1F, 0x15, 0x11, // 69
			0x1F, 0x05, 0x01, // 70
			0x0E, 0x11, 0x1D, // 71
			0x1F, 0x04, 0x1F, // 72
			0x1F, // 73
			0x10, 0x0F, // 74
			0x1F, 0x04, 0x1A, // 75
			0x1F, 0x10, 0x10, // 76
			0x1F, 0x02, 0x1F, // 77
			0x1F, 0x01, 0x1E, // 78
			0x1E, 0x11, 0x0F, // 79
			0x1F, 0x05, 0x06, // 80
			0x0E, 0x19, 0x1E, // 81
			0x1F, 0x05, 0x1A, // 82
			0x12, 0x15, 0x09, // 83
			0x01, 0x1F, 0x01, // 84
			0x0F, 0x10, 0x1F, // 85
			0x0F, 0x10, 0x0F, // 86
			0x1F, 0x08, 0x1F, // 87
			0x1B, 0x04, 0x1B, // 88
			0x07, 0x1C, 0x07, // 89
			0x19, 0x15, 0x13, // 90
			0x1F, 0x11, // 91
			0x01, 0x0E, 0x10, // 92
			0x11, 0x1F, // 93
			0x02, 0x01, 0x02, // 94
			0x10, 0x10, 0x10, // 95
			0x01, 0x02, // 96
			0x1E, 0x05, 0x1E, // 97
			0x1F, 0x15, 0x0A, // 98
			0x0E, 0x11, 0x11, // 99
			0x1F, 0x11, 0x0E, // 100
			0x1F, 0x15, 0x11, // 101
			0x1F, 0x05, 0x01, // 102
			0x0E, 0x11, 0x1D, // 103
			0x1F, 0x04, 0x1F, // 104
			0x1F, // 105
			0x10, 0x0F, // 106
			0x1F, 0x04, 0x1A, // 107
			0x1F, 0x10, 0x10, // 108
			0x1F, 0x02, 0x1F, // 109
			0x1F, 0x01, 0x1E, // 110
			0x1E, 0x11, 0x0F, // 111
			0x1F, 0x05, 0x06, // 112
			0x0E, 0x19, 0x1E, // 113
			0x1F, 0x05, 0x1A, // 114
			0x12, 0x15, 0x09, // 115
			0x01, 0x1F, 0x01, // 116
			0x0F, 0x10, 0x1F, // 117
			0x0F, 0x10, 0x0F, // 118
			0x1F, 0x08, 0x1F, // 119
			0x1B, 0x04, 0x1B, // 120
			0x07, 0x1C, 0x07, // 121
			0x19, 0x15, 0x13, // 122
			0x04, 0x0E, 0x11, // 123
			0x1F, // 124
			0x11, 0x0E, 0x04, // 125
			0x04, 0x06, 0x04, // 126
			0x00, 0x00, 0x00, // 127
		};
	}
}

