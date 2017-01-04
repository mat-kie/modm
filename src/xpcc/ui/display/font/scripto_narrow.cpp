/*
 * Copyright (c) 2010-2012, Fabian Greif
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
		FLASH_STORAGE(uint8_t ScriptoNarrow[]) =
		{
			0x7E, 0x01, // total size of this array
			3,	// width (may vary)
			7,	// height
			0,	// hspace
			1, 	// vspace
			32,	// first char
			95,	// char count
			
			// char widths
			// for each character the separate width in pixels
			 3,  1,  3,  5,  5,  6,  4,  1,  2,  2, 
			 4,  3,  2,  3,  2,  2,  3,  2,  3,  3, 
			 3,  3,  3,  3,  3,  3,  1,  2,  3,  3, 
			 3,  4,  3,  3,  3,  3,  3,  3,  3,  3, 
			 3,  1,  3,  3,  3,  4,  3,  3,  3,  3, 
			 3,  3,  3,  3,  3,  4,  4,  3,  3,  2, 
			 2,  2,  3,  3,  2,  3,  3,  3,  3,  3, 
			 2,  3,  3,  1,  2,  3,  1,  5,  3,  3, 
			 3,  3,  3,  4,  3,  3,  3,  5,  3,  3, 
			 4,  3,  1,  3,  5, 
			
			// font data
			// bit field of all characters
			0x00, 0x00, 0x00, // 32
			0x2F, // 33
			0x03, 0x00, 0x03, // 34
			0x12, 0x3F, 0x12, 0x3F, 0x12, // 35
			0x24, 0x2A, 0x7F, 0x2A, 0x12, // 36
			0x26, 0x15, 0x0B, 0x34, 0x2A, 0x19, // 37
			0x3A, 0x2D, 0x13, 0x20, // 38
			0x03, // 39
			0x1E, 0x21, // 40
			0x21, 0x1E, // 41
			0x2A, 0x1C, 0x1C, 0x2A, // 42
			0x08, 0x1C, 0x08, // 43
			0x40, 0x20, // 44
			0x10, 0x10, 0x10, // 45
			0x40, 0x20, // 46
			0x38, 0x07, // 47
			0x3E, 0x21, 0x1F, // 48
			0x02, 0x3F, // 49
			0x39, 0x25, 0x22, // 50
			0x21, 0x25, 0x3A, // 51
			0x0C, 0x0A, 0x3F, // 52
			0x27, 0x25, 0x39, // 53
			0x1E, 0x25, 0x1D, // 54
			0x01, 0x39, 0x07, // 55
			0x3A, 0x25, 0x1B, // 56
			0x2E, 0x29, 0x1E, // 57
			0x14, // 58
			0x40, 0x24, // 59
			0x08, 0x14, 0x22, // 60
			0x14, 0x14, 0x14, // 61
			0x22, 0x14, 0x08, // 62
			0x02, 0x29, 0x05, 0x02, // 63
			0x3F, 0x21, 0x2F, // 64
			0x3F, 0x09, 0x3E, // 65
			0x3F, 0x25, 0x3A, // 66
			0x3E, 0x21, 0x21, // 67
			0x3F, 0x21, 0x3E, // 68
			0x3F, 0x25, 0x21, // 69
			0x3F, 0x05, 0x01, // 70
			0x3E, 0x29, 0x39, // 71
			0x3F, 0x04, 0x3F, // 72
			0x3F, // 73
			0x20, 0x20, 0x1F, // 74
			0x3F, 0x04, 0x3B, // 75
			0x3F, 0x20, 0x20, // 76
			0x3F, 0x02, 0x02, 0x3F, // 77
			0x3F, 0x02, 0x3F, // 78
			0x3E, 0x21, 0x1F, // 79
			0x3F, 0x09, 0x0E, // 80
			0x1E, 0x31, 0x3E, // 81
			0x3F, 0x09, 0x36, // 82
			0x22, 0x25, 0x19, // 83
			0x01, 0x3F, 0x01, // 84
			0x1F, 0x20, 0x3F, // 85
			0x1F, 0x20, 0x1F, // 86
			0x3F, 0x10, 0x10, 0x3F, // 87
			0x21, 0x1E, 0x1E, 0x21, // 88
			0x07, 0x38, 0x07, // 89
			0x39, 0x25, 0x23, // 90
			0x3F, 0x21, // 91
			0x07, 0x38, // 92
			0x21, 0x3F, // 93
			0x02, 0x01, 0x02, // 94
			0x40, 0x40, 0x40, // 95
			0x01, 0x02, // 96
			0x38, 0x24, 0x3C, // 97
			0x3F, 0x24, 0x38, // 98
			0x38, 0x24, 0x24, // 99
			0x38, 0x24, 0x3F, // 100
			0x38, 0x2C, 0x28, // 101
			0x3E, 0x05, // 102
			0x5C, 0x52, 0x3E, // 103
			0x3F, 0x04, 0x38, // 104
			0x3D, // 105
			0x40, 0x3D, // 106
			0x3F, 0x0C, 0x34, // 107
			0x3F, // 108
			0x3C, 0x04, 0x38, 0x04, 0x38, // 109
			0x3C, 0x04, 0x38, // 110
			0x38, 0x24, 0x1C, // 111
			0x7E, 0x12, 0x1C, // 112
			0x1C, 0x12, 0x7E, // 113
			0x3C, 0x08, 0x04, // 114
			0x28, 0x2C, 0x34, 0x14, // 115
			0x04, 0x3F, 0x04, // 116
			0x1C, 0x20, 0x3C, // 117
			0x1C, 0x20, 0x1C, // 118
			0x3C, 0x20, 0x1C, 0x20, 0x1C, // 119
			0x24, 0x18, 0x24, // 120
			0x5E, 0x50, 0x3E, // 121
			0x24, 0x34, 0x2C, 0x24, // 122
			0x08, 0x36, 0x41, // 123
			0x33, // 124
			0x41, 0x36, 0x08, // 125
			0x08, 0x04, 0x0C, 0x08, 0x04, // 126
		};
	}
}

