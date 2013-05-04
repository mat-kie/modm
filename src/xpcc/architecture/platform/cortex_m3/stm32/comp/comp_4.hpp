// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2013, Roboterclub Aachen e.V.
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
 */
// ----------------------------------------------------------------------------
/*
 * WARNING: This file is generated automatically, do not edit!
 * Please modify the corresponding *.in file instead and rebuild this file. 
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_STM32__COMP4_HPP
#define XPCC_STM32__COMP4_HPP

#if defined(STM32F3XX)
namespace xpcc
{
	namespace stm32
	{
		/**
		 * @brief		Comparator Class for STM32F3 series
		 * 
		 * The class provides an interface to the comparators 1-7 availably on
		 * ST's STM32F3 microcontroller series.
		 * 
		 * @internal
		 * @ingroup		stm32
		 */
		class Comp4
		{
		public:
			enum Mode
			{
				ULTRA_LOW_POWER = 0b00,
				LOW_POWER = COMP4_CSR_COMP4MODE_0,		//0b01
				MEDIUM_POWER = COMP4_CSR_COMP4MODE_1,		//0b10
				HIGH_SPEED = 	COMP4_CSR_COMP4MODE_0 |
								COMP4_CSR_COMP4MODE_1		//0b11
			};

			enum InvertingInput
			{
				VREF_1_4 = 0b000,	//0b000 1/4 of Vrefint
				VREF_1_2 =			// 1/2 of Vrefint
							COMP4_CSR_COMP4INSEL_0,	//0b001
				VREF_3_4 =			// 3/4 of Vrefint
							COMP4_CSR_COMP4INSEL_1,	//0b010
				VREF = 				// Vrefint
							COMP4_CSR_COMP4INSEL_1 |
							COMP4_CSR_COMP4INSEL_0,	//0b011
				// Common Port Aliases
				PA4 =	COMP4_CSR_COMP4INSEL_2,		//0b100
				DAC1 =	COMP4_CSR_COMP4INSEL_2,		//0b100
				PA5 = 	COMP4_CSR_COMP4INSEL_2 |
						COMP4_CSR_COMP4INSEL_0,		//0b101
				DAC2 = 	COMP4_CSR_COMP4INSEL_2 |
						COMP4_CSR_COMP4INSEL_0,		//0b101
			// PA4 or DAC1 output if enabled
				COMP4_INM4 = 	COMP4_CSR_COMP4INSEL_2,	//0b100
				// PA5 or DAC2 output if enabled
				COMP4_INM5 = 	COMP4_CSR_COMP4INSEL_2 |
								COMP4_CSR_COMP4INSEL_0,	//0b101
				// PE8
				COMP4_INM6 = 	COMP4_CSR_COMP4INSEL_2 |
								COMP4_CSR_COMP4INSEL_1,	//0b110
				// PB2
				COMP4_INM7 = 	COMP4_CSR_COMP4INSEL_2 |
								COMP4_CSR_COMP4INSEL_1 |
								COMP4_CSR_COMP4INSEL_0,	//0b111
				// Port Alias
				PE8 = 			COMP4_CSR_COMP4INSEL_2 |
								COMP4_CSR_COMP4INSEL_1,	//0b110
				PB2 = 			COMP4_CSR_COMP4INSEL_2 |
								COMP4_CSR_COMP4INSEL_1 |
								COMP4_CSR_COMP4INSEL_0	//0b111
			};

enum NonInvertingInput
			{
			PB0 = 0b0,
				PE7 = COMP4_CSR_COMP4NONINSEL		//0b1
			};
enum Output
			{
				NOT_CONNECTED = 0b0000,
				TIMER_1_BREAK_INPUT =
								COMP4_CSR_COMP4OUTSEL_0,	//0b0001
				TIMER_1_BREAK_INPUT_2 =
								COMP4_CSR_COMP4OUTSEL_1,	//0b0010
				TIMER_8_BREAK_INPUT =
								COMP4_CSR_COMP4OUTSEL_1 |
								COMP4_CSR_COMP4OUTSEL_0,	//0b0011
				TIMER_8_BREAK_INPUT_2 =
								COMP4_CSR_COMP4OUTSEL_2,	//0b0100
				TIMER_8_OR_1_BREAK_INPUTS_2 =
								COMP4_CSR_COMP4OUTSEL_2 |
								COMP4_CSR_COMP4OUTSEL_0,	//0b0101

			TIMER_3_INPUT_CAPTURE_3 =
								COMP4_CSR_COMP4OUTSEL_2 |
								COMP4_CSR_COMP4OUTSEL_1,	//0b0110
				TIMER_8_OCREF_CLEAR_INPUT =
								COMP4_CSR_COMP4OUTSEL_2 |
								COMP4_CSR_COMP4OUTSEL_1 |
								COMP4_CSR_COMP4OUTSEL_0,	//0b0111
				TIMER_15_INPUT_CAPTURE_2 =
								COMP4_CSR_COMP4OUTSEL_3,	//0b1000
				TIMER_4_INPUT_CAPTURE_2 =
								COMP4_CSR_COMP4OUTSEL_3 |
								COMP4_CSR_COMP4OUTSEL_0,	//0b1001
				TIMER_15_OCREF_CLEAR_INPUT =
								COMP4_CSR_COMP4OUTSEL_3 |
								COMP4_CSR_COMP4OUTSEL_1,	//0b1010
				TIMER_3_OCREF_CLEAR_INPUT =
								COMP4_CSR_COMP4OUTSEL_3 |
								COMP4_CSR_COMP4OUTSEL_1 |
								COMP4_CSR_COMP4OUTSEL_0	//0b1011
			};

			enum Polarity
			{
				NOT_INVERTED = 0b0,
				INVERTED = COMP4_CSR_COMP4POL		//0b1
			};

			enum Hysteresis
			{
				NO_HYSTERESIS = 0b00,
				LOW_HYSTERESIS = COMP4_CSR_COMP4HYST_0,	//0b01
				MEDIUM_HYSTERESIS = COMP4_CSR_COMP4HYST_1,//0b10
				HIGH_HYSTERESIS = COMP4_CSR_COMP4HYST_1 |
									COMP4_CSR_COMP4HYST_0	//0b11
			};

			enum BlankingSource
			{
				NO_BLANKING = 0b000,
			TIMER_3_OC4 = COMP4_CSR_COMP4BLANKING_0,	// 0b001
				TIMER_8_OC5 = COMP4_CSR_COMP4BLANKING_1,	// 0b010
				TIMER_15_OC1 = COMP4_CSR_COMP4BLANKING_1 |
								COMP4_CSR_COMP4BLANKING_0	// 0b011
			};

		public:

			/**
			 * Initialize and enable the comparator.
			 *
			 * Enables the comperator and sets important values.
			 * 
			 * Do NOT set lock = true if you want to be able to set other values
			 * later.
			 */
			static inline void
			initialize(
						InvertingInput n_in,
					NonInvertingInput p_in,
					Output out = Output::NOT_CONNECTED,
						Hysteresis hyst = Hysteresis::NO_HYSTERESIS,
						Mode mode = Mode::HIGH_SPEED,
						Polarity pol = Polarity::NOT_INVERTED,
						bool lock_comp = false)
			{
				setInvertingInput(n_in);
			setNonInvertingInput(p_in);
			setOutputSelection(out);
				setHysteresis(hyst);
				setMode(mode);
				setPolarity(pol);
				setEnabled(true);	// enable comparator
				if(lock_comp) lock();
			}
			


			/**
			 * \brief	Enable/Disable the comparator.
			 */
			static inline void
			setEnabled(bool enabled)
			{
				if(enabled)
					COMP4->CSR |= COMP4_CSR_COMP4EN;
				else
					COMP4->CSR &= ~COMP4_CSR_COMP4EN;
			}

			/**
			 * \brief	Returns whether the comparator is enabled.
			 */
			static inline bool
			isEnabled()
			{
				return COMP4->CSR & COMP4_CSR_COMP4EN;
			}

		/**
			 * \brief	Sets the mode that determins speed/power consumption.
			 * 
			 * This setting is also called "output mode".
			 */
			static inline void
			setMode(Mode mode)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4MODE)
					| static_cast<uint32_t>(mode);
			}

			/**
			 * \brief	Sets the mode that determins speed/power consumption.
			 * 
			 * This setting is also called "output mode".
			 */
			static inline Mode
			getMode()
			{
				return static_cast<Mode>
					(COMP4->CSR & COMP4_CSR_COMP4MODE);
			}

			/**
			 * \brief	Selects what the inverting input is connected to.
			 */
			static inline void
			setInvertingInput(InvertingInput input)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4INSEL)
					| static_cast<uint32_t>(input);
			}

			/**.
			 * \brief	Returns what is connected to the inverting input.
			 */
			static inline InvertingInput
			getInvertingInput()
			{
				return static_cast<InvertingInput>
					(COMP4->CSR & COMP4_CSR_COMP4INSEL);
			}

		/**
			 * \brief	Selects what the non-inverting input is connected to.
			 */
			static inline void
			setNonInvertingInput(NonInvertingInput input)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4NONINSEL)
					| static_cast<uint32_t>(input);
			}

			/**.
			 * \brief	Returns what is connected to the non-inverting input.
			 */
			static inline NonInvertingInput
			getNonInvertingInput()
			{
				return static_cast<NonInvertingInput>
					(COMP4->CSR & COMP4_CSR_COMP4NONINSEL);
			}
		/**
			 * \brief Enable/Disable window mode for COMP3/4.
			 */
			static inline void
			setWindowMode(bool enabled)
			{
				if(enabled)
					COMP4->CSR |= COMP4_CSR_COMP4WNDWEN;
				else
					COMP4->CSR &= ~COMP4_CSR_COMP4WNDWEN;
			}

			/**
			 * \brief Returns true if win mode for COMP3/4 on.
			 */
			static inline bool
			isWindowModeEnabled()
			{
				return COMP4->CSR & COMP4_CSR_COMP4WNDWEN;
			}
		/**
			 * \brief	Selects what the output is connected to.
			 */
			static inline void
			setOutputSelection(Output output)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4OUTSEL)
					| static_cast<uint32_t>(output);
			}

			/**.
			 * \brief	Returns what is connected to the output.
			 */
			static inline Output
			getOutputSelection()
			{
				return static_cast<Output>
					(COMP4->CSR & COMP4_CSR_COMP4OUTSEL);
			}

			/**
			 * \brief	Selects output polarity.
			 */
			static inline void
			setPolarity(Polarity pol)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4POL)
					| static_cast<uint32_t>(pol);
			}

			/**.
			 * \brief	Returns output polarity.
			 */
			static inline Polarity
			getPolarity()
			{
				return static_cast<Polarity>
					(COMP4->CSR & COMP4_CSR_COMP4POL);
			}

			/**
			 * \brief	Selects the hysteresis.
			 */
			static inline void
			setHysteresis(Hysteresis hyst)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4HYST)
					| static_cast<uint32_t>(hyst);
			}

			/**.
			 * \brief	Returns the hysteresis.
			 */
			static inline Hysteresis
			getHysteresis()
			{
				return static_cast<Hysteresis>
					(COMP4->CSR & COMP4_CSR_COMP4HYST);
			}

			/**
			 * \brief	Selects the blanking source.
			 */
			static inline void
			setBlankingSource(BlankingSource blanking)
			{
				COMP4->CSR |=
					(COMP4->CSR & ~COMP4_CSR_COMP4BLANKING)
					| static_cast<uint32_t>(blanking);
			}

			/**.
			 * \brief	Returns the blanking source.
			 */
			static inline BlankingSource
			getBlankingSource()
			{
				return static_cast<BlankingSource>
					(COMP4->CSR & COMP4_CSR_COMP4BLANKING);
			}

			/**
			 * \brief	Returns the current Comparator output.
			 */
			static inline bool
			getOutput()
			{
				return COMP4->CSR & COMP4_CSR_COMP4OUT;
			}

			/**
			 * \brief	Locks the comparator. 
			 *
			 * Comparator can only be unlocked by a system reset.
			 */
			static inline void
			lock(void)
			{
				COMP4->CSR |= COMP4_CSR_COMP4LOCK;
			}

			/**
			 * \brief	Returns true if the comparator is locked.
			 */
			static inline bool
			isLocked(void)
			{
				return COMP4->CSR & COMP4_CSR_COMP4LOCK;
			}
		};
	}
}
#else	// defined(STM32F3XX)
	#  error 	"STM32F3XX not defined. Are you sure the comparator hw " \
				"feature is enabled on you platform?"
#endif

#endif	//  XPCC_STM32__COMP4_HPP