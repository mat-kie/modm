/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2009, Thorsten Lajewski
 * Copyright (c) 2012, 2015, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef	XPCC_INTERPOLATION__LAGRANGE_HPP
#define	XPCC_INTERPOLATION__LAGRANGE_HPP

#include <stdint.h>

#include <modm/utils/arithmetic_traits.hpp>
#include <modm/container/pair.hpp>
#include <modm/architecture/driver/accessor.hpp>

#include <modm/utils/template_metaprogramming.hpp>

namespace xpcc
{
	namespace interpolation
	{
		/**
		 * \brief	Lagrange Interpolation
		 * 
		 * Example:
		 * \code
		 * typedef xpcc::Pair<float, float> Point;
		 * 
		 * // interpolate x^2 over the range of 1 <= x <= 3
		 * Point points[3] =
		 * {
		 *     { 1, 1 },
		 *     { 2, 4 },
		 *     { 3, 9 }
		 * };
		 * 
		 * xpcc::interpolation::Lagrange<Point> value(points, 3);
		 * 
		 * ...
		 * float output = value.interpolate(1.5f);
		 * // output => 2.25;
		 * \endcode
		 * 
		 * \see http://en.wikipedia.org/wiki/Lagrange_interpolation
		 * 
		 * \warning	Only floating points types are allowed as second type of
		 * 			xpcc::Pair, otherwise the calculation will deliver wrong
		 * 			results!
		 * 
		 * \tparam	T	Any specialization of xpcc::Pair<> with a floating
		 * 				point type as second template argument.
		 * \tparam	Accessor	Accessor class. Can be xpcc::accessor::Ram,
		 * 						xpcc::accessor::Flash or any self defined
		 * 						accessor class.
		 * 						Default is xpcc::accessor::Ram.
		 * 
		 * \ingroup	interpolation
		 */
		template <typename T,
				  template <typename> class Accessor = ::xpcc::accessor::Ram>
		class Lagrange
		{
		public:
			typedef typename T::FirstType InputType;
			typedef typename T::SecondType OutputType;
			
			// WARNING:
			// Only floating point types are allowed as second type of xpcc::Pair
			// because the calculation will deliver wrong results otherwise!
			static_assert(xpcc::ArithmeticTraits<OutputType>::isFloatingPoint, 
					"Only floating point types are allowed as second type of xpcc::Pair");
		public:
			/**
			 * \brief	Constructor
			 * 
			 * \param	supportingPoints	Supporting points of the curve.
			 * 								Needs to be an Array of xpcc::Pair<>.
			 * \param	numberOfPoints		length of \p supportingPoints
			 */
			Lagrange(Accessor<T> supportingPoints, uint8_t numberOfPoints);
			
			/**
			 * \brief	Perform a Lagrange-interpolation
			 * 
			 * \param 	value	input value
			 * \return	interpolated value
			 */
			OutputType 
			interpolate(const InputType& value) const;
			
		private:
			const Accessor<T> supportingPoints;
			const uint8_t numberOfPoints; 
		};
	}
}

#include "lagrange_impl.hpp"

#endif	// XPCC_INTERPOLATION__LAGRANGE_HPP

