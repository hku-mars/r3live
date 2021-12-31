/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2006                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef __VCGLIB_LEGENDRE_H
#define __VCGLIB_LEGENDRE_H

#include <vcg/math/base.h>

namespace vcg {
namespace math {

/*
 * Contrary to their definition, the Associated Legendre Polynomials presented here are
 * only computed for positive m values.
 *
 */
template <typename ScalarType>
class Legendre {

protected :

	/**
	 * Legendre Polynomial three term Recurrence Relation
	 */
	static inline ScalarType legendre_next(unsigned l, ScalarType P_lm1, ScalarType P_lm2, ScalarType x)
	{
		return ((2 * l + 1) * x * P_lm1 - l * P_lm2) / (l + 1);
	}

	/**
	 * Associated Legendre Polynomial three term Recurrence Relation.
	 * Raises the band index.
	 */
	static inline double legendre_next(unsigned l, unsigned m, ScalarType P_lm1, ScalarType P_lm2, ScalarType x)
	{
		return ((2 * l + 1) * x * P_lm1 - (l + m) * P_lm2) / (l + 1 - m);
	}

	/**
	 * Recurrence relation to compute P_m_(m+1) given P_m_m at the same x
	 */
	static inline double legendre_P_m_mplusone(unsigned m, ScalarType p_m_m, ScalarType x)
	{
		return x * (2.0 * m + 1.0) * p_m_m;
	}

	/**
	 * Starting relation to compute P_m_m according to the formula:
	 *
	 * pow(-1, m) * double_factorial(2 * m - 1) * pow(1 - x*x, abs(m)/2)
	 *
	 * which becomes, if x = cos(theta) :
	 *
	 * pow(-1, m) * double_factorial(2 * m - 1) * pow(sin(theta), abs(m)/2)
	 */
	static inline double legendre_P_m_m(unsigned m, ScalarType sin_theta)
	{
		ScalarType p_m_m = 1.0;

		if (m > 0)
		{
			ScalarType fact = 1.0; //Double factorial here
			for (unsigned i = 1; i <= m; ++i)
			{
				p_m_m *= fact * sin_theta; //raising sin_theta to the power of m/2
				fact += 2.0;
			}

			if (m&1) //odd m
			{
				// Condon-Shortley Phase term
				p_m_m *= -1;
			}
		}

		return p_m_m;
	}

	static inline double legendre_P_l(unsigned l, ScalarType x)
	{
		ScalarType p0 = 1;
		ScalarType p1 = x;

		if (l == 0) return p0;

		for (unsigned n = 1; n < l; ++n)
		{
			std::swap(p0, p1);
			p1 = legendre_next(n, p0, p1, x);
		}

		return p1;
	}

	/**
	 * Computes the Associated Legendre Polynomial for any given
	 * positive m and l, with m <= l and -1 <= x <= 1.
	 */
	static inline double legendre_P_l_m(unsigned l, unsigned m, ScalarType cos_theta, ScalarType sin_theta)
	{
		if(m > l) return 0;
		if(m == 0) return legendre_P_l(l, cos_theta); //OK
		else
		{
			ScalarType p_m_m = legendre_P_m_m(m, sin_theta); //OK

			if (l == m) return p_m_m;

			ScalarType p_m_mplusone = legendre_P_m_mplusone(m, p_m_m, cos_theta); //OK

			if (l == m + 1) return p_m_mplusone;

			unsigned n = m + 1;

			while(n < l)
			{
				std::swap(p_m_m, p_m_mplusone);
				p_m_mplusone = legendre_next(n, m, p_m_m, p_m_mplusone, cos_theta);
				++n;
			}

			return p_m_mplusone;
		}
	}

public :

	static double Polynomial(unsigned l, ScalarType x)
	{
		assert (x <= 1 && x >= -1);
		return legendre_P_l(l, x);
	}

	static double AssociatedPolynomial(unsigned l, unsigned m, ScalarType x)
	{
		assert (m <= l && x <= 1 && x >= -1);
		return legendre_P_l_m(l, m, x, Sqrt(1.0 - x * x) );
	}

	static double AssociatedPolynomial(unsigned l, unsigned m, ScalarType cos_theta, ScalarType sin_theta)
	{
		assert (m <= l && cos_theta <= 1 && cos_theta >= -1 && sin_theta <= 1 && sin_theta >= -1);
		return legendre_P_l_m(l, m, cos_theta, Abs(sin_theta));
	}
};


template <typename ScalarType, int MAX_L>
class DynamicLegendre : public Legendre<ScalarType>
{

private:
	ScalarType matrix[MAX_L][MAX_L]; //dynamic table
	ScalarType _x; //table is conserved only across consistent x invocations
	ScalarType _sin_theta;

	void generate(ScalarType cos_theta, ScalarType sin_theta)
	{
		//generate all 'l's with m = 0

		matrix[0][0] = 1;
		matrix[0][1] = cos_theta;

		for (unsigned l = 2; l < MAX_L; ++l)
		{
			matrix[0][l] = legendre_next(l-1, matrix[0][l-1], matrix[0][l-2], cos_theta);
		}

		for(unsigned l = 1; l < MAX_L; ++l)
		{
			for (unsigned m = 1; m <= l; ++m)
			{
				if (l == m) matrix[m][m] = legendre_P_m_m(m, sin_theta);
				else if (l == m + 1) matrix[m][l] = legendre_P_m_mplusone(m, matrix[m][m], cos_theta);
				else{
					matrix[m][l] = legendre_next(l-1, m, matrix[m][l-1], matrix[m][l-2], cos_theta);
				}
			}
		}

		_x = cos_theta;
	}

public :

	DynamicLegendre() : _x(2), _sin_theta(2) {}

	double AssociatedPolynomial(unsigned l, unsigned m, ScalarType x)
	{
		assert (m <= l && x <= 1 && x >= -1);
		if (x != _x){
			_sin_theta = Sqrt(1.0 - x * x);
			generate(x, _sin_theta);
		}
		return matrix[m][l];
	}

	double AssociatedPolynomial(unsigned l, unsigned m, ScalarType cos_theta, ScalarType sin_theta)
	{
		assert (m <= l && cos_theta <= 1 && cos_theta >= -1 && sin_theta <= 1 && sin_theta >= -1);
		if (cos_theta != _x){
			_sin_theta = sin_theta;
			generate(cos_theta, _sin_theta);
		}
		return matrix[m][l];
	}

	double Polynomial(unsigned l, ScalarType x)
	{
		assert (x <= 1 && x >= -1);
		if (x != _x){
			_sin_theta = Sqrt(1.0 - x * x);
			generate(x, _sin_theta);
		}
		return matrix[0][l];
	}
};

}} //vcg::math namespace

#endif
