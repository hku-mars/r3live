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

#ifndef __VCGLIB_SPHERICAL_HARMONICS_H
#define __VCGLIB_SPHERICAL_HARMONICS_H

#include <climits>

#include <vcg/math/base.h>
#include <vcg/math/random_generator.h>
#include <vcg/math/legendre.h>
#include <vcg/math/factorial.h>

namespace vcg{
namespace math{

template <typename ScalarType>
class DummyPolarFunctor{
	public:
		inline ScalarType operator()(ScalarType theta, ScalarType phi) {return ScalarType(0);}
};


template <typename ScalarType, int MAX_BAND = 4>
class ScalingFactor
{
private :

	ScalarType k_factor[MAX_BAND][MAX_BAND];

	static ScalingFactor sf;

	ScalingFactor()
	{
		for (unsigned l = 0; l < MAX_BAND; ++l)
			for (unsigned m = 0; m <= l; ++m)
				k_factor[l][m] = Sqrt( ( (2.0*l + 1.0) * Factorial<ScalarType>(l-m) ) / (4.0 * M_PI * Factorial<ScalarType>(l + m)) );
	}

public :
	static ScalarType K(unsigned l, unsigned m)
	{
		return sf.k_factor[l][m];
	}
};

template <typename ScalarType, int MAX_BAND>
ScalingFactor<ScalarType, MAX_BAND> ScalingFactor<ScalarType, MAX_BAND>::sf;

/**
 * Although the Real Spherical Harmonic Function is correctly defined over any
 * positive l and any -l <= m <= l, the two internal functions computing the
 * imaginary and real parts of the Complex Spherical Harmonic Functions are defined
 * for positive m only.
 */
template <typename ScalarType, int MAX_BAND = 4>
class SphericalHarmonics{

private :

	static DynamicLegendre<ScalarType, MAX_BAND> legendre;

	static ScalarType scaling_factor(unsigned l, unsigned m)
	{
		return ScalingFactor<ScalarType, MAX_BAND>::K(l,m);
	}

	inline static ScalarType complex_spherical_harmonic_re(unsigned l, unsigned m, ScalarType theta, ScalarType phi)
	{
		return scaling_factor(l, m) * legendre.AssociatedPolynomial(l, m, Cos(theta), Sin(theta)) * Cos(m * phi);
	}

	inline static ScalarType complex_spherical_harmonic_im(unsigned l, unsigned m, ScalarType theta, ScalarType phi)
	{
		return scaling_factor(l, m) * legendre.AssociatedPolynomial(l, m, Cos(theta), Sin(theta)) * Sin(m * phi);
	}

	ScalarType coefficients[MAX_BAND * MAX_BAND];

public :

	/**
	 * Returns the Real Spherical Harmonic Function
	 *
	 * l is any positive integer,
	 * m is such that -l <= m <= l
	 * theta is inside [0, PI]
	 * phi is inside [0, 2*PI]
	 */
	static ScalarType Real(unsigned l, int m, ScalarType theta, ScalarType phi)
	{
		assert((int)-l <= m && m <= (int)l && theta >= 0.0 && theta <= (ScalarType)M_PI && phi >= 0.0 && phi <= (ScalarType)(2.0 * M_PI));

		if (m > 0) return SQRT_TWO * complex_spherical_harmonic_re(l, m, theta, phi);

		else if (m == 0) return scaling_factor(l, 0) * legendre.Polynomial(l, Cos(theta));

		else return SQRT_TWO * complex_spherical_harmonic_im(l, -m, theta, phi);
	}

	template <typename PolarFunctor>
	static SphericalHarmonics Project(PolarFunctor * fun, unsigned n_samples)
	{
		const ScalarType weight = 4 * M_PI;

		unsigned sqrt_n_samples = (unsigned int) Sqrt((int)n_samples);
		unsigned actual_n_samples = sqrt_n_samples * sqrt_n_samples;
		unsigned n_coeff = MAX_BAND * MAX_BAND;

		ScalarType one_over_n = 1.0/(ScalarType)sqrt_n_samples;

		MarsenneTwisterRNG rand;
		SphericalHarmonics sph;

		int i = 0;

		for (unsigned k = 0; k < n_coeff; k++ ) sph.coefficients[k] = 0;

		for (unsigned a = 0; a < sqrt_n_samples; ++a )
		{
			for (unsigned b = 0; b < sqrt_n_samples; ++b)
			{
				ScalarType x = (a + ScalarType(rand.generate01())) * one_over_n;
				ScalarType y = (b + ScalarType(rand.generate01())) * one_over_n;

				ScalarType theta = 2.0 * Acos(Sqrt(1.0 - x));
				ScalarType phi = 2.0 * M_PI * y;

				for (int l = 0; l < (int)MAX_BAND; ++l)
				{
					for (int m = -l; m <= l; ++m)
					{
						int index = l * (l+1) + m;
						sph.coefficients[index] += (*fun)(theta, phi) * Real(l, m, theta, phi);
					}
				}
				i++;
			}
		}

		ScalarType factor = weight / actual_n_samples;
		for(i = 0; i < (int)n_coeff; ++i)
		{
			sph.coefficients[i] *= factor;
		}

		return sph;
	}

	static SphericalHarmonics Wrap(ScalarType * _coefficients)
	{
		SphericalHarmonics sph;
                for(int i = 0; i < (int) MAX_BAND *  MAX_BAND; ++i) sph.coefficients[i] = _coefficients[i];
		return sph;
	}

	ScalarType operator()(ScalarType theta, ScalarType phi)
	{
		ScalarType f = 0;

		for (int l = 0; l < MAX_BAND; ++l)
		{
			for (int m = -l; m <= l; ++m)
			{
				int index = l * (l+1) + m;
				f += (coefficients[index] * Real(l, m, theta, phi));
			}
		}

		return f;
	}
};

template <typename ScalarType, int MAX_BAND>
DynamicLegendre<ScalarType, MAX_BAND> SphericalHarmonics<ScalarType, MAX_BAND>::legendre;

}} //namespace vcg::math

#endif
