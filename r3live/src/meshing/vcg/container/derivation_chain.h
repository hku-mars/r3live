/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
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

#ifndef __VCG_DERIVATION_CHAIN
#define __VCG_DERIVATION_CHAIN

namespace vcg{
/*------------------------------------------------------------------*/ 

// Metaprogramming Core
template < typename T=int>
class DefaultDeriver : public T {};

template <
					class Base,
          template <typename> class A> 
					class Arity1: public A<Base > {
          };

template <
					class Base,
          template <typename> class A, template <typename> class B> 
					class Arity2: public B<Arity1<Base, A> > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C > 
					class Arity3: public C<Arity2<Base, A, B> > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D> 
					class Arity4: public D<Arity3<Base, A, B, C> > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E > 
					class Arity5: public E<Arity4<Base, A, B, C, D> > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F > 
					class Arity6: public F<Arity5<Base, A, B, C, D, E> > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
					template <typename> class G> 
					class Arity7: public G<Arity6<Base, A, B, C, D, E, F> > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
					template <typename> class G, template <typename> class H> 
					class Arity8: public H<Arity7<Base, A, B, C, D, E, F, G > > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
					template <typename> class G, template <typename> class H,
					template <typename> class I>
					class Arity9: public I<Arity8<Base, A, B, C, D, E, F, G, H > > {};
					
template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
					template <typename> class G, template <typename> class H,
					template <typename> class I, template <typename> class J>
					class Arity10: public J<Arity9<Base, A, B, C, D, E, F, G, H, I > > {};

template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
          template <typename> class G, template <typename> class H,
          template <typename> class I, template <typename> class J,
          template <typename> class K>
					class Arity11: public K<Arity10<Base, A, B, C, D, E, F, G, H, I, J> > {};


template <
					class Base,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
          template <typename> class G, template <typename> class H,
          template <typename> class I, template <typename> class J,
          template <typename> class K, template <typename> class L>
					class Arity12: public L<Arity11<Base, A, B, C, D, E, F, G, H, I, J, K> > {};


// chain with 2 template arguments on the derivers
template <
					class Base,
					class TA,
					template <typename,typename> class A >
					class MArity1: public A<  Base, TA  > {
					};

template <
					class Base,
					class TA,
					template <typename,typename> class A,
					class TB,
					template <typename,typename> class B>
					class MArity2: public B< MArity1<Base, TA, A>,TB > {};

template <
					class Base,
					class TA,
					template <typename,typename> class A,
					class TB,
					template <typename,typename> class B,
					class TC,
					template <typename,typename> class C >
					class MArity3: public C<MArity2<Base, TA,A,TB, B>,TC > {};

template <
					class Base,
					class TA,
					template <typename,typename> class A,
					class TB,
					template <typename,typename> class B,
					class TC,
					template <typename,typename> class C,
					class TD,
					template <typename,typename> class D>
					class MArity4: public D<MArity3<Base, TA,A,TB, B, TC,C>,TD > {};



class DumClass {};

}// end namespace vcg
#endif
