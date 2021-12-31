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

/** \file face/pos.h
 * Definition of vcg:face::Pos class.
 * This file contain the definition of vcg::face::Pos class and the derived vcg::face::PosN class.
 */

#ifndef __VCG_FACE_POS
#define __VCG_FACE_POS

#include <assert.h>

namespace vcg {
namespace face {

/** \addtogroup face */
/*@{*/

// Needed Prototypes (pos is include before topology)
template <class FaceType>
bool IsBorder(FaceType const & f,  const int j );
template <class FaceType>
bool IsManifold(FaceType const & f,  const int j );

/**  Templated over the class face, it stores a \em position over a face in a mesh.
	It contain a pointer to the current face,
	the index of one edge and a pointer to one of the vertices of the edge.
	See also the JumpingPos in jumping_pos.h for an iterator that loops
	around the faces of a vertex without requiring the VF topology.
 */


template <class FaceType>
class Pos
{
public:

	/// The vertex type
	typedef typename FaceType::VertexType VertexType;
	///The Pos type
	typedef Pos<FaceType> PosType;
	/// The scalar type
	typedef typename VertexType::ScalarType ScalarType;

	/// Pointer to the face of the half-edge
	typename FaceType::FaceType *f;
	/// Index of the edge
	int z;
	/// Pointer to the vertex
	VertexType *v;

	/// Default constructor
	Pos(){}
	/// Constructor which associates the half-edge element with a face, its edge and its vertex
	/// \note that the input must be consistent, e.g. it should hold that \c vp==fp->V0(zp) or \c vp==fp->V1(zp)
	Pos(FaceType * const fp, int const zp, VertexType * const vp)
	{
	  f=fp; z=zp; v=vp;
	  assert((vp==fp->V0(zp))||(vp==fp->V1(zp)));
	}
	Pos(FaceType * const fp, int const zp){f=fp; z=zp; v=f->V(zp);}
	Pos(FaceType * const fp, VertexType * const vp)
	{
		f = fp;
		v = vp;
		for(int i = 0; i < f->VN(); ++i)
			if (f->V(i) == v) { z = f->Prev(i); break;}
	}

    // Official Access functions functions
   VertexType *& V(){ return v; }
     int         & E(){ return z; }
   FaceType   *& F(){ return f; }

   VertexType * V() const { return v; }
   int          E() const { return z; }
   FaceType   * F() const { return f; }

// Returns the face index of the vertex inside the face.
// Note that this is DIFFERENT from using the z member that denotes the edge index inside the face.
// It should holds that Vind != (z+1)%3   &&   Vind == z || Vind = z+2%3
   int VInd()
     {
         for(int i = 0; i < f->VN(); ++i) if(v==f->V(i)) return i;
         assert(0);
         return -1;
     }


	/// Operator to compare two half-edge
	inline bool operator == ( PosType const & p ) const {
			return (f==p.f && z==p.z && v==p.v);
	}

	/// Operator to compare two half-edge
	inline bool operator != ( PosType const & p ) const {
			return (f!=p.f || z!=p.z || v!=p.v);
	}
	/// Operator to order half-edge; it's compare at the first the face pointers, then the index of the edge and finally the vertex pointers
	inline bool operator <= ( PosType const & p) const {
	return	(f!=p.f)?(f<p.f):
						(z!=p.z)?(z<p.z):
						(v<=p.v);
	}

	/// Assignment operator
	inline PosType & operator = ( const PosType & h ){
		f=h.f;
		z=h.z;
		v=h.v;
		return *this;
		}

	/// Set to null the half-edge
	void SetNull(){
		f=0;
		v=0;
		z=-1;
	}
	/// Check if the half-edge is null
	bool IsNull() const {
		return f==0 || v==0 || z<0;
	}

	//Cambia Faccia lungo z
	// e' uguale a FlipF solo che funziona anche per non manifold.
	/// Change face via z
	void NextF()
	{
		FaceType * t = f;
		f = t->FFp(z);
		z = t->FFi(z);
	}

		// Paolo Cignoni 19/6/99
		// Si muove sulla faccia adiacente a f, lungo uno spigolo che
		// NON e' j, e che e' adiacente a v
		// in questo modo si scandiscono tutte le facce incidenti in un
		// vertice f facendo Next() finche' non si ritorna all'inizio
		// Nota che sul bordo rimbalza, cioe' se lo spigolo !=j e' di bordo
		// restituisce sempre la faccia f ma con nj che e' il nuovo spigolo di bordo
		// vecchi parametri:     	FaceType * & f, VertexType * v, int & j

	/// It moves on the adjacent face incident to v, via a different edge that j
	void NextE()
	{
		assert( f->V(z)==v || f->V(f->Next(z))==v ); // L'edge j deve contenere v
		FlipE();
		FlipF();
		assert( f->V(z)==v || f->V(f->Next(z))==v );
	}
	// Cambia edge mantenendo la stessa faccia e lo stesso vertice
	/// Changes edge maintaining the same face and the same vertex
	void FlipE()
	{
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V((z+0)%f->VN())==v));
		if(f->V(f->Next(z))==v) z=f->Next(z);
		else z= f->Prev(z);
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V((z))==v));
	}

	// Cambia Faccia mantenendo lo stesso vertice e lo stesso edge
	// Vale che he.flipf.flipf= he
	// Se l'he e' di bordo he.flipf()==he
	// Si puo' usare SOLO se l'edge e' 2manifold altrimenti
	// si deve usare nextf

	/// Changes face maintaining the same vertex and the same edge
	void FlipF()
	{
		assert( f->FFp(z)->FFp(f->FFi(z))==f );  // two manifoldness check
		// Check that pos vertex is one of the current z-th edge and it is different from the vert opposite to the edge.
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V((z))==v));
		FaceType *nf=f->FFp(z);
		int nz=f->FFi(z);
		assert(nf->V(nf->Prev(nz))!=v && (nf->V(nf->Next(nz))==v || nf->V((nz))==v));
		f=nf;
		z=nz;
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V(z)==v));
	}

	/// Changes vertex maintaining the same face and the same edge
	void FlipV()
	{
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V(z)==v));

		if(f->V(f->Next(z))==v)
			v=f->V(z);
		else
			v=f->V(f->Next(z));

		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V(z)==v));
	}

  /// return the vertex that it should have if we make FlipV;
    VertexType *VFlip() const
    {
        assert(f->cV(f->Prev(z))!=v && (f->cV(f->Next(z))==v || f->cV(z)==v));
        if(f->cV(f->Next(z))==v)	return f->cV(z);
                                else			return f->cV(f->Next(z));
    }

  /// return the face that it should have if we make FlipF;
    FaceType *FFlip() const
    {
//        assert( f->FFp(z)->FFp(f->FFi(z))==f );
//        assert(f->V(f->Prev(z))!=v);
//        assert(f->V(f->Next(z))==v || f->V((z+0)%f->VN())==v);
        FaceType *nf=f->FFp(z);
        return nf;
  }


	// Trova il prossimo half-edge di bordo (nhe)
	// tale che
	// --nhe.f adiacente per vertice a he.f
	// --nhe.v adiacente per edge di bordo a he.v
	// l'idea e' che se he e' un half edge di bordo
	// si puo scorrere tutto un bordo facendo
	//
	//		hei=he;
	//		do
	//			hei.Nextb()
	//		while(hei!=he);

	/// Finds the next half-edge border
	void NextB( )
	{
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V(z)==v));
		assert(f->FFp(z)==f); // f is border along j
	// Si deve cambiare faccia intorno allo stesso vertice v
	//finche' non si trova una faccia di bordo.
		do
			NextE();
	  while(!IsBorder());

		// L'edge j e' di bordo e deve contenere v
		assert(IsBorder() &&( f->V(z)==v || f->V(f->Next(z))==v ));

		FlipV();
		assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V(z)==v));
		assert(f->FFp(z)==f); // f is border along j
	}

	/// Checks if the half-edge is of border
	bool IsBorder()
	{
	return face::IsBorder(*f,z);
	}

  bool IsManifold()
    {
    return face::IsManifold(*f,z);
    }

	/*!
	 * Returns the number of vertices incident on the vertex pos is currently pointing to.
	 */
	int NumberOfIncidentVertices()
	{
		int  count		 = 0;
		bool on_border = false;
		CheckIncidentFaces(count, on_border);
		if(on_border) return (count/2)+1;
		else					return count;
	}

	/*!
	* Returns the number of faces incident on the vertex pos is currently pointing to.
	*/
	int NumberOfIncidentFaces()
	{
		int  count		 = 0;
		bool on_border = false;
		CheckIncidentFaces(count, on_border);
		if(on_border) return count/2;
		else					return count;
	}



  /*!
  * Returns the number of faces incident on the edge the pos is currently pointing to.
  * useful to compute the complexity of a non manifold edge
  */
  int NumberOfFacesOnEdge() const
  {
    int  count		 = 0;
    PosType ht = *this;
    do
    {
      ht.NextF();
      ++count;
    }
    while (ht!=*this);
    return count;
  }
    /** Function to inizialize an half-edge.
        @param fp Puntatore alla faccia
        @param zp Indice dell'edge
        @param vp Puntatore al vertice
    */
    void Set(FaceType  * const fp, int const zp,  VertexType  * const vp)
    {
        f=fp;z=zp;v=vp;
        assert(f->V(f->Prev(z))!=v && (f->V(f->Next(z))==v || f->V(z)==v));
    }

	void Set(FaceType  * const pFace, VertexType  * const pVertex)
	{
		f = pFace;
		v = pVertex;
		for(int i  = 0; i < f->VN(); ++i) if(f->V(i) == v ) {z = f->Prev(i);break;}
	}

	void Assert()
	#ifdef _DEBUG
	{
		FaceType ht=*this;
		ht.FlipF();
		ht.FlipF();
		assert(ht==*this);

		ht.FlipE();
		ht.FlipE();
		assert(ht==*this);

		ht.FlipV();
		ht.FlipV();
		assert(ht==*this);
	}
	#else
	{}
	#endif


	protected:
		void CheckIncidentFaces(int & count, bool & on_border)
		{
			PosType ht = *this;
			do
			{
				++count;
				ht.NextE();
				if(ht.IsBorder()) on_border=true;
			} while (ht != *this);
		}
};

/** Class VFIterator.
    This class is used as an iterator over the VF adjacency.
  It allow to easily traverse all the faces around a given vertex v;
  The faces are traversed in no particular order. No Manifoldness requirement.

  typical example:

    VertexPointer v;
    vcg::face::VFIterator<FaceType> vfi(v);
    for (;!vfi.End();++vfi)
            vfi.F()->ClearV();

        // Alternative

	vcg::face::VFIterator<FaceType> vfi(f, 1);
		while (!vfi.End()){
			vfi.F()->ClearV();
			++vfi;
		}


	See also the JumpingPos in jumping_pos.h for an iterator that loops
	around the faces of a vertex using FF topology and without requiring the VF topology.

 */

template <typename FaceType>
class VFIterator
{
public:

	/// The vertex type
	typedef typename FaceType::VertexType VertexType;
	/// The Base face type
	typedef  FaceType  VFIFaceType;
	/// The vector type
	typedef typename VertexType::CoordType CoordType;
	/// The scalar type
	typedef typename VertexType::ScalarType ScalarType;

	/// Pointer to the face of the half-edge
	FaceType *f;
	/// Index of the vertex
	int z;

	/// Default constructor
	VFIterator(){}
	/// Constructor which associates the half-edge elementet with a face and its vertex
	VFIterator(FaceType * _f,  const int &  _z){f = _f; z = _z;  assert(z>=0 && "VFAdj must be initialized");}

	/// Constructor which takes a pointer to vertex
	VFIterator(VertexType * _v){f = _v->VFp(); z = _v->VFi(); assert(z>=0 && "VFAdj must be initialized");}

	VFIFaceType *&	F() { return f;}
	int	&					  I() { return z;}

  // Access to the vertex. Having a VFIterator vfi, it corresponds to
  // vfi.V() = vfi.F()->V(vfi.I())
  inline VertexType *V() const { return f->V(z);}

  inline VertexType * const & V0() const { return f->V0(z);}
  inline VertexType * const & V1() const { return f->V1(z);}
  inline VertexType * const & V2() const { return f->V2(z);}

  bool End() const {return f==0;}
  void operator++() {
    FaceType* t = f;
        f = t->VFp(z);
        z = t->VFi(z);
  }

};

/*@}*/
}	 // end namespace
}	 // end namespace
#endif
