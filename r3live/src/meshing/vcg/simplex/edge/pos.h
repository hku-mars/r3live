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
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.2  2004/05/10 14:40:47  ganovelli
name of adhacency function updated

Revision 1.1  2004/05/10 14:01:56  ganovelli
created

****************************************************************************/


#ifndef __VCG_EDGE_POS
#define __VCG_EDGE_POS

namespace vcg {
namespace edge {

// Needed Prototypes (pos is include before topology)
template <class EDGETYPE>
bool IsEdgeBorder(EDGETYPE const & e,  const int j );
template <class EDGETYPE>
bool IsEdgeManifold(EDGETYPE const & e,  const int j );

/*
 Vertex_Edge: run over the fan of a vertex (no order is specified)
*/
/** Class VertexStar
	@param EDGETYPE Specifies the type of the faces
 */
template <class EDGETYPE> 
class VertexStar
{
public:
	/// Pointer to an edge
	EDGETYPE *e;
	/// Local index of the vertex
	int z;
	/// Default Constructor
	VertexStar() {}
	/// Constructor which associates the EdgePos elementet with a face and its edge
	VertexStar(EDGETYPE  * const ep, int const zp)
	{
		e=ep;
		z=zp;
	}

	/// Function to jump on the next face of the list of vertex z
	void NextF()
	{
		EDGETYPE * t = e;
		e = (EDGETYPE *)t->VEp(z);
		z = t->VEi(z);
	}
};


/*
 
*/
/** Class Pos.
	This structure is equivalent to a half-edge. 
	@param MFTYPE (Template-Parameter) Specifies the type of the edges
 */
template <class EDGETYPE> 
class Pos
{
public:

	/// The vertex type
	typedef	typename EDGETYPE::VertexType VertexType;
	/////The HEdgePos type
	typedef Pos< EDGETYPE> POSTYPE;
	///// The vector type
	//typedef typename MVTYPE::coord_type vectorial_type;
	///// The scalar type
	//typedef typename MVTYPE::scalar_type scalar_type;

	/// Pointer to the face of the half-edge
	EDGETYPE *e;
	/// Pointer to the vertex
	VertexType *v;

	/// Default constructor
	Pos(){}
	/// Constructor which associates the half-edge elementet with a face, its edge and its vertex
  Pos(EDGETYPE  * ep, int zp) {e=ep;v=ep->V(zp);}
  Pos(EDGETYPE  * ep, VertexType  *vp){e=ep;v=vp;}


  // Official Access functions functions
   VertexType *& V(){ return v; }
   EDGETYPE  *& E(){ return e; }
   int VInd(){
     return (e->V(0)==v)?0:1;
     }

	/// Operator to compare two half-edge
	inline bool operator == ( POSTYPE const & p ) const {
			return (e==p.e &&v==p.v);
	} 

	/// Operator to compare two half-edge
	inline bool operator != ( POSTYPE const & p ) const {
			return (e!=p.e || v!=p.v);
	} 
	/// Operator to order half-edge; it's compare at the first the face pointers, then the index of the edge and finally the vertex pointers
	inline bool operator <= ( POSTYPE const & p) const {
    return	(e!=p.e)?(e<p.e):
						(v<=p.v);
	}	

	/// Assignment operator
	inline POSTYPE & operator = ( const POSTYPE & h ){
		e=h.e;
		v=h.v;
		return *this;
		}
	/// Set to null the half-edge
	void SetNull(){
		e=0;
		v=0;
	}
	/// Check if the half-edge is null
	bool IsNull() const {
		return e==0 || v==0 ;
	}


	/*! \brief It advances the current Pos along the edge chain.
	 *
	 * Note that a Pos implicitly encode an ordering in the chain:
	 * the one denoted by the classical arrow shaped icon of a pos.
	 * In other words
	 *
	 *    o---------o
	 *         |  /
	 *         |/
	 *
	 * Meaningful only for 1-manifold edge chain.
	 */

	void NextE()
	{
       FlipE();
       FlipV();
	}
  
		// Paolo Cignoni 19/6/99
		// Si muove sulla faccia adiacente a f, lungo uno spigolo che
		// NON e' j, e che e' adiacente a v 
		// in questo modo si scandiscono tutte le facce incidenti in un 
		// vertice f facendo Next() finche' non si ritorna all'inizio
		// Nota che sul bordo rimbalza, cioe' se lo spigolo !=j e' di bordo
		// restituisce sempre la faccia f ma con nj che e' il nuovo spigolo di bordo 
		// vecchi parametri:     	MFTYPE * & f, MVTYPE * v, int & j

	// Cambia edge mantenendo la stessa faccia e lo stesso vertice
	/// Changes edge maintaining the same face and the same vertex
	void FlipV()
	{
		v = (e->V(0)==v)?e->V(1):e->V(0);
	}
	void FlipE()
	{
		assert( (e->V(0)==v) ||(e->V(1)==v));
		e = (e->V(0)==v)?e->EEp(0):e->EEp(1);
	}
  // return the vertex that it should have if we make FlipV;
	VertexType *VFlip()
	{
		return (e->V(0)==v)?e->V(1):e->V(0);
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
	

  /// Checks if the half-edge is of border
  bool IsBorder()
  {
    return edge::IsEdgeBorder(*e,VInd());
  }

  bool IsManifold()
  {
    return edge::IsEdgeManifold(*e,VInd());
  }


	/** Function to inizialize an half-edge.
		@param fp Puntatore alla faccia
		@param zp Indice dell'edge
		@param vp Puntatore al vertice
	*/
  void Set(EDGETYPE  * const ep, VertexType  * const vp)
	{
		e=ep;v=vp;
	}

};



/** Class VEIterator.
    This class is used as an iterator over the VE adjacency.
  It allow to easily traverse all the edges around a given vertex v;
  The edges are traversed in no particular order. No Manifoldness requirement.

  typical example:

    VertexPointer v;
    vcg::edge::VEIterator<EdgeType> vei(v);
    for (;!vei.End();++vei)
            vei.E()->ClearV();

        // Alternative

	vcg::edge::VEIterator<EdgeType> vei(f, 1);
		while (!vei.End()){
			vei.E()->ClearV();
			++vei;
		}


	See also the JumpingPos in jumping_pos.h for an iterator that loops
	around the faces of a vertex using FF topology and without requiring the VF topology.

 */

template <typename EdgeType>
class VEIterator
{
public:

	/// The vertex type
	typedef typename EdgeType::VertexType VertexType;
	/// The Base face type
	typedef  EdgeType  VFIEdgeType;
	/// The vector type
	typedef typename VertexType::CoordType CoordType;
	/// The scalar type
	typedef typename VertexType::ScalarType ScalarType;

	/// Pointer to the face of the half-edge
	EdgeType *e;
	/// Index of the vertex
	int z;

	/// Default constructor
	VEIterator(){}
	/// Constructor which associates the half-edge elementet with a face and its vertex
	VEIterator(EdgeType * _e,  const int &  _z){e = _e; z = _z;}

	/// Constructor which takes a pointer to vertex
	VEIterator(VertexType * _v){
	  e = _v->VEp(); z = _v->VEi();
	  assert(z>=0 && "VE adjacency not initialized");
	}

	VFIEdgeType * &E() { return e;}
	int	&					  I() { return z;}

  // Access to the vertex. Having a VEIterator vfi, it corresponds to
  // vfi.V() = vfi.I()->V(vfi.I())
  inline VertexType *V() const { return e->V(z);}

  inline VertexType * const & V0() const { return e->V0(z);}
  inline VertexType * const & V1() const { return e->V1(z);}

  bool End() const {return e==0;}
  VFIEdgeType *operator++() {
    EdgeType* t = e;
        e = e->VEp(z);
        z = t->VEi(z);
    return e;
  }

};


	}	 // end namespace
}	 // end namespace
#endif
