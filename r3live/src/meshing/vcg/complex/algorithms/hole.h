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
#ifndef __VCG_TRI_UPDATE_HOLE
#define __VCG_TRI_UPDATE_HOLE

#include <vcg/complex/algorithms/clean.h>

// This file contains three Ear Classes
// - TrivialEar
// - MinimumWeightEar
// - SelfIntersectionEar
// and a static class Hole for filling holes that is templated on the ear class

namespace vcg
{
namespace tri
{

/*
An ear is identified by TWO pos.
The Three vertexes of an Ear are:
e0.VFlip().v
e0.v
e1.v
Invariants:
  e1 == e0.NextB();
  e1.FlipV() == e0;

Situazioni ear non manifold, e degeneri (buco triangolare)

T  XXXXXXXXXXXXX    A        /XXXXX        B      en/XXXXX
/XXXXXXXXXXXXXXX            /XXXXXX                /XXXXXX
XXXXXXep==en XXX     ep\   /en XXXX               /e1 XXXX
XXXXXX ----/| XX   ------ ----/| XX       ------ ----/|XXX
XXXXXX|   /e1 XX   XXXXXX|   /e1 XX       XXXXXX|  o/e0 XX
XXXXXX|  /XXXXXX   XXXXXX|  /XXXXXX       XXXXXX|  /XXXXXX
XXX e0|o/XXXXXXX   XXX e0|o/XXXXXXX       XXX ep| /XXXXXXX
XXX  \|/XXXXXXXX   XXX  \|/XXXXXXXX       XXX  \|/XXXXXXXX
XXXXXXXXXXXXXXXX   XXXXXXXXXXXXXXXX       XXXXXXXXXXXXXXXX
*/
template < class MESH >
class TrivialEar
{
  public:
    typedef typename MESH::FaceType        FaceType;
    typedef typename MESH::FacePointer     FacePointer;
    typedef typename MESH::VertexPointer   VertexPointer;
    typedef typename face::Pos< FaceType > PosType;
    typedef typename MESH::ScalarType      ScalarType;
    typedef typename MESH::CoordType       CoordType;

    PosType     e0;
    PosType     e1;
    CoordType   n; // the normal of the face defined by the ear
    const char *Dump() { return 0; }
    // The following members are useful to consider the Ear as a generic <triangle>
    // with p0 the 'center' of the ear.
    const CoordType &cP( int i ) const { return P( i ); }
    const CoordType &P( int i ) const
    {
        switch ( i )
        {
        case 0:
            return e0.v->P();
        case 1:
            return e1.v->P();
        case 2:
            return e0.VFlip()->P();
        default:
            assert( 0 );
        }
        return e0.v->P();
    }

    ScalarType quality;
    ScalarType angleRad;
    TrivialEar() {}
    TrivialEar( const PosType &ep )
    {
        e0 = ep;
        assert( e0.IsBorder() );
        e1 = e0;
        e1.NextB();
        n = vcg::Normal< TrivialEar >( *this );
        ComputeQuality();
        ComputeAngle();
    }

    /// Compute the angle of the two edges of the ear.
    // it tries to make the computation in a precision safe way.
    // the angle computation takes into account the case of reversed ears
    void ComputeAngle()
    {
        angleRad = Angle( cP( 2 ) - cP( 0 ), cP( 1 ) - cP( 0 ) );
        ScalarType flipAngle = n.dot( e0.v->N() );
        if ( flipAngle < 0 )
            angleRad = ( 2.0 * ( ScalarType ) M_PI ) - angleRad;
    }

    virtual inline bool operator<( const TrivialEar &c ) const { return quality < c.quality; }

    bool IsNull() { return e0.IsNull() || e1.IsNull(); }
    void SetNull()
    {
        e0.SetNull();
        e1.SetNull();
    }
    virtual void ComputeQuality() { quality = QualityFace( *this ); }
    bool         IsUpToDate() { return ( e0.IsBorder() && e1.IsBorder() ); }
    // An ear is degenerated if both of its two endpoints are non manifold.
    bool IsDegen( const int nonManifoldBit )
    {
        if ( e0.VFlip()->IsUserBit( nonManifoldBit ) && e1.V()->IsUserBit( nonManifoldBit ) )
            return true;
        else
            return false;
    }
    bool IsConcave() const { return ( angleRad > ( float ) M_PI ); }

    // When you close an ear you have to check that the newly added triangle does not create non manifold situations
    // This can happen if the new edge already exists in the mesh.
    // We test that looping around one extreme of the ear we do not find the other vertex
    bool CheckManifoldAfterEarClose()
    {
        PosType       pp = e1;
        VertexPointer otherV = e0.VFlip();
        assert( pp.IsBorder() );
        do
        {
            pp.FlipE();
            pp.FlipF();
            if ( pp.VFlip() == otherV )
                return false;
        } while ( !pp.IsBorder() );
        return true;
    }

    virtual bool Close( PosType &np0, PosType &np1, FaceType *f )
    {
        // simple topological check
        if ( e0.f == e1.f )
        {
            // printf("Avoided bad ear");
            return false;
        }

        // usato per generare una delle due nuove orecchie.
        PosType ep = e0;
        ep.FlipV();
        ep.NextB();
        ep.FlipV(); // he precedente a e0
        PosType en = e1;
        en.NextB(); // he successivo a e1
        if ( ep != en )
            if ( !CheckManifoldAfterEarClose() )
                return false;

        ( *f ).V( 0 ) = e0.VFlip();
        ( *f ).V( 1 ) = e0.v;
        ( *f ).V( 2 ) = e1.v;
        face::ComputeNormalizedNormal( *f );

        face::FFAttachManifold( f, 0, e0.f, e0.z );
        face::FFAttachManifold( f, 1, e1.f, e1.z );
        face::FFSetBorder( f, 2 );

        // caso ear degenere per buco triangolare
        if ( ep == en )
        {
            // printf("Closing the last triangle");
            face::FFAttachManifold( f, 2, en.f, en.z );
            np0.SetNull();
            np1.SetNull();
        }
        // Caso ear non manifold a
        else if ( ep.v == en.v )
        {
            // printf("Ear Non manif A\n");
            PosType enold = en;
            en.NextB();
            face::FFAttachManifold( f, 2, enold.f, enold.z );
            np0 = ep;
            np1 = en;
        }
        // Caso ear non manifold b
        else if ( ep.VFlip() == e1.v )
        {
            // printf("Ear Non manif B\n");
            PosType epold = ep;
            ep.FlipV();
            ep.NextB();
            ep.FlipV();
            face::FFAttachManifold( f, 2, epold.f, epold.z );
            np0 = ep; // assign the two new
            np1 = en; // pos that denote the ears
        }
        else // caso standard // Now compute the new ears;
        {
            np0 = ep;
            np1 = PosType( f, 2, e1.v );
        }

        return true;
    }

}; // end TrivialEar Class

// Ear with FillHoleMinimumWeight's quality policy
template < class MESH >
class MinimumWeightEar : public TrivialEar< MESH >
{
  public:
    static float &DiedralWeight()
    {
        static float _dw = 0.1;
        return _dw;
    }
    typedef TrivialEar< MESH > TE;
    typename MESH::ScalarType  dihedralRad;
    typename MESH::ScalarType  aspectRatio;
    const char *               Dump()
    {
        static char buf[ 200 ];
        if ( this->IsConcave() )
            sprintf( buf, "Dihedral -(deg) %6.2f Quality %6.2f\n", math::ToDeg( dihedralRad ), aspectRatio );
        else
            sprintf( buf, "Dihedral  (deg) %6.2f Quality %6.2f\n", math::ToDeg( dihedralRad ), aspectRatio );
        return buf;
    }

    MinimumWeightEar() {}
    MinimumWeightEar( const typename face::Pos< typename MESH::FaceType > &ep ) : TrivialEar< MESH >( ep ) { ComputeQuality(); }

    // In the heap, by default, we retrieve the LARGEST value,
    // so if we need the ear with minimal dihedral angle, we must reverse the sign of the comparison.
    // The concave elements must be all in the end of the heap, sorted accordingly,
    // So if only one of the two ear is Concave that one is always the minimum one.
    // the pow function is here just to give a way to play with different weighting schemas, balancing in a different way

    virtual inline bool operator<( const MinimumWeightEar &c ) const
    {
        if ( TE::IsConcave() && !c.IsConcave() )
            return true;
        if ( !TE::IsConcave() && c.IsConcave() )
            return false;

        return aspectRatio - ( dihedralRad / M_PI ) * DiedralWeight() < c.aspectRatio - ( c.dihedralRad / M_PI ) * DiedralWeight();

        //      return (pow((float)dihedralRad,(float)DiedralWeight())/aspectRatio) > (pow((float)c.dihedralRad,(float)DiedralWeight())/c.aspectRatio);
    }

    // the real core of the whole hole filling strategy.
    virtual void ComputeQuality()
    {
        // compute quality by (dihedral ancgle, area/sum(edge^2) )
        typename MESH::CoordType n1 = TE::e0.FFlip()->cN();
        typename MESH::CoordType n2 = TE::e1.FFlip()->cN();

        dihedralRad = std::max( Angle( TE::n, n1 ), Angle( TE::n, n2 ) );
        aspectRatio = QualityFace( *this );
    }

}; // end class MinimumWeightEar

// Ear for selfintersection algorithm
template < class MESH >
class SelfIntersectionEar : public MinimumWeightEar< MESH >
{
  public:
    typedef typename MESH::FaceType        FaceType;
    typedef typename MESH::FacePointer     FacePointer;
    typedef typename face::Pos< FaceType > PosType;
    typedef typename MESH::ScalarType      ScalarType;
    typedef typename MESH::CoordType       CoordType;

    static std::vector< FacePointer > &AdjacencyRing()
    {
        static std::vector< FacePointer > ar;
        return ar;
    }

    SelfIntersectionEar() {}
    SelfIntersectionEar( const PosType &ep ) : MinimumWeightEar< MESH >( ep ) {}

    virtual bool Close( PosType &np0, PosType &np1, FacePointer f )
    {
        PosType ep = this->e0;
        ep.FlipV();
        ep.NextB();
        ep.FlipV(); // he precedente a e0
        PosType en = this->e1;
        en.NextB(); // he successivo a e1
                    //    bool triangularHole = false;
                    //    if(en==ep || en-) triangularHole=true;

        // costruisco la faccia e poi testo, o copio o butto via.
        ( *f ).V( 0 ) = this->e0.VFlip();
        ( *f ).V( 1 ) = this->e0.v;
        ( *f ).V( 2 ) = this->e1.v;
        face::FFSetBorder( f, 0 );
        face::FFSetBorder( f, 1 );
        face::FFSetBorder( f, 2 );

        typename std::vector< FacePointer >::iterator it;
        for ( it = this->AdjacencyRing().begin(); it != this->AdjacencyRing().end(); ++it )
        {
            if ( !( *it )->IsD() )
            {
                if ( tri::Clean< MESH >::TestFaceFaceIntersection( f, *it ) )
                    return false;
                // We must also check that the newly created face does not have any edge in common with other existing surrounding faces
                // Only the two faces of the ear can share an edge with the new face
                if ( face::CountSharedVertex( f, *it ) == 2 )
                {
                    int  e0, e1;
                    bool ret = face::FindSharedEdge( f, *it, e0, e1 );
                    assert( ret );
                    if ( !face::IsBorder( **it, e1 ) )
                        return false;
                }
            }
        }
        bool ret = TrivialEar< MESH >::Close( np0, np1, f );
        if ( ret )
            AdjacencyRing().push_back( f );
        return ret;
    }
}; // end class SelfIntersectionEar

// Funzione principale per chiudier un buco in maniera topologicamente corretta.
// Gestisce situazioni non manifold ragionevoli
// (tutte eccetto quelle piu' di 2 facce per 1 edge).
// Controlla che non si generino nuove situazioni non manifold chiudendo orecchie
// che sottendono un edge che gia'esiste.

template < class MESH >
class Hole
{
  public:
    typedef typename MESH::VertexType        VertexType;
    typedef typename MESH::VertexPointer     VertexPointer;
    typedef typename MESH::ScalarType        ScalarType;
    typedef typename MESH::FaceType          FaceType;
    typedef typename MESH::FacePointer       FacePointer;
    typedef typename MESH::FaceIterator      FaceIterator;
    typedef typename MESH::CoordType         CoordType;
    typedef typename vcg::Box3< ScalarType > Box3Type;
    typedef typename face::Pos< FaceType >   PosType;

  public:
    class Info
    {
      public:
        Info() {}
        Info( PosType const &pHole, int const pHoleSize, Box3< ScalarType > &pHoleBB )
        {
            p = pHole;
            size = pHoleSize;
            bb = pHoleBB;
        }

        PosType  p;
        int      size;
        Box3Type bb;

        bool operator<( const Info &hh ) const { return size < hh.size; }

        ScalarType Perimeter()
        {
            ScalarType sum = 0;
            PosType    ip = p;
            do
            {
                sum += Distance( ip.v->cP(), ip.VFlip()->cP() );
                ip.NextB();
            } while ( ip != p );
            return sum;
        }

        // Support function to test the validity of a single hole loop
        // for now it test only that all the edges are border;
        // The real test should check if all non manifold vertices
        // are touched only by edges belonging to this hole loop.
        bool CheckValidity()
        {
            if ( !p.IsBorder() )
                return false;
            PosType ip = p;
            ip.NextB();
            for ( ; ip != p; ip.NextB() )
            {
                if ( !ip.IsBorder() )
                    return false;
            }
            return true;
        }
    };

    class EdgeToBeAvoided
    {
        VertexPointer v0, v1;
        EdgeToBeAvoided( VertexPointer _v0, VertexPointer _v1 ) : v0( _v0 ), v1( _v1 )
        {
            if ( v0 > v1 )
                swap( v0, v1 );
        }
        bool operator<( const EdgeToBeAvoided &e )
        {
            if ( this->v0 != e.v0 )
                return this->v0 < e.v0;
            return this->v1 < e.v1;
        }
    };
    /// Main Single Hole Filling Function
    /// Given a specific hole (identified by the Info h) it fills it
    /// It also update a vector of face pointers
    /// It uses an heap to choose the best ear to be closed

    template < class EAR >
    static void FillHoleEar( MESH &                        m, // The mesh to be filled
                             Info &                        h, // the particular hole to be filled
                             std::vector< FacePointer * > &facePointersToBeUpdated )
    {
        // Aggiungo le facce e aggiorno il puntatore alla faccia!
        FaceIterator f = tri::Allocator< MESH >::AddFaces( m, h.size - 2, facePointersToBeUpdated );

        assert( h.p.f >= &*m.face.begin() );
        assert( h.p.f <= &m.face.back() );
        assert( h.p.IsBorder() );

        std::vector< EAR > EarHeap;
        EarHeap.reserve( h.size );
        int nmBit = VertexType::NewBitFlag(); // non manifoldness bit

        // First loops around the hole to mark non manifold vertices.
        PosType ip = h.p; // Pos iterator
        do
        {
            ip.V()->ClearUserBit( nmBit );
            ip.V()->ClearV();
            ip.NextB();
        } while ( ip != h.p );

        ip = h.p; // Re init the pos iterator for another loop (useless if everithing is ok!!)
        do
        {
            if ( !ip.V()->IsV() )
                ip.V()->SetV(); // All the vertexes that are visited more than once are non manifold
            else
                ip.V()->SetUserBit( nmBit );
            ip.NextB();
        } while ( ip != h.p );

        PosType fp = h.p;
        do
        {
            EAR appEar = EAR( fp );
            EarHeap.push_back( appEar );
            // printf("Adding ear %s ",app.Dump());
            fp.NextB();
            assert( fp.IsBorder() );
        } while ( fp != h.p );

        int cnt = h.size;

        make_heap( EarHeap.begin(), EarHeap.end() );

        // finche' il buco non e' chiuso o non ci sono piu' orecchie da analizzare.
        while ( cnt > 2 && !EarHeap.empty() )
        {
            // printf("Front of the heap is %s", H.front().Dump());
            pop_heap( EarHeap.begin(), EarHeap.end() ); // retrieve the MAXIMUM value and put in the back;
            EAR BestEar = EarHeap.back();
            EarHeap.pop_back();

            if ( BestEar.IsUpToDate() && !BestEar.IsDegen( nmBit ) )
            {
                if ( ( *f ).HasPolyInfo() )
                    ( *f ).Alloc( 3 );
                PosType ep0, ep1;
                if ( BestEar.Close( ep0, ep1, &*f ) )
                {
                    if ( !ep0.IsNull() )
                    {
                        EarHeap.push_back( EAR( ep0 ) );
                        push_heap( EarHeap.begin(), EarHeap.end() );
                    }
                    if ( !ep1.IsNull() )
                    {
                        EarHeap.push_back( EAR( ep1 ) );
                        push_heap( EarHeap.begin(), EarHeap.end() );
                    }
                    --cnt;
                    ++f;
                }
            } // is update()
        }     // fine del while principale.

        while ( f != m.face.end() )
        {
            tri::Allocator< MESH >::DeleteFace( m, *f );
            f++;
        }

        VertexType::DeleteBitFlag( nmBit ); // non manifoldness bit
    }

    template < class EAR >
    static int EarCuttingFill( MESH &m, int sizeHole, bool Selected = false, CallBackPos *cb = 0 )
    {
        std::vector< Info > vinfo;
        GetInfo( m, Selected, vinfo );

        typename std::vector< Info >::iterator ith;
        int                                    indCb = 0;
        int                                    holeCnt = 0;
        std::vector< FacePointer * >           facePtrToBeUpdated;
        for ( ith = vinfo.begin(); ith != vinfo.end(); ++ith )
            facePtrToBeUpdated.push_back( &( *ith ).p.f );

        for ( ith = vinfo.begin(); ith != vinfo.end(); ++ith )
        {
            indCb++;
            if ( cb )
                ( *cb )( indCb * 10 / vinfo.size(), "Closing Holes" );
            if ( ( *ith ).size < sizeHole )
            {
                holeCnt++;
                FillHoleEar< EAR >( m, *ith, facePtrToBeUpdated );
            }
        }
        return holeCnt;
    }

    /// Main Hole Filling function.
    /// Given a mesh search for all the holes smaller than a given size and fill them
    /// It returns the number of filled holes.

    template < class EAR >
    static int EarCuttingIntersectionFill( MESH &m, const int maxSizeHole, bool Selected, CallBackPos *cb = 0 )
    {
        std::vector< Info > vinfo;
        GetInfo( m, Selected, vinfo );
        typename std::vector< Info >::iterator ith;

        // collect the face pointer that has to be updated by the various addfaces
        std::vector< FacePointer * > vfpOrig;
        for ( ith = vinfo.begin(); ith != vinfo.end(); ++ith )
            vfpOrig.push_back( &( *ith ).p.f );

        int indCb = 0;
        int holeCnt = 0;
        for ( ith = vinfo.begin(); ith != vinfo.end(); ++ith )
        {
            indCb++;
            if ( cb )
                ( *cb )( indCb * 10 / vinfo.size(), "Closing Holes" );
            if ( ( *ith ).size < maxSizeHole )
            {
                std::vector< FacePointer * > facePtrToBeUpdated;
                holeCnt++;
                facePtrToBeUpdated = vfpOrig;
                EAR::AdjacencyRing().clear();
                // Loops around the hole to collect the faces that have to be tested for intersection.
                PosType ip = ( *ith ).p;
                do
                {
                    PosType inp = ip;
                    do
                    {
                        inp.FlipE();
                        inp.FlipF();
                        EAR::AdjacencyRing().push_back( inp.f );
                    } while ( !inp.IsBorder() );
                    ip.NextB();
                } while ( ip != ( *ith ).p );

                typename std::vector< FacePointer >::iterator fpi;
                for ( fpi = EAR::AdjacencyRing().begin(); fpi != EAR::AdjacencyRing().end(); ++fpi )
                    facePtrToBeUpdated.push_back( &*fpi );

                FillHoleEar< EAR >( m, *ith, facePtrToBeUpdated );
                EAR::AdjacencyRing().clear();
            }
        }
        return holeCnt;
    }

    static void GetInfo( MESH &m, bool Selected, std::vector< Info > &VHI )
    {
        tri::UpdateFlags< MESH >::FaceClearV( m );
        for ( FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi )
        {
            if ( !( *fi ).IsD() )
            {
                if ( Selected && !( *fi ).IsS() )
                {
                    // se devo considerare solo i triangoli selezionati e
                    // quello che sto considerando non lo e' lo marchio e vado avanti
                    ( *fi ).SetV();
                }
                else
                {
                    for ( int j = 0; j < 3; ++j )
                    {
                        if ( face::IsBorder( *fi, j ) && !( *fi ).IsV() )
                        { // Trovato una faccia di bordo non ancora visitata.
                            ( *fi ).SetV();
                            PosType sp( &*fi, j, ( *fi ).V( j ) );
                            PosType fp = sp;
                            int     holesize = 0;

                            Box3Type hbox;
                            hbox.Add( sp.v->cP() );
                            // printf("Looping %i : (face %i edge %i) \n", VHI.size(),sp.f-&*m.face.begin(),sp.z);
                            sp.f->SetV();
                            do
                            {
                                sp.f->SetV();
                                hbox.Add( sp.v->cP() );
                                ++holesize;
                                sp.NextB();
                                sp.f->SetV();
                                assert( sp.IsBorder() );
                            } while ( sp != fp );

                            // ho recuperato l'inofrmazione su tutto il buco
                            VHI.push_back( Info( sp, holesize, hbox ) );
                        }
                    } // for sugli edge del triangolo
                }     // S & !S
            }         //! IsD()
        }             // for principale!!!
    }

    // Minimum Weight Algorithm
    class Weight
    {
      public:
        Weight()
        {
            ang = 180;
            ar = FLT_MAX;
        }
        Weight( float An, float Ar )
        {
            ang = An;
            ar = Ar;
        }
        ~Weight() {}

        float angle() const { return ang; }
        float area() const { return ar; }

        Weight operator+( const Weight &other ) const { return Weight( std::max( angle(), other.angle() ), area() + other.area() ); }
        bool operator<( const Weight &rhs ) const { return ( angle() < rhs.angle() || ( angle() == rhs.angle() && area() < rhs.area() ) ); }

      private:
        float ang;
        float ar;
    };

    /*
\ /        \/
v1*---------*v4
/ \       /
/   \     /
/ 	  \   /
/ear	   \ /
*---------*-
| v3      v2\
*/

    static float ComputeDihedralAngle( CoordType p1, CoordType p2, CoordType p3, CoordType p4 )
    {
        CoordType n1 = NormalizedNormal( p1, p3, p2 );
        CoordType n2 = NormalizedNormal( p1, p2, p4 );
        return math::ToDeg( AngleN( n1, n2 ) );
    }

    static bool existEdge( PosType pi, PosType pf )
    {
        PosType app = pi;
        PosType appF = pi;
        PosType tmp;
        assert( pi.IsBorder() );
        appF.NextB();
        appF.FlipV();
        do
        {
            tmp = app;
            tmp.FlipV();
            if ( tmp.v == pf.v )
                return true;
            app.FlipE();
            app.FlipF();

            if ( app == pi )
                return false;
        } while ( app != appF );
        return false;
    }

    static Weight computeWeight( int i, int j, int k, std::vector< PosType > pv, std::vector< std::vector< int > > v )
    {
        PosType pi = pv[ i ];
        PosType pj = pv[ j ];
        PosType pk = pv[ k ];

        // test complex edge
        if ( existEdge( pi, pj ) || existEdge( pj, pk ) || existEdge( pk, pi ) )
        {
            return Weight();
        }
        // Return an infinite weight, if one of the neighboring patches
        // could not be created.
        if ( v[ i ][ j ] == -1 )
        {
            return Weight();
        }
        if ( v[ j ][ k ] == -1 )
        {
            return Weight();
        }

        // calcolo il massimo angolo diedrale, se esiste.
        float   angle = 0.0f;
        PosType px;
        if ( i + 1 == j )
        {
            px = pj;
            px.FlipE();
            px.FlipV();
            angle = std::max< float >( angle, ComputeDihedralAngle( pi.v->P(), pj.v->P(), pk.v->P(), px.v->P() ) );
        }
        else
        {
            angle = std::max< float >( angle, ComputeDihedralAngle( pi.v->P(), pj.v->P(), pk.v->P(), pv[ v[ i ][ j ] ].v->P() ) );
        }

        if ( j + 1 == k )
        {
            px = pk;
            px.FlipE();
            px.FlipV();
            angle = std::max< float >( angle, ComputeDihedralAngle( pj.v->P(), pk.v->P(), pi.v->P(), px.v->P() ) );
        }
        else
        {
            angle = std::max< float >( angle, ComputeDihedralAngle( pj.v->P(), pk.v->P(), pi.v->P(), pv[ v[ j ][ k ] ].v->P() ) );
        }

        if ( i == 0 && k == ( int ) v.size() - 1 )
        {
            px = pi;
            px.FlipE();
            px.FlipV();
            angle = std::max< float >( angle, ComputeDihedralAngle( pk.v->P(), pi.v->P(), pj.v->P(), px.v->P() ) );
        }

        ScalarType area = ( ( pj.v->P() - pi.v->P() ) ^ ( pk.v->P() - pi.v->P() ) ).Norm() * 0.5;

        return Weight( angle, area );
    }

    static void calculateMinimumWeightTriangulation( MESH &m, FaceIterator f, std::vector< PosType > vv )
    {
        std::vector< std::vector< Weight > > w;  // matrice dei pesi minimali di ogni orecchio preso in conzideraione
        std::vector< std::vector< int > >    vi; // memorizza l'indice del terzo vertice del triangolo

        // hole size
        int nv = vv.size();

        w.clear();
        w.resize( nv, std::vector< Weight >( nv, Weight() ) );

        vi.resize( nv, std::vector< int >( nv, 0 ) );

        // inizializzo tutti i pesi possibili del buco
        for ( int i = 0; i < nv - 1; ++i )
            w[ i ][ i + 1 ] = Weight( 0, 0 );

        // doppio ciclo for per calcolare di tutti i possibili triangoli i loro pesi.
        for ( int j = 2; j < nv; ++j )
        {
            for ( int i = 0; i + j < nv; ++i )
            {
                // per ogni triangolazione mi mantengo il minimo valore del peso tra i triangoli possibili
                Weight minval;

                // indice del vertice che da il peso minimo nella triangolazione corrente
                int minIndex = -1;

                // ciclo tra i vertici in mezzo a i due prefissati
                for ( int m = i + 1; m < i + j; ++m )
                {
                    Weight a = w[ i ][ m ];
                    Weight b = w[ m ][ i + j ];
                    Weight newval = a + b + computeWeight( i, m, i + j, vv, vi );
                    if ( newval < minval )
                    {
                        minval = newval;
                        minIndex = m;
                    }
                }
                w[ i ][ i + j ] = minval;
                vi[ i ][ i + j ] = minIndex;
            }
        }

        // Triangulate
        int i, j;
        i = 0;
        j = nv - 1;

        triangulate( m, f, i, j, vi, vv );

        while ( f != m.face.end() )
        {
            ( *f ).SetD();
            ++f;
            m.fn--;
        }
    }

    static void triangulate( MESH &m, FaceIterator &f, int i, int j, std::vector< std::vector< int > > vi, std::vector< PosType > vv )
    {
        if ( i + 1 == j )
        {
            return;
        }
        if ( i == j )
            return;

        int k = vi[ i ][ j ];

        if ( k == -1 )
            return;

        // Setto i vertici
        f->V( 0 ) = vv[ i ].v;
        f->V( 1 ) = vv[ k ].v;
        f->V( 2 ) = vv[ j ].v;

        f++;
        triangulate( m, f, i, k, vi, vv );
        triangulate( m, f, k, j, vi, vv );
    }

    static void MinimumWeightFill( MESH &m, int holeSize, bool Selected )
    {
        std::vector< PosType >       vvi;
        std::vector< FacePointer * > vfp;

        std::vector< Info >                    vinfo;
        typename std::vector< Info >::iterator VIT;
        GetInfo( m, Selected, vinfo );

        for ( VIT = vinfo.begin(); VIT != vinfo.end(); ++VIT )
        {
            vvi.push_back( VIT->p );
        }

        typename std::vector< PosType >::iterator       ith;
        typename std::vector< PosType >::iterator       ithn;
        typename std::vector< VertexPointer >::iterator itf;

        std::vector< PosType >       app;
        PosType                      ps;
        std::vector< FaceType >      tr;
        std::vector< VertexPointer > vf;

        for ( ith = vvi.begin(); ith != vvi.end(); ++ith )
        {
            tr.clear();
            vf.clear();
            app.clear();
            vfp.clear();

            ps = *ith;
            getBoundHole( ps, app );

            if ( app.size() <= size_t( holeSize ) )
            {
                typename std::vector< PosType >::iterator itP;
                std::vector< FacePointer * >              vfp;

                for ( ithn = vvi.begin(); ithn != vvi.end(); ++ithn )
                    vfp.push_back( &( ithn->f ) );

                for ( itP = app.begin(); itP != app.end(); ++itP )
                    vfp.push_back( &( *itP ).f );

                // aggiungo le facce
                FaceIterator f = tri::Allocator< MESH >::AddFaces( m, ( app.size() - 2 ), vfp );

                calculateMinimumWeightTriangulation( m, f, app );
            }
        }
    }

    static void getBoundHole( PosType sp, std::vector< PosType > &ret )
    {
        PosType fp = sp;
        // take vertex around the hole
        do
        {
            assert( fp.IsBorder() );
            ret.push_back( fp );
            fp.NextB();
        } while ( sp != fp );
    }

}; // close class Hole

} // end namespace tri
} // end namespace vcg
#endif
