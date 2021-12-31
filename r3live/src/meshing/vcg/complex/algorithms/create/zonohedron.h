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

#ifndef __VCGLIB_ZONOHEDRON
#define __VCGLIB_ZONOHEDRON

namespace vcg {
namespace tri {
/** \addtogroup trimesh */
//@{
    /**
                A class to build a Zonohedron.

                Given a set of input vectors, a zonohedron is defined
                as the convex hull of all the points which can be costructed by summing
                together any subset of input vectors.
                The surface closing this solid is composed only of flat parallelograms,
                (which have the input vectors as sides).
                It is always point-symmetric.

                Mesh created by this class are pure-quad meshes (triangular bit-quad),
                (when coplanar vectors are fed, then planar groups of quads can be seen as
                forming planar faces with more than 4 vertices).

                USAGE:
                    1) Instantiate a Zonohedron.
                    2) Add input vectors at will to it, with addVector(s)
                    3) When you are done, call createMesh.

    */


template <class Scalar>
class Zonohedron{
public:

    typedef Point3<Scalar> Vec3;

    Zonohedron(){}

    void addVector(Scalar x, Scalar y, Scalar z);
    void addVector(Vec3 v);
    void addVectors(const std::vector< Vec3 > );

    const std::vector< Vec3 >& vectors() const {
        return vec;
    }

    template<class MeshType>
    void createMesh( MeshType& output );

private:

    /* classes for internal use */
    /****************************/

    typedef int VecIndex; //  a number in [0..n)

    /* the signature of a vertex (a 0 or 1 per input vector) */
    struct Signature {
        std::vector< bool > v;
        Signature(){}
        Signature(int n){ v.resize(n,false); }

        bool operator == (const Signature & b) const {
            return (b.v == v);
        }
        bool operator < (const Signature & b) const {
            return (b.v < v);
        }
        Signature& set(VecIndex i, bool value){
            v[i] = value;
            return *this;
        }
        Signature& set(VecIndex i, bool valueI, VecIndex j, bool valueJ){
            v[i] = valueI;
            v[j] = valueJ;
            return *this;
        }
    };

    struct Face {
        int vert[4]; // index to vertex array
    };

    /* precomputed cross products for all pairs of vectors */
    std::vector< Vec3 > precomputedCross;

    void precompteAllCrosses(){
        precomputedCross.resize(n*n);
        for (int i=0; i<n; i++) for (int j=0; j<n; j++) {
            precomputedCross[i*n+j] =  vec[i] ^ vec[j] ;
        }
    }

    Vec3 cross(VecIndex i, VecIndex j){
        return precomputedCross[i*n+j];
    }

    // given a vector, returns a copy pointing a unique verse
    static Vec3 uniqueVerse(Vec3 v){
        if (v.X()>0) return v;
        else if (v.X()<0) return -v;
        else if (v.Y()>0) return v;
        else if (v.Y()<0) return -v;
        else if (v.Z()>0) return v;
        return -v;
    }

    static Vec3 altVec(int i) {
        return Vec3(1, i, i*i);
    }

    static Scalar tripleProduct( const Vec3 &a, const Vec3 &b, const Vec3 & c){
        return ( a ^ b ) * c;
    }

    // returns signof:  (i x j) * k
    bool signOf_IxJoK(VecIndex i, VecIndex j, VecIndex k){
        const float EPSILON_SQUARED = 1e-12;
        bool invert = false;
        // sort i,j,k
        if (i<j) { std::swap(i,j); invert = !invert; }
        if (j<k) { std::swap(j,k); invert = !invert;
            if (i<j) { std::swap(i,j); invert = !invert; }
        }

        //Scalar res = Vec3::dot( Vec3::cross( vec[i] , vec[j] ) , vec[k] );
        Scalar res =  cross( i , j ) * vec[k] ;

        if (res*res<=EPSILON_SQUARED) {
            // three coplanar vectors!
            // use derivative...
            //res =  uniqueVerse( cross(i,j) ) * cross(j,k) ;
            res =  tripleProduct( altVec(i), vec[j], vec[k]) +
                   tripleProduct( vec[i], altVec(j), vec[k]) +
                   tripleProduct( vec[i], vec[j], altVec(k)) ;
            if (res*res<=EPSILON_SQUARED) {
                // zero derivative (happens, if three colinear vectors, or...)
                res =  tripleProduct( vec[i], altVec(j), altVec(k)) +
                       tripleProduct( altVec(i), vec[j], altVec(k)) +
                       tripleProduct( altVec(i), altVec(j), vec[k]) ;
            }
            if (res*res<=EPSILON_SQUARED) {
                // zero second derivative (happens if three zero-vectors, i.e. never? or...)
                res = tripleProduct( altVec(i), altVec(j), altVec(k) );
            }
        }

        return ( (res>=0) != invert ); // XOR
    }

    int n; // number of input vectors
    std::vector<Vec3> vec; // input vectors

    int vertCount;
    std::vector<Face> _face;

    typedef std::map< Signature, int > VertexMap;
    VertexMap vertexMap;

    // given a vertex signature, returns index of vert (newly created or not)
    VecIndex vertexIndex(const Signature &s){
        typename VertexMap::iterator i;
        //Vec3 pos = s; //toPos(s);
        i = vertexMap.find( s );
        if (i!= vertexMap.end() ) return i->second;
        else {
            int newVertex = vertCount++;
            //vertexMap.insert(s)
            vertexMap[s] = newVertex;
            return newVertex;
        }
    }

    // given two index of vectors, returns face
    Face& face(VecIndex i, VecIndex j){
        assert(i!=j);
        assert( i*n + j < (int) _face.size() );
        return _face[i*n + j];
    }

    Vec3 toPos(const Signature &s) const{
        Vec3 res(0,0,0);
        for (int i=0; i<n; i++)
            if (s.v[i]) res += vec[i];
        return res;
    }

    void createInternalMesh() {

        n = vec.size();
        precompteAllCrosses();

        // allocate faces
        _face.resize( n*n );

        vertCount = 0;
        vertexMap.clear();

        for (int i=0; i<n; i++) {
            //::showProgress(i,n);
            for (int j=0; j<n; j++) if(i!=j)  {
                Signature s(n);
                for (int k=0; k<n; k++) if ((k!=j) && (k!=i))
                {
                    s.set( k , signOf_IxJoK( i,j,k ) );
                }
                face(i,j).vert[0] = vertexIndex( s.set(i,false, j,false) );
                face(i,j).vert[1] = vertexIndex( s.set(i,false, j,true ) );
                face(i,j).vert[2] = vertexIndex( s.set(i,true,  j,true ) );
                face(i,j).vert[3] = vertexIndex( s.set(i,true,  j,false) );
            }
        }
    }


};


template<class Scalar>
void Zonohedron<Scalar>::addVectors(std::vector< Zonohedron<Scalar>::Vec3 > input){
    for (size_t i=0; i<input.size(); i++) {
        addVector( input[i]);
    }
}

template<class Scalar>
void Zonohedron<Scalar>::addVector(Scalar x, Scalar y, Scalar z) {
    addVector( Vec3(x,y,z) );
}


template<class Scalar>
void Zonohedron<Scalar>::addVector(Zonohedron<Scalar>::Vec3 v){
    vec.push_back(v);
}


template<class Scalar>
template<class MeshType>
void Zonohedron<Scalar>::createMesh(MeshType &m){
    typedef MeshType Mesh;
    typedef typename Mesh::VertexPointer  MeshVertexPointer;
    typedef typename Mesh::VertexIterator MeshVertexIterator;
    typedef typename Mesh::FaceIterator   MeshFaceIterator;
    typedef typename Mesh::FaceType   MeshFace;

    createInternalMesh();

    m.Clear();
    Allocator<MeshType>::AddVertices(m,vertexMap.size());
  Allocator<MeshType>::AddFaces(m,n*(n-1) * 2);

    // assign vertex positions
    MeshVertexIterator vi=m.vert.begin();
    for (typename VertexMap::iterator i=vertexMap.begin(); i!=vertexMap.end(); i++){
        (vi + i->second )->P() = toPos( i->first );
    }

    // assegn FV connectivity
  MeshFaceIterator fi=m.face.begin();

    for (int i=0; i<n; i++) {
        for (int j=0; j<n; j++) if (i!=j) {
            const Face &f( face(i,j) );
            for (int k=0; k<2; k++) { // two tri faces per quad
                for (int w=0; w<3; w++) {
                    fi->V(w) = &* (vi + f.vert[(w+k*2)%4] );
                }
                if (tri::HasPerFaceNormal(m)) {
                    fi->N() = cross(i,j).normalized();
                }
                if (tri::HasPerFaceFlags(m)) {
                    fi->SetF(2); // quad diagonals are faux
                }
                fi++;
            }
        }
    }


}


//@}

} // End Namespace TriMesh
} // End Namespace vcg

#endif // __VCGLIB_ZONOHEDRON
