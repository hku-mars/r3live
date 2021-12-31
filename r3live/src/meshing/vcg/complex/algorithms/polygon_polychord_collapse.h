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
#ifndef POLYGON_POLYCHORD_COLLAPSE_H
#define POLYGON_POLYCHORD_COLLAPSE_H

#include <list>
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/jumping_pos.h>

namespace vcg {
namespace tri {
/** \addtogroup trimesh */

/**
* @brief The PolychordCollapse class provides methods to semplify a quad mesh, by collapsing the polychords.
*
* This class is an implementation of a method very similar to that for mesh semplification proposed
* by Daniels et al. in "Quadrilateral mesh simplification", see http://www.cs.utah.edu/~jdaniels/research/asia2008_qms.htm
* The main function is PolychordCollapse::CollapsePolychord() which deletes all the quadrilateral faces in a polychord.
* The polychords that can be collapsed in this case are those forming a closed loop (a ring) or that start and end to
* mesh borders. A way to preserve the structure of the singularities is also provided.
* The convenient method PolychordCollapse::CollapseAllPolychords() finds and collapses all the polychords on a mesh.
* The input mesh should be polygonal, i.e. it should have the vcg::face::PolyInfo component. Even though a generic
* triangle mesh can be given, actually the class does not perform any collapsing operation since it sees only triangles,
* in fact it does not consider faux edges.
*/
template < typename PolyMeshType >
class PolychordCollapse {
public:
  typedef typename PolyMeshType::CoordType      CoordType;
  typedef typename PolyMeshType::VertexType     VertexType;
  typedef typename PolyMeshType::VertexPointer  VertexPointer;
  typedef typename PolyMeshType::VertexIterator VertexIterator;
  typedef typename PolyMeshType::FaceType       FaceType;
  typedef typename PolyMeshType::FacePointer    FacePointer;
  typedef typename PolyMeshType::FaceIterator   FaceIterator;

  /**
  * @brief The PC_ResultCode enum codifies the result type of a polychord collapse operation.
  */
  enum PC_ResultCode {
    PC_SUCCESS = 0,
    PC_NOTMANIF = 1,
    PC_NOTQUAD = 2,
    PC_NOLINKCOND = 4,
    PC_SINGBOTH = 8,
    PC_SELFINTERSECT = 16,
    PC_VOID = 32
  };

  /**
  * @brief The PC_Chord struct identifies a coord of a polychord passing through a quad.
  */
  struct PC_Chord {
    unsigned long mark;
    PC_ResultCode q;
    PC_Chord * prev;
    PC_Chord * next;
    PC_Chord() : mark(std::numeric_limits<unsigned long>::max()), q(PC_VOID), prev(NULL), next(NULL) { }
    inline void Reset() {
      mark = std::numeric_limits<unsigned long>::max();
      q = PC_VOID;
      prev = next = NULL;
    }
  };

  /**
  * @brief The PC_Chords class gives efficient access to each coord (relative to a face).
  */
  class PC_Chords {
  public:
    /**
     * @brief PC_Chords constructor.
     * @note Since each face corresponds to two chords, the actual size of the vector of chords is 2*mesh.face.size().
     * @param mesh
     */
    PC_Chords (const PolyMeshType &mesh) : _Chords(2*mesh.face.size()), _currentChord(NULL) {
      Reset(mesh);
    }

    /**
     * @brief ResetMarks
     */
    void ResetMarks() {
      typename std::vector<PC_Chord>::iterator it = _Chords.begin();
      for (; it != _Chords.end(); it++)
        (*it).mark = std::numeric_limits<unsigned long>::max();
    }

    /**
     * @brief Reset rearrages the container.
     * @note Since each face corresponds to two chords, the actual size of the vector of chords is 2*mesh.face.size().
     * @param mesh
     */
    void Reset(const PolyMeshType &mesh) {
      _Chords.resize(2*mesh.face.size());
      for (size_t j = 0; j < _Chords.size(); j++)
        _Chords[j].Reset();
      _currentChord = NULL;

      PC_Chord *chord = NULL;
      long long j = 0;
      for (size_t i = 0; i < _Chords.size(); i++) {
        // set the prev
        chord = NULL;
        if ((long long)i-1 >= 0) {
          chord = &_Chords[i-1];
          if (vcg::tri::HasPerFaceFlags(mesh)) {
            j = i-1;
            while (j >= 0 && mesh.face[j/2].IsD())
              j--;
            if (j >= 0)
              chord = &_Chords[j];
            else
              chord = NULL;
          }
        }
        _Chords[i].prev = chord;

        // set the next
        chord = NULL;
        if (i+1 < _Chords.size()) {
          chord = &_Chords[i+1];
          if (vcg::tri::HasPerFaceFlags(mesh)) {
            j = i+1;
            while (j < (long long)_Chords.size() && mesh.face[j/2].IsD())
              j++;
            if (j < (long long)_Chords.size())
              chord = &_Chords[j];
            else
              chord = NULL;
          }
        }
        _Chords[i].next = chord;
      }
      if (mesh.face.size() > 0) {
        // set the current coord (first - not deleted - face)
        _currentChord = &_Chords[0];
        if (vcg::tri::HasPerFaceFlags(mesh) && mesh.face[0].IsD())
          _currentChord = _currentChord->next;
      }
    }

    /**
     * @brief operator [], given a face index and an offset, it returns (a reference to) its corresponding PC_Chord.
     * @param face_edge A std::pair<size_t, unsigned char>(face_index, offset). The offset should be 0 or 1.
     * @return A reference to the corresponding PC_Chord.
     */
    inline PC_Chord & operator[] (const std::pair<size_t, unsigned char> &face_edge) {
      assert(face_edge.first >= 0 && 2*face_edge.first+face_edge.second < _Chords.size());
      return _Chords[2*face_edge.first + face_edge.second];
    }
    /**
     * @brief operator [], given a face index and an offset, it returns (a const reference to) its corresponding PC_Chord.
     * @param face_edge A std::pair<size_t, unsigned char>(face_index, offset). The offset should be 0 or 1.
     * @return A reference to the corresponding PC_Chord.
     */
    inline const PC_Chord & operator[] (const std::pair<size_t, unsigned char> &face_edge) const {
      assert(face_edge.first >= 0 && 2*face_edge.first+face_edge.second < _Chords.size());
      return _Chords[2*face_edge.first + face_edge.second];
    }

    /**
     * @brief operator [], given a coord, it returns its corresponding face index and edge.
     * @param coord The coord pointer.
     * @return A std::pair <size_t, unsigned char>(face_index, offset) with offset being 0 or 1.
     */
    inline std::pair<size_t, unsigned char> operator[] (PC_Chord const * const coord) {
      assert(coord >= &_Chords[0] && coord < &_Chords[0]+_Chords.size());
      return std::pair<size_t, unsigned char>((coord - &_Chords[0])/2, (coord - &_Chords[0])%2);
    }

    /**
     * @brief UpdateCoord updates the coord information and links.
     * @param coord The coord to update.
     * @param mark The mark of the polychord.
     * @param resultCode The code for the type of the polychord.
     */
    inline void UpdateCoord (PC_Chord &coord, const unsigned long mark, const PC_ResultCode resultCode) {
      // update prev and next
      if (coord.q == PC_VOID) {
        if (coord.prev != NULL && &coord != _currentChord)
          coord.prev->next = coord.next;
        if (coord.next != NULL && &coord != _currentChord)
          coord.next->prev = coord.prev;
        coord.mark = mark;
      }
      coord.q = resultCode;
    }

    /**
     * @brief Next, if it's not at the end, it goes to the next coord.
     */
    inline void Next () {
      if (_currentChord != NULL)
        _currentChord = _currentChord->next;
    }

    /**
     * @brief GetCurrent returns the current FaceType pointer and edge.
     * @param face_edge A std::pair where to store the FaceType pointer and the edge index.
     */
    inline void GetCurrent (std::pair<size_t, unsigned char> &face_edge) {
      if (_currentChord != NULL) {
        face_edge.first = (_currentChord - &_Chords[0])/2;
        face_edge.second = (_currentChord - &_Chords[0])%2;
      } else {
        face_edge.first = std::numeric_limits<size_t>::max();
        face_edge.second = 0;
      }
    }

    /**
     * @brief End says if an end has been reached.
     * @return true if an end has been reached, false otherwise.
     */
    inline bool End () {
      return _currentChord == NULL;
    }

  private:
    std::vector<PC_Chord>   _Chords;
    PC_Chord                *_currentChord;
  };

  /**
   * @brief The LinkCondition class provides a tool to check if a polychord satisfies the link conditions.
   */
  class LinkConditions {
  private:
    struct LCEdge;
    struct LCVertex;

    typedef std::set<LCVertex *> LCVertexStar;  // define the star of a vertex
    typedef std::set<LCEdge *> LCEdgeStar;      // define the set of edges whose star involves a vertex

  public:
    /**
     * @brief LinkCondition constructor.
     * @param size The number of vertices of the mesh.
     */
    LinkConditions (const size_t size) : _lcVertices(size) { }

    /**
     * @brief Resize just resets the size of the container.
     * @param size
     */
    inline void Resize(const size_t size) {
      _lcVertices.resize(size);
    }

    /**
     * @brief CheckLinkConditions checks if collapsing the polychord starting from startPos
     * satisfies the link conditions.
     * @warning The polychord starts from startPos and ends to itself (if it's a loop) or to a border. In the latter case,
     * call this method starting from the opposite border of the strip of quads.
     * @param mesh The mesh for getting the vertex index.
     * @param startPos The starting position of the polychord.
     * @return true if satisfied, false otherwise.
     */
    bool CheckLinkConditions (const PolyMeshType &mesh, const vcg::face::Pos<FaceType> &startPos) {
      assert(!startPos.IsNull());
      assert(mesh.vert.size() == _lcVertices.size());
      std::list<LCEdge> lcEdges;
      LCEdge *e = NULL;
      LCVertexStar intersection;

      // reset the stars
      LC_ResetStars(mesh, startPos);

      // compute the stars
      LC_computeStars(mesh, startPos, lcEdges);

      // for each edge e = (v1,v2)
      // if intersection( star(v1) , star(v2) ) == star(e)
      //      then collapse e
      // else
      //      return false (i.e. link conditions not satisfied)
      for (typename std::list<LCEdge>::iterator eIt = lcEdges.begin(); eIt != lcEdges.end(); eIt++) {
        e = &*eIt;
        // compute the intersetion
        SetIntersection(e->v1->star, e->v2->star, intersection);
        // if intersection( star(v1) , star(v2) ) != star(e) then return false
        if (intersection != e->star)
            return false;
        // else simulate the collapse
        LC_SimulateEdgeCollapse(*e);
      }
      // at this point all collapses are possible, thus return true
      return true;
    }

  private:
    /**
     * @brief SetIntersection computes the set intersection between two sets.
     * @param set1
     * @param set2
     * @param result The set resulting from the intersection.
     */
    static void SetIntersection (const LCVertexStar &set1, const LCVertexStar &set2, LCVertexStar &result) {
      typename LCVertexStar::const_iterator set1It = set1.begin();
      typename LCVertexStar::const_iterator set2It = set2.begin();
      result.clear();
      while (set1It != set1.end() && set2It != set2.end()) {
        if (*set1It < *set2It) ++set1It;
        else if (*set2It < *set1It) ++set2It;
        else {
          result.insert(*set1It);
          ++set1It;
          ++set2It;
        }
      }
    }

    /**
     * @brief LC_ResetStars resets the stars on a polychord.
     * @param mesh The mesh for getting the vertex index.
     * @param startPos
     */
    void LC_ResetStars (const PolyMeshType &mesh, const vcg::face::Pos<FaceType> &startPos) {
      assert(!startPos.IsNull());
      assert(mesh.vert.size() == _lcVertices.size());
      vcg::face::Pos<FaceType> runPos = startPos;
      vcg::face::JumpingPos<FaceType> vStarPos;
      // reset the stars
      do {
        // reset the star of this edge endpoints
        _lcVertices[vcg::tri::Index(mesh, runPos.V())].edges.clear();
        _lcVertices[vcg::tri::Index(mesh, runPos.V())].star.clear();
        _lcVertices[vcg::tri::Index(mesh, runPos.VFlip())].edges.clear();
        _lcVertices[vcg::tri::Index(mesh, runPos.VFlip())].star.clear();
        // reset the stars of the vertices in the star of the second vertex
        runPos.FlipV();
        vStarPos.Set(runPos.F(), runPos.E(), runPos.V());
        do {
          vStarPos.FlipV();
          vStarPos.FlipE();
          while (vStarPos.V() != runPos.V()) {
            _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].edges.clear();
            _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].star.clear();
            vStarPos.FlipV();
            vStarPos.FlipE();
          }
          vStarPos.NextFE();
        } while (vStarPos != runPos);
        // reset the stars of the vertices in the star of the first vertex
        runPos.FlipV();
        vStarPos.Set(runPos.F(), runPos.E(), runPos.V());
        do {
          vStarPos.FlipV();
          vStarPos.FlipE();
          while (vStarPos.V() != runPos.V()) {
            _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].edges.clear();
            _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].star.clear();
            vStarPos.FlipV();
            vStarPos.FlipE();
          }
          vStarPos.NextFE();
        } while (vStarPos != runPos);
        // when arrive to a border, return
        if (runPos != startPos && runPos.IsBorder())
          break;
        // go on the next edge
        runPos.FlipE();
        runPos.FlipV();
        runPos.FlipE();
        runPos.FlipF();
      } while (runPos != startPos);
    }

    /**
     * @brief LC_computeStars computes the stars of edges and vertices of the polychord from the starting pos
     * either to itself (if it's a loop) or to the border edge.
     * @param mesh The mesh for getting the vertex index.
     * @param startPos Starting position.
     * @param lcEdges List of edge stars.
     */
    void LC_computeStars (const PolyMeshType &mesh, const vcg::face::Pos<FaceType> &startPos, std::list<LCEdge> &lcEdges)
    {
      assert(!startPos.IsNull());
      assert(mesh.vert.size() == _lcVertices.size());
      LCEdge *lcedgeP = NULL;
      vcg::face::Pos<FaceType> runPos = startPos;
      vcg::face::JumpingPos<FaceType> vStarPos;
      vcg::face::Pos<FaceType> eStarPos;

      lcEdges.clear();
      /// compute the star of all the vertices and edges seen from the polychord
      runPos = startPos;
      do {
        // create a lcedge
        lcEdges.push_back(LCEdge());
        lcedgeP = &lcEdges.back();
        // set lcvertices references
        lcedgeP->v1 = &_lcVertices[vcg::tri::Index(mesh, runPos.V())];
        lcedgeP->v2 = &_lcVertices[vcg::tri::Index(mesh, runPos.VFlip())];
        // add this edge to its vertices edge-stars
        lcedgeP->v1->edges.insert(lcedgeP);
        lcedgeP->v2->edges.insert(lcedgeP);
        // compute the star of this edge
        lcedgeP->star.insert(lcedgeP->v1);  // its endpoints, clearly
        lcedgeP->star.insert(lcedgeP->v2);  // its endpoints, clearly
        // navigate over the other vertices of this facet
        eStarPos = runPos;
        eStarPos.FlipE();
        eStarPos.FlipV();
        while (eStarPos.V() != runPos.VFlip()) {
          // add current vertex to the star of this edge
          lcedgeP->star.insert(&_lcVertices[vcg::tri::Index(mesh, eStarPos.V())]);
          // add this edge to the edge-star of the current vertex
          _lcVertices[vcg::tri::Index(mesh, eStarPos.V())].edges.insert(lcedgeP);
          // go on
          eStarPos.FlipE();
          eStarPos.FlipV();
        }
        // go on the opposite facet
        if (!runPos.IsBorder()) {
          eStarPos = runPos;
          eStarPos.FlipF();
          eStarPos.FlipE();
          eStarPos.FlipV();
          while (eStarPos.V() != runPos.VFlip()) {
            // add current vertex to the star of this edge
            lcedgeP->star.insert(&_lcVertices[vcg::tri::Index(mesh, eStarPos.V())]);
            // add this edge to the edge-star of the current vertex
            _lcVertices[vcg::tri::Index(mesh, eStarPos.V())].edges.insert(lcedgeP);
            // go on
            eStarPos.FlipE();
            eStarPos.FlipV();
          }
        }

        // compute the star of vertex v2
        runPos.FlipV();
        vStarPos.Set(runPos.F(), runPos.E(), runPos.V());
        // v2 is in its star
        _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].star.insert(&_lcVertices[vcg::tri::Index(mesh, vStarPos.V())]);
        do {
          vStarPos.FlipV();
          vStarPos.FlipE();
          while (vStarPos.V() != runPos.V()) {
            // add the current vertex to the v2 star
            _lcVertices[vcg::tri::Index(mesh, runPos.V())].star.insert(&_lcVertices[vcg::tri::Index(mesh, vStarPos.V())]);
            // add v2 to the star of the current vertex
            _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].star.insert(&_lcVertices[vcg::tri::Index(mesh, runPos.V())]);
            vStarPos.FlipV();
            vStarPos.FlipE();
          }
          vStarPos.NextFE();
        } while (vStarPos != runPos);

        // compute the star of vertex v1
        runPos.FlipV();
        vStarPos.Set(runPos.F(), runPos.E(), runPos.V());
        // v1 is in its star
        _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].star.insert(&_lcVertices[vcg::tri::Index(mesh, vStarPos.V())]);
        do {
          vStarPos.FlipV();
          vStarPos.FlipE();
          while (vStarPos.V() != runPos.V()) {
            // add the current vertex to the v2 star
            _lcVertices[vcg::tri::Index(mesh, runPos.V())].star.insert(&_lcVertices[vcg::tri::Index(mesh, vStarPos.V())]);
            // add v2 to the star of the current vertex
            _lcVertices[vcg::tri::Index(mesh, vStarPos.V())].star.insert(&_lcVertices[vcg::tri::Index(mesh, runPos.V())]);
            vStarPos.FlipV();
            vStarPos.FlipE();
          }
          vStarPos.NextFE();
        } while (vStarPos != runPos);

        // when arrive to a border, stop
        if (runPos != startPos && runPos.IsBorder())
          break;

        // go on the next edge
        runPos.FlipE();
        runPos.FlipV();
        runPos.FlipE();
        runPos.FlipF();
      } while (runPos != startPos);

      // check if the starting pos or the border has been reached
      assert(runPos == startPos || runPos.IsBorder());
    }

    /**
     * @brief LC_SimulateEdgeCollapse simulates an edge collapse by updating the stars involved.
     * @param edge The edge to collapse.
     */
    void LC_SimulateEdgeCollapse (LCEdge &edge) {
      // let v1 and v2 be the two end points
      LCVertex *v1 = edge.v1;
      LCVertex *v2 = edge.v2;
      assert(v1 && v2);
      LCVertex *v = NULL;
      LCEdge *e = NULL;

      /// v2 merges into v1:
      // star(v1) = star(v1) U star(v2)
      v1->star.insert(v2->star.begin(), v2->star.end());
      v1->star.erase(v2);     // remove v2 from v1-star
      v2->star.erase(v1);     // remove v1 from v2-star
      // foreach v | v2 \in star(v) [i.e. v \in star(v2)]
      //      star(v) = star(v) U {v1} \ {v2}
      for (typename LCVertexStar::iterator vIt = v2->star.begin(); vIt != v2->star.end(); vIt++) {
        v = *vIt;
        v->star.insert(v1);
        v->star.erase(v2);
      }
      /// update the star of the edges which include v1 and v2 in their star
      // foreach e | v1 \in star(e) ^ v2 \in star(e)
      //      star(e) = star(e) \ {v1,v2} U {v1}
      for (typename LCEdgeStar::iterator eIt = v1->edges.begin(); eIt != v1->edges.end(); eIt++) {
        e = *eIt;
        e->star.erase(v2);
      }
      for (typename LCEdgeStar::iterator eIt = v2->edges.begin(); eIt != v2->edges.end(); eIt++) {
        e = *eIt;
        e->star.erase(v2);
        e->star.insert(v1);
      }
    }

    /**
     * @brief The LCVertex struct represents a vertex for the Link Conditions.
     */
    struct LCVertex {
      LCVertexStar star;  // vertex star
      LCEdgeStar edges;   // list of edges whose star involves this vertex
    };

    /**
     * @brief The LCEdge struct represents an edge for the Link Conditions.
     */
    struct LCEdge {
      LCVertex *v1, *v2;          // endpoints
      LCVertexStar star;          // edge star
      LCEdge() {v1 = v2 = NULL;}  // default contructor
    };

    /**
     * @brief _lcVertices is a vector of vertex stars for the link conditions.
     */
    std::vector<LCVertex> _lcVertices;
  };


  // PolychordCollapse's methods begin here::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

  /**
   * @brief CollapsePolychord performs all checks and then collapses the polychord.
   *
   * @warning This function deletes faces and vertices by calling
   * vcg::tri::Allocator<PolyMeshType>::DeleteFace() and
   * vcg::tri::Allocator<PolyMeshType>::DeleteVertex().
   * The object PC_Chords chords is used to track the polychords, and it has got
   * a size proportional to that of the mesh face container. If you actually
   * delete faces and vertices by calling vcg::tri::Allocator<PolyMeshType>::CompactFaceVector()
   * and vcg::tri::Allocator<PolyMeshType>::CompactVertexVector() after this function,
   * object PC_Chords chords then is not valid any more, so you MUST rearrange it
   * by calling PC_Chords.Reset(). For the same reason, you MUST rearrange LinkConditions linkConditions
   * by calling LinkConditions.Resize().
   * However, for efficiency, you SHOULD compact vertex and face containers at the end of all your
   * polychord collapsing operations, without having to rearrange chords and linkConditions.
   * The function CollapseAllPolychords() does this for you.
   *
   * @note Vertex flags, face flags, FF adjacency and FV adjacency are required. Not anything else.
   * Such components are automatically updated here. If the mesh has other components that may be
   * affected by this editing, you should update them later by yourself.
   *
   * @param mesh The polygonal mesh used for getting the face index and deleting the faces
   * (it SHOULD have the vcg::face::PolyInfo component).
   * @param pos Position of the polychord.
   * @param mark Mark for the current polychord.
   * @param chords Vector of chords.
   * @param linkConditions Link conditions checker.
   * @param checkSing true if singularities on both sides are not allowed.
   * @return A PC_ResultCode resulting from checks or PC_SUCCESS if the collapse has been performed.
   */
  static PC_ResultCode CollapsePolychord (PolyMeshType &mesh,
                                          const vcg::face::Pos<FaceType> &pos,
                                          const unsigned long mark,
                                          PC_Chords &chords,
                                          LinkConditions &linkConditions,
                                          const bool checkSing = true) {
    vcg::tri::RequirePerVertexFlags(mesh);
    vcg::tri::RequirePerFaceFlags(mesh);

    if (mesh.face.size() == 0)
      return PC_VOID;

    if (pos.IsNull())
      return PC_VOID;

    vcg::face::Pos<FaceType> tempPos, startPos;

    // check if the sequence of facets is a polychord and find the starting coord
    PC_ResultCode resultCode = CheckPolychordFindStartPosition(pos, startPos, checkSing);
    // if not successful, visit the sequence for marking it and return
    if (resultCode != PC_SUCCESS) {
      // if not manifold, visit the entire polychord ending on the non-manifold edge
      if (resultCode == PC_NOTMANIF) {
        tempPos = pos;
        VisitPolychord(mesh, tempPos, chords, mark, resultCode);
        if (tempPos.IsManifold() && !tempPos.IsBorder()) {
          tempPos.FlipF();
          VisitPolychord(mesh, tempPos, chords, mark, resultCode);
        }
        return resultCode;
      }
      // if not quad, visit all the polychords passing through this coord
      if (resultCode == PC_NOTQUAD) {
        tempPos = startPos;
        do {
          if (!tempPos.IsBorder()) {
            tempPos.FlipF();
            VisitPolychord(mesh, tempPos, chords, mark, resultCode);
            tempPos.FlipF();
          }
          tempPos.FlipV();
          tempPos.FlipE();
        } while (tempPos != startPos);
      }
      VisitPolychord(mesh, startPos, chords, mark, resultCode);
      return resultCode;
    }
    // check if the link conditions are satisfied
    bool lc = linkConditions.CheckLinkConditions(mesh, startPos);
    // if not satisfied, visit the sequence for marking it and return
    if (!lc) {
      VisitPolychord(mesh, startPos, chords, mark, PC_NOLINKCOND);
      return PC_NOLINKCOND;
    }
    // check if the polychord does not intersect itself
    bool si = IsPolychordSelfIntersecting(mesh, startPos, chords, mark);
    // if it self-intersects, visit the polychord for marking it and return
    if (si) {
      VisitPolychord(mesh, startPos, chords, mark, PC_SELFINTERSECT);
      return PC_SELFINTERSECT;
    }
    // at this point the polychord is collapsable, visit it for marking
    VisitPolychord(mesh, startPos, chords, mark, PC_SUCCESS);

    // now collapse
    CoordType point;
    int valenceA = 0, valenceB = 0;
    vcg::face::Pos<FaceType> runPos = startPos;
    vcg::face::JumpingPos<FaceType> tmpPos;
    bool onSideA = false, onSideB = false;
    vcg::face::Pos<FaceType> sideA, sideB;
    typedef std::queue<VertexPointer *> FacesVertex;
    typedef std::pair<VertexPointer, FacesVertex> FacesVertexPair;
    typedef std::queue<FacesVertexPair> FacesVertexPairQueue;
    FacesVertexPairQueue vQueue;
    typedef std::pair<FacePointer *, FacePointer> FFpPair;
    typedef std::pair<char *, char> FFiPair;
    typedef std::pair<FFpPair, FFiPair> FFPair;
    typedef std::queue<FFPair> FFQueue;
    FFQueue ffQueue;
    std::queue<VertexPointer> verticesToDeleteQueue;
    std::queue<FacePointer> facesToDeleteQueue;

    if (checkSing) {
      do {
        runPos.FlipV();
        valenceB = runPos.NumberOfIncidentVertices();
        tmpPos.Set(runPos.F(), runPos.E(), runPos.V());
        if (tmpPos.FindBorder())
          valenceB++;
        runPos.FlipV();
        valenceA = runPos.NumberOfIncidentVertices();
        tmpPos.Set(runPos.F(), runPos.E(), runPos.V());
        if (tmpPos.FindBorder())
          valenceA++;
        if (valenceA != 4)
          onSideA = true;
        if (valenceB != 4)
          onSideB = true;
        assert(!onSideA || !onSideB);

        if (runPos != startPos && runPos.IsBorder())
          break;

        // go on next edge/face
        runPos.FlipE();
        runPos.FlipV();
        runPos.FlipE();
        runPos.FlipF();
      } while (runPos != startPos);
    }

    runPos = startPos;
    do {
      // compute new vertex
      point = (runPos.V()->P() + runPos.VFlip()->P()) / 2.f;
      if (checkSing) {
        if (onSideA)
          point = runPos.V()->P();
        if (onSideB)
          point = runPos.VFlip()->P();
      }
      runPos.V()->P() = point;
      // list the vertex pointer of the faces on the other side to be updated
      vQueue.push(FacesVertexPair());
      vQueue.back().first = runPos.V();
      tmpPos.Set(runPos.F(), runPos.E(), runPos.V());
      tmpPos.FlipV();
      tmpPos.NextFE();    // go to next face
      while (tmpPos.F() != runPos.F()) {
        if (tmpPos.F() != runPos.FFlip())
          vQueue.back().second.push(&tmpPos.F()->V(tmpPos.VInd()));
        tmpPos.NextFE();    // go to next face
      }

      // enqueue to delete the other vertex
      verticesToDeleteQueue.push(runPos.VFlip());

      // list the adjacencies
      sideA = runPos;
      sideA.FlipE();
      sideA.FlipF();
      sideB = runPos;
      sideB.FlipV();
      sideB.FlipE();
      sideB.FlipF();
      // first side
      if (!sideA.IsBorder()) {
        ffQueue.push(FFPair(FFpPair(),FFiPair()));
        ffQueue.back().first.first = &sideA.F()->FFp(sideA.E());
        ffQueue.back().second.first = &sideA.F()->FFi(sideA.E());
        if (!sideB.IsBorder()) {
          ffQueue.back().first.second = sideB.F();
          ffQueue.back().second.second = sideB.E();
        } else {
          ffQueue.back().first.second = sideA.F();
          ffQueue.back().second.second = sideA.E();
        }
      }
      // second side
      if (!sideB.IsBorder()) {
        ffQueue.push(FFPair(FFpPair(),FFiPair()));
        ffQueue.back().first.first = &sideB.F()->FFp(sideB.E());
        ffQueue.back().second.first = &sideB.F()->FFi(sideB.E());
        if (!sideA.IsBorder()) {
          ffQueue.back().first.second = sideA.F();
          ffQueue.back().second.second = sideA.E();
        } else {
          ffQueue.back().first.second = sideB.F();
          ffQueue.back().second.second = sideB.E();
        }
      }

      // enqueue to delete the face
      facesToDeleteQueue.push(runPos.F());

      // go on next edge/face
      runPos.FlipE();
      runPos.FlipV();
      runPos.FlipE();
      runPos.FlipF();
    } while (runPos != startPos && !runPos.IsBorder());
    assert(runPos == startPos || vcg::face::IsBorder(*startPos.F(),startPos.E()));
    if (runPos.IsBorder()) {
      // compute new vertex on the last (border) edge
      point = (runPos.V()->P() + runPos.VFlip()->P()) / 2.f;
      if (checkSing) {
        if (onSideA)
          point = runPos.V()->P();
        if (onSideB)
          point = runPos.VFlip()->P();
      }
      runPos.V()->P() = point;
      // list the vertex pointer of the faces on the other side to be updated
      vQueue.push(FacesVertexPair());
      vQueue.back().first = runPos.V();
      tmpPos.Set(runPos.F(), runPos.E(), runPos.V());
      tmpPos.FlipV();
      tmpPos.NextFE();    // go to next face
      while (tmpPos.F() != runPos.F()) {
        vQueue.back().second.push(&tmpPos.F()->V(tmpPos.VInd()));
        tmpPos.NextFE();
      }

      // enqueue to delete the other vertex
      verticesToDeleteQueue.push(runPos.VFlip());
    }

    // update vertices
    while (!vQueue.empty()) {
      while (!vQueue.front().second.empty()) {
        *vQueue.front().second.front() = vQueue.front().first;
        vQueue.front().second.pop();
      }
      vQueue.pop();
    }

    // update adjacencies
    while (!ffQueue.empty()) {
      *ffQueue.front().first.first = ffQueue.front().first.second;
      *ffQueue.front().second.first = ffQueue.front().second.second;
      ffQueue.pop();
    }

    // delete faces
    while (!facesToDeleteQueue.empty()) {
      vcg::tri::Allocator<PolyMeshType>::DeleteFace(mesh, *facesToDeleteQueue.front());
      facesToDeleteQueue.pop();
    }

    // delete vertices
    while (!verticesToDeleteQueue.empty()) {
      vcg::tri::Allocator<PolyMeshType>::DeleteVertex(mesh, *verticesToDeleteQueue.front());
      verticesToDeleteQueue.pop();
    }

    return PC_SUCCESS;
  }

  /**
   * @brief CollapseAllPolychords finds and collapses all the polychords.
   * @param mesh The input polygonal mesh (it SHOULD have the vcg::face::PolyInfo component).
   * @param checkSing true if singularities on both sides of a polychord are not allowed.
   */
  static void CollapseAllPolychords (PolyMeshType &mesh, const bool checkSing = true) {
    vcg::tri::RequireFFAdjacency(mesh);

    if (mesh.FN() == 0)
      return;

    vcg::face::Pos<FaceType> pos;
    PC_ResultCode resultCode;
    std::pair<size_t, unsigned char> face_edge;
    // construct the link conditions checker
    LinkConditions linkConditions(mesh.vert.size());
    // construct the vector of chords
    PC_Chords chords(mesh);
    unsigned long mark = 0;

    // iterate over all the chords
    while (!chords.End()) {
      // get the current coord
      chords.GetCurrent(face_edge);
      // construct a pos on the face and edge of the current coord
      pos.Set(&mesh.face[face_edge.first], face_edge.second, mesh.face[face_edge.first].V(face_edge.second));
      // (try to) collapse the polychord
      resultCode = CollapsePolychord(mesh, pos, mark, chords, linkConditions, checkSing);
      // go to the next coord
      chords.Next();

      // increment the mark
      mark++;
      if (mark == std::numeric_limits<unsigned long>::max()) {
        chords.ResetMarks();
        mark = 0;
      }
    }
  }

  /**
   * @brief SplitPolychord splits a polychord into n polychords by inserting all the needed faces.
   * @param mesh is the input polygonal mesh.
   * @param pos is a position into the polychord (not necessarily the starting border).
   * @param n is the number of polychords to replace the input one.
   * @param facesToUpdate is a vector of face pointers to be updated after re-allocation.
   * @param verticesToUpdate is a vector of vertex pointers to be updated after re-allocation.
   */
  static void SplitPolychord (PolyMeshType &mesh, const vcg::face::Pos<FaceType> &pos, const size_t n,
                              std::vector<FacePointer *> &facesToUpdate = std::vector<FacePointer *>(),
                              std::vector<VertexPointer *> &verticesToUpdate = std::vector<VertexPointer *>()) {
    if (mesh.IsEmpty())
      return;
    if (pos.IsNull())
      return;
    if (n <= 1)
      return;

    // find the real starting position (is the polychord a strip or a ring?) and count how many faces there are
    size_t fn = 0;
    bool polyBorderFound = false;
    vcg::face::Pos<FaceType> startPos = pos;
    do {
      // check if all faces are 4-sided
      if (startPos.F()->VN() != 4)
        return;
      // check manifoldness
      if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
        return;

      // increase the number of faces
      fn++;

      // go on the opposite edge
      startPos.FlipE();
      startPos.FlipV();
      startPos.FlipE();

      // if the first border has been reached, go on the other direction to find the other border
      if (!polyBorderFound && startPos != pos && startPos.IsBorder()) {
        // check manifoldness
        if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
          return;
        startPos = pos;
        polyBorderFound = true;
      }

      // if the other border has been reached, stop
      if (polyBorderFound && startPos.IsBorder()) {
        // check manifoldness
        if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
          return;
        break;
      }

      // check manifoldness
      if (!startPos.IsManifold())
        return;
      // go onto the next face
      startPos.FlipF();
    } while (startPos != pos);

    // as every face has an orientation, ensure that the new polychords are inserted on the right of the starting pos
    startPos.FlipE();
    int e = startPos.E();
    startPos.FlipE();
    if (startPos.F()->Next(startPos.E()) != e)
      startPos.FlipV();

    // compute the number of faces and vertices that must be added to the mesh in order to insert the new polychords
    size_t FN = fn * (n - 1);
    size_t VN = FN;
    if (startPos.IsBorder())
      VN += n - 1;

    // add the starting position's face and vertex pointers to the list of things to update after re-allocation
    facesToUpdate.push_back(&startPos.F());
    verticesToUpdate.push_back(&startPos.V());

    // add faces to the mesh
    FaceIterator firstAddedFaceIt = vcg::tri::Allocator<PolyMeshType>::AddFaces(mesh, FN, facesToUpdate);
    // add vertices to the mesh
    VertexIterator firstAddedVertexIt = vcg::tri::Allocator<PolyMeshType>::AddVertices(mesh, VN, verticesToUpdate);

    // delete the added starting position's face and vertex pointers
    facesToUpdate.pop_back();
    verticesToUpdate.pop_back();

    // allocate and initialize 4 vertices and ffAdj for each new face
    for (FaceIterator fIt = firstAddedFaceIt; fIt != mesh.face.end(); fIt++) {
      fIt->Alloc(4);
      for (size_t j = 0; j < 4; j++) {
        fIt->FFp(j) = &*fIt;
        fIt->FFi(j) = j;
      }
    }

    // some variables
    size_t ln = fn;
    if (startPos.IsBorder())
      ln++;
    FacePointer lf = NULL;        // face on the left to the current one
    int lfre = 0;                 // right edge of lf
    VertexPointer * lfbrV = NULL; // address of the bottom-right vertex pointer of lf
    VertexPointer * lftrV = NULL; // address of the top-right vertex pointer of lf
    CoordType lvP;
    CoordType svP;
    typedef std::pair<FacePointer,int>    FaceEdge;
    typedef std::pair<FaceEdge,FaceEdge>  FaceFaceAdj;
    typedef std::pair<VertexPointer *,VertexPointer>  FaceVertexAdj;
    std::queue<FaceFaceAdj>   ffAdjQueue;   // face-to-face adjacency queue
    std::queue<FaceVertexAdj> fvAdjQueue;   // face-to-vertex adjacency queue
    bool currentFaceBottomIsBorder = false;

    // scan the polychord and add adj into queues
    vcg::face::Pos<FaceType> runPos = startPos;
    for (size_t i = 0; i < fn; i++) {
      // store links to the current left face
      lf = runPos.F();
      currentFaceBottomIsBorder = runPos.IsBorder();
      lvP = runPos.VFlip()->P();
      svP = (runPos.V()->P() - lvP) / n;
      runPos.FlipE();
      lfre = runPos.E();
      lfbrV = &runPos.F()->V(runPos.VInd());
      runPos.FlipV();
      lftrV = &runPos.F()->V(runPos.VInd());
      // set the current line's last face's right ff adjacency
      if (!runPos.IsBorder())
        ffAdjQueue.push(FaceFaceAdj(FaceEdge(&*(firstAddedFaceIt + (i+1)*(n-1) - 1), 1), FaceEdge(runPos.FFlip(), runPos.F()->FFi(runPos.E()))));
      // set the current line's last face's bottom right vertex's coords
      (firstAddedVertexIt + (i+1)*(n-1) - 1)->P() = lvP + svP * (n - 1);
      // set the current line's last face's bottom right vertex
      fvAdjQueue.push(FaceVertexAdj(&(firstAddedFaceIt + (i+1)*(n-1) - 1)->V(1), runPos.VFlip()));
      // set the current face's top left vertex
      fvAdjQueue.push(FaceVertexAdj(&(firstAddedFaceIt + (i+1)*(n-1) - 1)->V(2), runPos.V()));
      runPos.FlipE();
      if (!runPos.IsBorder())
        runPos.FlipF();
      else
        for (size_t j = 0; j < n-1; j++)
          // set the current face's bottom right vertex's coords
          (firstAddedVertexIt + (i+1)*(n-1) + j)->P() = runPos.VFlip()->P() +
                                                        (runPos.V()->P() - runPos.VFlip()->P()) / n * (j+1);

      // run horizontally on the current line of the grid
      for (size_t j = 0; j < n-1; j++) {
        // set the current face's left ff adj
        ffAdjQueue.push(FaceFaceAdj(FaceEdge(lf, lfre), FaceEdge(&*(firstAddedFaceIt + i*(n-1) + j), 3)));
        // set the current face's bottom ff adjacency
        if (!currentFaceBottomIsBorder)
          ffAdjQueue.push(FaceFaceAdj(FaceEdge(&*(firstAddedFaceIt + ((i+fn-1)%fn)*(n-1) + j), 2), FaceEdge(&*(firstAddedFaceIt + i*(n-1) + j), 0)));
        // set the current face's bottom right vertex's coords
        (firstAddedVertexIt + i*(n-1) + j)->P() = lvP + svP * (j+1);
        // set the left face's bottom right vertex
        fvAdjQueue.push(FaceVertexAdj(lfbrV, &*(firstAddedVertexIt + i*(n-1) + j)));
        // set the current face's bottom left vertex
        fvAdjQueue.push(FaceVertexAdj(&(firstAddedFaceIt + i*(n-1) + j)->V(0), &*(firstAddedVertexIt + i*(n-1) + j)));
        // set the left face's top right vertex
        fvAdjQueue.push(FaceVertexAdj(lftrV, &*(firstAddedVertexIt + ((i+1)%ln)*(n-1) + j)));
        // set the current face's top left vertex
        fvAdjQueue.push(FaceVertexAdj(&(firstAddedFaceIt + i*(n-1) + j)->V(3), &*(firstAddedVertexIt + ((i+1)%ln)*(n-1) + j)));

        // update temporary variables
        lf = &*(firstAddedFaceIt + i*(n-1) + j);
        lfre = 1;
        lfbrV = &(firstAddedFaceIt + i*(n-1) + j)->V(1);
        lftrV = &(firstAddedFaceIt + i*(n-1) + j)->V(2);
      }
    }

    // now apply ff adj changes
    while (!ffAdjQueue.empty()) {
      // the left/bottom face links to the right/top face
      ffAdjQueue.front().first.first->FFp(ffAdjQueue.front().first.second) = ffAdjQueue.front().second.first;
      ffAdjQueue.front().first.first->FFi(ffAdjQueue.front().first.second) = ffAdjQueue.front().second.second;
      // the right/top face links to the left/bottom face
      ffAdjQueue.front().second.first->FFp(ffAdjQueue.front().second.second) = ffAdjQueue.front().first.first;
      ffAdjQueue.front().second.first->FFi(ffAdjQueue.front().second.second) = ffAdjQueue.front().first.second;
      // pop from queue
      ffAdjQueue.pop();
    }

    // and apply fv adj changes
    while (!fvAdjQueue.empty()) {
      *fvAdjQueue.front().first = fvAdjQueue.front().second;
      fvAdjQueue.pop();
    }
  }

private:
  /**
   * @brief IsVertexAdjacentToAnyNonManifoldEdge checks if a vertex is adjacent to any non-manifold edge.
   * @param pos The starting position.
   * @return true if adjacent to non-manifold edges, false otherwise.
   */
  static bool IsVertexAdjacentToAnyNonManifoldEdge (const vcg::face::Pos<FaceType> &pos) {
    assert(!pos.IsNull());
    vcg::face::JumpingPos<FaceType> jmpPos;
    jmpPos.Set(pos.F(), pos.E(), pos.V());
    do {
      if (!jmpPos.IsManifold())
        return true;
      jmpPos.NextFE();
    } while (jmpPos != pos);
    return false;
  }

  /**
   * @brief CheckPolychordFindStartPosition checks if it's a collapsable polychord.
   * @param pos Input The starting position.
   * @param startPos Output the new starting position (in case of borders).
   * @param checkSing true if singularities on both sides are not allowed.
   * @return PC_SUCCESS if it's a collapsable polychord, otherwise the code for the cause (startPos is on it).
   */
  static PC_ResultCode CheckPolychordFindStartPosition (const vcg::face::Pos<FaceType> &pos,
                                                        vcg::face::Pos<FaceType> &startPos,
                                                        const bool checkSing = true) {
    assert(!pos.IsNull());
    int valence = 0;
    bool singSideA = false, singSideB = false;
    bool borderSideA = false, borderSideB = false;
    bool polyBorderFound = false;
    vcg::face::JumpingPos<FaceType> jmpPos;

    startPos = pos;
    do {
      // check if it is a quad
      if (startPos.F()->VN() != 4)
        return PC_NOTQUAD;
      // check manifoldness
      if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
        return PC_NOTMANIF;
      startPos.FlipV();
      if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
        return PC_NOTMANIF;
      startPos.FlipV();

      // check if side A is on border
      startPos.FlipE();
      if (startPos.IsBorder())
        borderSideA = true;
      startPos.FlipE();
      // check if side B is on border
      startPos.FlipV();
      startPos.FlipE();
      if (startPos.IsBorder())
        borderSideB = true;
      startPos.FlipE();
      startPos.FlipV();

      // check if singularities are not in both sides
      if (checkSing) {
        // compute the valence of the vertex on side B
        startPos.FlipV();
        valence = startPos.NumberOfIncidentVertices();
        // if the vertex is on border increment its valence by 1 (virtually connect it to a dummy vertex)
        jmpPos.Set(startPos.F(), startPos.E(), startPos.V());
        if (jmpPos.FindBorder())
          valence++;
        if (valence != 4)
          singSideB = true;
        // a 2-valence internl vertex cause a polychord to touch itself, producing non-2manifoldness
        // in that case, a 2-valence vertex is dealt as 2 singularities in both sides
        if (valence == 2 && !borderSideB)
          singSideA = true;
        // compute the valence of the vertex on side A
        startPos.FlipV();
        valence = startPos.NumberOfIncidentVertices();
        // if the vertex is on border increment its valence by 1 (virtually connect it to a dummy vertex)
        jmpPos.Set(startPos.F(), startPos.E(), startPos.V());
        if (jmpPos.FindBorder())
          valence++;
        if (valence != 4)
          singSideA = true;
        // a 2-valence internal vertex cause a polychord to touch itself, producing non-2manifoldness
        // in that case, a 2-valence vertex is dealt as 2 singularities in both sides
        if (valence == 2 && !borderSideA)
          singSideB = true;
      }

      // if the first border has been reached, go on the other direction to find the other border
      if (startPos != pos && startPos.IsBorder() && !polyBorderFound) {
        startPos = pos;
        startPos.FlipF();
        polyBorderFound = true;
      }

      // if the other border has been reached, return
      if (polyBorderFound && startPos.IsBorder())
        break;

      // go to the next edge
      startPos.FlipE();
      startPos.FlipV();
      startPos.FlipE();
      // check manifoldness
      if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
        return PC_NOTMANIF;
      startPos.FlipV();
      if (IsVertexAdjacentToAnyNonManifoldEdge(startPos))
        return PC_NOTMANIF;
      startPos.FlipV();
      // go to the next face
      startPos.FlipF();
    } while (startPos != pos);

    // polychord with singularities on both sides can not collapse
    if ((singSideA && singSideB) ||
        (singSideA && borderSideB) ||
        (singSideB && borderSideA))
      return PC_SINGBOTH;

    // polychords that are rings and have borders on both sides can not collapse
    if (!polyBorderFound && borderSideA && borderSideB)
      return PC_SINGBOTH;

    return PC_SUCCESS;
  }

  /**
   * @brief IsPolychordSelfIntersecting checks if the input polychord intersects itself.
   * @warning Don't call this function without being sure that it's a polychord
   * (i.e. call CheckPolychordFindStartPoint() before calling IsPolychordSelfIntersecting().
   * @param mesh The mesh used for getting the face index.
   * @param startPos The starting position.
   * @param chords The vector of chords.
   * @param mark The current mark, used to identify quads already visited.
   * @return true if it intersects itself, false otherwise.
   */
  static bool IsPolychordSelfIntersecting (const PolyMeshType &mesh,
                                           const vcg::face::Pos<FaceType> &startPos,
                                           const PC_Chords &chords,
                                           const unsigned long mark) {
    assert(!startPos.IsNull());
    vcg::face::Pos<FaceType> runPos = startPos;
    vcg::face::Pos<FaceType> tmpPos;
    std::pair<size_t, unsigned char> face_edge(std::numeric_limits<size_t>::max(), 0);
    do {
      assert(runPos.F()->VN() == 4);
      // check if we've already crossed this face
      face_edge.first = vcg::tri::Index(mesh, runPos.F());
      face_edge.second = (runPos.E()+1)%2;
      if (chords[face_edge].mark == mark)
        return true;
      // if this coord is adjacent to another coord of the same polychord
      // i.e., this polychord touches itself without intersecting
      // it might cause a wrong collapse, producing holes and non-2manifoldness
      tmpPos = runPos;
      tmpPos.FlipE();
      if (!tmpPos.IsBorder()) {
        tmpPos.FlipF();
        face_edge.first = vcg::tri::Index(mesh, tmpPos.F());
        face_edge.second = (tmpPos.E()+1)%2;
        if (chords[face_edge].mark == mark)
          return true;
      }
      tmpPos = runPos;
      tmpPos.FlipV();
      tmpPos.FlipE();
      if (!tmpPos.IsBorder()) {
        tmpPos.FlipF();
        face_edge.first = vcg::tri::Index(mesh, tmpPos.F());
        face_edge.second = (tmpPos.E()+1)%2;
        if (chords[face_edge].mark == mark)
          return true;
      }
      runPos.FlipE();
      runPos.FlipV();
      runPos.FlipE();
      runPos.FlipF();
    } while (runPos != startPos && !runPos.IsBorder());

    return false;
  }

  /**
   * @brief VisitPolychord updates the information of a polychord.
   * @param mesh The mesh used for getting the face index.
   * @param startPos The starting position.
   * @param chords The vector of chords.
   * @param mark The mark.
   * @param q The visiting type.
   */
  static void VisitPolychord (const PolyMeshType &mesh,
                              const vcg::face::Pos<FaceType> &startPos,
                              PC_Chords &chords,
                              const unsigned long mark,
                              const PC_ResultCode q) {
    assert(!startPos.IsNull());
    vcg::face::Pos<FaceType> tmpPos, runPos = startPos;
    std::pair<size_t, unsigned char> face_edge(std::numeric_limits<size_t>::max(), 0);

    if (runPos.F()->VN() != 4)  // non-quads are not visited
      return;

    // follow the sequence of quads
    do {
      // check manifoldness
      tmpPos = runPos;
      do {
        if (!tmpPos.IsManifold()) {
          // update current coord
          face_edge.first = vcg::tri::Index(mesh, tmpPos.F());
          face_edge.second = tmpPos.E()%2;
          chords.UpdateCoord(chords[face_edge], mark, q);
          face_edge.second = (tmpPos.E()+1)%2;
          chords.UpdateCoord(chords[face_edge], mark, q);
          return;
        }
        tmpPos.FlipV();
        tmpPos.FlipE();
      } while (tmpPos != runPos);

      // update current coord
      face_edge.first = vcg::tri::Index(mesh, runPos.F());
      face_edge.second = runPos.E()%2;
      chords.UpdateCoord(chords[face_edge], mark, q);
      // if the polychord has to collapse, i.e. q == PC_SUCCESS, also visit the orthogonal coord
      if (q == PC_SUCCESS) {
        face_edge.second = (runPos.E()+1)%2;
        chords.UpdateCoord(chords[face_edge], mark, q);
      }

      runPos.FlipE();
      runPos.FlipV();
      runPos.FlipE();
      runPos.FlipF();
    } while (runPos != startPos && !runPos.IsBorder() && runPos.F()->VN() == 4);
    assert(runPos == startPos || vcg::face::IsBorder(*startPos.F(),startPos.E())
           || runPos.F()->VN() != 4 || startPos.FFlip()->VN() != 4);
  }
};

}
}

#endif // POLYGON_Polychord_COLLAPSE_H
