#ifndef MLS_ADVANCE_H
#define MLS_ADVANCE_H

#include <iostream>
#include <list>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/flag.h>

namespace vcg {
  namespace tri {

/* An active edge on the advancing front.
 * belong to a triangle (v0,v1,v2)
 * v0, v1 the active edge
 * v2     internal vertex
*/
class FrontEdge {
 public:
  int v0, v1, v2;   //v0, v1 represent the FrontEdge, v2 the other vertex
                    //in the face this FrontEdge belongs to
  bool active; //keep tracks of wether it is in front or in deads

  //the loops in the front are mantained as a double linked list
  std::list<FrontEdge>::iterator next;
  std::list<FrontEdge>::iterator previous;

  FrontEdge() {}
  FrontEdge(int _v0, int _v1, int _v2):
             v0(_v0), v1(_v1), v2(_v2), active(true) {
               assert(v0 != v1 && v1 != v2 && v0 != v2);
  }

    bool operator==(const FrontEdge& f) const
    {
        return ((v0 == f.v0) && (v1 == f.v1) && (v2 == f.v2) );
    }
};

template <class MESH> class AdvancingFront {
 public:

  typedef typename MESH::VertexType     VertexType;
  typedef typename MESH::FaceType       FaceType;
  typedef typename MESH::FaceIterator       FaceIterator;
  typedef typename MESH::ScalarType     ScalarType;
  typedef typename MESH::VertexType::CoordType   Point3x;

  //class FrontEdgeLists
  //{

  //};


// protected:
  std::list<FrontEdge> front;
  std::list<FrontEdge> deads;
  std::vector<int> nb; //number of fronts a vertex is into,
                       //this is used for the Visited and Border flags
                       //but adding topology may not be needed anymore

 public:

  MESH &mesh;           //this structure will be filled by the algorithm

  AdvancingFront(MESH &_mesh): mesh(_mesh) {


    UpdateFlags<MESH>::FaceBorderFromNone(mesh);
    UpdateFlags<MESH>::VertexBorderFromFace(mesh);

    nb.clear();
    nb.resize(mesh.vert.size(), 0);

    CreateLoops();
  }
  virtual ~AdvancingFront() {}

  void BuildMesh(CallBackPos call = NULL, int interval = 512)
  {
    float finalfacesext = mesh.vert.size() * 2.0f;
    if(call) call(0, "Advancing front");
    while(1) {

      for(int i = 0; i < interval; i++) {
        if(!front.size() && !SeedFace()) return;
        AddFace();
        if(call)
        {
            float rap = float(mesh.face.size()) / finalfacesext;
            int perc = (int) (100.0f * rap);
            (*call)(perc,"Adding Faces");
        }
      }
    }
  }

protected:
  //Implement these functions in your subclass
  enum ListID {FRONT,DEADS};
  typedef std::pair< ListID,std::list<FrontEdge>::iterator > ResultIterator;
  virtual bool Seed(int &v0, int &v1, int &v2) = 0;
  // This function must find a vertex to be added to edge 'e'.
  // return -1 in case of failure
  virtual int Place(FrontEdge &e, ResultIterator &touch) = 0;

  //create the FrontEdge loops from seed faces
  void CreateLoops()
  {
    for(size_t i = 0; i < mesh.face.size(); i++)
    {
      FaceType &f = mesh.face[i];
      if(f.IsD()) continue;

      for(int k = 0; k < 3; k++) {
        if(f.IsB(k)) {
          addNewEdge(FrontEdge(tri::Index(mesh,f.V0(k)),tri::Index(mesh,f.V1(k)),tri::Index(mesh,f.V2(k))) );
          nb[tri::Index(mesh,f.V0(k))]++;
        }
      }
    }

    for(std::list<FrontEdge>::iterator s = front.begin(); s != front.end(); s++) {
      (*s).previous = front.end();
      (*s).next = front.end();
    }
    //now create loops:
    for(std::list<FrontEdge>::iterator s = front.begin(); s != front.end(); s++) {
      for(std::list<FrontEdge>::iterator j = front.begin(); j != front.end(); j++) {
        if(s == j) continue;
        if((*s).v1 != (*j).v0) continue;
        if((*j).previous != front.end()) continue;
        (*s).next = j;
        (*j).previous = s;
        break;
      }
    }
    for(std::list<FrontEdge>::iterator s = front.begin(); s != front.end(); s++) {
      assert((*s).next != front.end());
      assert((*s).previous != front.end());
    }
  }

  bool SeedFace() {
    int v[3];
    bool success = Seed(v[0], v[1], v[2]);
    if(!success) return false;

    nb.resize(mesh.vert.size(), 0);

     //create the border of the first face
    std::list<FrontEdge>::iterator e = front.end();
    std::list<FrontEdge>::iterator last = e;
    std::list<FrontEdge>::iterator first;

    for(int i = 0; i < 3; i++) {
      int v0 = v[i];
      int v1 = v[((i+1)%3)];
      int v2 = v[((i+2)%3)];

      mesh.vert[v0].SetB();
      nb[v[i]]++;

      e = front.insert(front.begin(), FrontEdge(v0, v1, v2));
      if(i != 0) {
        (*last).next = e;
        (*e).previous = last;
      } else
        first = e;

      last = e;
    }
    //connect last and first
    (*last).next = first;
    (*first).previous = last;

    AddFace(v[0], v[1], v[2]);
    return true;
  }

public:
  bool AddFace() {
    if(!front.size()) return false;

    std::list<FrontEdge>::iterator ei = front.begin();
    FrontEdge &current = *ei;
    FrontEdge &previous = *current.previous;
    FrontEdge &next = *current.next;

    int v0 = current.v0, v1 = current.v1;
    assert(nb[v0] < 10 && nb[v1] < 10);

    ResultIterator touch;
    touch.first = FRONT;
    touch.second = front.end();
    int v2 = Place(current, touch);

    if(v2 == -1) {
      KillEdge(ei);
      return false;
    }

    assert(v2 != v0 && v2 != v1);

    if ( ( (touch.first == FRONT) && (touch.second != front.end()) ) ||
         ( (touch.first == DEADS) && (touch.second != deads.end()) )    )

    {
      //check for orientation and manifoldness

      //touch == current.previous?
      if(v2 == previous.v0) {
        if(!CheckEdge(v2, v1)) {
          KillEdge(ei);
          return false;
        }
          /*touching previous FrontEdge  (we reuse previous)
                                    next
             ------->v2 -----> v1------>
                      \       /
                       \     /
               previous \   / current
                         \ /
                          v0           */

        Detach(v0);

        std::list<FrontEdge>::iterator up = addNewEdge(FrontEdge(v2, v1, v0));
        MoveFront(up);
        (*up).previous = previous.previous;
        (*up).next = current.next;
        (*previous.previous).next = up;
        next.previous = up;
        Erase(current.previous);
        Erase(ei);
        Glue(up);

      //touch == (*current.next).next
      } else if(v2 == next.v1) {
        if(!CheckEdge(v0, v2)) {
          KillEdge(ei);
          return false;
        }
        /*touching next FrontEdge  (we reuse next)
          previous
             ------->v0 -----> v2------>
                      \       /
                       \     /
                        \   / next
                         \ /
                          v1           */

        Detach(v1);
        std::list<FrontEdge>::iterator up = addNewEdge(FrontEdge(v0, v2, v1));
        MoveFront(up);
        (*up).previous = current.previous;
        (*up).next = (*current.next).next;
        previous.next = up;
        (*next.next).previous = up;
        Erase(current.next);
        Erase(ei);
        Glue(up);
      } else {
        if(!CheckEdge(v0, v2) || !CheckEdge(v2, v1)) {
          KillEdge(ei);
          return false;
        }
      //touching some loop: split (or merge it is local does not matter.
      //like this
      /*
                  left        right
                <--------v2-<------
                          /|\
                         /   \
                     up /     \ down
                       /       \
                      /         V
                 ----v0 - - - > v1---------
                        current                         */
        std::list<FrontEdge>::iterator left = touch.second;
        std::list<FrontEdge>::iterator right = (*touch.second).previous;

        //this would be a really bad join
        if(v1 == (*right).v0 || v0 == (*left).v1) {
          KillEdge(ei);
          return false;
        }

        nb[v2]++;

        std::list<FrontEdge>::iterator down = addNewEdge(FrontEdge(v2, v1, v0));
        std::list<FrontEdge>::iterator up = addNewEdge(FrontEdge(v0, v2, v1));

        (*right).next = down;
        (*down).previous = right;

        (*down).next = current.next;
        next.previous = down;

        (*left).previous = up;
        (*up).next = left;

        (*up).previous = current.previous;
        previous.next = up;
        Erase(ei);
      }


    }
    else if (((touch.first == FRONT) && (touch.second == front.end())) ||
             ((touch.first == DEADS) && (touch.second == deads.end()))    )
    {
//        assert(CheckEdge(v0, v2));
//        assert(CheckEdge(v2, v1));
        /*  adding a new vertex

                           v2
                          /|\
                         /   \
                     up /     \ down
                       /       \
                      /         V
                 ----v0 - - - > v1--------- */
        assert(!mesh.vert[v2].IsB()); //fatal error! a new point is already a border?
        nb[v2]++;
        mesh.vert[v2].SetB();

        std::list<FrontEdge>::iterator down = addNewEdge(FrontEdge(v2, v1, v0));
        std::list<FrontEdge>::iterator up = addNewEdge(FrontEdge(v0, v2, v1));

        (*down).previous = up;
        (*up).next = down;
        (*down).next = current.next;
        next.previous = down;
        (*up).previous = current.previous;
        previous.next = up;
        Erase(ei);
      }

      AddFace(v0, v2, v1);
      return false;
  }

protected:
  void AddFace(int v0, int v1, int v2) {
    assert(v0 < (int)mesh.vert.size() && v1 < (int)mesh.vert.size() && v2 < (int)mesh.vert.size());
    FaceIterator fi = vcg::tri::Allocator<MESH>::AddFaces(mesh,1);
    fi->ClearFlags();
    fi->V(0) = &mesh.vert[v0];
    fi->V(1) = &mesh.vert[v1];
    fi->V(2) = &mesh.vert[v2];
    ComputeNormalizedNormal(*fi);
    if(tri::HasVFAdjacency(mesh))
    {
      for(int j=0;j<3;++j)
      {
        (*fi).VFp(j) = (*fi).V(j)->VFp();
        (*fi).VFi(j) = (*fi).V(j)->VFi();
        (*fi).V(j)->VFp() = &(*fi);
        (*fi).V(j)->VFi() = j;
      }
    }
  }

  void AddVertex(VertexType &vertex) {
    VertexType *oldstart = NULL;
    if(mesh.vert.size()) oldstart = &*mesh.vert.begin();
    mesh.vert.push_back(vertex);
    mesh.vn++;
    VertexType *newstart = &*mesh.vert.begin();
    if(oldstart && oldstart != newstart) {
      for(int i = 0; i < mesh.face.size(); i++) {
        FaceType &face = mesh.face[i];
        for(int k = 0; k < 3; k++)
          face.V(k) = newstart + (face.V(k) - oldstart);
      }
    }
    nb.push_back(0);
  }

  // Given a possible new edge v0-v1
  // it checks that:
  // 1) the orientation is consistent (all the faces with vertex v0 and v1 have the edge in the opposite way)
  // 2) the edge appears at least once

  bool CheckEdge(int v0, int v1) {
    int tot = 0;
    VertexType *vv0 = &(mesh.vert[v0]);
    VertexType *vv1 = &(mesh.vert[v1]);
    if(tri::HasVFAdjacency(mesh))
    {
      face::VFIterator<FaceType> vfi(vv0);
      for (;!vfi.End();++vfi)
      {
        FaceType *f = vfi.F();
        for(int k = 0; k < 3; k++) {
          if(vv0 == f->V0(k) && vv1 == f->V1(k))  //orientation non constistent
             return false;
          else if(vv1 == f->V0(k) && vv0 == f->V1(k)) ++tot;
        }
      }
      return true;
    }
    for(int i = 0; i < (int)mesh.face.size(); i++) {
      FaceType &f = mesh.face[i];
      for(int k = 0; k < 3; k++) {
        if(vv0 == f.V0(k) && vv1 == f.V1(k))  //orientation non constistent
           return false;
        else if(vv1 == f.V0(k) && vv0 == f.V1(k)) ++tot;
      }
      if(tot >= 2) { //non manifold
        return false;
      }
    }
    return true;
  }
  //front management:

  //Add a new FrontEdge to the back of the queue
  std::list<FrontEdge>::iterator addNewEdge(FrontEdge e) {
    return front.insert(front.end(), e);
  }

  //move an Edge among the dead ones
  void KillEdge(std::list<FrontEdge>::iterator e)
  {
    if (e->active)
    {
        (*e).active = false;
        //std::list<FrontEdge>::iterator res = std::find(front.begin(),front.end(),e);
        FrontEdge tmp = *e;
        deads.splice(deads.end(), front, e);
        std::list<FrontEdge>::iterator newe = std::find(deads.begin(),deads.end(),tmp);
        tmp.previous->next = newe;
        tmp.next->previous = newe;
    }
  }

  void Erase(std::list<FrontEdge>::iterator e) {
    if((*e).active) front.erase(e);
    else deads.erase(e);
  }

  //move an FrontEdge to the back of the queue
  void MoveBack(std::list<FrontEdge>::iterator e) {
    front.splice(front.end(), front, e);
  }

  void MoveFront(std::list<FrontEdge>::iterator e) {
    front.splice(front.begin(), front, e);
  }

  //check if e can be sewed with one of oits neighbours
  bool Glue(std::list<FrontEdge>::iterator e) {
    return Glue((*e).previous, e) || Glue(e, (*e).next);
  }

  //Glue toghether a and b (where a.next = b
  bool Glue(std::list<FrontEdge>::iterator a, std::list<FrontEdge>::iterator b) {
    if((*a).v0 != (*b).v1) return false;

    std::list<FrontEdge>::iterator previous = (*a).previous;
    std::list<FrontEdge>::iterator next = (*b).next;
    (*previous).next = next;
    (*next).previous = previous;
    Detach((*a).v1);
    Detach((*a).v0);
    Erase(a);
    Erase(b);
    return true;
  }

  void Detach(int v) {
    assert(nb[v] > 0);
    if(--nb[v] == 0) {
      mesh.vert[v].ClearB();
    }
  }
};

template <class MESH> class AdvancingTest: public AdvancingFront<MESH> {
 public:
  typedef typename MESH::VertexType     VertexType;
  typedef typename MESH::VertexIterator VertexIterator;
  typedef typename MESH::FaceType       FaceType;
  typedef typename MESH::FaceIterator   FaceIterator;

  typedef typename MESH::ScalarType     ScalarType;
  typedef typename MESH::VertexType::CoordType   Point3x;

  AdvancingTest(MESH &_mesh): AdvancingFront<MESH>(_mesh) {}

  bool Seed(int &v0, int &v1, int &v2) {
    VertexType v[3];
    v[0].P() = Point3x(0, 0, 0);
    v[1].P() = Point3x(1, 0, 0);
    v[2].P() = Point3x(0, 1, 0);
    v[0].ClearFlags();
    v[1].ClearFlags();
    v[2].ClearFlags();

    v0 = this->mesh.vert.size();
    AddVertex(v[0]);
    v1 = this->mesh.vert.size();
    AddVertex(v[1]);
    v2 = this->mesh.vert.size();
    AddVertex(v[2]);
    return true;
  }

  int Place(FrontEdge &e, typename AdvancingFront<MESH>::ResultIterator &touch)
  {
     Point3f p[3];
     p[0] = this->mesh.vert[e.v0].P();
     p[1] = this->mesh.vert[e.v1].P();
     p[2] = this->mesh.vert[e.v2].P();
     Point3f point = p[0] + p[1] - p[2];

     int vn = this->mesh.vert.size();
     for(int i = 0; i < this->mesh.vert.size(); i++)
     {
       if((this->mesh.vert[i].P() - point).Norm() < 0.1)
       {
            vn = i;
            //find the border
            assert(this->mesh.vert[i].IsB());
            for(std::list<FrontEdge>::iterator k = this->front.begin(); k != this->front.end(); k++)
                if((*k).v0 == i)
                {
                                        touch.first = AdvancingFront<MESH>::FRONT;
                    touch.second = k;
                }

            for(std::list<FrontEdge>::iterator k = this->deads.begin(); k != this->deads.end(); k++)
                if((*k).v0 == i)
                    if((*k).v0 == i)
                    {
                                                touch.first = AdvancingFront<MESH>::FRONT;
                        touch.second = k;
                    }
            break;
       }
     }
     if(vn == this->mesh.vert.size()) {
       VertexType v;
       v.P() = point;
       v.ClearFlags();
       AddVertex(v);
     }
     return vn;
  }
};

}//namespace tri
}//namespace vcg

#endif
