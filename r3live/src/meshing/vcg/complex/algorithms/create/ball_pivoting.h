#ifndef BALL_PIVOTING_H
#define BALL_PIVOTING_H

#include "advancing_front.h"
#include <vcg/space/index/kdtree/kdtree.h>

#include <vcg/complex/algorithms/closest.h>

/* Ball pivoting algorithm:
   1) the vertices used in the new mesh are marked as visited
   2) the border vertices of the new mesh are marked as border
   3) the vector nb is used to keep track of the number of borders a vertex belongs to
   4) usedBit flag is used to select the points in the mesh already processed
  
*/   
namespace vcg {
  namespace tri {

template <class MESH> class BallPivoting: public AdvancingFront<MESH> {
 public:
  typedef typename MESH::VertexType     VertexType;
  typedef typename MESH::FaceType       FaceType;
  typedef typename MESH::ScalarType     ScalarType;
  typedef typename MESH::VertexIterator     VertexIterator;
  typedef typename MESH::VertexType::CoordType   Point3x;

  float radius;          //radius of the ball
  float min_edge;        //min length of an edge 
  float max_edge;        //min length of an edge 
  float max_angle;       //max angle between 2 faces (cos(angle) actually)  
  
 public:

	// if radius ==0 an autoguess for the ball pivoting radius is attempted 
	// otherwise the passed value (in absolute mesh units) is used.
	
  BallPivoting(MESH &_mesh, float _radius = 0, 
               float minr = 0.2, float angle = M_PI/2): 
                     
    AdvancingFront<MESH>(_mesh), radius(_radius), 
    min_edge(minr), max_edge(1.8), max_angle(cos(angle)),
    last_seed(-1) {
                  
    //compute bbox
    baricenter = Point3x(0, 0, 0);
    UpdateBounding<MESH>::Box(_mesh);
		for(VertexIterator vi=this->mesh.vert.begin();vi!=this->mesh.vert.end();++vi)
			if( !(*vi).IsD() )  baricenter += (*vi).P();
    
    baricenter /= this->mesh.vn;
		
		assert(this->mesh.vn > 3);
    if(radius == 0) // radius ==0 means that an auto guess should be attempted. 
      radius = sqrt((this->mesh.bbox.Diag()*this->mesh.bbox.Diag())/this->mesh.vn);
      

    min_edge *= radius;
    max_edge *= radius;    
      
    VertexConstDataWrapper<MESH> ww(this->mesh);
    tree = new KdTree<float>(ww);
    tree->setMaxNofNeighbors(16);
    
    usedBit = VertexType::NewBitFlag();
    UpdateFlags<MESH>::VertexClear(this->mesh,usedBit);
    UpdateFlags<MESH>::VertexClearV(this->mesh);    
    
    for(int i = 0; i < (int)this->mesh.face.size(); i++) {
      FaceType &f = this->mesh.face[i];
      if(f.IsD()) continue;
      for(int k = 0; k < 3; k++) {
        Mark(f.V(k));
      }
    }    
  }
  
  ~BallPivoting() {
    VertexType::DeleteBitFlag(usedBit);
    delete tree;
  }
  
  bool Seed(int &v0, int &v1, int &v2) {               
    //get a sphere of neighbours
    std::vector<VertexType *> targets;      
    while(++last_seed < (int)(this->mesh.vert.size())) {
      VertexType &seed = this->mesh.vert[last_seed];
      if(seed.IsD() || seed.IsUserBit(usedBit)) continue;                      
      
      seed.SetUserBit(usedBit);       

      tree->doQueryK(seed.P());
      int nn = tree->getNofFoundNeighbors();
      for(int i=0;i<nn;++i)
      {
        VertexType *vp = &this->mesh.vert[tree->getNeighborId(i)];
        if(Distance(seed.P(),vp->cP()) > 2*radius) continue;
        targets.push_back(vp);
      }

      int n = targets.size();
      if(n<3) continue;

      bool success = true;
      //find the closest visited or boundary
      for(int i = 0; i < n; i++) {         
        VertexType &v = *(targets[i]);
        if(v.IsV()) {        
          success = false;
          break;
        }
      }
      if(!success) continue;
      
      VertexType *vv0, *vv1, *vv2;
      success = false;
      //find a triplet that does not contains any other point
      Point3x center;
      for(int i = 0; i < n; i++) {
        vv0 = targets[i];
        if(vv0->IsD()) continue;
        Point3x &p0 = vv0->P();        
    
        for(int k = i+1; k < n; k++) {
          vv1 = targets[k];            
          if(vv1->IsD()) continue;
          Point3x &p1 = vv1->P();      
          float d2 = (p1 - p0).Norm();    
          if(d2 < min_edge || d2 > max_edge) continue;
    
          for(int j = k+1; j < n; j++) {
            vv2 = targets[j];
            if(vv2->IsD()) continue;
            Point3x &p2 = vv2->P();            
            float d1 = (p2 - p0).Norm();
            if(d1 < min_edge || d1 > max_edge) continue;            
            float d0 = (p2 - p1).Norm();
            if(d0 < min_edge || d0 > max_edge) continue;
            
            Point3x normal = (p1 - p0)^(p2 - p0);
            if(normal.dot(p0 - baricenter) < 0) continue;
/*            if(use_normals) {             
              if(normal * vv0->N() < 0) continue;
              if(normal * vv1->N() < 0) continue;
              if(normal * vv2->N() < 0) continue;
            }*/
            
            if(!FindSphere(p0, p1, p2, center)) {
              continue;
            }
                       
            //check no other point inside
            int t;
            for(t = 0; t < n; t++) {              
              if((center - targets[t]->P()).Norm() <= radius)
                break;              
            }
            if(t < n) {
              continue;                         
            }
            
            //check on the other side there is not a surface
            Point3x opposite = center + normal*(((center - p0).dot(normal))*2/normal.SquaredNorm());
            for(t = 0; t < n; t++) {
              VertexType &v = *(targets[t]);
              if((v.IsV()) && (opposite - v.P()).Norm() <= radius) 
                break;              
            }
            if(t < n) {
              continue;                         
            }
            success = true;
            i = k = j = n;
          }
        }
      }
      
      if(!success) { //see bad luck above
        continue;
      }
      Mark(vv0);
      Mark(vv1);
      Mark(vv2);            
      
      v0 = tri::Index(this->mesh,vv0);
      v1 = tri::Index(this->mesh,vv1);
      v2 = tri::Index(this->mesh,vv2);
      return true;      
    }
    return false;    
  }
  
  // Given an edge select a new vertex, mark as Visited and mark as usedBit all neighbours (less than min_edge)
  int Place(FrontEdge &edge, typename AdvancingFront<MESH>::ResultIterator &touch) {
    Point3x v0 = this->mesh.vert[edge.v0].P();
    Point3x v1 = this->mesh.vert[edge.v1].P();  
    Point3x v2 = this->mesh.vert[edge.v2].P();  
    /* TODO why using the face normals everything goes wrong? should be
       exactly the same................................................
    
       Point3x &normal = mesh.face[edge.face].N(); ?
    */
    
    Point3x normal = ((v1 - v0)^(v2 - v0)).Normalize();        
    Point3x middle = (v0 + v1)/2;    
    Point3x center;    

    if(!FindSphere(v0, v1, v2, center)) {
//      assert(0);
      return -1;
    }
    
    Point3x start_pivot = center - middle;          
    Point3x axis = (v1 - v0);
    
    ScalarType axis_len = axis.SquaredNorm();
    if(axis_len > 4*radius*radius) {
      return -1;
    }
    axis.Normalize();
    
    // r is the radius of the thorus of all possible spheres passing throug v0 and v1
    ScalarType r = sqrt(radius*radius - axis_len/4);

    
    tree->doQueryK(middle);
    int nn = tree->getNofFoundNeighbors();
    if(nn==0) return -1;
    
    VertexType *candidate = NULL;
    ScalarType min_angle = M_PI;
    //
    // Loop over all the nearest vertexes and choose the best one according the ball pivoting strategy.
    //
    for (int i = 0; i < nn; i++) {
      int vInd = tree->getNeighborId(i);
      VertexType *v = &this->mesh.vert[vInd];
      if(Distance(middle,v->cP()) > r + radius) continue;

      // this should always be true IsB => IsV , IsV => IsU
      if(v->IsB()) assert(v->IsV());
      if(v->IsV()) assert(v->IsUserBit(usedBit));
      
      
      if(v->IsUserBit(usedBit) && !(v->IsB())) continue;
      if(vInd == edge.v0 || vInd == edge.v1 || vInd == edge.v2) continue;
        
      Point3x p = this->mesh.vert[vInd].P();
                                
      /* Find the sphere through v0, p, v1 (store center on end_pivot */
      if(!FindSphere(v0, p, v1, center)) {
        continue;      
      }
      
      /* Angle between old center and new center */
      ScalarType alpha = OrientedAngleRad(start_pivot, center - middle, axis);
    
      /* adding a small bias to already chosen vertices.
         doesn't solve numerical problems, but helps. */
    //          if(this->mesh.vert[id].IsB()) alpha -= 0.001;
      
      /* Sometimes alpha might be little less then M_PI while it should be 0,
         by numerical errors: happens for example pivoting 
         on the diagonal of a square. */
      
/*      if(alpha > 2*M_PI - 0.8) {               
        // Angle between old center and new *point* 
        //TODO is this really overshooting? shouldbe enough to alpha -= 2*M_PI
        Point3x proj = p - axis * (axis * p - axis * middle);
        ScalarType beta = angle(start_pivot, proj - middle, axis);
      
        if(alpha > beta) alpha -= 2*M_PI;
      } */
      if(candidate == NULL || alpha < min_angle) {
        candidate = v;
        min_angle = alpha;
      } 
    }
    if(min_angle >= M_PI - 0.1) {
      return -1;
    }

    if(candidate == NULL) {
      return -1;
    }
    if(!candidate->IsB()) {
      assert((candidate->P() - v0).Norm() > min_edge);
      assert((candidate->P() - v1).Norm() > min_edge);    
    }
    
    int candidateIndex = tri::Index(this->mesh,candidate);
    assert(candidateIndex != edge.v0 && candidateIndex != edge.v1);
    
    Point3x newnormal = ((candidate->P() - v0)^(v1 - v0)).Normalize();
    if(normal.dot(newnormal) < max_angle || this->nb[candidateIndex] >= 2) {
      return -1;
    }

	//test if id is in some border (to return touch
	for(std::list<FrontEdge>::iterator k = this->front.begin(); k != this->front.end(); k++)
	{
	  if((*k).v0 == candidateIndex)
	  {
		touch.first = AdvancingFront<MESH>::FRONT;
		touch.second = k;
	  }
	}
	for(std::list<FrontEdge>::iterator k = this->deads.begin(); k != this->deads.end(); k++)
	{
	  if((*k).v0 == candidateIndex)
	  {
		touch.first = AdvancingFront<MESH>::DEADS;
		touch.second = k;
	  }
	}

    //mark vertices close to candidate
    Mark(candidate);
    return candidateIndex;
  }
  
 private:
  int last_seed;     //used for new seeds when front is empty
  int usedBit;       //use to detect if a vertex has been already processed.
  Point3x baricenter;//used for the first seed.  
  KdTree<float> *tree;

    
  /* returns the sphere touching p0, p1, p2 of radius r such that
     the normal of the face points toward the center of the sphere */
     
  bool FindSphere(const Point3x &p0, const Point3x &p1, const Point3x &p2, Point3x &center) {
    //we want p0 to be always the smallest one.
    Point3x p[3];
    
    if(p0 < p1 && p0 < p2) {
      p[0] = p0;
      p[1] = p1;
      p[2] = p2;          
    } else if(p1 < p0 && p1 < p2) {
      p[0] = p1;
      p[1] = p2;
      p[2] = p0;
    } else {
      p[0] = p2;
      p[1] = p0;
      p[2] = p1;
    }
    Point3x q1 = p[1] - p[0];
    Point3x q2 = p[2] - p[0];  
  
    Point3x up = q1^q2;
    ScalarType uplen = up.Norm();
  
    //the three points are aligned
    if(uplen < 0.001*q1.Norm()*q2.Norm()) {
      return false;
    }
    up /= uplen;
    
  
    ScalarType a11 = q1.dot(q1);
    ScalarType a12 = q1.dot(q2);
    ScalarType a22 = q2.dot(q2);
  
    ScalarType m = 4*(a11*a22 - a12*a12);
    ScalarType l1 = 2*(a11*a22 - a22*a12)/m;
    ScalarType l2 = 2*(a11*a22 - a12*a11)/m;
  
    center = q1*l1 + q2*l2;
    ScalarType circle_r = center.Norm();
    if(circle_r > radius) {
      return false; //need too big a sphere
    }
  
    ScalarType height = sqrt(radius*radius - circle_r*circle_r);
    center += p[0] + up*height;
  
    return true;
  }         
  
  /* compute angle from p to q, using axis for orientation */
  ScalarType OrientedAngleRad(Point3x p, Point3x q, Point3x &axis) {
    p.Normalize();
    q.Normalize();
    Point3x vec = p^q;
    ScalarType angle = acos(p.dot(q));
    if(vec.dot(axis) < 0) angle = -angle;
    if(angle < 0) angle += 2*M_PI;
    return angle;
  }          
  
  void Mark(VertexType *v) {
    tree->doQueryK(v->cP());
    int n = tree->getNofFoundNeighbors();
    for (int i = 0; i < n; i++) {
      VertexType *vp = &this->mesh.vert[tree->getNeighborId(i)];
      if(Distance(v->cP(),vp->cP())<min_edge)
        vp->SetUserBit(usedBit);
    }
    v->SetV();
  }
};

} //namespace tri
} //namespace vcg
#endif
