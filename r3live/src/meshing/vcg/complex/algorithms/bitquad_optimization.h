#ifndef _BITQUAD_OPTIMIZATION
#define _BITQUAD_OPTIMIZATION

namespace vcg{namespace tri{

template <class BQ>
class BitQuadOptimization{
  
typedef typename BQ::MeshType MeshType;
typedef typename BQ::Pos Pos;

typedef typename MeshType::ScalarType ScalarType;
typedef typename MeshType::CoordType CoordType;
typedef typename MeshType::FaceType FaceType;
typedef typename MeshType::FaceType* FaceTypeP;
typedef typename MeshType::VertexType VertexType;
typedef typename MeshType::FaceIterator FaceIterator;
typedef typename MeshType::VertexIterator VertexIterator;

//typedef BitQuad<MeshType> BQ; // static class to make basic quad operatins

public:

// helper function: mark a quadface, setting Q at 0, and neight at .75, 0.5...
static void MarkFace(FaceType* f, MeshType &m){
  
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
     fi->Q() = 1; 
  }
  
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) f->FFp(i)->FFp(j)->Q() = 0.75;
  }
  for (int i=0; i<3; i++) {
    f->FFp(i)->Q() = 0.50;
  }
  f->Q() = 0;
  
}

// helper function: mark a quadface, setting Q at 0, and neight at .75, 0.5...
static void MarkVertex(FaceType* f, int wedge, MeshType &m){

  VertexType *v = f->V(wedge);
  
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
     if (fi->V0(0)==v || fi->V1(0)==v ||fi->V2(0)==v ) fi->Q() = 0; 
     // else fi->Q() = 1; 
  }
  
}

static bool MarkSmallestEdge(MeshType &m, bool perform)
{
  ScalarType min = std::numeric_limits<ScalarType>::max();
  
  FaceType *fa=NULL; int w=0;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD())
  for (int k=0; k<3; k++) {
    FaceType *f=&*fi;
    
    if (f->IsF(k)) continue;
    if (f->FFp(k) == f ) continue; // skip borders
    
    ScalarType score;

    score = (f->P0(k) - f->P1(k)).Norm();
    if (score<min) {
      min=score; 
      fa = f;
      w = k;
    }
        
  }
  if (fa) {
    if (perform) {
      return BQ::CollapseEdge(*fa,w,m);
    } else {
      fa->Q()=0.0;
      fa->FFp(w)->Q()=0.0;
      return true;
    }
  }
  return false;
}

static ScalarType Importance(const CoordType  &p){
  //return ::proceduralImportance(p);
	return 1;
}

// returns: 0 if fail. 1 if edge. 2 if diag.
static int MarkSmallestEdgeOrDiag(MeshType &m, ScalarType edgeMult, bool perform, Pos* affected=NULL)
{
  ScalarType min = std::numeric_limits<ScalarType>::max();
  
  FaceType *fa=NULL; int w=0; bool counterDiag = false;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD())
  for (int k=0; k<3; k++) {
    FaceType *f=&*fi;
    
    if (f->FFp(k) >= f ) continue; // skip borders (==), and do it one per edge
    
    ScalarType score;

    score = (f->P0(k) - f->P1(k)).Norm();
    
    ScalarType imp = Importance( (f->P0(k) + f->P1(k))/2 );
    
    score /= imp;
    
    if (!f->IsF(k)) score*=edgeMult; // edges are supposed to be smaller!
    
    
    
    if (score<min) {
      min=score; 
      fa = f;
      w = k;
      counterDiag=false;
    }
    
    if (f->IsF(k)) { // for diag faces, test counterdiag too
      score = BQ::CounterDiag(f).Norm();
      score /= imp;

      if (score<min) {
        min=score; 
        fa = f;
        w = k;
        counterDiag=true;
      }
    }
    

        
  }
  if (fa) {
    if (perform) {
      if (fa->IsF(w)) {
        if (counterDiag) {
          if (BQ::CollapseCounterDiag(*fa, BQ::PosOnDiag(*fa,true), m , affected)) return 2;
        } else {
          if (BQ::CollapseDiag(*fa, BQ::PosOnDiag(*fa,false), m ,affected)) return 2;
        }
      } else {
        if  (BQ::CollapseEdge(*fa,w,m, affected)) return 1;
      }
    } else {
      fa->Q()=0.0;
      fa->FFp(w)->Q()=0.0;
      if (fa->IsF(w)) return 2; else return 1;
    }
  }
  return 0;
}


static void MarkSmallestDiag(MeshType &m)
{
  ScalarType min = std::numeric_limits<ScalarType>::max();
  
  FaceType *fa=NULL; 
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    FaceType *f=&*fi;
    
    ScalarType score;

    score = BQ::Diag(f).Norm();
    if (score<min) {
      min=score; 
      fa = f;
    }
    
    score = BQ::CounterDiag(f).Norm();
    if (score<min) {
      min=score; 
      fa = f;
    }
    
  }
  if (fa) {
    fa->Q()=0.0;
    fa->FFp(BQ::FauxIndex(fa))->Q()=0.0;
  }

}



static bool IdentifyAndCollapseSmallestDiag(MeshType &m){
  ScalarType min = std::numeric_limits<ScalarType>::max();
  
  FaceType *fa=NULL; bool flip;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    FaceType *f=&*fi;
    
    ScalarType score;
    
    score = BQ::Diag(f).Norm();
    if (score<min) {
      min=score; 
      fa = f;
      flip = false;
    }
    
    score = BQ::CounterDiag(f).Norm();
    if (score<min) {
      min=score; 
      fa = f;
      flip = true;
    }
    
  }
  if (!fa) return false;
  
  if (BQ::TestAndRemoveDoublet(*fa,0,m)) { return true; }
  if (BQ::TestAndRemoveDoublet(*fa,1,m)) { return true; }
  if (BQ::TestAndRemoveDoublet(*fa,2,m)) { return true; }
  int k = BQ::FauxIndex(fa);
  if (BQ::TestAndRemoveDoublet( *fa->FFp(k),(fa->FFi(k)+2)%3, m )) return true;

  if (flip) {
    if (!BQ::CheckFlipDiag(*fa) ) {
      // I can't collapse (why?)
      MarkFace(fa,m);
      return false;
    } else 
    BQ::CollapseCounterDiag(*fa, BQ::PosOnDiag(*fa,true), m );
  }
  else  {
    BQ::CollapseDiag(*fa, BQ::PosOnDiag(*fa,false), m );
  }
  return true;
}



/*
seeks and removes all doublets (a pair of quads sharing two consecutive edges)
by merging them into a single quad (thus removing one vertex and two tri faces)-
Returns number of removed Doublets
*/
static int RemoveDoublets(MeshType &m, Pos *p=NULL)
{
  int res=0;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    fi->Q()=1;
    for (int k=0; k<3; k++) {
      if ( BQ::IsDoublet(*fi,k) ){
        res++;
        BQ::RemoveDoublet(*fi,k,m,p);
        if (fi->IsD()) break; // break wedge circle, if face disappeard
        if (p) return res;
      }
    }
  }
  return res;
}

/*
marks (Quality=0) and approx. counts profitable vertex rotations
(vertex rotations which make edge shorter
*/
template <bool perform>
static int MarkVertexRotations(MeshType &m, Pos *affected=NULL)
{
  int res=0;
  for (VertexIterator vi = m.vert.begin();  vi!=m.vert.end(); vi++) if (!vi->IsD()) vi->ClearV();
  if (!perform)
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) fi->Q()=1.0;

  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    
    for (int k=0; k<3; k++) {
      if (fi->V(k)->IsV()) continue;
      if (BQ::TestVertexRotation(*fi,k)) {
        res++;
        fi->V(k)->SetV();
        if (!perform) {
          res++; MarkVertex(&*fi, k, m); //fi->Q()=0;
        }
        else {
          if (BQ::RotateVertex(*fi, k, m, affected)) res++; //fi->Q()=0;
          if (affected) return res; // uncomment for only one rotation
        }
      }
    }
  }
  return res;
}

// mark (and count) all edges that are worth rotating
// if perform == true, actually rotate them
template <bool perform>
static int MarkEdgeRotations(MeshType &m, Pos *p=NULL)
{
  int count = 0;
  
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) fi->Q()=1;
  
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    //if (count>0) break;
    for (int k=0; k<3; k++) {
      if (fi->IsF(k)) continue;
      if (fi->FFp(k)<= &*fi) continue; // only once per real (non faux) edge, and only for non border ones
      int best = BQ::TestEdgeRotation(*fi, k);
      if (perform) {
        if (best==+1) if (BQ::template RotateEdge< true>(*fi, k, m, p)) count++;
        if (best==-1) if (BQ::template RotateEdge<false>(*fi, k, m, p)) count++;
        if (p) if (count>0) return count;
      }
      else {
        if (best!=0) { fi->Q()=0; fi->FFp(k)->Q()=0; count++; }
      }
    }
  }

  return count;
}

/*
marks (Quality=0) and approx. counts doublets (a pair of quads sharing two consecutive edges)
*/
static int MarkDoublets(MeshType &m)
{
  int res=0;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    fi->Q()=1;
    for (int k=0; k<3; k++) {
      if ( BQ::IsDoublet(*fi,k) ){
        res++;
        if (fi->IsF((k+1)%3)) res++; // counts for a quad
        fi->Q()=0;
      }
    }
  }
  assert (res%2==0);
  return res/4; // return doublet pairs (approx, as a quad could be a part of many pairs)
}

/*
marks (Quality=0) and counts singlets (vertex B in an A-B-A-C quad)
*/
static int MarkSinglets(MeshType &m)
{
  int res=0;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    fi->Q()=1;
    for (int k=0; k<3; k++) {
      if ( BQ::IsSinglet(*fi,k) ){
        res++;
        fi->Q()=0;
      }
    }
  }
  assert (res%2==0);
  return res/2; // return number of  singlet pairs
}

/*
deletes singlets, reutrns number of
*/
static int RemoveSinglets(MeshType &m, Pos *p=NULL)
{
  int res=0;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    for (int k=0; k<3; k++) {
      if ( BQ::IsSinglet(*fi,k) ){
        res++;
        BQ::RemoveSinglet(*fi,k,m, p);
        if (p) return res;
        break;
      }
    }
  }
  return res; // return singlet pairs (approx, as a quad could be a part of many pairs)
}


/* returns average quad quality, and assigns it to triangle quality
*/
static ScalarType MeasureQuality(MeshType &m)
{
  assert(MeshType::HasPerFaceFlags());
  ScalarType res = 0;
  int div = 0;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    if (fi->IsAnyF()) {
      
      ScalarType q = BQ::quadQuality( &*fi, BQ::FauxIndex(&*fi) );
      
      if (MeshType::HasPerFaceQuality()) fi->Q() = q;
      res += q;
      div++;
    }
  }
  if (!div) return 0; else return res / div;
}

};
}} // end namespace vcg::tri

#endif
