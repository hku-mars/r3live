#ifndef VCG_BITQUAD_SUPPORT
#define VCG_BITQUAD_SUPPORT
#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/space/planar_polygon_tessellation.h>
#include <vcg/complex/algorithms/update/quality.h>

/** BIT-QUAD creation support:
    a few basic operations to work with bit-quads simplices
    (quads defined by faux edges over a tri mesh backbone)


   [ basic operations: ]

   bool IsDoublet(const FaceType& f, int wedge)
   void RemoveDoublet(FaceType &f, int wedge, MeshType& m)
    - identifies and removed "Doublets" (pair of quads sharing two consecutive edges)

   bool IsSinglet(const FaceType& f, int wedge)
   void RemoveSinglet(FaceType &f, int wedge, MeshType& m)

   void FlipDiag(FaceType &f)
    - rotates the faux edge of a quad (quad only change internally)

   bool RotateEdge(FaceType& f, int w0a);
    - rotate a quad edge (clockwise or counterclockwise, specified via template)

   bool RotateVertex(FaceType &f, int w0)
    - rotate around a quad vertex ("wind-mill" operation)

   void CollapseDiag(FaceType &f, ... p , MeshType& m)
    - collapses a quad on its diagonal.
    - p identifies the pos of collapsed point
      (as either the parametric pos on the diagonal, or a fresh coordtype)


   [ helper functions: ]

   ScalarType quadQuality( ... );
    - returns the quality for a given quad
    - (should be made into a template parameter for methods using it)
    - currently measures how squared each angle is

   int FauxIndex(const FaceType* f);
    - returns index of the only faux edge of a quad (otherwise, assert)

   int CountBitPolygonInternalValency(const FaceType& f, int wedge)
    - returns valency of vertex in terms  of polygons (quads, tris...)


*/

// these should become a parameter in the corresponding class
#define DELETE_VERTICES 1
// Reason not to delete vertices:
// if not vertex TwoManyfold, the vertex could still be used elsewhere...

// if one, use length to determine if rotations are profitable
// if zero, maximize conformal quality
#define LENGTH_CRITERION 1

namespace vcg{namespace tri{

/* simple geometric-interpolation mono-function class used
as a default template parameter to BitQuad class */
template <class VertexType>
class GeometricInterpolator{
public:
  typedef typename VertexType::ScalarType ScalarType;
  static void Apply( const VertexType &a,  const VertexType &b, ScalarType t, VertexType &res){
    /*assert (&a != &b);*/
    res.P() = a.cP()*(1-t) + b.cP()*(t);
    if (a.IsB()||b.IsB()) res.SetB();
  }
};

template <
  // first template parameter: the tri mesh (with face-edges flagged)
  class _MeshType,
  // second template parameter: used to define interpolations between points
  class Interpolator = GeometricInterpolator<typename _MeshType::VertexType>
>
class BitQuad{
public:

typedef _MeshType MeshType;
typedef typename MeshType::ScalarType ScalarType;
typedef typename MeshType::CoordType CoordType;
typedef typename MeshType::FaceType FaceType;
typedef typename MeshType::FaceType* FaceTypeP;
typedef typename MeshType::VertexType VertexType;
typedef typename MeshType::FaceIterator FaceIterator;
typedef typename MeshType::VertexIterator VertexIterator;
typedef typename MeshType::VertexPointer VertexPointer;

class Pos{
  FaceType *f;
  int e;
public:
  enum{ PAIR, AROUND , NOTHING } mode;
  FaceType* &F(){return f;}
  FaceType* F() const {return f;}
  VertexType* V() {return f->V(e);}
  const VertexType* cV() const {return f->V(e);}
  int& E(){return e;}
  int E() const {return e;}


  Pos(){ f=NULL; e=0; mode=AROUND;}

  Pos(FaceType* _f, int _e){f=_f; e=_e;}
  Pos NextE()const {return Pos(f, (e+1)%3); }
  Pos PrevE(){return Pos(f, (e+2)%3); }
  bool IsF(){return f->IsF(e);}
  Pos FlipF(){return Pos(f->FFp(e), f->FFi(e)); }

};



static void MarkFaceF(FaceType *f){
  f->V(0)->SetS();
  f->V(1)->SetS();
  f->V(2)->SetS();
  int i=FauxIndex(f);
  f->FFp( i )->V2( f->FFi(i) )->SetS();
  f->V(0)->SetV();
  f->V(1)->SetV();
  f->V(2)->SetV();
  f->FFp( i )->V2( f->FFi(i) )->SetV();
}


template <bool verse>
static bool RotateEdge(FaceType& f, int w0a, MeshType &m, Pos *affected=NULL){
  FaceType *fa = &f;
  assert(! fa->IsF(w0a) );

  VertexType *v0, *v1;
  v0= fa->V0(w0a);
  v1= fa->V1(w0a);

//  int w1a = (w0a+1)%3;
  int w2a = (w0a+2)%3;

  FaceType *fb = fa->FFp(w0a);

  MarkFaceF(fa);
  MarkFaceF(fb);

  int w0b = fa->FFi(w0a);
//  int w1b = (w0b+1)%3;
  int w2b = (w0b+2)%3;

  if (fa->IsF(w2a) == verse) {
    if (!CheckFlipDiag(*fa)) return false;
    FlipDiag(*fa);
    // hack: recover edge index, so that (f, w0a) identifies the same edge as before
    fa = fb->FFp(w0b);
    w0a = fb->FFi(w0b);
  }

  if (fb->IsF(w2b) == verse) {
    if (!CheckFlipDiag(*fb)) return false;
    FlipDiag(*fb);
  }

  if (!CheckFlipEdge(*fa,w0a)) return false;
  FlipEdge(*fa,w0a,m);
  if (affected) {
    affected->F() = fa;
    affected->E() = (FauxIndex(fa)+2)%3;
    affected->mode = Pos::PAIR;
  }
  return true;
}

/* small helper function which returns the index of the only
   faux index, assuming there is exactly one (asserts out otherwise)
*/
static int FauxIndex(const FaceType* f){
  if (f->IsF(0)) return 0;
  if (f->IsF(1)) return 1;
  assert(f->IsF(2));
  return 2;
}

// rotates the diagonal of a quad
static void FlipDiag(FaceType &f){
  int faux = FauxIndex(&f);
  FaceType* fa = &f;
  FaceType* fb = f.FFp(faux);
  vcg::face::FlipEdge(f, faux);
  // ripristinate faux flags
  fb->ClearAllF();
  fa->ClearAllF();
  for (int k=0; k<3; k++) {
    if (fa->FFp(k) == fb) fa->SetF(k);
    if (fb->FFp(k) == fa) fb->SetF(k);
  }
}


// given a vertex (i.e. a face and a wedge),
// this function tells us how the totale edge length around a vertex would change
// if that vertex is rotated
static ScalarType EdgeLenghtVariationIfVertexRotated(const FaceType &f, int w0)
{
  assert(!f.IsD());

  ScalarType
    before=0, // sum of quad edges (originating from v)
    after=0;  // sum of quad diag (orginating from v)
  int guard = 0;

  // rotate arond vertex
  const FaceType* pf = &f;
  int pi = w0;
  int n = 0; // vertex valency
  int na = 0;
  do {
    ScalarType triEdge = (pf->P0(pi) - pf->P1(pi) ).Norm();
    if (pf->IsF(pi)) { after += triEdge; na++;}
    else { before+= triEdge; n++; }
    if ( pf->IsF((pi+1)%3)) { after += CounterDiag( pf ).Norm(); na++; }

    const FaceType *t = pf;
    t = pf->FFp( pi );
    if (pf == t ) return std::numeric_limits<ScalarType>::max(); // it's a mesh border! flee!
    pi = pf->cFFi( pi );
    pi = (pi+1)%3; // FaceType::Next( pf->FFi( pi ) );
    pf = t;
    guard++;
    assert(guard<100);
  } while (pf != &f);
  assert (na == n);
  return (after-before);
}

// given a vertex (i.e. a face and a wedge),
// this function tells us how the totale edge length around a vertex would change
// if that vertex is rotated
static ScalarType QuadQualityVariationIfVertexRotated(const FaceType &f, int w0)
{
  assert(!f.IsD());

  ScalarType
    before=0, // sum of quad quality around v
    after=0;  // same after the collapse
  int guard = 0;

  // rotate arond vertex
  const FaceType* pf = &f;
  int pi = w0;
  std::vector<const VertexType *> s; // 1 star around v
  do {
    // ScalarType triEdge = (pf->P0(pi) - pf->P1(pi) ).Norm();
    if (!pf->IsF(pi)) {
      if ( pf->IsF((pi+1)%3)) {
        s.push_back(pf->cFFp((pi+1)%3)->V2( pf->cFFi((pi+1)%3) ));
      } else {
        s.push_back( pf->V2(pi) );
      }

      s.push_back( pf->V1(pi) );
    }

    const FaceType *t = pf;
    t = pf->FFp( pi );
    if (pf == t ) return std::numeric_limits<ScalarType>::max(); // it's a mesh border! flee!
    pi = pf->cFFi( pi );
    pi = (pi+1)%3; // FaceType::Next( pf->FFi( pi ) );
    pf = t;
    guard++;
    assert(guard<100);
  } while (pf != &f);

  assert(s.size()%2==0);
  int N = s.size();
  for (int i=0; i<N; i+=2) {
    int h = (i+N-1)%N;
    int j = (i  +1)%N;
    int k = (i  +2)%N;
    before+=   quadQuality( s[i]->P(),s[j]->P(),s[k]->P(),f.P(w0) );
    after+=quadQuality( s[h]->P(),s[i]->P(),s[j]->P(),f.P(w0) );
  }

  return (after-before);
}

/*
  const FaceType* pf = &f;
  int pi = wedge;
  int res = 0, guard=0;
  do {
    if (!pf->IsAnyF()) return false; // there's a triangle!
    if (!pf->IsF(pi)) res++;
    const FaceType *t = pf;
    t = pf->FFp( pi );
    if (pf == t ) return false;
    pi = pf->cFFi( pi );
    pi = (pi+1)%3; // FaceType::Next( pf->FFi( pi ) );
    pf = t;
    assert(guard++<100);
  } while (pf != &f);
*/

// given a vertex (i.e. a face and a wedge),
// this function tells us if it should be rotated or not
// (currently, we should iff it is shortened)
static bool TestVertexRotation(const FaceType &f, int w0)
{
  assert(!f.IsD());

#if (LENGTH_CRITERION)
  // rotate vertex IFF this way edges become shorter:
  return EdgeLenghtVariationIfVertexRotated(f,w0)<0;
#else
  // rotate vertex IFF overall Quality increase
#endif
  return QuadQualityVariationIfVertexRotated(f,w0)<0;
}


static bool RotateVertex(FaceType &f, int w0, MeshType &/*m*/, Pos *affected=NULL)
{

//  int guard = 0;

  FaceType* pf = &f;
  int pi = w0;

  if (pf->IsF((pi+2) % 3)) {
      pi = (pi+2)%3;
      // do one step back
      int tmp = pf->FFi(pi); pf = pf->FFp(pi); pi = tmp;  // flipF
  }

  const FaceType* stopA = pf;
  const FaceType* stopB = pf->FFp(FauxIndex(pf));

  // rotate around vertex, flipping diagonals if necessary,
  do {
    bool mustFlip;
    if (pf->IsF(pi)) {
      // if next edge is faux, move on other side of quad
      int tmp = (pf->FFi(pi)+1)%3; pf = pf->FFp(pi); pi = tmp;  // flipF
      mustFlip = false;
    }
    else {
      mustFlip = true;
    }

    FaceType *lastF = pf;

    int tmp = (pf->FFi(pi)+1)%3; pf = pf->FFp(pi); pi = tmp;  // flipF

    if (mustFlip) {
      if (!CheckFlipDiag(*lastF)) return false; // cannot flip??
      FlipDiag(*lastF);
    }
    MarkFaceF(pf);
  } while (pf != stopA && pf!= stopB);

  // last pass: rotate arund vertex again, changing faux status
  stopA=pf;
  do {
    int j = pi;
    if (pf->IsF(j))
      { pf->ClearF(j); IncreaseValency(pf->V1(j));  }
    else
      { pf->SetF(j); DecreaseValencySimple(pf->V1(j),1); }

    j = (j+2)%3;
    if (pf->IsF(j)) pf->ClearF(j); else pf->SetF(j);
    int tmp = (pf->FFi(pi)+1)%3; pf = pf->FFp(pi); pi = tmp;  // flipF flipV
  } while (pf != stopA );

  if (affected) {
    affected->F() = pf;
    affected->E()=pi;
  }
  return true;
}





// flips the faux edge of a quad
static void FlipEdge(FaceType &f, int k, MeshType &m){
  assert(!f.IsF(k));
  FaceType* fa = &f;
  FaceType* fb = f.FFp(k);
  assert(fa!=fb); // else, rotating a border edge

  // backup prev other-quads-halves
  FaceType* fa2 = fa->FFp( FauxIndex(fa) );
  FaceType* fb2 = fb->FFp( FauxIndex(fb) );

  IncreaseValency( fa->V2(k) );
  IncreaseValency( fb->V2(f.FFi(k)) );
  //DecreaseValency( fa->V0(k) );
  //DecreaseValency( fa->V1(k) );
  DecreaseValency(fa, k ,m);
  DecreaseValency(fa,(k+1)%3,m );


  vcg::face::FlipEdge(*fa, k);

  // ripristinate faux flags
  fb->ClearAllF();
  fa->ClearAllF();
  for (int k=0; k<3; k++) {
    //if (fa->FFp(k) == fa2) fa->SetF(k);
    //if (fb->FFp(k) == fb2) fb->SetF(k);
    if (fa->FFp(k)->IsF( fa->FFi(k) )) fa->SetF(k);
    if (fb->FFp(k)->IsF( fb->FFi(k) )) fb->SetF(k);
  }

}

// check if a quad diagonal can be topologically flipped
static bool CheckFlipDiag(FaceType &f){
  return (vcg::face::CheckFlipEdge(f, FauxIndex(&f) ) );
}

// given a face (part of a quad), returns its diagonal
static CoordType Diag(const FaceType* f){
  int i = FauxIndex(f);
  return f->P1( i ) - f->P0( i );
}


// given a face (part of a quad), returns other diagonal
static CoordType CounterDiag(const FaceType* f){
  int i = FauxIndex(f);
  return f->cP2( i ) - f->cFFp( i )->cP2(f->cFFi(i) ) ;
}

/* helper function:
   collapses a single face along its faux edge.
   Updates FF adj of other edges. */
static void _CollapseDiagHalf(FaceType &f, int faux, MeshType& /*m*/)
{
  int faux1 = (faux+1)%3;
  int faux2 = (faux+2)%3;

  FaceType* fA = f.FFp( faux1 );
  FaceType* fB = f.FFp( faux2 );

  MarkFaceF(fA);
  MarkFaceF(fB);

  int iA = f.FFi( faux1 );
  int iB = f.FFi( faux2 );

  if (fA==&f && fB==&f) {
    // both non-faux edges are borders: tri-face disappears, just remove the vertex
    //if (DELETE_VERTICES)
    //if (GetValency(f.V(faux2))==0) Allocator<MeshType>::DeleteVertex(m,*(f.V(faux2)));
  } else {
    if (fA==&f) {
      fB->FFp(iB) = fB;  fB->FFi(iB) = iB;
    } else {
      fB->FFp(iB) = fA;  fB->FFi(iB) = iA;
    }

    if (fB==&f) {
      fA->FFp(iA) = fA;  fA->FFi(iA) = iA;
    } else {
      fA->FFp(iA) = fB;  fA->FFi(iA) = iB;
    }
  }


  //DecreaseValency(&f,faux2,m); // update valency
  //Allocator<MeshType>::DeleteFace(m,f);

}

static void RemoveDoublet(FaceType &f, int wedge, MeshType& m, Pos* affected=NULL){
  if (f.IsF((wedge+1)%3) ) {
    VertexType *v = f.V(wedge);
    FlipDiag(f);
    // quick hack: recover wedge index after flip
    if (f.V(0)==v) wedge = 0;
    else if (f.V(1)==v) wedge = 1;
    else {
      assert(f.V(2)==v);
      wedge = 2;
    }
  }
  ScalarType k=(f.IsF(wedge))?1:0;
  CollapseDiag(f, k, m, affected);
  VertexType *v = f.V(wedge);
}

static void RemoveSinglet(FaceType &f, int wedge, MeshType& m, Pos* affected=NULL){
  if (affected) affected->mode = Pos::NOTHING; // singlets leave nothing to update behind

  if (f.V(wedge)->IsB()) return; // hack: lets detect

  FaceType *fa, *fb; // these will die
  FaceType *fc, *fd; // their former neight
  fa = & f;
  fb = fa->FFp(wedge);
  int wa0 = wedge;
  int wa1 = (wa0+1)%3 ;
  int wa2 = (wa0+2)%3 ;
  int wb0 = (fa->FFi(wa0)+1)%3;
  int wb1 = (wb0+1)%3 ;
//  int wb2 = (wb0+2)%3 ;
  assert (fb == fa->FFp( wa2 ) ); // otherwise, not a singlet

  // valency decrease
  DecreaseValency(fa, wa1, m);
  DecreaseValency(fa, wa2, m);
  if (fa->IsF(wa0)) {
    DecreaseValency(fa,wa2,m); // double decrease of valency on wa2
  } else {
    DecreaseValency(fa,wa1,m); // double decrease of valency on wa1
  }

  // no need to MarkFaceF !

  fc = fa->FFp(wa1);
  fd = fb->FFp(wb1);
  int wc = fa->FFi(wa1);
  int wd = fb->FFi(wb1);
  fc->FFp(wc) = fd;
  fc->FFi(wc) = wd;
  fd->FFp(wd) = fc;
  fd->FFi(wd) = wc;
  // faux status of survivors: unchanged
  assert( ! ( fc->IsF( wc) ) );
  assert( ! ( fd->IsF( wd) ) );

  Allocator<MeshType>::DeleteFace( m,*fa );
  Allocator<MeshType>::DeleteFace( m,*fb );

  DecreaseValency(fa,wedge,m );
  //if (DELETE_VERTICES)
  //if (GetValency(fa->V(wedge))==0) Allocator<MeshType>::DeleteVertex( m,*fa->V(wedge) );
}


static bool TestAndRemoveDoublet(FaceType &f, int wedge, MeshType& m){
  if (IsDoublet(f,wedge)) {
     RemoveDoublet(f,wedge,m);
     return true;
  }
  return false;
}

static bool TestAndRemoveSinglet(FaceType &f, int wedge, MeshType& m){
  if (IsSinglet(f,wedge)) {
     RemoveSinglet(f,wedge,m);
     return true;
  }
  return false;
}

// given a face and a wedge, counts its valency in terms of quads (and triangles)
// uses only FF, assumes twomanyfold
// returns -1 if border
static int CountBitPolygonInternalValency(const FaceType& f, int wedge){
  const FaceType* pf = &f;
  int pi = wedge;
  int res = 0;
  do {
    if (!pf->IsF(pi)) res++;
    const FaceType *t = pf;
    t = pf->FFp( pi );
    if (pf == t ) return -1;
    pi = (pi+1)%3; // FaceType::Next( pf->FFi( pi ) );
    pf = t;
  } while (pf != &f);
  return res;
}

// given a face and a wedge, returns if it host a doubet
// assumes tri and quad only. uses FF topology only.
static bool IsDoubletFF(const FaceType& f, int wedge){
  const FaceType* pf = &f;
  int pi = wedge;
  int res = 0, guard=0;
  do {
    if (!pf->IsAnyF()) return false; // there's a triangle!
    if (!pf->IsF(pi)) res++;
    const FaceType *t = pf;
    t = pf->FFp( pi );
    if (pf == t ) return false;
    pi = pf->cFFi( pi );
    pi = (pi+1)%3; // FaceType::Next( pf->FFi( pi ) );
    pf = t;
    guard++;
    assert(guard<100);
  } while (pf != &f);
  return (res == 2);
}

// version that uses vertex valency
static bool IsDoublet(const FaceType& f, int wedge){
  return (GetValency( f.V(wedge)) == 2) && (!f.V(wedge)->IsB() ) ;
}

static bool IsDoubletOrSinglet(const FaceType& f, int wedge){
  return (GetValency( f.V(wedge)) <= 2) && (!f.V(wedge)->IsB() ) ;
}

static bool RemoveDoubletOrSinglet(FaceType& f, int wedge, MeshType& m, Pos* affected=NULL){
 if (GetValency( f.V(wedge)) == 2) { RemoveDoublet(f,wedge,m,affected) ; return true; }
 assert (GetValency( f.V(wedge)) == 1) ;
 RemoveSinglet(f,wedge,m,affected) ;
 return true;
}

// given a face and a wedge, returns if it host a singlets
// assumes tri and quad only. uses FF topology only.
static bool IsSingletFF(const FaceType& f, int wedge){
  const FaceType* pf = &f;
  int pi = wedge;
  int res = 0, guard=0;
  do {
    if (!pf->IsAnyF()) return false; // there's a triangle!
    if (!pf->IsF(pi)) res++;
    const FaceType *t = pf;
    t = pf->FFp( pi );
    if (pf == t ) return false;
    pi = pf->cFFi( pi );
    pi = (pi+1)%3; // FaceType::Next( pf->FFi( pi ) );
    pf = t;
    guard++;
    assert(guard<100);
  } while (pf != &f);
  return (res == 1);
}

// version that uses vertex valency
static bool IsSinglet(const FaceType& f, int wedge){
  return (GetValency( f.cV(wedge) ) == 1) && (!f.cV(wedge)->IsB() ) ;
}

static bool CollapseEdgeDirect(FaceType &f, int w0, MeshType& m){
  FaceType * f0 = &f;

  assert( !f0->IsF(w0) );

  VertexType *v0, *v1;
  v0 = f0->V0(w0);
  v1 = f0->V1(w0);

  if (!RotateVertex(*f0,w0,m)) return false;

  // quick hack: recover original wedge
  if      (f0->V(0) == v0) w0 = 0;
  else if (f0->V(1) == v0) w0 = 1;
  else if (f0->V(2) == v0) w0 = 2;
  else assert(0);

  assert( f0->V1(w0) == v1 );
  assert( f0->IsF(w0) );

  return CollapseDiag(*f0,PosOnDiag(*f0,false), m);
}

// collapses an edge. Optional output pos can be iterated around to find affected faces
static bool CollapseEdge(FaceType &f, int w0, MeshType& m, Pos *affected=NULL){
  FaceTypeP f0 = &f;
  assert(!f0->IsF(w0)); // don't use this method to collapse diag.

  if (IsDoubletOrSinglet(f,w0)) return false; //{ RemoveDoubletOrSinglet(f,w0,m, affected); return true;}
  if (IsDoubletOrSinglet(f,(w0+1)%3)) return false; //{ RemoveDoubletOrSinglet(f,(w0+1)%3,m, affected); return true;}

  if (affected) {
    int w1 = 3-w0-FauxIndex(f0); // the edge whihc is not the collapsed one nor the faux
    affected->F() = f0->FFp(w1);
    affected->E() = (f0->FFi(w1)+2+w1-FauxIndex(f0))%3;
  }

  FaceTypeP f1 = f0->FFp(w0);
  int w1 = f0->FFi(w0);

  assert(f0!=f1); // can't collapse border edges!

  // choose: rotate around V0 or around V1?
  if (
    EdgeLenghtVariationIfVertexRotated(*f0,w0)
    <
    EdgeLenghtVariationIfVertexRotated(*f1,w1)
  )    return CollapseEdgeDirect(*f0,w0,m);
  else return CollapseEdgeDirect(*f1,w1,m);
}



/** collapses a quad diagonal a-b
  forming the new vertex in between the two old vertices.
   if k == 0, new vertex is in a
   if k == 1, new vertex is in b
   if k == 0.5, new vertex in the middle, etc
*/
static bool CollapseCounterDiag(FaceType &f, ScalarType interpol, MeshType& m, Pos* affected=NULL){
  if (!CheckFlipDiag(f)) return false;
  FlipDiag(f);
  return CollapseDiag(f,interpol,m,affected);
}

// rotates around vertex
class Iterator{
private:
  typedef typename face::Pos<FaceType> FPos;
  Pos  start, cur;
  bool over;
public:
  Iterator(Pos& pos){
    if (pos.mode==Pos::NOTHING) {over = true; return; }
    start = pos; //FPos(pos.F(), pos.E());
    if (start.F()->IsD()) { over = true; return;}
    assert(!start.F()->IsD());
    if (pos.mode==Pos::AROUND) {
      if (start.F()->IsF((start.E()+2)%3))
      {
          int i = start.F()->FFi( start.E() );
            start.F() = start.F()->FFp( start.E() );
            start.E() = (i+1)%3;
      }
    }
      cur=start;
    over = false;
  }
  bool End() const {
    return over;
  }
  void operator ++ () {
    if (start.mode==Pos::PAIR)  {
      if (cur.F()!=start.F()) over=true;
      int i = (cur.E()+2)%3;
      cur.E() = (cur.F()->FFi( i )+1)%3;
      cur.F() = cur.F()->FFp( i );
    } else {
      if (cur.F()->IsF(cur.E())) {
        // jump over faux diag
        int i = cur.F()->FFi( cur.E() );
        cur.F() = cur.F()->FFp( cur.E() );
        cur.E() = (i+1)%3;
      }
      // jump over real edge
      FaceType *f =cur.F()->FFp( cur.E() );
      if (f==cur.F()) over=true; // border found
      cur.E() = (cur.F()->FFi( cur.E() ) +1 )%3;
      cur.F() = f;
      if (cur.F()==start.F()) over=true;
    }
  }

  Pos GetPos(){
    return cur;
  }
};

static bool CollapseDiag(FaceType &f, ScalarType interpol, MeshType& m, Pos* affected=NULL){

  FaceType* fa = &f; // fa lives
  int fauxa = FauxIndex(fa);

  //if (IsDoubletOrSinglet(f,fauxa)) { RemoveDoubletOrSinglet(f,fauxa,m, affected); return true;}
//  if (IsDoubletOrSinglet(f,(fauxa+2)%3)) { RemoveDoubletOrSinglet(f,(fauxa+2)%3,m, affected); return true;}
  if (IsDoubletOrSinglet(f,(fauxa+2)%3)) return false;
  if (IsDoubletOrSinglet(*(f.FFp(fauxa)),(f.FFi(fauxa)+2)%3)) return false;

  if (affected) {
    int w1 = (fauxa+2)%3; // any edge but not the faux
    affected->F() = fa->FFp(w1);
    affected->E() = fa->FFi(w1);
    if (affected->F() == fa){
      int w1 = (fauxa+1)%3; // any edge but not the faux
      affected->F() = fa->FFp(w1);
      affected->E() = (fa->FFi(w1)+2)%3;
    }
  }

  FaceType* fb = fa->FFp(fauxa);  // fb dies
  assert (fb!=fa); // otherwise, its a singlet
  int fauxb = FauxIndex(fb);

  VertexType* va = fa->V(fauxa); // va lives
  VertexType* vb = fb->V(fauxb); // vb dies

  Interpolator::Apply( *(f.V0(fauxa)), *(f.V1(fauxa)), interpol, *va);

  bool border = false;
  int val =0; // number of faces around vb, which dies

  // update FV...

  // rotate around vb, (same-sense-as-face)-wise
  int pi = fauxb;
  FaceType* pf = fb; /* pf, pi could be put in a Pos<FaceType> p(pb, fauxb) */
     do {
    //pf->V(pi) = va;
    if (((pf->V2(pi) == va)||(pf->V1(pi) == va))
            &&(pf!=fa)&&(pf!=fb))
            return false;
    pi=(pi+2)%3;
    FaceType *t = pf->FFp(pi);
    if (t==pf) { border= true; break; }
    pi = pf->FFi(pi);
    pf = t;
  } while ((pf!=fb));

    pi = fauxb;
    pf = fb;

  do {
    pf->V(pi) = va;

    pi=(pi+2)%3;
    FaceType *t = pf->FFp(pi);
    if (t==pf) { border= true; break; }
    if (!pf->IsF(pi)) val++;
    pi = pf->FFi(pi);
    pf = t;
  } while (pf!=fb);

  // of found a border, also rotate around vb, (counter-sense-as-face)-wise
  if (border) {
    val++;
    int pi = fauxa;
    FaceType* pf = fa; /* pf, pi could be a Pos<FaceType> p(pf, pi) */
    do {
      pi=(pi+1)%3;
      pf->V(pi) = va;
      FaceType *t = pf->FFp(pi);
      if (t==pf) break;
      if (!pf->IsF(pi)) val++;
      pi = pf->FFi(pi);
      pf = t;
    } while (pf!=fb);
  }

  // update FF, delete faces
  _CollapseDiagHalf(*fb, fauxb, m);
  _CollapseDiagHalf(*fa, fauxa, m);

  SetValency(va, GetValency(va)+val-2);
  DecreaseValency(fb,(fauxb+2)%3,m); // update valency
  DecreaseValency(fa,(fauxa+2)%3,m); // update valency
  Allocator<MeshType>::DeleteFace(m,*fa);
  Allocator<MeshType>::DeleteFace(m,*fb);

  //assert(val == GetValency(vb));


  DecreaseValencyNoSingletTest(vb, val, m);
  // note: don't directly kill vb. In non-twomanifold, it could still be referecned
  // but: don't hunt for doublets either.

  assert(GetValency(vb)!=1 || vb->IsB());
  // if this asserts, you are in trouble.
  // It means  that the vertex that was supposed to die is still attached
  // somewhere else (non-twomanifold)
  // BUT in its other attachments it is a singlet, and that singlet cannot be
  // found now (would require VF)


  return true;
}




// helper function: find a good position on a diag to collapse a point
// currently, it is point in the middle,
//    unless a mixed border-non border edge is collapsed, then it is an exreme
static ScalarType PosOnDiag(const FaceType& f, bool counterDiag){
  bool b0, b1, b2, b3; // which side of the quads are border

  const FaceType* fa=&f;
  int ia = FauxIndex(fa);
  const FaceType* fb=fa->cFFp(ia);
  int ib = fa->cFFi(ia);

  b0 = fa->FFp((ia+1)%3) == fa;
  b1 = fa->FFp((ia+2)%3) == fa;
  b2 = fb->FFp((ib+1)%3) == fb;
  b3 = fb->FFp((ib+2)%3) == fb;

  if (counterDiag) {
    if (  (b0||b1) && !(b2||b3) ) return 1;
    if ( !(b0||b1) &&  (b2||b3) ) return 0;
  } else {
    if (  (b1||b2) && !(b3||b0) ) return 0;
    if ( !(b1||b2) &&  (b3||b0) ) return 1;
  }
  //if (f->FF( FauxIndex(f) )->IsB(
  return 0.5f;
}

// trick! hide valency in flags
typedef enum { VALENCY_FLAGS = 24 } ___; // this bit and the 4 successive one are devoted to store valency

static void SetValency(VertexType *v, int n){
  //v->Q() = n;
  assert(n>=0 && n<=255);
  v->Flags()&= ~(255<<VALENCY_FLAGS);
  v->Flags()|= n<<VALENCY_FLAGS;
}

static int GetValency(const VertexType *v){
  //return (int)(v->cQ());
  return ( v->cFlags() >> (VALENCY_FLAGS) ) & 255;
}

static void IncreaseValency(VertexType *v, int dv=1){
#ifdef NDEBUG
  v->Flags() += dv<<VALENCY_FLAGS;
#else
  SetValency( v, GetValency(v)+dv );
#endif
}

/*
static void DecreaseValency(VertexType *v, int dv=1){
#ifdef NDEBUG
  v->Flags() -= dv<<VALENCY_FLAGS;
#else
  SetValency( v, GetValency(v)-dv );
#endif
}
*/

// decrease valency, kills singlets on sight, remove unreferenced vertices too...
static void DecreaseValency(FaceType *f, int wedge, MeshType &m){
  VertexType *v = f->V(wedge);
  int val = GetValency(v)-1;
  SetValency( v, val );
  if (val==0) Allocator<MeshType>::DeleteVertex(m,*v);
  if (val==1) // singlet!
    RemoveSinglet(*f,wedge,m); // this could be recursive...
}

// decrease valency, remove unreferenced vertices too, but don't check for singlets...
static void DecreaseValencyNoSingletTest(VertexType *v, int dv,  MeshType &m){
  int val = GetValency(v)-dv;
  SetValency( v, val );
  if (DELETE_VERTICES)
  if (val==0) Allocator<MeshType>::DeleteVertex(m,*v);
}

static void DecreaseValencySimple(VertexType *v, int dv){
  int val = GetValency(v)-dv;
  SetValency( v, val );
}

static void UpdateValencyInFlags(MeshType& m){
  for (VertexIterator vi = m.vert.begin();  vi!=m.vert.end(); vi++) if (!vi->IsD()) {
    SetValency(&*vi,0);
  }
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
     for (int w=0; w<3; w++)
     if (!fi->IsF(w))
       IncreaseValency( fi->V(w));
  }
}

static void UpdateValencyInQuality(MeshType& m){
  tri::UpdateQuality<MeshType>::VertexConstant(m,0);

  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
     for (int w=0; w<3; w++)
         fi->V(w)->Q() += (fi->IsF(w)||fi->IsF((w+2)%3) )? 0.5f:1;
  }
}

static bool HasConsistentValencyFlag(MeshType &m) {
  UpdateValencyInQuality(m);
  bool isok=true;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    for (int k=0; k<3; k++)
      if (GetValency(fi->V(k))!=fi->V(k)->Q()){
        MarkFaceF(&*fi);
        isok=false;
      }
  }
  return isok;
}

// helper function:
// returns quality of a given (potential) quad
static ScalarType quadQuality(FaceType *f, int edgeInd){

  CoordType
    a = f->V0(edgeInd)->P(),
    b = f->FFp(edgeInd)->V2( f->FFi(edgeInd) )->P(),
    c = f->V1(edgeInd)->P(),
    d = f->V2(edgeInd)->P();

  return quadQuality(a,b,c,d);
}

/**
helper function:
given a quad edge, retruns:
   0 if that edge should not be rotated
  +1 if it should be rotated clockwise (+1)
  -1 if it should be rotated counterclockwise (-1)
Currently an edge is rotated iff it is shortened by that rotations
(shortcut criterion)
*/
static int TestEdgeRotation(const FaceType &f, int w0, ScalarType *gain=NULL)
{
  const FaceType *fa = &f;
  assert(! fa->IsF(w0) );
  ScalarType q0,q1,q2;
  CoordType v0,v1,v2,v3,v4,v5;
  int w1 = (w0+1)%3;
  int w2 = (w0+2)%3;

  v0 = fa->P(w0);
  v3 = fa->P(w1);

  if (fa->IsF(w2) ) {
    v1 = fa->cFFp(w2)->V2( fa->cFFi(w2) )->P();
    v2 = fa->P(w2);
  } else {
    v1 = fa->P(w2);
    v2 = fa->cFFp(w1)->V2( fa->cFFi(w1) )->P();
  }

  const FaceType *fb = fa->cFFp(w0);
  w0 = fa->cFFi(w0);

  w1 = (w0+1)%3;
  w2 = (w0+2)%3;
  if (fb->IsF(w2) ) {
    v4 = fb->cFFp(w2)->V2( fb->cFFi(w2) )->P();
    v5 = fb->P(w2);
  } else {
    v4 = fb->P(w2);
    v5 = fb->cFFp(w1)->V2( fb->cFFi(w1) )->P();
  }


#if (!LENGTH_CRITERION)
  //  max overall CONFORMAL quality criterion:
  q0 = quadQuality(v0,v1,v2,v3) +  quadQuality(v3,v4,v5,v0); // keep as is?
  q1 = quadQuality(v1,v2,v3,v4) +  quadQuality(v4,v5,v0,v1); // rotate CW?
  q2 = quadQuality(v5,v0,v1,v2) +  quadQuality(v2,v3,v4,v5); // rotate CCW?

  if (q0>=q1 && q0>=q2) return 0;
  if (q1>=q2) return 1;

#else
  // min distance (shortcut criterion)
  q0 = (v0 - v3).SquaredNorm();
  q1 = (v1 - v4).SquaredNorm();
  q2 = (v5 - v2).SquaredNorm();

  if (q0<=q1 && q0<=q2) return 0; // there's no rotation shortening this edge

  //static int stop=0;
  //static int go=0;
  //if ((stop+go)%100==99) printf("Stop: %4.1f%%\n",(stop*100.0/(stop+go)) );

  if (q1<=q2) {
    if (gain) *gain = sqrt(q1)-sqrt(q0);
    // test: two diagonals should become shorter (the other two reamin the same)
    if (
      (v0-v2).SquaredNorm() < (v4-v2).SquaredNorm() ||
      (v3-v5).SquaredNorm() < (v1-v5).SquaredNorm()
    ) {
      //stop++;
      return 0;
    }
    //go++;
    return 1;
  }

  {
    if (gain) *gain = sqrt(q2)-sqrt(q0);
    // diagonal test, as above:
    if (
      (v0-v4).SquaredNorm() < (v2-v4).SquaredNorm() ||
      (v3-v1).SquaredNorm() < (v5-v1).SquaredNorm()
    ) {
      //stop++;
      return 0;
    }
    //go++;
    return -1;
  }
#endif
}

private:

// helper function:
// returns quality of a quad formed by points a,b,c,d
// quality is computed as "how squared angles are"
static ScalarType quadQuality(const CoordType &a, const CoordType &b, const CoordType &c, const CoordType &d){
  ScalarType score = 0;
  score += 1 - math::Abs( Cos( a,b,c) );
  score += 1 - math::Abs( Cos( b,c,d) );
  score += 1 - math::Abs( Cos( c,d,a) );
  score += 1 - math::Abs( Cos( d,a,b) );
  return score / 4;
}




private:

// helper function:
// cos of angle abc. This should probably go elsewhere
static ScalarType Cos(const CoordType &a, const CoordType &b, const CoordType &c )
{
  CoordType
    e0 = b - a,
    e1 = b - c;
  ScalarType d =  (e0.Norm()*e1.Norm());
  if (d==0) return 0.0;
  return (e0*e1)/d;
}
public:
/**
  Generic quad triangulation function.
  It take in input 4 vertex pointrs and rotate them so that a simple fan triangulation is Ok.
  It uses geometric criteria for avoiding bad shaped triangles, and folds
  and it use an internal set of already created diagonal to avoid the creation of non manifold situations.
  At the begin you shoud call this function with an empty vector to reset the set of existing diagonals.
  */
static void QuadTriangulate(std::vector<VertexPointer> &q)
{
  typedef typename std::set<std::pair<VertexPointer,VertexPointer> > diagSetType;
  static diagSetType diagSet; // the set of already created diagonals
  if(q.size()!=4)
  {
    diagSet.clear();
    return;
  }
  const CoordType &P0=q[0]->cP();
  const CoordType &P1=q[1]->cP();
  const CoordType &P2=q[2]->cP();
  const CoordType &P3=q[3]->cP();

  CoordType N00 = Normal(P0,P1,P2);
  CoordType N01 = Normal(P0,P2,P3);
  CoordType N10 = Normal(P1,P2,P3);
  CoordType N11 = Normal(P1,P3,P0);

  ScalarType Angle0Rad=Angle(N00,N01);
  ScalarType Angle1Rad=Angle(N10,N11);

  // QualityRadii is inradius/circumradius; bad when close to zero.
  // swap diagonal if the worst triangle improve.
  bool qualityImprove = std::min<ScalarType>(QualityRadii(P0,P1,P2),QualityRadii(P0,P2,P3)) < std::min<ScalarType>(QualityRadii(P1,P2,P3),QualityRadii(P1,P3,P0));
  bool swapCauseFlip = (Angle1Rad > M_PI/2.0) && (Angle0Rad <M_PI/2.0);

  if ( qualityImprove && ! swapCauseFlip)
         std::rotate(q.begin(), q.begin()+1, q.end());

  std::pair<typename diagSetType::iterator,bool> res;
  if(q[0]<q[2]) res= diagSet.insert(std::make_pair(q[0],q[2]));
  else res= diagSet.insert(std::make_pair(q[2],q[0]));

  if(!res.second) // res.second is false if an element with the same value existed; in that case rotate again!
    std::rotate(q.begin(), q.begin()+1, q.end());
}
};
}} // end namespace vcg::tri

#endif
