#include <vcg/complex/algorithms/bitquad_support.h>

/** BIT-QUAD creation support:
    a collection of methods that,
    starting from a triangular mesh, will create your quad-pure or quad-domainant mesh.

    They all require:
      - per face Q, and FF connectivity, 2-manyfold meshes,
      - and tri- or quad- meshes (no penta, etc) (if in need, use MakeBitTriOnly)


[ list of available methods: ]

void MakePureByRefine(Mesh &m)
   - adds a vertex for each tri or quad present
   - thus, miminal complexity increase is the mesh is quad-dominant already
   - old non-border edges are made faux
   - never fails

void MakePureByCatmullClark(MeshType &m)
   - adds a vertex in each (non-faux) edge.
   - twice complexity increase w.r.t. "ByRefine" method.
   - preserves edges: old edges are still edges
   - never fails

bool MakePureByFlip(MeshType &m [, int maxdist] )
   - does not increase # vertices, just flips edges
   - call in a loop until it returns true  (temporary hack)
   - fails if number of triangle is odd (only happens in open meshes)
   - add "StepByStep" to method name if you want it to make a single step (debugging purposes)

bool MakeTriEvenBySplit(MeshType& m)
bool MakeTriEvenByDelete(MeshType& m)
   - two simple variants that either delete or split *at most one* border face
     so that the number of tris will be made even. Return true if it did it.
   - useful to use the previous method, when mesh is still all triangle

void MakeDominant(MeshType &m, int level)
   - just merges traingle pairs into quads, trying its best
   - various heuristic available, see descr. for parameter "level"
   - provides good starting point for make-Quad-Only methods
   - uses an ad-hoc measure for "quad quality" (which is hard-wired, for now)

void MakeBitTriOnly(MeshType &m)
   - inverse process: returns to tri-only mesh

int SplitNonFlatQuads(MeshType &m, ScalarType toleranceDeg=0){
   - as above, but splits only non flat quads

TESTING METHODS:

bool IsTriOnly(const MeshType &m);  // only triangles
bool IsQuadOnly(const MeshType &m); // only quads
bool IsTriQuadOnly(const MeshType &m); // only quads or triangles

(more info in comments before each method)

*/
#ifndef VCG_BITQUAD_CRE
#define VCG_BITQUAD_CRE

namespace vcg{namespace tri{

template <class _MeshType,
          class Interpolator = GeometricInterpolator<typename _MeshType::VertexType> >
class BitQuadCreation{

public:

typedef _MeshType MeshType;
typedef typename MeshType::ScalarType ScalarType;
typedef typename MeshType::CoordType CoordType;
typedef typename MeshType::FaceType FaceType;
typedef typename MeshType::FaceType* FaceTypeP;
typedef typename MeshType::VertexType VertexType;
typedef typename MeshType::FaceIterator FaceIterator;
typedef typename MeshType::VertexIterator VertexIterator;
typedef typename MeshType::ConstFaceIterator ConstFaceIterator;

typedef BitQuad<MeshType> BQ; // static class to make basic quad operations

// helper function:
// given a triangle, merge it with its best neightboord to form a quad
template <bool override>
static void selectBestDiag(FaceType *fi){

  if (!override) {
    if (fi->IsAnyF()) return;
  }

  // select which edge to make faux (if any)...
  int whichEdge = -1;
  ScalarType bestScore = fi->Q();

  whichEdge=-1;

  for (int k=0; k<3; k++){

    // todo: check creases? (continue if edge k is a crease)

    if (!override) {
      if (fi->FFp(k)->IsAnyF()) continue;
    }
    if (fi->FFp(k)==fi) continue; // never make a border faux

    ScalarType score = BQ::quadQuality( &*fi, k );
    if (override) {
      // don't override anyway iff other face has a better match
      if (score < fi->FFp(k)->Q()) continue;
    }
    if (score>bestScore) {
      bestScore = score;
      whichEdge = k;
    }
  }

  // ...and make it faux
  if (whichEdge>=0) {
    //if (override && fi->FFp(whichEdge)->IsAnyF()) {
      // new score is the average of both scores
    //  fi->Q() = fi->FFp(whichEdge)->Q() = ( bestScore + fi->FFp(whichEdge)->Q() ) /2;
    //} else {
    //}

    if (override) {
      // clear any faux edge of the other face
      for (int k=0; k<3; k++)
      if (fi->FFp(whichEdge)->IsF(k)) {
        fi->FFp(whichEdge)->ClearF(k);
        fi->FFp(whichEdge)->FFp(k)->ClearF( fi->FFp(whichEdge)->FFi(k) );
        fi->FFp(whichEdge)->FFp(k)->Q()=0.0; // other face's ex-buddy is now single and sad :(
      }

      // clear all faux edges of this face...
      for (int k=0; k<3; k++)
      if (fi->IsF(k)) {
        fi->ClearF(k);
        fi->FFp(k)->ClearF( fi->FFi(k) );
        fi->FFp(k)->Q()= 0.0; // my ex-buddy is now sad
      }
    }
    // set (new?) quad
    fi->SetF(whichEdge);
    fi->FFp(whichEdge)->SetF( fi->FFi(whichEdge) );
    fi->Q() = fi->FFp(whichEdge)->Q() = bestScore;

  }


}



// helper funcion:
// a pass though all triangles to merge triangle pairs into quads
template <bool override> // override previous decisions?
static void MakeDominantPass(MeshType &m){

  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    selectBestDiag<override>(&(*fi));
  }

}
/**
 * This function split a face along the specified border edge it does not compute any property of the new vertex. It only do the topological work.
 * @param edge Index of the edge
 */
//                                  sideF
//          sideF         V2(e) ------------- v2
//  V0 -------------V2 V2(e)  \               /
//  |             /     |  \    \  newF     /
//  |           /       |    \    \       / e
//  |   f     /         |      \    \   /
//  |       / e         | f   V1(e)=newV =
//  |     /             |      /
//  |   /               |    /
//  | /                 |  /
//  V1                  V0(e)
//

static std::pair<typename MeshType::FaceType *, typename MeshType::VertexType *> FaceSplitBorderEdge(MeshType &m, typename MeshType::FaceType &f, int edge, typename MeshType::FaceType *newFace, typename MeshType::VertexType *newVert )
{

    typename MeshType::FaceType *sideFFp;
    int sideFFi;

    assert(tri::HasFFAdjacency(m));
    assert(face::IsBorder(f,edge));
    //qDebug("OldFacePRE  %i %i %i",tri::Index(m,f.V(0)),tri::Index(m,f.V(1)),tri::Index(m,f.V(2)));
    if(newFace==0) newFace=&*tri::Allocator<MeshType>::AddFaces(m,1);
    if(newVert==0) {
        newVert=&*tri::Allocator<MeshType>::AddVertices(m,1);
        newVert->P()=(f.P0(edge)+f.P1(edge))/2.0;
    }
    newFace->V0(edge)=newVert;
    newFace->V1(edge)=f.V1(edge);
    newFace->V2(edge)=f.V2(edge);

    f.V1(edge)=newVert;

    //qDebug("NewFace %i %i %i",tri::Index(m,newFace->V(0)),tri::Index(m,newFace->V(1)),tri::Index(m,newFace->V(2)));
    //qDebug("OldFace %i %i %i",tri::Index(m,f.V(0)),tri::Index(m,f.V(1)),tri::Index(m,f.V(2)));

   // Topology

    newFace->FFp((edge+2)%3) = &f;
    newFace->FFi((edge+2)%3) = (edge+1)%3;

    newFace->FFp((edge+0)%3) = newFace;
    newFace->FFi((edge+0)%3) = (edge+0)%3;

    newFace->FFp((edge+1)%3) = f.FFp((edge+1)%3);
    newFace->FFi((edge+1)%3) = f.FFi((edge+1)%3);

    sideFFp = f.FFp((edge+1)%3);
    sideFFi = f.FFi((edge+1)%3);

    f.FFp((edge+1)%3) = newFace;
    f.FFi((edge+1)%3) = (edge+2)%3;

    sideFFp->FFp(sideFFi)=newFace;
    sideFFp->FFi(sideFFi)=(edge+1)%3;

    assert(face::IsBorder(f,edge));
    assert(face::IsBorder(*newFace,edge));

    return std::make_pair(newFace,newVert);
}
// make tri count even by splitting a single triangle...
//
//  V0 -------V2    V0 --------V2
//  |       /       |  \ Fnew /
//  |     /         |    Vnew
//  |   /           |    /
//  | /             |  /
//  V1              V1
//

static bool MakeTriEvenBySplit(MeshType& m){
  if (m.fn%2==0) return false; // it's already Even
  // Search for a triangle on the border
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++)
  {
      if(!(*fi).IsD())
      {
          for (int k=0; k<3; k++) {
              if (face::IsBorder(*fi,k)){
                  // We have found a face with a border
                  int index=tri::Index(m,*fi);
                  VertexIterator vnew=tri::Allocator<MeshType>::AddVertices(m,1);
                  (*vnew).P()=((*fi).P0(k)+(*fi).P1(k))/2.0;

                  FaceIterator fnew=tri::Allocator<MeshType>::AddFaces(m,1);

                  FaceSplitBorderEdge(m,m.face[index],k,&*fnew,&*vnew);
                     return true;
              }
          }
      }

  }
     return true;
}

// make tri count even by delete...
static bool MakeTriEvenByDelete(MeshType& m)
{
  if (m.fn%2==0) return false; // it's already Even

  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) {
    for (int k=0; k<3; k++) {
      if (face::IsBorder(*fi,k) ) {
        FFDetachManifold(*fi,(k+1)%3);
        FFDetachManifold(*fi,(k+2)%3);
        Allocator<MeshType>::DeleteFace(m,*fi);
        return true;
      }
    }
  }
  assert(0); // no border face found? then how could the number of tri be Odd?
  return true;
}


/*
  Splits any quad that makes an angle steeper than given degrees
*/
static int SplitNonFlatQuads(MeshType &m, ScalarType deg=0){
  int res=0;
  float th = math::Cos(math::ToRad(deg));
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    if (fi->IsAnyF()) {
      int faux = BQ::FauxIndex(&*fi);
      FaceType *fb = fi->FFp(faux);
      if (fb->N()*fi->N()<th) {
        fi->ClearF(faux);
        fb->ClearF(fi->FFi(faux));
        res++;
      }
    }
  }
  return res;
}


/**
  Given a mesh, makes it bit trianglular (makes all edges NOT faux)
*/
static void MakeBitTriOnly(MeshType &m){
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) {
    fi->ClearAllF();
  }
}

/** given a quad-and-tree mesh, enforces the "faux edge is 2nd edge" convention.
 * Requires (and updates): FV and FF structure
 * Updates: faux flags
 * Updates: per wedge attributes, if any
 * Other connectivity structures, and per edge and per wedge flags are ignored
 */
static bool MakeBitTriQuadConventional(MeshType &/*m*/){
  assert(0); // todo
  return false;
}

/* returns true if mesh is a "conventional" quad mesh.
   I.e. if it is all quads, with third edge faux for all triangles*/
static bool IsBitTriQuadConventional(const MeshType &m){
  for (ConstFaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    if (fi->IsAnyF())
    if ( (fi->Flags() & FaceType::FAUX012 ) != FaceType::FAUX2 ) {
      return false;
    }
  }
  return true;
}

/* returns true if mesh is a pure tri-mesh. (no faux edges) */
static bool IsTriOnly(const MeshType &m){
  for (ConstFaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    if (fi->IsAnyF()) return false;
  }
  return true;
}

/* returns true if mesh is a pure quad-mesh.  */
static bool IsQuadOnly(const MeshType &m){
  for (ConstFaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
        int count = 0;
        if (fi->IsF(0)) count++;
        if (fi->IsF(1)) count++;
        if (fi->IsF(2)) count++;
        if (count!=1) return false;
  }
  return true;
}

/* returns true if mesh has only tris and quads (no penta etc) */
static bool IsTriQuadOnly(const MeshType &m){
  for (ConstFaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
        int count = 0;
        if (fi->IsF(0)) count++;
        if (fi->IsF(1)) count++;
        if (fi->IsF(2)) count++;
        if (count>1) return false;
  }
  return true;
}


static void CopyTopology(FaceType *fnew, FaceType * fold)
{
    fnew->FFp(0)=fold->FFp(0); fnew->FFi(0)=fold->FFi(0);
    fnew->FFp(1)=fold->FFp(1); fnew->FFi(1)=fold->FFi(1);
    fnew->FFp(2)=fold->FFp(2); fnew->FFi(2)=fold->FFi(2);
    fnew->V(0) = fold->V(0);
    fnew->V(1) = fold->V(1);
    fnew->V(2) = fold->V(2);
}
/**
 makes any mesh quad only by refining it so that a quad is created over all
 previous diags
 requires that the mesh is made only of quads and tris.
*/
static void MakePureByRefine(MeshType &m){

  // todo: update VF connectivity if present


  int ev = 0; // EXTRA vertices (times 2)
  int ef = 0; // EXTRA faces

  // first pass: count triangles to be added
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    int k=0;
     if (face::IsBorder(*fi,0)) k++;
      if (face::IsBorder(*fi,1)) k++;
      if (face::IsBorder(*fi,2)) k++;
    if (!fi->IsAnyF()) {
      // it's a triangle
      if (k==0) // add a vertex in the center of the face, splitting it in 3
       { ev+=2; ef+=2; }
      if (k==1) // add a vertex in the border edge, splitting it in 2
       { }
      if (k==2) // do nothing, just mark the non border edge as faux
       { }
      if (k==3) // disconnected single triangle (all borders): make one edge as faus
       { }
    }
    else {
      // assuming is a quad (not a penta, etc), i.e. only one faux
      // add a vertex in the center of the faux edge, splitting the face in 2
      ev+=1; ef+=1;
      assert(k!=3);
    }
  }
  assert(ev%2==0); // should be even by now
  ev/=2; // I was counting each of them twice

  //int originalFaceNum = m.fn;
  FaceIterator nfi = tri::Allocator<MeshType>::AddFaces(m,ef);
  VertexIterator nvi = tri::Allocator<MeshType>::AddVertices(m,ev);

  tri::UpdateFlags<MeshType>::FaceClearV(m);

  // second pass: add faces and vertices
  int nsplit=0; // spits to be done on border in the third pass
  for (FaceIterator fi = m.face.begin(), fend = nfi;  fi!=fend; fi++) if (!fi->IsD() && !fi->IsV() ) {

    fi->SetV();

    if (!fi->IsAnyF()) {
      // it's a triangle

      int k=0; // number of borders
      if (face::IsBorder(*fi,0)) k++;
      if (face::IsBorder(*fi,1)) k++;
      if (face::IsBorder(*fi,2)) k++;

      if (k==0) // add a vertex in the center of the face, splitting it in 3
      {
        assert(nvi!=m.vert.end());
        VertexType *nv = &*nvi; nvi++;
        //*nv = *fi->V0( 0 ); // lazy: copy everything from the old vertex
                nv->ImportData(*(fi->V0( 0 ))); // lazy: copy everything from the old vertex

        nv->P() = ( fi->V(0)->P() + fi->V(1)->P() + fi->V(2)->P() )  /3.0;
        FaceType *fa = &*fi;
        FaceType *fb = &*nfi; nfi++;
        FaceType *fc = &*nfi; nfi++;

                fb->ImportData(*fa); CopyTopology(fb,fa);
                fc->ImportData(*fa); CopyTopology(fc,fa);

        fa->V(0) = nv;
        fb->V(1) = nv;
        fc->V(2) = nv;

        fb->FFp(2)=fa->FFp(2); fb->FFi(2)=fa->FFi(2);
                fc->FFp(0)=fa->FFp(0); fc->FFi(0)=fa->FFi(0);

        assert( fa->FFp(1)->FFp(fa->FFi(1)) == fa );
        /*    */fb->FFp(2)->FFp(fb->FFi(2)) =  fb;
        /*    */fc->FFp(0)->FFp(fc->FFi(0)) =  fc;

        fa->FFp(0) = fc; fa->FFp(2) = fb; fa->FFi(0) = fa->FFi(2) = 1;
        fb->FFp(1) = fa; fb->FFp(0) = fc; fb->FFi(0) = fb->FFi(1) = 2;
        fc->FFp(1) = fa; fc->FFp(2) = fb; fc->FFi(1) = fc->FFi(2) = 0;

        if (fb->FFp(2)==fa) fb->FFp(2)=fb; // recover border status
        if (fc->FFp(0)==fa) fc->FFp(0)=fc;

        fa->ClearAllF();
        fb->ClearAllF();
        fc->ClearAllF();
        fa->SetF(1);
        fb->SetF(2);
        fc->SetF(0);

        fa->SetV();fb->SetV();fc->SetV();
      }
      if (k==1) { // make a border face faux, anf other two as well
        fi->SetF(0);
        fi->SetF(1);
        fi->SetF(2);
        nsplit++;
      }
      if (k==2) // do nothing, just mark the non border edge as faux
      {
        fi->ClearAllF();
        for (int w=0; w<3; w++) if (fi->FFp(w) != &*fi) fi->SetF(w);
      }
      if (k==3) // disconnected single triangle (all borders): use catmull-clark (tree vertices, split it in 6
      {
        fi->ClearAllF();
        fi->SetF(2);
        nsplit++;
      }
    }
    else {
      // assuming is a part of quad (not a penta, etc), i.e. only one faux
      FaceType *fa = &*fi;
      int ea2 = BQ::FauxIndex(fa); // index of the only faux edge
      FaceType *fb = fa->FFp(ea2);
      int eb2 = fa->FFi(ea2);
      assert(fb->FFp(eb2)==fa) ;
      assert(fa->IsF(ea2));
      //assert(fb->IsF(eb2)); // reciprocal faux edge

      int ea0 = (ea2+1) %3;
      int ea1 = (ea2+2) %3;
      int eb0 = (eb2+1) %3;
      int eb1 = (eb2+2) %3;

      // create new vert in center of faux edge
      assert(nvi!=m.vert.end());
      VertexType *nv = &*nvi; nvi++;
      // *nv = * fa->V0( ea2 );
            nv->ImportData(*(fa->V0( ea2 ) )); // lazy: copy everything from the old vertex
      //nv->P() = ( fa->V(ea2)->P() + fa->V(ea0)->P() ) /2.0;
      Interpolator::Apply(*(fa->V(ea2)),*(fa->V(ea0)),0.5,*nv);
      // split faces: add 2 faces (one per side)
      assert(nfi!=m.face.end());
      FaceType *fc = &*nfi; nfi++;
      assert(nfi!=m.face.end());
      FaceType *fd = &*nfi; nfi++;

            fc->ImportData(*fa ); CopyTopology(fc,fa); // lazy: copy everything from the old vertex
            fd->ImportData(*fb ); CopyTopology(fd,fb);// lazy: copy everything from the old vertex

      fa->V(ea2) = fc->V(ea0) =
      fb->V(eb2) = fd->V(eb0) = nv ;

      fa->FFp(ea1)->FFp( fa->FFi(ea1) ) = fc;
      fb->FFp(eb1)->FFp( fb->FFi(eb1) ) = fd;

      fa->FFp(ea1) = fc ;  fa->FFp(ea2) = fd;
      fa->FFi(ea1) = ea0;  fa->FFi(ea2) = eb2;
      fb->FFp(eb1) = fd ;  fb->FFp(eb2) = fc;
      fb->FFi(eb1) = eb0;  fb->FFi(eb2) = ea2;
      fc->FFp(ea0) = fa ;  fc->FFp(ea2) = fb;
      fc->FFi(ea0) = ea1;  fc->FFi(ea2) = eb2;
      fd->FFp(eb0) = fb ;  fd->FFp(eb2) = fa;
      fd->FFi(eb0) = eb1;  fd->FFi(eb2) = ea2;

      // detect boundaries
      bool ba = fa->FFp(ea0)==fa;
      bool bc = fc->FFp(ea1)==fa;
      bool bb = fb->FFp(eb0)==fb;
      bool bd = fd->FFp(eb1)==fb;

      if (bc) fc->FFp(ea1)=fc; // repristinate boundary status
      if (bd) fd->FFp(eb1)=fd; // of new faces

      fa->SetV();
      fb->SetV();
      fc->SetV();
      fd->SetV();

      fa->ClearAllF();
      fb->ClearAllF();
      fc->ClearAllF();
      fd->ClearAllF();

      fa->SetF( ea0 );
      fb->SetF( eb0 );
      fc->SetF( ea1 );
      fd->SetF( eb1 );

      // fix faux mesh boundary... if two any consecutive, merge it in a quad
      if (ba&&bc) {
        fa->ClearAllF(); fa->SetF(ea1);
        fc->ClearAllF(); fc->SetF(ea0);
        ba = bc = false;
      }
      if (bc&&bb) {
        fc->ClearAllF(); fc->SetF(ea2);
        fb->ClearAllF(); fb->SetF(eb2);
        bc = bb = false;
      }
      if (bb&&bd) {
        fb->ClearAllF(); fb->SetF(eb1);
        fd->ClearAllF(); fd->SetF(eb0);
        bb = bd = false;
      }
      if (bd&&ba) {
        fd->ClearAllF(); fd->SetF(eb2);
        fa->ClearAllF(); fa->SetF(ea2);
        bd = ba = false;
      }
      // remaninig boudaries will be fixed by splitting in the last pass
      if (ba) nsplit++;
      if (bb) nsplit++;
      if (bc) nsplit++;
      if (bd) nsplit++;
    }
  }
  assert(nfi==m.face.end());
  assert(nvi==m.vert.end());

  // now and there are no tris left, but there can be faces with ONE edge border & faux ()


  // last pass: add vertex on faux border faces... (if any)
  if (nsplit>0) {
    FaceIterator nfi = tri::Allocator<MeshType>::AddFaces(m,nsplit);
    VertexIterator nvi = tri::Allocator<MeshType>::AddVertices(m,nsplit);
    for (FaceIterator fi = m.face.begin(), fend = nfi;  fi!=fend; fi++) if (!fi->IsD()) {
      FaceType* fa = &*fi;
      int ea2 = -1; // border and faux face (if any)
      if (fa->FFp(0)==fa &&  fa->IsF(0) ) ea2=0;
      if (fa->FFp(1)==fa &&  fa->IsF(1) ) ea2=1;
      if (fa->FFp(2)==fa &&  fa->IsF(2) ) ea2=2;

      if (ea2 != -1) { // ea2 edge is naughty (border AND faux)

        int ea0 = (ea2+1) %3;
        int ea1 = (ea2+2) %3;

        // create new vert in center of faux edge
        VertexType *nv = &*nvi; nvi++;
        //*nv = * fa->V0( ea2 );
                nv->ImportData(*(fa->V0( ea2 ) )); // lazy: copy everything from the old vertex
        nv->P() = ( fa->V(ea2)->P() + fa->V(ea0)->P() ) /2.0;
        Interpolator::Apply(*(fa->V(ea2)),*(fa->V(ea0)),0.5,*nv);
        // split face: add 1 face
        FaceType *fc = &*nfi; nfi++;

                fc->ImportData(*fa);CopyTopology(fc,fa); // lazy: copy everything from the old vertex

        fa->V(ea2) = fc->V(ea0) = nv ;

        fc->FFp(ea2) = fc;

        fa->FFp(ea1)->FFp( fa->FFi(ea1) ) = fc;

        fa->FFp(ea1) = fc ;
        fa->FFi(ea1) = ea0;
        fc->FFp(ea0) = fa ;  fc->FFp(ea2) = fc;
        fc->FFi(ea0) = ea1;

        if (fc->FFp(ea1)==fa) fc->FFp(ea1)=fc; // recover border status

        assert(fa->IsF(ea0) == fa->IsF(ea1) );
        bool b = fa->IsF(ea1);

        fa->ClearAllF();
        fc->ClearAllF();

        if (b) {
          fa->SetF( ea0 );
          fc->SetF( ea1 );
        } else {
          fa->SetF( ea1 );
          fc->SetF( ea0 );
        }
      }
    }
  }


}


// uses Catmull Clark to enforce quad only meshes
// each old edge (but not faux) is split in two.
static void MakePureByCatmullClark(MeshType &m){
  MakePureByRefine(m);
  MakePureByRefine(m);
  // done
}

// Helper funcion:
// marks edge distance froma a given face.
// Stops at maxDist or at the distance when a triangle is found
static FaceType * MarkEdgeDistance(MeshType &m, FaceType *startF, int maxDist){
    assert(tri::HasPerFaceQuality(m));

  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++)  if (!fi->IsD()) {
    fi->Q()=maxDist;
  }

  FaceType * firstTriangleFound = NULL;

  startF->Q() =  0;
  std::vector<FaceType*> stack;
  int stackPos=0;
  stack.push_back(startF);

  while ( stackPos<int(stack.size())) {
    FaceType *f = stack[stackPos++];
    for (int k=0; k<3; k++) {
      assert(FFCorrectness(*f,k));
      FaceType *fk = f->FFp(k);
      int fq = int(f->Q()) + ( ! f->IsF(k) );
      if (fk->Q()> fq && fq <= maxDist) {
        if (!fk->IsAnyF()) { firstTriangleFound = fk; maxDist = fq;}
        fk->Q() = fq;
        stack.push_back(fk);
      }
    }
  }
  return firstTriangleFound;
}


/*
  given a tri-quad mesh,
  uses edge rotates to make a tri move toward another tri and to merges them into a quad.

  Retunrs number of surviving triangles (0, or 1), or -1 if not done yet.
  StepbyStep: makes just one step!
  use it in a loop as long as it returns 0 or 1.

  maxdist is the maximal edge distance where to look for a companion triangle
*/
static int MakePureByFlipStepByStep(MeshType &m, int maxdist=10000, int restart=false){

  static FaceType *ta, *tb; // faces to be matched into a quad

  static int step = 0; // hack

  if (restart) { step=0; return false; }
if (step==0) {

  // find a triangular face ta
  ta = NULL;
  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
    if (!fi->IsAnyF()) { ta=&*fi; break; }
  }
  if (!ta) return 0; // success: no triangle left (done?)


  tb = MarkEdgeDistance(m,ta,maxdist);
  if (!tb) return 1; // fail: no matching triagle found (increase maxdist?)

  step=1;

} else {
  int marriageEdge=-1;
  bool done = false;
  while (!done) {

    int bestScore = int(tb->Q());
    int edge = -1;
    bool mustDoFlip;

    // select which edge to use
    for (int k=0; k<3; k++) {
      if (tb->FFp(k) == tb) continue; // border

      FaceType* tbk = tb->FFp(k);

      if (!tbk->IsAnyF()) {done=true; marriageEdge=k; break; } // found my match

      int back = tb->FFi(k);
      int faux = BQ::FauxIndex(tbk);
      int other = 3-back-faux;

      int scoreA = int(tbk->FFp(other)->Q());

      FaceType* tbh = tbk->FFp(faux);
      int fauxh = BQ::FauxIndex(tbh);

      int scoreB = int(tbh->FFp( (fauxh+1)%3 )->Q());
      int scoreC = int(tbh->FFp( (fauxh+2)%3 )->Q());

      int scoreABC = std::min( scoreC, std::min( scoreA, scoreB ) );
      if (scoreABC<bestScore) {
        bestScore = scoreABC;
        edge = k;
        mustDoFlip = !(scoreB == scoreABC || scoreC == scoreABC);
      }
    }

    if (done) break;

    // use that edge to proceed
    if (mustDoFlip) {
      BQ::FlipDiag( *(tb->FFp(edge)) );
    }

    FaceType* next = tb->FFp(edge)->FFp( BQ::FauxIndex(tb->FFp(edge))  );

    // create new edge
    next->ClearAllF();
    tb->FFp(edge)->ClearAllF();

    // dissolve old edge
    tb->SetF(edge);
    tb->FFp(edge)->SetF( tb->FFi(edge) );
    tb->FFp(edge)->Q() = tb->Q();

    tb = next;
break;
  }

  if (marriageEdge!=-1) {
    // consume the marriage (two tris = one quad)
    assert(!(tb->IsAnyF()));
    assert(!(tb->FFp(marriageEdge)->IsAnyF()));
    tb->SetF(marriageEdge);
    tb->FFp(marriageEdge)->SetF(tb->FFi(marriageEdge));

    step=0;
  }
}
  return -1; // not done yet
}

/*
  given a tri-quad mesh,
  uses edge rotates to make a tri move toward another tri and to merges them into a quad.
  - maxdist is the maximal edge distance where to look for a companion triangle
  - retunrs true if all triangles are merged (always, unless they are odd, or maxdist not enough).
*/
static bool MakePureByFlip(MeshType &m, int maxdist=10000)
{
  MakePureByFlipStepByStep(m, maxdist, true); // restart
  int res=-1;
  while (res==-1) res = MakePureByFlipStepByStep(m, maxdist);
  return res==0;
}

/**
  given a triangle mesh, makes it quad dominant by merging triangle pairs into quads
  various euristics:
      level = 0: maximally greedy. Leaves fewest triangles
      level = 1: smarter: leaves more triangles, but makes better quality quads
      level = 2: even more so (marginally)
*/
static void MakeDominant(MeshType &m, int level){

  for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) {
    fi->ClearAllF();
    fi->Q() = 0;
  }


  MakeDominantPass<false> (m);
  if (level>0)  MakeDominantPass<true> (m);
  if (level>1)  MakeDominantPass<true> (m);
  if (level>0)  MakeDominantPass<false> (m);
}

};
}} // end namespace vcg::tri
#endif
