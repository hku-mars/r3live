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
#ifndef __VCG_RectPacker__
#define __VCG_RectPacker__

#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <vector>
#include <ctime>
#include <vcg/space/point2.h>
#include <vcg/math/similarity2.h>

namespace vcg
{

template <class SCALAR_TYPE>
class RectPacker
{
  typedef typename vcg::Box2<SCALAR_TYPE> Box2x;
  typedef typename vcg::Point2<SCALAR_TYPE> Point2x;
  typedef typename vcg::Similarity2<SCALAR_TYPE> Similarity2x;
public:
  class Stat
  {
  public:
    void clear() {
      pack_attempt_num=0;
      pack_attempt_time=0;
      pack_total_time=0;
    }

    int pack_attempt_num;
    float pack_attempt_time;
    float pack_total_time;
  };


  static Stat &stat() { static Stat _s; return _s; }


  static  bool Pack(const std::vector<Box2x > & rectVec,  /// the set of rectangles that have to be packed (generic floats, no req.)
                    const Point2i containerSizeX,         /// the size of the container where they has to be fitted (usually in pixel size)
                    std::vector<Similarity2x> &trVec,     /// the result, a set of similarity transformation that have to be applied to the rect to get their position
                    Point2x &coveredContainer)            /// the sub portion of the container covered by the solution.
{
  float bestOccupancy=0,currOccupancy=0.1f;
  std::vector<Similarity2x> currTrVec;
  Point2x currCovered;
  int start_t=clock();
  stat().clear();
  bool ret=true;
  while(ret)
  {
    stat().pack_attempt_num++;
    int t0=clock();
    ret=PackOccupancy(rectVec,containerSizeX,currOccupancy,currTrVec,currCovered);
    stat().pack_attempt_time = float(clock()-t0)/float(CLOCKS_PER_SEC);
    if(ret)
    {
      assert(currOccupancy>bestOccupancy);
      bestOccupancy = currOccupancy;
      trVec=currTrVec;
      coveredContainer=currCovered;
      currOccupancy = (2.0*currOccupancy+1.0)/3.0;
    }
  }
  stat().pack_total_time=float(clock()-start_t)/float(CLOCKS_PER_SEC);;
  if(bestOccupancy>0) return true;
  return false;
}


static  bool PackOccupancy(const std::vector<Box2x > & rectVec,  /// the set of rectangles that have to be packed
                  const Point2i containerSizeX,         /// the size of the container where they has to be fitted (usually in pixel size)
                  const SCALAR_TYPE occupancyRatio,     /// the expected percentage of the container that has to be covered
                  std::vector<Similarity2x> &trVec,     /// the result, a set of similarity transformation that have to be applied to the rect to get their position
                  Point2x &coveredContainer)            /// the sub portion of the container covered by the solution.
  {
    Point2x maxSize(0,0);
    const vcg::Point2i containerSize=Point2i::Construct(containerSizeX);
    SCALAR_TYPE areaSum=0;
    SCALAR_TYPE areaContainer = containerSize[0]*containerSize[1];

    for (size_t i=0;i<rectVec.size();++i)
    {
      maxSize[0]=std::max(maxSize[0],rectVec[i].DimX());
      maxSize[1]=std::max(maxSize[1],rectVec[i].DimY());
      areaSum += rectVec[i].DimX() * rectVec[i].DimY();
    }

    Point2x scaleFactor2(containerSize[0]/maxSize[0],containerSize[1]/maxSize[1]);

//    SCALAR_TYPE unitScaleFactor = std::min(scaleFactor2[0],scaleFactor2[1]);

    SCALAR_TYPE scaleFactor = (sqrt(areaContainer)/sqrt(areaSum))*sqrt(occupancyRatio);

//    printf("unitScaleFactor %6.3f\n",unitScaleFactor);
//    printf("scaleFactor %6.3f\n",scaleFactor);
//    printf("areaContainer %6.3f\n",areaContainer);
//    printf("areaSum %6.3f\n",areaSum);
    std::vector<vcg::Point2i> sizes(rectVec.size());
    for (size_t i=0;i<rectVec.size();++i)
    {
      sizes[i][0]=ceil(rectVec[i].DimX()*scaleFactor);
      sizes[i][1]=ceil(rectVec[i].DimY()*scaleFactor);
    }

    std::vector<vcg::Point2i> posiz;
    vcg::Point2i global_size;

    bool res = PackInt(sizes,containerSize,posiz,global_size);
    if(!res) return false;

    trVec.resize(rectVec.size());
    for (size_t i=0;i<rectVec.size();++i)
    {
      trVec[i].tra = Point2x::Construct(posiz[i]) - rectVec[i].min*scaleFactor;
      trVec[i].sca = scaleFactor;

//      qDebug("rectVec[ %5i ] (%6.2f %6.2f) - (%6.2f %6.2f) : SizeI (%6i %6i) Posiz (%6i %6i)",i,
//             rectVec[i].min[0],rectVec[i].min[1],  rectVec[i].max[0],rectVec[i].max[1],
//             sizes[i][0],sizes[i][1],  posiz[i][0],posiz[i][1]);
    }
//    printf("globalSize (%6i %6i)\n",global_size[0],global_size[1]);
    coveredContainer = Point2x::Construct(global_size);
    return true;
  }
static  bool PackMulti(const std::vector<Box2x > & rectVec,  /// the set of rectangles that have to be packed (generic floats, no req.)
                  const Point2i containerSizeI,         /// the size of the container where they has to be fitted (usually in pixel size)
                  const int containerNum,
                  std::vector<Similarity2x> &trVec,     /// the result, a set of similarity transformation that have to be applied to the rect to get their position
                  std::vector<int> &indVec,
                  std::vector<Point2x> &coveredContainer)            /// the sub portion of the container covered by the solution.
{
  float bestOccupancy=0,currOccupancy=0.1f;
  std::vector<Similarity2x> currTrVec;
  std::vector<int> currIndVec;
  std::vector<Point2x> currCovered;
  int start_t=clock();
  stat().clear();
  bool ret=true;
  while(ret && bestOccupancy < 0.99f)
  {
    stat().pack_attempt_num++;
    int t0=clock();
    ret=PackOccupancyMulti(rectVec,containerSizeI,containerNum,currOccupancy,currTrVec, currIndVec, currCovered);
    stat().pack_attempt_time = float(clock()-t0)/float(CLOCKS_PER_SEC);
    if(ret)
    {
      printf("CurrOccupancy %f\n",currOccupancy);
      assert(currOccupancy>bestOccupancy);
      bestOccupancy = currOccupancy;
      trVec=currTrVec;
      indVec=currIndVec;
      coveredContainer=currCovered;
      currOccupancy = (2.0*currOccupancy+1.0)/3.0;
    }
  }
  stat().pack_total_time=float(clock()-start_t)/float(CLOCKS_PER_SEC);;
  if(bestOccupancy>0) return true;
  return false;
}

static  bool PackOccupancyMulti(const std::vector<Box2x > & rectVec,  /// the set of rectangles that have to be packed
                                const Point2i containerSizeX,         /// the size of the container where they has to be fitted (usually in pixel size)
                                const int containerNum,
                                const SCALAR_TYPE occupancyRatio,     /// the expected percentage of the container that has to be covered
                                std::vector<Similarity2x> &trVec,     /// the result, a set of similarity transformation that have to be applied to the rect to get their position
                                std::vector<int> &indVec,
                                std::vector<Point2x> &coveredContainer)            /// the sub portion of the container covered by the solution.
{
    Point2x maxSize(0,0);
    const vcg::Point2i containerSize=Point2i::Construct(containerSizeX);
    SCALAR_TYPE areaSum=0;
    SCALAR_TYPE areaContainer = containerSize[0]*containerSize[1]*containerNum;

    for (size_t i=0;i<rectVec.size();++i)
    {
      maxSize[0]=std::max(maxSize[0],rectVec[i].DimX());
      maxSize[1]=std::max(maxSize[1],rectVec[i].DimY());
      areaSum += rectVec[i].DimX() * rectVec[i].DimY();
    }

    Point2x scaleFactor2(containerSize[0]/maxSize[0],containerSize[1]/maxSize[1]);

    SCALAR_TYPE scaleFactor = (sqrt(areaContainer)/sqrt(areaSum))*sqrt(occupancyRatio);

//    printf("unitScaleFactor %6.3f\n",unitScaleFactor);
//    printf("scaleFactor %6.3f\n",scaleFactor);
//    printf("areaContainer %6.3f\n",areaContainer);
//    printf("areaSum %6.3f\n",areaSum);
    std::vector<vcg::Point2i> sizes(rectVec.size());
    for (size_t i=0;i<rectVec.size();++i)
    {
      sizes[i][0]=ceil(rectVec[i].DimX()*scaleFactor);
      sizes[i][1]=ceil(rectVec[i].DimY()*scaleFactor);
    }

    std::vector<vcg::Point2i> posiz;
    std::vector<vcg::Point2i> global_sizeVec;

    bool res = PackIntMulti(sizes,containerNum,containerSize,posiz,indVec,global_sizeVec);
    if(!res) return false;

    trVec.resize(rectVec.size());
    for (size_t i=0;i<rectVec.size();++i)
    {
      trVec[i].tra = Point2x::Construct(posiz[i]) - rectVec[i].min*scaleFactor;
      trVec[i].sca = scaleFactor;

//      qDebug("rectVec[ %5i ] (%6.2f %6.2f) - (%6.2f %6.2f) : SizeI (%6i %6i) Posiz (%6i %6i)",i,
//             rectVec[i].min[0],rectVec[i].min[1],  rectVec[i].max[0],rectVec[i].max[1],
//             sizes[i][0],sizes[i][1],  posiz[i][0],posiz[i][1]);
    }
//    printf("globalSize (%6i %6i)\n",global_size[0],global_size[1]);
    coveredContainer.resize(containerNum);
    for(int i=0;i<containerNum;++i)
      coveredContainer[i] = Point2x::Construct(global_sizeVec[i]);
    return true;
  }

private:


class ComparisonFunctor
{
public:
  const std::vector<vcg::Point2i> & v;
  inline ComparisonFunctor( const std::vector<vcg::Point2i> & nv ) : v(nv) { }

  inline bool operator() ( int a, int b )
  {
    const Point2i &va=v[a];
    const Point2i &vb=v[b];

    return	 (va[1]!=vb[1])?(va[1]>vb[1]):
                (va[0]>vb[0]);
  }
};


/* This is the low level function that packs a set of int rects onto a grid.

   Based on the criptic code written by Claudio Rocchini

   Greedy algorithm.
   Sort the rect according their height (larger first)
   and then place them in the position that minimize the area of the bbox of all the placed rectangles

   To efficiently skip occupied areas it fills the grid with the id of the already placed rectangles.
  */
static bool PackInt(const std::vector<vcg::Point2i> & sizes, // the sizes of the rect to be packed
                    const vcg::Point2i & max_size,           // the size of the container
                    std::vector<vcg::Point2i> & posiz,       // the found positionsof each rect
                    vcg::Point2i & global_size)              // the size of smallest rect covering all the packed rect
{
  int n = (int)(sizes.size());
  assert(n>0 && max_size[0]>0 && max_size[1]>0);

  int gridSize = max_size[0]*max_size[1];		// Size dell griglia
  int i,j,x,y;

  posiz.resize(n,Point2i(-1,-1));
  std::vector<int> grid(gridSize,0);			// Creazione griglia

  #define Grid(q,w)	(grid[(q)+(w)*max_size[0]])

  // Build a permutation that keeps the reordiering of the sizes vector according to their width
  std::vector<int> perm(n);
  for(i=0;i<n;i++) perm[i] = i;
  ComparisonFunctor cmp(sizes);
  sort(perm.begin(),perm.end(),cmp);

  if(sizes[perm[0]][0]>max_size[0] || sizes[perm[0]][1]>max_size[1] )
    return false;

  // Posiziono il primo
  j = perm[0];
  global_size = sizes[j];
  posiz[j] = Point2i(0,0);

  // Fill the grid with the id(+1) of the first
  for(y=0;y<global_size[1];y++)
    for(x=0;x<global_size[0];x++)
    {
      assert(x>=0 && x<max_size[0]);
      assert(y>=0 && y<max_size[1]);
        grid[x+y*max_size[0]] = j+1;
    }

  // Posiziono tutti gli altri
  for(i=1;i<n;++i)
  {
        j = perm[i];
    assert(j>=0 && j<n);
        assert(posiz[j][0]==-1);

    int bestx,besty,bestsx,bestsy,bestArea;

    bestArea = -1;

    int sx = sizes[j][0];	// Pe comodita' mi copio la dimensione
    int sy = sizes[j][1];
    assert(sx>0 &&  sy>0);

    // Calcolo la posizione limite
    int lx = std::min(global_size[0],max_size[0]-sx);
    int ly = std::min(global_size[1],max_size[1]-sy);

    assert(lx>0 && ly>0);

    int finterior = 0;

	for(y=0;y<=ly;y++)
		{
	  for(x=0;x<=lx;)
			{
				int px;
		int c = Grid(x,y+sy-1);
		// Intersection check
		if(!c) c = Grid(x+sx-1,y+sy-1);
				if(!c)
				{
					for(px=x;px<x+sx;px++)
					{
						c = Grid(px,y);
						if(c) break;
					}
				}

				if(c)	// Salto il rettangolo
				{
		  --c;  // we store id+1...
		  assert(c>=0 && c<n);
					assert(posiz[c][0]!=-1);
					x = posiz[c][0] + sizes[c][0];
				}
		else // x,y are an admissible position where we can put the rectangle
				{
		  int nsx = std::max(global_size[0],x+sx);
		  int nsy = std::max(global_size[1],y+sy);
		  int area   = nsx*nsy;

		  if(bestArea==-1 || bestArea>area)
					{
						bestx  = x;
						besty  = y;
						bestsx = nsx;
						bestsy = nsy;
			bestArea  = area;
						if( bestsx==global_size[0] && bestsy==global_size[1] )
							finterior = 1;
					}
					break;
				}
				if(finterior) break;
			}
			if( finterior ) break;
		}

	if(bestArea==-1)
		{
			return false;
		}

		posiz[j][0] = bestx;
		posiz[j][1] = besty;
		global_size[0] = bestsx;
		global_size[1] = bestsy;
		for(y=posiz[j][1];y<posiz[j][1]+sy;y++)
			for(x=posiz[j][0];x<posiz[j][0]+sx;x++)
			{
		assert(x>=0 && x<max_size[0]);
		assert(y>=0 && y<max_size[1]);
				grid[x+y*max_size[0]] = j+1;
			}
	}

#undef Grid

	return true;
}

// Versione multitexture
static bool PackIntMulti( const std::vector<Point2i> & sizes,
                          const int ntexture,
                          const vcg::Point2i & max_size,
                          std::vector<Point2i> & posiz,
                          std::vector<int> & texin,
                          std::vector<Point2i> & globalsize  )
{
  int n = sizes.size();
  assert(n>0);
  assert(max_size[0]>0);
  assert(max_size[1]>0);


  int gdim = max_size[0]*max_size[1];		// Size dell griglia

  int i,j,k,x,y;

  globalsize.resize(ntexture);		// creazione globalsize

  posiz.resize(n);
  texin.resize(n);
  for(i=0;i<n;i++)					// Azzero le posizioni e indici
  {
    posiz[i].X() = -1;
    texin[i]    = -1;
  }

  std::vector< std::vector<int> > grid;			// Creazione griglie
  grid.resize(ntexture);
  for(k=0;k<ntexture;++k)
  {
    grid[k].resize(gdim);
    for(i=0;i<gdim;++i)
      grid[k][i] = 0;
  }

#define Grid(k,q,w)	(grid[k][(q)+(w)*max_size[0]])

  std::vector<int> perm(n);			// Creazione permutazione
  for(i=0;i<n;i++) perm[i] = i;
  ComparisonFunctor conf(sizes);
  sort(perm.begin(),perm.end(),conf);

  if(sizes[perm[0]].X()>max_size[0] ||	// Un pezzo piu' grosso del contenitore
     sizes[perm[0]].Y()>max_size[1] )
    return false;

  if(n<ntexture)						// Piu' contenitore che pezzi
    return false;

  // Posiziono i primi
  for(k=0;k<ntexture;++k)
  {
    j = perm[k];
    globalsize[k].X() = sizes[j].X();
    globalsize[k].Y() = sizes[j].Y();
    posiz[j].X() = 0;
    posiz[j].Y() = 0;
    texin[j]     = k;
    for(y=0;y<globalsize[k].Y();y++)
      for(x=0;x<globalsize[k].X();x++)
      {
        assert(x>=0);
        assert(x<max_size[0]);
        assert(y>=0);
        assert(y<max_size[1]);
        Grid(k,x,y) = j+1;
      }
  }

  // Posiziono tutti gli altri
  for(i=ntexture;i<n;++i)
  {
    j = perm[i];
    assert(j>=0);
    assert(j<n);
    assert(posiz[j].X()==-1);


    int sx = sizes[j].X();	// Pe comodita' mi copio la dimensione
    int sy = sizes[j].Y();
    assert(sx>0);
    assert(sy>0);


    int gbestx,gbesty,gbestsx,gbestsy,gbestk;
    int gbesta = -1;

    for(k=0;k<ntexture;++k)
    {
      int bestx,besty,bestsx,bestsy,besta;
      int starta;

      besta = -1;

      // Calcolo la posizione limite
      int lx = std::min(globalsize[k].X(),max_size[0]-sx);
      int ly = std::min(globalsize[k].Y(),max_size[1]-sy);

      starta = globalsize[k].X()*globalsize[k].Y();

      assert(lx>0);
      assert(ly>0);

      int finterior = 0;

      for(y=0;y<=ly;y++)
      {
        for(x=0;x<=lx;)
        {
          int px;
          int c;
          // Controllo intersezione
          c = Grid(k,x,y+sy-1);
          if(!c) c = Grid(k,x+sx-1,y+sy-1);
          if(!c)
          {
            for(px=x;px<x+sx;px++)
            {
              c = Grid(k,px,y);
              if(c) break;
            }
          }

          if(c)	// Salto il rettangolo
          {
            --c;
            assert(c>=0);
            assert(c<n);
            assert(posiz[c].X()!=-1);
            x = posiz[c].X() + sizes[c].X();
          }
          else
          {
            int nsx = std::max(globalsize[k].X(),x+sx);
            int nsy = std::max(globalsize[k].Y(),y+sy);
            int a   = nsx*nsy;

            if(besta==-1 || besta>a)
            {
              bestx  = x;
              besty  = y;
              bestsx = nsx;
              bestsy = nsy;
              besta  = a;
              if( bestsx==globalsize[k].X() && bestsy==globalsize[k].Y() )
                finterior = 1;
            }
            break;
          }
          if(finterior) break;
        }
        if( finterior ) break;
      }

      if(besta==-1) continue;		// non c'e' spazio

      besta -= starta;

      if(gbesta==-1 || gbesta>besta)
      {
        gbesta  = besta;
        gbestx  = bestx;
        gbesty  = besty;
        gbestsx = bestsx;
        gbestsy = bestsy;
        gbestk  = k;
      }
    }

    if(gbesta==-1)
    {
      return false;
    }

    assert(gbestx>=0);
    assert(gbesty>=0);
    assert(gbestk>=0);
    assert(gbestx<=max_size[0]);
    assert(gbesty<=max_size[1]);
    assert(gbestk<ntexture);

    posiz[j].X() = gbestx;
    posiz[j].Y() = gbesty;
    texin[j]     = gbestk;
    globalsize[gbestk].X() = gbestsx;
    globalsize[gbestk].Y() = gbestsy;
    for(y=posiz[j].Y();y<posiz[j].Y()+sy;y++)
      for(x=posiz[j].X();x<posiz[j].X()+sx;x++)
      {
        assert(x>=0);
        assert(x<max_size[0]);
        assert(y>=0);
        assert(y<max_size[1]);
        Grid(gbestk,x,y) = j+1;
      }
  }
#undef Grid

  return true;
}

}; // end class
} // end namespace vcg
#endif
