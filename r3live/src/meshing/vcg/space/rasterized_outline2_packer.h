/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2013                                                \/)\/    *
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

#ifndef __RASTERIZED_OUTLINE2_PACKER_H__
#define __RASTERIZED_OUTLINE2_PACKER_H__

#include <vcg/space/rect_packer.h>
#include <vcg/complex/algorithms/outline_support.h>

namespace vcg
{


class RasterizedOutline2
{
private:
    //the grid is the "bounding grid" of the polygon, which is returned by the rasterization process
    //this is a vector of "bounding grids", there is one for each rasterization (different rotation or whatever)
    std::vector < std::vector< std::vector<int> > > grids;

    //points: the points which make the polygon
    std::vector<Point2f> points;

    //top: a vector containing the number of cells (for the i-th column starting from left) from the
    //FIRST NON-EMTPY cell at the bottom to the LAST NON-EMPTY CELL at the top (there is one top vector for each rasterization)
    std::vector< std::vector<int> > deltaY;

    //bottom: a vector containing the number of EMPTY cells found starting from the bottom
    //until the first NON-EMPTY cell is found (there is one bottom vector for each rasterization)
    std::vector< std::vector<int> > bottom;

    //right: a vector containing the number of cells (for the i-th row starting from bottom) from the
    //FIRST NON-EMTPY cell at the left to the LAST NON-EMPTY CELL at the right (there is one right vector for each rasterization)
    std::vector< std::vector<int> > deltaX;

    //left: a vector containing the number of EMPTY cells found starting from the left (at the i-th row starting from the bottom)
    //until the first NON-EMPTY cell is found (there is one left vector for each rasterization)
    std::vector< std::vector<int> > left;

    //the area, measured in cells, of the discrete representations of the polygons
    std::vector<int> discreteAreas;

public:
    RasterizedOutline2() { }
    int gridHeight(int i) { return grids.at(i).size(); }
    int gridWidth( int i) { return grids.at(i).at(0).size(); }

    std::vector<Point2f>&  getPoints()           { return points; }
    const std::vector<Point2f>&  getPointsConst() const{ return points; }
    std::vector< std::vector<int> >& getGrids(int rast_i)  { return grids[rast_i]; }

    //get top/bottom/left/right vectors of the i-th rasterization
    std::vector<int>& getDeltaY(int i) { return deltaY[i]; }
    std::vector<int>& getBottom(int i) { return bottom[i]; }
    std::vector<int>& getDeltaX(int i) { return deltaX[i]; }
    std::vector<int>& getLeft(int i) { return left[i]; }
    int& getDiscreteArea(int i) { return discreteAreas[i]; }
    void addPoint(Point2f& newpoint) { points.push_back(newpoint); }
    void setPoints(std::vector<Point2f>& newpoints) { points = newpoints; }

    //resets the state of the poly and resizes all the states vectors
    void resetState(int totalRasterizationsNum) {
        discreteAreas.clear();
        deltaY.clear();
        bottom.clear();
        deltaX.clear();
        left.clear();
        grids.clear();

        discreteAreas.resize(totalRasterizationsNum);
        deltaY.resize(totalRasterizationsNum);
        bottom.resize(totalRasterizationsNum);
        deltaX.resize(totalRasterizationsNum);
        left.resize(totalRasterizationsNum);
        grids.resize(totalRasterizationsNum);
    }

    void initFromGrid(int rast_i) {
        std::vector< std::vector<int> >& tetrisGrid = grids[rast_i];
        int gridWidth = tetrisGrid[0].size();
        int gridHeight = tetrisGrid.size();
        //compute bottom,
        //where bottom[i] = empty cells from the bottom in the column i
        for (int col = 0; col < gridWidth; col++) {
            int bottom_i = 0;
            for (int row = gridHeight - 1; row >= 0; row--) {
                if (tetrisGrid[row][col] == 0) {
                    bottom_i++;
                }
                else {
                    bottom[rast_i].push_back(bottom_i);
                    break;
                }
            }
        }
        if (bottom[rast_i].size() == 0) assert("ERROR: EMPTY BOTTOM VECTOR"==0);

        //compute top
        //IT ASSUMES THAT THERE IS AT LEAST ONE NON-0 ELEMENT (which should always be the case, even if the poly is just a point)
        //deltaY[i] = for the column i, it stores the number of cells which are between the bottom and the top side of the poly
        for (int col = 0; col < gridWidth; col++) {
            int deltay_i = gridHeight - bottom[rast_i][col];
            for (int row = 0; row < gridHeight; row++) {
                if (tetrisGrid[row][col] == 0) {
                    deltay_i--;
                }
                else {
                    break;
                }
            }
            deltaY[rast_i].push_back(deltay_i);
        }
        if (deltaY[rast_i].size() == 0) assert("ERROR: EMPTY deltaY VECTOR"==0);

        //same meaning as bottom, but for the left side
        //we want left/right sides vector to be ordered so that index 0 is at poly's bottom
        int left_i;
        for (int row = gridHeight-1; row >= 0; --row) {
            //for (int row = 0; row < gridHeight; ++row) {
            left_i = 0;
            for (int col = 0; col < gridWidth; col++) {
                if (tetrisGrid[row][col] == 0) ++left_i;
                else {
                    left[rast_i].push_back(left_i);
                    break;
                }
            }
        }
        if (left[rast_i].size() == 0) assert("ERROR: EMPTY leftSide VECTOR"==0);

        //we want left/right sides vector to be ordered so that index 0 is at poly's bottom
        int deltax_i;
        for (int row = gridHeight-1; row >= 0; --row) {
            //for (int row = 0; row < gridHeight; ++row) {
            deltax_i = gridWidth - left[rast_i][gridHeight - 1 - row];
            for (int col = gridWidth - 1; col >= 0; --col) {
                if (tetrisGrid[row][col] == 0) --deltax_i;
                else {
                    break;
                }
            }
            deltaX[rast_i].push_back(deltax_i);
        }
        if (deltaX[rast_i].size() == 0) assert("ERROR: EMPTY rightSide VECTOR"==0);

        //compute the discreteArea: IT IS THE AREA (measured in grid cells) BETWEEN THE TOP AND BOTTOM SIDES...
        int discreteArea = 0;
        for (size_t i = 0; i < deltaY[rast_i].size(); i++) {
            discreteArea += deltaY[rast_i][i];
        }
        discreteAreas[rast_i] = discreteArea;
    }


};

template <class ScalarType>
class ComparisonFunctor
{

public:
    std::vector<RasterizedOutline2> & v;
    inline ComparisonFunctor( std::vector<RasterizedOutline2> & nv ) : v(nv) { }

    inline bool operator() ( int a, int b )
    {
        float area1 = tri::OutlineUtil<ScalarType>::Outline2Area(v[a].getPoints());
        float area2 = tri::OutlineUtil<ScalarType>::Outline2Area(v[b].getPoints());

        return area1 > area2;
    }
};

template <class SCALAR_TYPE, class RASTERIZER_TYPE>
class RasterizedOutline2Packer
{
    typedef typename vcg::Box2<SCALAR_TYPE> Box2x;
    typedef typename vcg::Point2<SCALAR_TYPE> Point2x;
    typedef typename vcg::Similarity2<SCALAR_TYPE> Similarity2x;
public:
  class Parameters
  {
  public:
      //size of one cell of the grid (square cells at the moment)
      int cellSize;

      //the number of rasterizations to create for each polygon; It must be a multiple of 4.
      int rotationNum;

      //THE PACKING ALGO TO USE:
      //0 - BOTTOM PACKING: it just uses bottom horizon and computes cost using the num of empty cells between the bottom side of the poly and the bottom horizon
      //1 - BOTTOM PACKING WITH PENALTY: it uses both bottom and left horizons, and it makes so that polys are placed the closest possible to the left horizon
      //2 - CORNER PACKING: 1) tries to drop poly from right to left and computes cost relative to the left horizon
      //                    2) tries to drop poly from top to bottom and computes cost relative to the bottom horizon
      //                    3) chooses the X,Y which minimize the cost
      //                    NOTE: IN THIS ALGO THE COST HAVE TO INCLUDE THE PENALTY, OTHERWISE THE TWO STRATEGIES (dropping from right and from top)
      //                          WILL COMPETE CAUSING A LOW PACKING EFFICIENCY
      enum costFuncEnum {
          MinWastedSpace,
          LowestHorizon,
          MixedCost
      };
      costFuncEnum costFunction;
      bool doubleHorizon;

      ///default constructor
      Parameters()
      {
          costFunction = LowestHorizon;
          doubleHorizon=true;
          rotationNum = 16;
          cellSize = 8;
      }
  };


  //THE CLASS WHICH HANDLES THE PACKING AND THE UPDATED STATE OF THE PACKING ALGORITHMS
  class packingfield
  {

  private:
      //the bottomHorizon stores the length of the i-th row in the current solution
      std::vector<int> mLeftHorizon;

      //the bottomHorizon stores the height of the i-th column in the current solution
      std::vector<int> mBottomHorizon;

      //the size of the packing grid
      vcg::Point2i mSize;

      //packing parameters
      Parameters params;

  public:
      packingfield(vcg::Point2i size, const Parameters& par)
      {
          mBottomHorizon.resize(size.X(), 0);
          mLeftHorizon.resize(size.Y(), 0);
          params = par;
          mSize = Point2i(size.X(), size.Y());
      }

      std::vector<int>& bottomHorizon() { return mBottomHorizon; }
      std::vector<int>& leftHorizon() { return mLeftHorizon; }
      vcg::Point2i& size() { return mSize; }

      //returns the score relative to the left horizon of that poly in that particular position, taking into account the choosen algo
      int getCostX(RasterizedOutline2& poly, Point2i pos, int rast_i) {
          switch (params.costFunction) {
          case 0: return emptyCellBetweenPolyAndLeftHorizon(poly, pos, rast_i);
          case 1: return maxXofPoly(poly, pos, rast_i);
          case 2: return costXWithPenaltyOnY(poly, pos, rast_i);
          }
          return 0;
      }

      //returns the score relative to the bottom horizon of that poly in that particular position, taking into account the choosen algo
      int getCostY(RasterizedOutline2& poly, Point2i pos, int rast_i) {
          switch (params.costFunction) {
          case 0: return emptyCellBetweenPolyAndBottomHorizon(poly, pos, rast_i);
          case 1: return maxYofPoly(poly, pos, rast_i);
          case 2: return costYWithPenaltyOnX(poly, pos, rast_i);
          }
          return 0;
      }

      //given a poly and the column at which it is placed,
      //this returns the Y at which the wasted space is minimum
      //i.e. the Y at which the polygon touches the horizon
      int dropY(RasterizedOutline2& poly, int col, int rast_i) {
          int tmp = INT_MAX;
          int adjacentIndex = 0;
          std::vector<int>& bottom = poly.getBottom(rast_i);

          //look for for index of the column at which the poly touches the bottom horizon first
          for (size_t i = 0; i < bottom.size(); ++i) {
              int diff = bottom[i] - mBottomHorizon[col + i];
              if (diff < tmp) {
                  adjacentIndex = i;
                  tmp = diff;
              }
          }
          //return the lowest Y of the dropped poly
          return mBottomHorizon[col + adjacentIndex] - bottom[adjacentIndex];
      }

      //given a poly and the row at which it is placed,
      //this returns the X at which the wasted space is minimum
      //i.e. the X at which the polygon touches the left horizon
      int dropX(RasterizedOutline2& poly, int row, int rast_i) {
          int tmp = INT_MAX;
          int adjacentIndex = 0;
          std::vector<int>& left = poly.getLeft(rast_i);

          //look for for index of the column at which the poly touches the left horizon first
          for (size_t i = 0; i < left.size(); ++i) {
              int diff = left[i] - mLeftHorizon[row + i];
              if (diff < tmp) {
                  adjacentIndex = i;
                  tmp = diff;
              }
          }
          //and return the lowest X of the dropped poly
          return mLeftHorizon[row + adjacentIndex] - left[adjacentIndex];
      }

      int costYWithPenaltyOnX(RasterizedOutline2& poly, Point2i pos, int rast_i) {
          std::vector<int>& left = poly.getLeft(rast_i);

          //get the standard cost on X axis
          int score = emptyCellBetweenPolyAndBottomHorizon(poly, pos, rast_i);

          //apply a penalty if the poly is the poly is far from the left horizon
          //thus preferring poly which are closer to the left horizon
          for (size_t i = 0; i < left.size(); ++i) {
              //ASSUMPTION: if the poly is (partially/fully) under the left horizon,
              //then we will count this as a good thing (subtracting a quantity from the cost) but since we don't have
              //a grid holding the current state of the packing field, we don't know the position of the polygons at our left side,
              //so we ASSUME that there isn't any polygon between the poly we're considering and the Y axis of the packing field,
              //and count the number of cells between us and the RIGHT end the packing field
              //(NOTE: ^^^^^^^ this implies that the closer we are to the left horizon, the lower the cost will get)
              if (pos.X() + left[i] < mLeftHorizon[pos.Y() + i])
                  //number of cells between us and the RIGHT end the packing field
                  score -= mSize.X() - pos.X() - left[i];
              else         //the number of cells between the bottom side of the poly at the (posY+i)-th row and the value of the horizon in that row
                  score += pos.X() + left[i] - mLeftHorizon[pos.Y() + i];
          }
          return score;
      }

      //returns the number of empty cells between poly's bottom side and the bottom horizon
      int emptyCellBetweenPolyAndBottomHorizon(RasterizedOutline2& poly, Point2i pos, int rast_i)
      {
          std::vector<int>& bottom = poly.getBottom(rast_i);
          int score = 0;
          int min = INT_MAX;

          //count the number of empty cells between poly's bottom side and the bottom horizon
          for (size_t i = 0; i < bottom.size(); ++i) {
              int diff = bottom[i] - mBottomHorizon[pos.X() + i];
              score += diff;
              if (diff < min) min = diff;
          }
          score += (-min*bottom.size());

          return score;
      }

      int costXWithPenaltyOnY(RasterizedOutline2& poly, Point2i pos, int rast_i) {
          std::vector<int>& bottom = poly.getBottom(rast_i);

          //get the standard cost on X axis
          int score = emptyCellBetweenPolyAndLeftHorizon(poly, pos, rast_i);

          //apply a penalty if the poly is the poly is far from the bottom horizon
          //thus preferring poly which are closer to the bottom horizon
          for (size_t i = 0; i < bottom.size(); ++i) {
              //ASSUMPTION: if the poly is (partially/fully) under the bottom horizon,
              //then we will count this as a good thing (subtracting a quantity from the cost) but since we don't have
              //a grid holding the current state of the packing field, we don't know the position of the polygons beneath us,
              //so we ASSUME that there isn't any polygon between the poly we're considering and the X axis of the packing field,
              //and count the number of cells between us and the TOP end the packing field
              //(NOTE: ^^^^^^^ this implies that the closer we are to the bottom horizon, the lower the cost will get)
              if (pos.Y() + bottom[i] < mBottomHorizon[pos.X() + i])
                  //number of cells between us and the TOP side the packing field
                  score -= (mSize.Y() - pos.Y() - bottom[i]);
              else         //the number of cells between the left side of the poly at the (posX+i)-th column and the value of the horizon in that column
                  score += pos.X() + bottom[i] - mBottomHorizon[pos.X() + i];
          }
          return score;
      }

      int maxYofPoly(RasterizedOutline2& poly, Point2i pos, int rast_i)
      {
          return pos.Y() + poly.gridHeight(rast_i);
      }

      int maxXofPoly(RasterizedOutline2& poly, Point2i pos, int rast_i)
      {
          return pos.X() + poly.gridWidth(rast_i);
      }

      //returns the number of empty cells between poly's left side and the left horizon
      int emptyCellBetweenPolyAndLeftHorizon(RasterizedOutline2& poly, Point2i pos, int rast_i)
      {
          std::vector<int>& left = poly.getLeft(rast_i);

          int score = 0;
          int min = INT_MAX;

          //count the number of empty cells between poly's left side and the left horizon
          for (size_t i = 0; i < left.size(); ++i) {
              int diff = left[i] - mLeftHorizon[pos.Y() + i];
              score += diff;
              if (diff < min) min = diff;
          }
          score += (-min*left.size());

          return score;
      }

      //updates the horizons according to the chosen position
      void placePoly(RasterizedOutline2& poly, Point2i pos, int rast_i) {

          std::vector<int>& bottom = poly.getBottom(rast_i);
          std::vector<int>& deltaY = poly.getDeltaY(rast_i);
          std::vector<int>& left = poly.getLeft(rast_i);
          std::vector<int>& deltaX = poly.getDeltaX(rast_i);

          //update bottom horizon
          for (int i = 0; i < poly.gridWidth(rast_i); i++) {
              //tmpHor = the highest Y reached by the poly, RELATIVE TO the packing field coords system
              int tmpHor = pos.Y() + bottom[i] + deltaY[i];
              //only update the horizon if it's higher than this value
              if (tmpHor > mBottomHorizon[pos.X() + i]) mBottomHorizon[pos.X() + i] = tmpHor;
          }

          if (params.costFunction != Parameters::MixedCost
                  && !params.doubleHorizon) return;

          //update leftHorizon
          for (int i = 0; i < poly.gridHeight(rast_i); i++) {
              //tmpHor = the highest X reached by the poly, RELATIVE TO the packing field coords system
              int tmpHor = pos.X() + left[i] + deltaX[i];
              //only update the horizon if it's higher than this value
              if (tmpHor > mLeftHorizon[pos.Y() + i]) mLeftHorizon[pos.Y() + i] = tmpHor;
          }
      }
  };


    static bool Pack(std::vector< std::vector< Point2x>  > &polyPointsVec,
                     Point2i containerSize,
                     std::vector<Similarity2x> &trVec,
                     const Parameters &packingPar)
    {
      std::vector<Point2i> containerSizes(1,containerSize);
      std::vector<int> polyToContainer;
      return Pack(polyPointsVec,containerSizes,trVec,polyToContainer,packingPar);
    }

    static bool Pack(std::vector< std::vector< Point2x>  > &polyPointsVec,
                     const std::vector<Point2i> &containerSizes,
                     std::vector<Similarity2x> &trVec,
                     std::vector<int> &polyToContainer,
                     const Parameters &packingPar)
    {

      int containerNum=containerSizes.size();

        float gridArea = 0;
        //if containerSize isn't multiple of cell size, crop the grid (leaving containerSize as it is)
        for (int i = 0; i < containerNum; i++) {
            Point2i gridSize(containerSizes[i].X() / packingPar.cellSize,
                             containerSizes[i].Y() / packingPar.cellSize);

            gridArea += (gridSize.X()*packingPar.cellSize * gridSize.Y()*packingPar.cellSize);
        }

        float totalArea = 0;
        for (size_t j = 0; j < polyPointsVec.size(); j++) {
            float curArea = tri::OutlineUtil<SCALAR_TYPE>::Outline2Area(polyPointsVec[j]);
            if(curArea<0) tri::OutlineUtil<SCALAR_TYPE>::ReverseOutline2(polyPointsVec[j]);
            totalArea += fabs(curArea);
        }

        //we first set it to the "optimal" scale
        float optimalScale = sqrt(gridArea/ totalArea);
        float currScale = optimalScale;
        float latestFailScale = 0;

        bool ret = false;
        //we look for the first scale factor which makes the packing algo succeed
        //we will use this value in the bisection method afterwards
        ret = PolyPacking(polyPointsVec, containerSizes, trVec, polyToContainer, packingPar, currScale);
        while (!ret) {
            latestFailScale = currScale;
            currScale *= 0.60;
            ret = PolyPacking(polyPointsVec, containerSizes, trVec, polyToContainer, packingPar, currScale);
        }
        //if it managed to pack with the optimal scale (VERY unlikely), just leave
        if (currScale == optimalScale) return true;

        //BISECTION METHOD
        float latestSuccessScale = currScale;
        float tmpScale = (latestSuccessScale + latestFailScale) / 2;
        while ( (latestFailScale / latestSuccessScale) - 1 > 0.001
                || ((latestFailScale / latestSuccessScale) - 1 < 0.001 && !ret) ) {

            tmpScale = (latestSuccessScale + latestFailScale) / 2;
            ret = PolyPacking(polyPointsVec, containerSizes, trVec, polyToContainer, packingPar, tmpScale);
            if (ret) latestSuccessScale = tmpScale;
            else latestFailScale = tmpScale;
        }

        float finalArea = 0;
        //compute occupied area
        for (size_t j = 0; j < polyPointsVec.size(); j++) {
            std::vector<Point2f> oldPoints = polyPointsVec[j];
            for (size_t k = 0; k < oldPoints.size(); k++) {
                oldPoints[k].Scale(latestSuccessScale, latestSuccessScale);
            }
            finalArea +=  tri::OutlineUtil<SCALAR_TYPE>::Outline2Area(oldPoints);
        }
//        printf("PACKING EFFICIENCY: %f with scale %f\n", finalArea/gridArea, latestSuccessScale);
    }

    //tries to pack polygons using the given gridSize and scaleFactor
    //stores the result, i.e. the vector of similarities, in trVec
    static bool PolyPacking(std::vector< std::vector< Point2x>  > &outline2Vec,
                            const std::vector<Point2i> &containerSizes,
                            std::vector<Similarity2x> &trVec,
                            std::vector<int> &polyToContainer,
                            const Parameters &packingPar,
                            float scaleFactor)
    {
        int containerNum = containerSizes.size();

        polyToContainer.clear();
        polyToContainer.resize(outline2Vec.size());
        trVec.resize(outline2Vec.size());

        //create packing fields, one for each container
        std::vector<Point2i> gridSizes;
        std::vector<packingfield> packingFields;
        for (int i=0; i < containerNum; i++) {
            //if containerSize isn't multiple of cell size, crop the grid (leaving containerSize as it is)
            gridSizes.push_back(Point2i(containerSizes[i].X() / packingPar.cellSize,
                                        containerSizes[i].Y() / packingPar.cellSize));

            packingfield one(gridSizes[i], packingPar);
            packingFields.push_back(one);
        }

        //create the vector of polys, starting for the poly points we received as parameter
        std::vector<RasterizedOutline2> polyVec(outline2Vec.size());
        for(size_t i=0;i<polyVec.size();i++) {
            polyVec[i].setPoints(outline2Vec[i]);
        }

        // Build a permutation that holds the indexes of the polys ordered by their area
        std::vector<int> perm(polyVec.size());
        for(size_t i=0;i<polyVec.size();i++) perm[i] = i;
        sort(perm.begin(),perm.end(),ComparisonFunctor<float>(polyVec));

//        printf("BEGIN OF PACKING\n");

        // **** First Step: Rasterize all the polygons ****
        for (size_t i = 0; i < polyVec.size(); i++) {
            polyVec[i].resetState(packingPar.rotationNum);
            for (int rast_i = 0; rast_i < packingPar.rotationNum/4; rast_i++) {
                //create the rasterization (i.e. fills bottom/top/grids/internalWastedCells arrays)
                RASTERIZER_TYPE::rasterize(polyVec[i],scaleFactor, rast_i,packingPar.rotationNum,packingPar.cellSize);
            }
        }

        // **** Second Step: iterate on the polys, and try to find the best position ****
        for (size_t currPoly = 0; currPoly < polyVec.size(); currPoly++) {

            int i = perm[currPoly];
            int bestRastIndex = -1;
            int bestCost = INT_MAX;
            int bestPolyX = -1;
            int bestPolyY = -1;
            int bestContainer = -1; //the container where the poly fits best

            //try all the rasterizations and choose the best fitting one
            for (int rast_i = 0; rast_i < packingPar.rotationNum; rast_i++) {

                //try to fit the poly in all containers, in all valid positions
                for (int grid_i = 0; grid_i < containerNum; grid_i++) {
                    int maxCol = gridSizes[grid_i].X() - polyVec[i].gridWidth(rast_i);
                    int maxRow = gridSizes[grid_i].Y() - polyVec[i].gridHeight(rast_i);

                    //look for the best position, dropping from top
                    for (int col = 0; col < maxCol; col++) {
                        //get the Y at which the poly touches the horizontal horizon
                        int currPolyY = packingFields[grid_i].dropY(polyVec[i],col, rast_i);

                        if (currPolyY + polyVec[i].gridHeight(rast_i) > gridSizes[grid_i].Y()) {
                            //skip this column, as the poly would go outside the grid if placed here
                            continue;
                        }

                        int currCost = packingFields[grid_i].getCostX(polyVec[i], Point2i(col, currPolyY), rast_i) +
                                packingFields[grid_i].getCostY(polyVec[i], Point2i(col, currPolyY), rast_i);

                        //if this rasterization is better than what we found so far
                        if (currCost < bestCost) {
                            bestContainer = grid_i;
                            bestCost = currCost;
                            bestRastIndex = rast_i;
                            bestPolyX = col;
                            bestPolyY = currPolyY;
                        }
                    }

                    if (!packingPar.doubleHorizon) continue;

                    for (int row = 0; row < maxRow; row++) {
                        //get the Y at which the poly touches the horizontal horizon
                        int currPolyX = packingFields[grid_i].dropX(polyVec[i],row, rast_i);

                        if (currPolyX + polyVec[i].gridWidth(rast_i) > gridSizes[grid_i].X()) {
                            //skip this column, as the poly would go outside the grid if placed here
                            continue;
                        }

                        int currCost = packingFields[grid_i].getCostY(polyVec[i], Point2i(currPolyX, row), rast_i) +
                                packingFields[grid_i].getCostX(polyVec[i], Point2i(currPolyX, row), rast_i);

                        //if this rasterization fits better than those we tried so far
                        if (currCost < bestCost) {
                            bestContainer = grid_i;
                            bestCost = currCost;
                            bestRastIndex = rast_i;
                            bestPolyX = currPolyX;
                            bestPolyY = row;
                        }
                    }
                }
            }

            //if we couldn't find a valid position for the poly return false, as we couldn't pack with the current scaleFactor
            if (bestRastIndex == -1) {
//                printf("Items didn't fit using %f as scaleFactor\n", scaleFactor);
                return false;
            }

            //we found the best position for a given poly,
            //let's place it, so that the horizons are updated accordingly
            packingFields[bestContainer].placePoly(polyVec[i], Point2i(bestPolyX, bestPolyY), bestRastIndex);

            //create the rotated bb which we will use to set the similarity translation prop
            float angleRad = float(bestRastIndex)*(M_PI*2.0)/float(packingPar.rotationNum);
            Box2f bb;
            std::vector<Point2f> points = polyVec[i].getPoints();
            for(size_t i=0;i<points.size();++i) {
                Point2f pp=points[i];
                pp.Rotate(angleRad);
                bb.Add(pp);
            }

            //associate the poly to the container where it fitted best
            polyToContainer[i] = bestContainer;

            //now we have bestPolyX/bestRastIndex
            //we have to update the similarities vector accordingly!
            float polyXInImgCoords = bestPolyX*packingPar.cellSize;
            float scaledBBWidth = bb.DimX()*scaleFactor;
            float polyWidthInImgCoords = polyVec[i].gridWidth(bestRastIndex)*packingPar.cellSize;
            float offsetX = (polyWidthInImgCoords - ceil(scaledBBWidth))/2.0;
            float scaledBBMinX = bb.min.X()*scaleFactor;

            //note: bestPolyY is 0 if the poly is at the bottom of the grid
            float imgHeight = containerSizes[bestContainer].Y();
            float polyYInImgCoords = bestPolyY*packingPar.cellSize;
            float polyHeightInImgCoords = polyVec[i].gridHeight(bestRastIndex)*packingPar.cellSize;
            float topPolyYInImgCoords = polyYInImgCoords + polyHeightInImgCoords;
            float scaledBBHeight = bb.DimY()*scaleFactor;
            float offsetY = (polyHeightInImgCoords - ceil(scaledBBHeight))/2.0;
            float scaledBBMinY = bb.min.Y()*scaleFactor;
            trVec[i].tra = Point2f(polyXInImgCoords - scaledBBMinX + offsetX,
                                   imgHeight - topPolyYInImgCoords - scaledBBMinY + offsetY);
            trVec[i].rotRad = angleRad;
            trVec[i].sca = scaleFactor;
        }

        //sort polyToContainer and trVec so that we have them ordered for dumping
//        printf("SUCCESSFULLY PACKED with scaleFactor %f\n", scaleFactor);
        return true;
    }

    static void printVector(std::vector<int>& vec) {
        for (size_t i = 0; i < vec.size(); i++) {
            printf("%d", vec[i]);
        }
        printf("\n");
    }
}; // end class



} // end namespace vcg

#endif // NEW_POLYPACKER_H
