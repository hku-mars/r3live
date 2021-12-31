#include <wrap/qt/outline2_rasterizer.h>
#include <wrap/qt/col_qt_convert.h>
#include "stdio.h"
#include "math.h"
#include <vcg/space/color4.h>
#include <wrap/qt/col_qt_convert.h>

using namespace vcg;
using namespace std;

void QtOutline2Rasterizer::rasterize(RasterizedOutline2 &poly,
                                 float scale,
                                 int rast_i,
                                 int rotationNum,
                                 int cellSize)
{

    float rotRad = M_PI*2.0f*float(rast_i) / float(rotationNum);

    //get polygon's BB, rotated according to the input parameter
    Box2f bb;
    vector<Point2f> pointvec = poly.getPoints();
    for(size_t i=0;i<pointvec.size();++i) {
        Point2f pp=pointvec[i];
        pp.Rotate(rotRad);
        bb.Add(pp);
    }

    ///CREATE ITS GRID. The grid has to be a multiple of CELLSIZE because this grid's cells have size CELLSIZE
    //we'll make so that sizeX and sizeY are multiples of CELLSIZE:
    //1) we round it to the next integer
    //2) add the number which makes it a multiple of CELLSIZE (only if it's not multiple already)
    int sizeX = (int)ceil(bb.DimX()*scale);
    int sizeY = (int)ceil(bb.DimY()*scale);
    if (sizeX % cellSize != 0) sizeX += (cellSize - ((int)ceil(bb.DimX()*scale) % cellSize));
    if (sizeY % cellSize != 0) sizeY += (cellSize - ((int)ceil(bb.DimY()*scale) % cellSize));

    //security measure: add a dummy column/row thus making the image bigger, and crop it afterwards
    //(if it hasn't been filled with anything)
    //this is due to the fact that if we have a rectangle which has bb 39.xxx wide, then it won't fit in a 40px wide QImage!! The right side will go outside of the image!! :/
    sizeX+=cellSize;
    sizeY+=cellSize;

    QImage img(sizeX,sizeY,QImage::Format_RGB32);
    QColor backgroundColor(Qt::transparent);
    img.fill(backgroundColor);

    ///SETUP OF DRAWING PROCEDURE
    QPainter painter;
    painter.begin(&img);
    QBrush br;
    br.setStyle(Qt::SolidPattern);
    QPen qp;
    qp.setWidthF(0);
    qp.setColor(Qt::yellow);
    painter.setBrush(br);
    painter.setPen(qp);

    painter.resetTransform();
    painter.translate(QPointF(-(bb.min.X()*scale) , -(bb.min.Y()*scale) ));
    painter.rotate(math::ToDeg(rotRad));
    painter.scale(scale,scale);

    //create the polygon to print it
    QVector<QPointF> points;
    vector<Point2f> newpoints = poly.getPoints();
    for (size_t i = 0; i < newpoints.size(); i++) {
        points.push_back(QPointF(newpoints[i].X(), newpoints[i].Y()));
    }
    painter.drawPolygon(QPolygonF(points));


    //CROPPING: it is enough to check for the (end - cellSize - 1)th row/col of pixels, if they're all black we can eliminate the last 8columns/rows of pixels
    bool cropX = true;
    bool cropY = true;
    for (int j=0; j<img.height(); j++) {
        const uchar* line = img.scanLine(j);
        if (j == img.height() - (cellSize - 1) - 1  ) {
            for (int x=0; x<img.width(); x++) {
                if (((QRgb*)line)[x] != backgroundColor.rgb()) {
                    cropY = false;
                    break;
                }
            }
        }
        else {
            if (((QRgb*)line)[img.width() - (cellSize - 1) - 1] != backgroundColor.rgb()) {
                cropX = false;
                break;
            }
        }
        if (!cropY) break;
    }


    if (cropX || cropY) {
        painter.end();
        img = img.copy(0, 0, img.width() - cellSize * cropX, img.height() - cellSize * cropY);
        painter.begin(&img);
        painter.setBrush(br);
        painter.setPen(qp);
    }


    //draw the poly for the second time, this time it is centered to the image
    img.fill(backgroundColor);

    painter.resetTransform();
    painter.translate(QPointF(-(bb.min.X()*scale) + (img.width() - ceil(bb.DimX()*scale))/2.0, -(bb.min.Y()*scale) + (img.height() - ceil(bb.DimY()*scale))/2.0));
    painter.rotate(math::ToDeg(rotRad));
    painter.scale(scale,scale);
    //create the polygon to print it
    QVector<QPointF> points2;
    vector<Point2f> newpoints2 = poly.getPoints();
    for (size_t i = 0; i < newpoints2.size(); i++) {
        points2.push_back(QPointF(newpoints2[i].X(), newpoints2[i].Y()));
    }
    painter.drawPolygon(QPolygonF(points2));

    //create the first grid, which will then be rotated 3 times.
    //we will reuse this grid to create the rasterizations corresponding to this one rotated by 90/180/270°
    vector<vector<int> > tetrisGrid;
    QRgb yellow = QColor(Qt::yellow).rgb();
    int gridWidth = img.width() / cellSize;
    int gridHeight = img.height() / cellSize;
    int x = 0;
    tetrisGrid.resize(gridHeight);
    for (int k = 0; k < gridHeight; k++) {
        tetrisGrid[k].resize(gridWidth, 0);
    }
    for (int y = 0; y < img.height(); y++) {
        int gridY = y / cellSize;
        const uchar* line = img.scanLine(y);
        x = 0;
        int gridX = 0;
        while(x < img.width()) {
            gridX = x/cellSize;
            if (tetrisGrid[gridY][gridX] == 1) {
                x+= cellSize - (x % cellSize); //align with the next x
                continue;
            }
            if (((QRgb*)line)[x] == yellow) tetrisGrid[gridY][gridX] = 1;
            ++x;
        }
    }

    //create the 4 rasterizations (one every 90°) using the discrete representation grid we've just created
    int rotationOffset = rotationNum/4;
    for (int j = 0; j < 4; j++) {
        if (j != 0)  {
            tetrisGrid = rotateGridCWise(tetrisGrid);
        }
        //add the grid to the poly's vector of grids
        poly.getGrids(rast_i + rotationOffset*j) = tetrisGrid;

        //initializes bottom/left/deltaX/deltaY vectors of the poly, for the current rasterization
        poly.initFromGrid(rast_i + rotationOffset*j);
    }

    painter.end();
}

// rotates the grid 90 degree clockwise (by simple swap)
// used to lower the cost of rasterization.
vector<vector<int> > QtOutline2Rasterizer::rotateGridCWise(vector< vector<int> >& inGrid) {
    vector<vector<int> > outGrid(inGrid[0].size());
    for (size_t i = 0; i < inGrid[0].size(); i++) {
        outGrid[i].reserve(inGrid.size());
        for (size_t j = 0; j < inGrid.size(); j++) {
            outGrid[i].push_back(inGrid[inGrid.size() - j - 1][i]);
        }
    }
    return outGrid;
}
