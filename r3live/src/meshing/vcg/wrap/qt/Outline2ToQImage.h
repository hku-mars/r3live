#ifndef POLYTOQIMAGE_H
#define POLYTOQIMAGE_H
#include <QImage>
#include <QSvgGenerator>
#include <QPainter>
#include <vcg/space/point2.h>
#include <vcg/space/color4.h>
#include <vcg/space/box2.h>
#include <vcg/math/similarity2.h>

///this class is used to pass global
///parameters to the polygonal dumper

///this class is used to draw polygons on an image could be vectorial or not
class Outline2Dumper
{
public:
  static float MM2PT(const float valueMM, float dpi)
  {
    float valueInch = valueMM / 25.4f;
    return valueInch * dpi;
  }

  class Param
  {
  public:
      /// the backgrround color
      vcg::Color4b backgroundColor;
      /// true if the polygons must be filled with color
      bool fill;
      /// true if the filling color is random
      bool randomColor;
      /// the filling color of polygons, used only if randomColor==false
      vcg::Color4b FillColor;
      /// The size of the font. If zero (default) it is automatically chosen.
      int fontSize;
      /// dimension of the image (in PNG are pixels, while in SVG is the workspace in points)
      int width;
      int height;
      vcg::Color4b labelColor;
      vcg::Color4b lineColor;

      /// DPI resolution, used only for SVG
      int dpi;
      float penWidth;

      void SetSVGPenInMM(float widthMM)
      {
        float widthInch = widthMM/25.4f;
        penWidth = widthInch*dpi;
      }

      ///Handy function for setting the size of the drawing
      void SetSVGDimInMm(float widthMM,float heightMM,float _dpi=72)
      {
        dpi=_dpi;
        width = MM2PT(widthMM,dpi);
        height = MM2PT(heightMM,dpi);
      }

      ///default constructor
      Param()
      {
          backgroundColor = vcg::Color4b::Gray;
          width=1024;
          height=1024;
          dpi=72;
          fontSize=0;
          fill=false;
          randomColor=true;
          FillColor=vcg::Color4b(0,0,0,255);
          lineColor=vcg::Color4b::Black;
          labelColor=vcg::Color4b::Black;

      }
  };
private:
	///this class draw a black mask fora given polygon, cenetered and scaled to fit with
	///the image size, it return the transformation to tranform back the polygon to 2D space
	static void DrawPolygonMask(const std::vector< std::vector<vcg::Point2f> > &polyVec,QImage &img,
								vcg::Similarity2f &ret,const vcg::Similarity2f &trans);

	///return the max radius of a point inside a polygon ,given the mask image
	///actually it evaluate the maximum bounding box
	static int getMaxMaskRadius(int x,int y,QImage &img);
public:

	///return the point inside the polygon with the bigger distance to the border,
	///this is used to write labels within the polygon, it handle polygons with holes too
	static vcg::Point2f GetIncenter(const std::vector< std::vector<vcg::Point2f> > &polyVec,
										const vcg::Similarity2f &tra1, float &radius, int resolution=100);

	static void rectSetToOutline2Vec(std::vector< vcg::Box2f > &rectVec, std::vector< std::vector<vcg::Point2f> > &polyVec);
	static void multiRectSetToSingleOutline2Vec(std::vector< vcg::Box2f > &rectVec, std::vector<vcg::Similarity2f> &trVec, std::vector<int> &indVec,
											int ind, std::vector< std::vector<vcg::Point2f> > &polyVec, std::vector<vcg::Similarity2f> &trPolyVec);
	static void multiOutline2VecToSingleOutline2Vec(const std::vector<std::vector<vcg::Point2f> > &multipolyVec,  const std::vector<vcg::Similarity2f> &trVec, const std::vector<int> &indVec,
												 int ind, std::vector< std::vector< vcg::Point2f> > &polyVec, std::vector< vcg::Similarity2f> &trPolyVec);

	///write a polygon on a PNG file, format of the polygon is vector of vector of contours...nested contours are holes
	///takes the name of the image in input, the set of polygons, the set of per polygons transformation,
	///the label to be written and the global parameter for drawing style
	static void dumpOutline2VecPNG(const char * imageName, std::vector< std::vector< std::vector<vcg::Point2f> > > &polyVecVec,
							std::vector<vcg::Similarity2f> &trVec, std::vector<std::vector<std::string> > &labelVecVec, Param &pp);
	//write a polygon on a SVG file, format of the polygon is vector of vector of contours...nested contours are holes
	///takes the name of the image in input, the set of polygons, the set of per polygons transformation,
	///the label to be written and the global parameter for drawing style
	static void dumpOutline2VecSVG(const char * imageName, std::vector< std::vector< std::vector<vcg::Point2f> > > &polyVecVec,
							   std::vector<vcg::Similarity2f> &trVec, std::vector< std::vector<std::string> > &labelVecVec,
							   std::vector<std::vector<vcg::Similarity2f> > &labelTrVecVec, std::vector<std::vector<float> > &labelRadVec, Param &pp);

	static void dumpOutline2VecPNG(const char * imageName, std::vector< std::vector< std::vector<vcg::Point2f> > > &polyVecVec,
							std::vector<vcg::Similarity2f> &trVec, std::vector<std::string> &labelVec, Param &pp);
	static void dumpOutline2VecSVG(const char * imageName, std::vector< std::vector< std::vector<vcg::Point2f> > > &polyVecVec,
							   std::vector<vcg::Similarity2f> &trVec, std::vector<std::string> &labelVec, Param &pp);

	static void dumpOutline2VecPNG(const char * imageName, std::vector< std::vector< std::vector<vcg::Point2f> > > &polyVecVec,
							std::vector<vcg::Similarity2f> &trVec, Param &pp);
	static void dumpOutline2VecSVG(const char * imageName, std::vector< std::vector< std::vector<vcg::Point2f> > > &polyVecVec,
							std::vector<vcg::Similarity2f> &trVec, Param &pp);

	static void dumpOutline2VecPNG(const char * imageName, std::vector<  std::vector<vcg::Point2f> > &polyVecVec,
							std::vector<vcg::Similarity2f> &trVec, Param &pp);
	static void dumpOutline2VecSVG(const char * imageName, std::vector<  std::vector<vcg::Point2f> > &outline2Vec,
							std::vector<vcg::Similarity2f> &trVec, Param &pp);
};
#endif // POLYTOQIMAGE_H
