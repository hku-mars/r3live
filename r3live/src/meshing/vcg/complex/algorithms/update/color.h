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

#ifndef __VCG_TRI_UPDATE_COLOR
#define __VCG_TRI_UPDATE_COLOR

#include <limits>
#include <math.h>
#include <time.h>
#include <vcg/space/color4.h>
#include <vcg/math/histogram.h>
#include <vcg/math/perlin_noise.h>
#include <vcg/math/random_generator.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/stat.h>

namespace vcg {
namespace tri {

/*!
\ingroup trimesh

\headerfile color.h vcg/complex/algorithms/update/color.h

\brief Generation and processing of per-vertex and per-face colors according to various strategy.

This class is used to compute per face or per vertex color with respect to a number of algorithms.
There is a wide range of algorithms for processing vertex color in a \i photoshop-like mode (changing for example contrast, white balance, gamma)
Basic Tools for mapping quality into a color according to standard color ramps are here.
*/

template <class MeshType>
class UpdateColor
{
public:
  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::FaceType       FaceType;
  typedef typename MeshType::FacePointer    FacePointer;
  typedef typename MeshType::FaceIterator   FaceIterator;
  typedef typename MeshType::ScalarType     ScalarType;

  class ColorAvgInfo
  {
  public:
    unsigned int r;
    unsigned int g;
    unsigned int b;
    unsigned int a;
    int cnt;
  };

  /*! \brief This function colores all (or the selected) the vertices of a mesh.
    */
  static int PerVertexConstant(MeshType &m, Color4b vs=Color4b::White,bool selected=false)
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");

    int cnt=0;
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
      if(!(*vi).IsD()){
        if(!selected || (*vi).IsS())
        {
          (*vi).C() = vs;
          ++cnt;
        }
      }
    return cnt;
  }

  /*! \brief This function colores all (or the selected) faces of a mesh.
  */
  static int PerFaceConstant(MeshType &m, Color4b vs=Color4b::White,bool selected=false)
  {
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");
    int cnt=0;
    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
      if(!(*fi).IsD()){
        if(!selected || (*fi).IsS())
        {
          (*fi).C() = vs;
          ++cnt;
        }
      }
    return cnt;
  }

  /** \brief Transfer face color onto vertex color

  Plain average of the color of the faces incident on a given vertex.
  No adjacency required.
  */
  static void PerVertexFromFace( MeshType &m)
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");

    ColorAvgInfo csi;
    csi.r=0; csi.g=0; csi.b=0; csi.cnt=0;
    SimpleTempData<typename MeshType::VertContainer, ColorAvgInfo> TD(m.vert,csi);

    FaceIterator fi;
    for(fi=m.face.begin();fi!=m.face.end();++fi)
      if(!(*fi).IsD())
        for(int j=0;j<3;++j)
        {
          TD[(*fi).V(j)].r+=(*fi).C()[0];
          TD[(*fi).V(j)].g+=(*fi).C()[1];
          TD[(*fi).V(j)].b+=(*fi).C()[2];
          TD[(*fi).V(j)].a+=(*fi).C()[3];
          ++TD[(*fi).V(j)].cnt;
        }

    VertexIterator vi;
    for(vi=m.vert.begin();vi!=m.vert.end();++vi)
      if(!(*vi).IsD() && TD[*vi].cnt>0 )
      {
        (*vi).C()[0] = TD[*vi].r / TD[*vi].cnt;
        (*vi).C()[1] = TD[*vi].g / TD[*vi].cnt;
        (*vi).C()[2] = TD[*vi].b / TD[*vi].cnt;
        (*vi).C()[3] = TD[*vi].a / TD[*vi].cnt;
      }
  }

  /*! \brief Transfer vertex color onto face color
  Plain average of the color of the vertexes on a given face.
  */
  static void PerFaceFromVertex( MeshType &m)
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");

    FaceIterator fi;
    for(fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
    {
      Color4f avg = (Color4f::Construct((*fi).V(0)->C()) +
                     Color4f::Construct((*fi).V(1)->C()) +
                     Color4f::Construct((*fi).V(2)->C()) )/ 3.0;
      (*fi).C().Import(avg);
    }
  }

  /*! \brief This function colores all the faces of a mesh with a hue color shade dependent on the quality.

  If no range of quality is passed it is automatically computed.
  */
  static void PerVertexQualityRamp(MeshType &m, float minq=0, float maxq=0)
  {
    if(!HasPerVertexQuality(m)) throw MissingComponentException("PerVertexQuality");
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");

    if(minq==maxq)
    {
      std::pair<float,float> minmax = Stat<MeshType>::ComputePerVertexQualityMinMax(m);
      minq=minmax.first;
      maxq=minmax.second;
    }
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
          if(!(*vi).IsD())
              (*vi).C().SetColorRamp(minq,maxq,(*vi).Q());
  }

  /*! \brief This function colores all the faces of a mesh with a hue color shade dependent on the quality.

  If no range of quality is passed it is automatically computed.
  */
  static void PerFaceQualityRamp(MeshType &m, float minq=0, float maxq=0, bool selected=false)
  {
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");
    if(!HasPerFaceQuality(m)) throw MissingComponentException("PerFaceQuality");

    if(minq==maxq)
    {
      std::pair<float,float> minmax = Stat<MeshType>::ComputePerFaceQualityMinMax(m);
      minq=minmax.first;
      maxq=minmax.second;
    }
    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
      if(!selected || (*fi).IsS())
        (*fi).C().SetColorRamp(minq,maxq,(*fi).Q());
  }

  /*! \brief This function colores all the vertices of a mesh with a gray shade dependent on the quality.

  If no range of quality is passed it is automatically computed.
  */
  static void PerVertexQualityGray(MeshType &m,  float minq,  float maxq)
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    if(!HasPerVertexQuality(m)) throw MissingComponentException("PerVertexQuality");
    if(minq==maxq)
    {
      std::pair<float,float> minmax = Stat<MeshType>::ComputePerVertexQualityMinMax(m);
      minq=minmax.first;
      maxq=minmax.second;
    }
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
          if(!(*vi).IsD())
              (*vi).C().SetGrayShade( ((*vi).Q()-minq)/(maxq-minq));
  }

  /*! \brief This function colores all the faces of a mesh with a gray shade dependent on the quality.

  If no range of quality is passed it is automatically computed.
  */
  static void PerFaceQualityGray(MeshType &m, float minq=0, float maxq=0)
  {
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");
    if(!HasPerFaceQuality(m)) throw MissingComponentException("PerFaceQuality");

    if(minq==maxq)
    {
      std::pair<float,float> minmax = Stat<MeshType>::ComputePerFaceQualityMinMax(m);
      minq=minmax.first;
      maxq=minmax.second;
    }
    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
          (*fi).C().SetGrayShade( ((*fi).Q()-minq)/(maxq-minq));
  }

  /** \brief Color the vertexes of the mesh that are on the border

It uses the information in the Vertex flags, and not necessarily any topology.
So it just require that you have correctly computed the flags; one way could be the following one:
\code
vcg::tri::UpdateTopology<Mesh>::FaceFace(m.cm);
vcg::tri::UpdateFlags<Mesh>::FaceBorderFromFF(m.cm);
vcg::tri::UpdateFlags<Mesh>::VertexBorderFromFace (m.cm);
vcg::tri::UpdateColor<Mesh>::PerVertexBorderFlag(m.cm);
\endcode
*/
  static void PerVertexBorderFlag( MeshType &m, Color4b BorderColor=Color4b::Blue, Color4b InternalColor=Color4b::White, Color4b MixColor=Color4b::Cyan)
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    Color4b BaseColor = Color4b::Green;

    VertexConstant(m,BaseColor);
    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
      for(int j=0;j<3;++j)
      {
        if((*fi).IsB(j)){
          if( (*fi).V(j)->C() == BaseColor)     (*fi).V(j)->C() = BorderColor;
          if( (*fi).V(j)->C() == InternalColor) (*fi).V(j)->C() = MixColor;
          if( (*fi).V1(j)->C() == BaseColor)     (*fi).V1(j)->C() = BorderColor;
          if( (*fi).V1(j)->C() == InternalColor) (*fi).V1(j)->C() = MixColor;
        } else
        {
          if( (*fi).V(j)->C() == BaseColor)     (*fi).V(j)->C() = InternalColor;
          if( (*fi).V(j)->C() == BorderColor) (*fi).V(j)->C() = MixColor;
          if( (*fi).V1(j)->C() == BaseColor)     (*fi).V1(j)->C() = InternalColor;
          if( (*fi).V1(j)->C() == BorderColor) (*fi).V1(j)->C() = MixColor;
        }
      }

  }

  /*! \brief This function colores the faces of connected components of a mesh randomly.

It require FaceFace Adjacency becouse it relies on the output of the ConnecteComponents();
*/
  static void PerFaceRandomConnectedComponent( MeshType &m)
  {
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");
    if(!HasFFAdjacency(m)) throw MissingComponentException("FFAdjacency");

    std::vector< std::pair<int, typename MeshType::FacePointer> > CCV;
    int ScatterSize= std::min (100,tri::Clean<MeshType>::ConnectedComponents(m, CCV)); // number of random color to be used. Never use too many.

    ConnectedComponentIterator<MeshType> ci;
    for(unsigned int i=0;i<CCV.size();++i)
    {
      Color4b BaseColor = Color4b::Scatter(ScatterSize, i%ScatterSize,.4f,.7f);
      std::vector<typename MeshType::FacePointer> FPV;
      for(ci.start(m,CCV[i].second);!ci.completed();++ci)
        (*ci)->C()=BaseColor;
    }
  }

  /*! \brief This function colores the face of a mesh randomly.

Note: The faux bit is used to color polygonal faces uniformly
*/
  static void PerFaceRandom(MeshType &m)
  {
    if(!HasPerFaceColor(m)) throw MissingComponentException("PerFaceColor");
    FaceIterator fi;
    Color4b BaseColor = Color4b::Black;
    PerFaceConstant(m,BaseColor);
    int id_num=0;
    for(fi=m.face.begin();fi!=m.face.end();++fi)
      if(!(*fi).IsD())
      {
        id_num++;
        if((*fi).C() == BaseColor) (*fi).C() = Color4b::Scatter(50, id_num%50,.4f,.7f);
        for(int j=0;j<3;++j)
          if((*fi).IsF(j))
          {
            assert(!IsBorder((*fi),j));
            (*fi).FFp(j)->C()= (*fi).C();
          }
      }
  }

  /*! \brief Perlin Noise.
  \return the number of changed vertexes (the selected ones)

  Simple Perlin noise. To make things weirder each color band can have its own offset and frequency.
  Period is expressed in absolute terms.
  So as period it is meaningful could be to use something in the range of 1/10 of the bbox diag.
  */
  static void PerVertexPerlinNoise(MeshType& m, Point3f period, Point3f offset=Point3f(0,0,0))
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    Point3<ScalarType> p[3];
    for(VertexIterator vi = m.vert.begin(); vi!=m.vert.end(); ++vi)
    {
      if(!(*vi).IsD()){
        // perlin noise is defined in 022
        p[0] = (vi->P()/period[0])+offset;
        p[1] = (vi->P()/period[1])+offset;
        p[2] = (vi->P()/period[2])+offset;
        (*vi).C() = Color4b( int(127+128.0*math::Perlin::Noise(p[0][0],p[0][1],p[0][2])),
                             int(127+128.0*math::Perlin::Noise(p[1][0],p[1][1],p[1][2])),
                             int(127+128.0*math::Perlin::Noise(p[2][0],p[2][1],p[2][2])),
                             255 );
      }
    }
  }

  /*! \brief Simple Noise adding function.
  It simply add signed noise to the color of the mesh. The noise has uniform distribution and the amplitude is +/-2^(noisebits-1).
  */
  static void PerVertexAddNoise(MeshType& m, int noiseBits)
  {
    if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    if(noiseBits>8) noiseBits = 8;
    if(noiseBits<1) return;

    math::SubtractiveRingRNG randomGen = math::SubtractiveRingRNG(time(NULL));
    for(VertexIterator vi = m.vert.begin(); vi!=m.vert.end(); ++vi)
    {
      if(!(*vi).IsD()){
        (*vi).C()[0] = math::Clamp<int>((*vi).C()[0] + randomGen.generate(int(2*pow(2.0f,noiseBits))) - int(pow(2.0f,noiseBits)),0,255);
        (*vi).C()[1] = math::Clamp<int>((*vi).C()[1] + randomGen.generate(int(2*pow(2.0f,noiseBits))) - int(pow(2.0f,noiseBits)),0,255);
        (*vi).C()[2] = math::Clamp<int>((*vi).C()[2] + randomGen.generate(int(2*pow(2.0f,noiseBits))) - int(pow(2.0f,noiseBits)),0,255);
      }
    }
  }


/*! \brief Reduces vertex color the mesh to two colors according to a threshold.
  */
static int PerVertexThresholding(MeshType &m, float threshold, Color4b c1 = Color4<unsigned char>::Black, Color4b c2 = Color4<unsigned char>::White, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        float value = ComputeLightness((*vi).C());

        if(value<=threshold) (*vi).C() = c1;
        else (*vi).C() = c2;
        ++counter;
      }
    }
  }
  return counter;
}

// Computes the lightness value for a specified color. lightness = 0.5*(Max(R,G,B)+Min(R,G,B))
static float ComputeLightness(Color4b c)
{
  float min_rgb = (float)math::Min(c[0],c[1],c[2]);
  float max_rgb = (float)math::Max(c[0],c[1],c[2]);
  return (max_rgb + min_rgb)/2;
}

/*! \brief Apply the brightness filter, with the given amount, to the mesh.
  */
static int PerVertexBrightness(MeshType &m, float amount, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
    int counter=0;
	VertexIterator vi;
	for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
	{
		if(!(*vi).IsD()) //if it has not been deleted...
		{
			if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = Color4b(
                            math::Clamp(int((*vi).C()[0]+amount),0,255),
                            math::Clamp(int((*vi).C()[1]+amount),0,255),
                            math::Clamp(int((*vi).C()[2]+amount),0,255),
                            255);
        ++counter;
      }
    }
  }
	return counter;
}

/*! \brief Apply Contrast filter to the mesh with the given contrast factor.
  */
static int PerVertexContrast(MeshType &m, float factor, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = ColorMul((*vi).C(),factor);
        ++counter;
      }
    }
  }
  return counter;
}

//Performs contrast operations on color, i.e expands or compress the histogram around
//the midpoint value.  NewValue = (OldValue - 128) ◊ factor + 128
static Color4b ColorMul(Color4b c, float factor)
{
  return Color4b( ValueMul(c[0], factor), ValueMul(c[1], factor), ValueMul(c[2], factor), 1);
}

static int ValueMul(int value, float factor)
{
  return math::Clamp<int>((int)((value - 128)*factor + 128), 0, 255);
}

/*! \brief Apply  Brightness and Contrast  filter to the mesh, with the given contrast factor and brightness amount.

Performs contrast and brightness operations on color, i.e NewValue = (OldValue - 128) * contrast + 128 + amount
The result is clamped just one time after all computations; this get a more accurate result.

The formula used here is the one of GIMP.

  */
static int PerVertexBrightnessContrast(MeshType &m, float brightness, float contrast, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = ColorBrightnessContrast((*vi).C(),brightness,contrast);
        ++counter;
      }
    }
  }
  return counter;
}

static Color4b ColorBrightnessContrast(Color4b c, float brightness, float contrast)
{
  return Color4b( ValueBrightnessContrast(c[0], brightness, contrast),
									ValueBrightnessContrast(c[1], brightness, contrast),
									ValueBrightnessContrast(c[2], brightness, contrast), 1 );
}

static int ValueBrightnessContrast(unsigned char ivalue, float brightness, float contrast)
{
	float value = float(ivalue)/255.0f;
  if (brightness < 0.0)  value = value * ( 1.0 + brightness);
                    else value = value + ((1.0 - value) * brightness);
	value = (value - 0.5) * (tan ((contrast + 1) * M_PI/4) ) + 0.5;
	return math::Clamp<int>(255.0*value, 0, 255);
}

/*! \brief Invert the colors of the mesh.

  \return the number of changed vertexes (the selected ones)
  */
static int PerVertexInvert(MeshType &m, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  for(VertexIterator vi=m.vert.begin(); vi!=m.vert.end(); ++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        Color4b &c=(*vi).C();
        c=Color4b( 255-c[0],255-c[1],255-c[2], 1);
        ++counter;
      }
    }
  }
  return counter;
}

/*! \brief Apply the gamma correction filter, with the given gamma exponet, to the mesh.
  \return the number of changed vertexes (the selected ones)
  */
static int PerVertexGamma(MeshType &m, float gamma, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;

  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = ColorPow((*vi).C(), 1/gamma);
        ++counter;
      }
    }
  }
  return counter;
}

//computes the standard gamma transformation on a given color, according to NewVal = OldVal^(1/gamma)
static Color4b ColorPow(Color4b c, float exponent)
{
  return vcg::Color4b(
                      math::Clamp((int)( ValuePow(float(c[0])/255, exponent)*255), 0, 255),
                      math::Clamp((int)( ValuePow(float(c[1])/255, exponent)*255), 0, 255),
                      math::Clamp((int)( ValuePow(float(c[2])/255, exponent)*255), 0, 255),
                      255);
}

static float ValuePow(float value, float exponent)
{
  return powf(value, exponent);
}

//useful bit masks for RGB channels, used for Levels filter.
enum rgbChMask {ALL_CHANNELS = 7, RED_CHANNEL = 4, GREEN_CHANNEL = 2, BLUE_CHANNEL = 1, NO_CHANNELS = 0 };

/*! \brief Adjusts color levels of the mesh

  \return the number of changed vertexes (the selected ones)

Adjusts color levels of the mesh. Filter can be applied to all RGB channels or to each channel separately.
in_min, gamma and in_max are respectively the black point, the gray point and the white point.
out_min and out_max are the output level for black and white respectively.
*/
static int PerVertexLevels(MeshType &m, float gamma, float in_min, float in_max, float out_min, float out_max, unsigned char rgbMask, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = ColorLevels((*vi).C(), gamma, in_min, in_max, out_min, out_max, rgbMask);
        ++counter;
      }
    }
  }
  return counter;
}

//Performs levels transformation on each channel set to 1 in the rgbMask.
static Color4b ColorLevels(Color4b c, float gamma, float in_min, float in_max, float out_min, float out_max, unsigned char rgbMask)
{
  unsigned char r = c[0], g = c[1], b = c[2];
  if(rgbMask & RED_CHANNEL) r = ValueLevels(c[0], gamma, in_min, in_max, out_min, out_max);
  if(rgbMask & GREEN_CHANNEL) g = ValueLevels(c[1], gamma, in_min, in_max, out_min, out_max);
  if(rgbMask & BLUE_CHANNEL) b = ValueLevels(c[2], gamma, in_min, in_max, out_min, out_max);
  return Color4b(r, g, b, 255);
}

//Transform on levels
static int ValueLevels(int value, float gamma, float in_min, float in_max, float out_min, float out_max)
{
  float fvalue = value/255.0f;
  // normalize
  fvalue = math::Clamp<float>(fvalue - in_min, 0.0f, 1.0f) / math::Clamp<float>(in_max - in_min, 1.0f/255.0f, 1.0f);
  // transform gamma
  fvalue = powf(fvalue,1/gamma);
  // rescale range
  fvalue = fvalue * (out_max - out_min) + out_min;
  //back in interval [0,255] and clamp
  return math::Clamp<int>((int)(fvalue * 255), 0, 255);
}

/*! \brief  Colorize the mesh toward a given color.
  \return the number of changed vertexes (the selected ones)

Colors the mesh. Color is blended to the mesh with the given intensity (0..1 ranged).
  */
static int PerVertexColourisation(MeshType &m, Color4b c, float intensity, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = ColorApplyDiff((*vi).C(), c, intensity);
        ++counter;
      }
    }
  }
  return counter;
}

// Perform colourisation operation.
// For each channel C:
// newC = origC + intensity * (newC - origC)
static Color4b ColorApplyDiff(Color4b old_color, Color4b new_color, float intensity)
{
  return Color4b( ValueApplyDiff(old_color[0], new_color[0], intensity),
                  ValueApplyDiff(old_color[1], new_color[1], intensity),
                  ValueApplyDiff(old_color[2], new_color[2], intensity), 255);
}

static int ValueApplyDiff(int old_value, int new_value, float intensity)
{
  return  math::Clamp<int>((int)(old_value + intensity * (new_value - old_value)), 0, 255);
}

//An useful ENUM to hold all desaturation methods.
enum DesaturationMethods {M_LIGHTNESS = 0, M_LUMINOSITY = 1, M_AVERAGE = 2};

/*! \brief  Desaturates the mesh according the a chosen desaturation method

\return the number of changed vertexes (the selected ones)

There are three possibilities
- \c M_LIGHTNESS where lightness = 0.5*(Max(R,G,B)+Min(R,G,B))
- \c M_LUMINOSITY where luminosity = 0.21*R+0.71*G+0.7*B
- \c M_AVERAGE Plain Average
  */
static int PerVertexDesaturation(MeshType &m, int method, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  int counter=0;
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C() = ColorDesaturate((*vi).C(), method);
        ++counter;
      }
    }
  }
  return counter;
}

//Desature the color. Ausiliary functions to calculate lightness/luminosity/average.
static Color4b ColorDesaturate(Color4b c, int method)
{
  switch(method){
    case M_LIGHTNESS:{
      int val = (int)ComputeLightness(c);
      return Color4b( val, val, val, 255);
    }
    case M_AVERAGE:{
      int val = (int)ComputeAvgLightness(c);
      return Color4b( val, val, val, 255);
    }
    case M_LUMINOSITY:{
      int val = (int)ComputeLuminosity(c);
      return Color4b( val, val, val, 255);
    }
    default: assert(0);
  }
}

//ausiliary function to compute average lightness. value = (R+G+B)/3
static float ComputeAvgLightness(Color4b c)
{
    return float(c[0]+c[1]+c[2])/3.0f;
}

//ausiliary function to compute luminosity. value = 0.21*R+0.71*G+0.7*B
static float ComputeLuminosity(Color4b c)
{
    return float(0.2126f*c[0]+0.7152f*c[1]+0.0722f*c[2]);
}

/*! \brief Histogram Color Equalization.
  \return the number of changed vertexes (the selected ones)

Equalize the histogram of colors. It can equalize any combination of rgb channels or it can work on lightness.
  */
static int PerVertexEqualize(MeshType &m, unsigned int rgbMask, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  //declares , resets and set up 4 histograms, for Red, Green, Blue and Lightness
  Histogramf Hl, Hr, Hg, Hb;
  Hl.Clear(); Hr.Clear(); Hg.Clear(); Hb.Clear();
  Hl.SetRange(0, 255, 255); Hr.SetRange(0, 255, 255); Hg.SetRange(0, 255, 255); Hb.SetRange(0, 255, 255);

  int counter=0;
  VertexIterator vi;

  //Scan the mesh to build the histograms
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, put it in the histograms
      {
        float v = ComputeLightness((*vi).C())+0.5; //compute and round lightness value
        Hl.Add(v); Hr.Add((float)(*vi).C()[0]); Hg.Add((float)(*vi).C()[1]); Hb.Add((float)(*vi).C()[2]);
      }
    }
  }

  //for each histogram, compute the cumulative distribution function, and build a lookup table
	int cdf_l[256], cdf_r[256], cdf_g[256], cdf_b[256];
	cdf_l[0] = Hl.BinCount(0); cdf_r[0] = Hr.BinCount(0); cdf_g[0] = Hg.BinCount(0); cdf_b[0] = Hb.BinCount(0);
	for(int i=1; i<256; i++){
    cdf_l[i] = Hl.BinCount(float(i)) + cdf_l[i-1];
    cdf_r[i] = Hr.BinCount(float(i)) + cdf_r[i-1];
    cdf_g[i] = Hg.BinCount(float(i)) + cdf_g[i-1];
    cdf_b[i] = Hb.BinCount(float(i)) + cdf_b[i-1];
  }

  //this loop aaplies the transformation to colors
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C()=ColorEqualize((*vi).C(), cdf_l, cdf_r, cdf_g, cdf_b, rgbMask);
        ++counter;
      }
    }
  }
  return counter;
}

//Applies equalization to the components of the color according to rgbmask
static Color4b ColorEqualize(Color4b c, int cdf_l[256], int cdf_r[256], int cdf_g[256], int cdf_b[256], unsigned int rgbMask)
{
  unsigned char r = c[0], g = c[1], b = c[2];
  if(rgbMask == NO_CHANNELS) //in this case, equalization is done on lightness
  {
    int v = ValueEqualize(cdf_l[(int)(ComputeLightness(c)+0.5f)], cdf_l[0], cdf_l[255]);
    return Color4b(v, v, v, 255); //return the equalized gray color
  }
  if(rgbMask & RED_CHANNEL) r = ValueEqualize(cdf_r[c[0]], cdf_r[0], cdf_r[255]); //Equalizes red
  if(rgbMask & GREEN_CHANNEL) g = ValueEqualize(cdf_g[c[1]], cdf_g[0], cdf_g[255]); //Equalizes green
  if(rgbMask & BLUE_CHANNEL) b = ValueEqualize(cdf_b[c[2]], cdf_b[0], cdf_b[255]); //Equalizes blue
  return Color4b(r, g, b, 255); //return the equalized color
}

//Compute the equalized value
static int ValueEqualize(int cdfValue, int cdfMin, int cdfMax)
{
  return int(float((cdfValue - cdfMin)/float(cdfMax - cdfMin)) * 255.0f);
}

/*! \brief Simple white balancing filter.
  \return the number of changed vertexes (the selected ones)

  It applies a simple white balancing filter. It may works on a provided user color that is supposed to be white.
  */
static int PerVertexWhiteBalance(MeshType &m, Color4b userColor, const bool ProcessSelected=false)
{
  if(!HasPerVertexColor(m)) throw MissingComponentException("PerVertexColor");
  Color4b unbalancedWhite= userColor;
  int counter=0;
  VertexIterator vi;

  //in this loop the transformation is applied to the mesh
  for(vi=m.vert.begin();vi!=m.vert.end();++vi) //scan all the vertex...
  {
    if(!(*vi).IsD()) //if it has not been deleted...
    {
      if(!ProcessSelected || (*vi).IsS()) //if this vertex has been selected, do transormation
      {
        (*vi).C()=ColorWhiteBalance((*vi).C(),unbalancedWhite);
        ++counter;
      }
    }
  }
  return counter;
}

//Balnce the white of the color, applying a correction factor based on the unbalancedWhite color.
static Color4b ColorWhiteBalance(Color4b c, Color4b unbalancedWhite)
{
  //sanity check to avoid division by zero...
  if(unbalancedWhite[0]==0) unbalancedWhite[0]=1;
  if(unbalancedWhite[1]==0) unbalancedWhite[1]=1;
  if(unbalancedWhite[2]==0) unbalancedWhite[2]=1;

  return Color4b(
                 math::Clamp<int>((int)(c[0]*(255.0f/unbalancedWhite[0])), 0, 255),
                 math::Clamp<int>((int)(c[1]*(255.0f/unbalancedWhite[1])), 0, 255),
                 math::Clamp<int>((int)(c[2]*(255.0f/unbalancedWhite[2])), 0, 255),
                 255);
}

};

}// end namespace
}// end namespace
#endif
