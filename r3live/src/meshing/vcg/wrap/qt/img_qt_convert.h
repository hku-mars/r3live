#ifndef IMG_QT_CONVERT_H_
#define IMG_QT_CONVERT_H_

// implementation of conversione between qimage and basic image types

#include <QImage>

namespace img {

template<typename ScalarType, bool Safe> 
inline void convert_QImage_to_Y(const QImage &source, Image<1,ScalarType,Safe> &destination)
{
  assert(!source.isNull());
  if(Safe){
    if(source.isNull())  throw ImageException("Null source image");
  }
  destination.setZero(source.width(),source.height());
  for (int y_coord = 0; y_coord < source.height(); ++y_coord)
    for (int x_coord = 0; x_coord < source.width(); ++x_coord){
      destination.setValue(x_coord, y_coord, 0, qGray(source.pixel(x_coord, y_coord)) );
    }
  destination.attributes.setRange(ScalarType(0.0),ScalarType(255.0));
}

template<typename ScalarType, bool Safe> 
inline void convert_QImage_to_RGB(const QImage &source, Image<3,ScalarType,Safe> &destination)
{
  assert(!source.isNull());
  if(Safe){
    if(source.isNull())  throw ImageException("Null source image");
  }
  destination.setZero(source.width(),source.height());
  for (int y_coord = 0; y_coord < source.height(); ++y_coord)
    for (int x_coord = 0; x_coord < source.width(); ++x_coord){
      QRgb qpixel = source.pixel(x_coord, y_coord);
      destination.setValue(x_coord, y_coord, 0, qRed(qpixel) );
      destination.setValue(x_coord, y_coord, 1, qGreen(qpixel) );
      destination.setValue(x_coord, y_coord, 2, qBlue(qpixel) );
    }
  destination.attributes.setRange(ScalarType(0.0),ScalarType(255.0));
}

template<typename ScalarType, bool Safe> 
inline void convert_QImage_to_RGBA(const QImage &source, Image<4,ScalarType,Safe> &destination)
{
  assert(!source.isNull());
  if(Safe){
    if(source.isNull())  throw ImageException("Null source image");
  }
  destination.setZero(source.width(),source.height());
  for (int y_coord = 0; y_coord < source.height(); ++y_coord)
    for (int x_coord = 0; x_coord < source.width(); ++x_coord){
      QRgb qpixel = source.pixel(x_coord, y_coord);
      destination.setValue(x_coord, y_coord, 0, qRed(qpixel) );
      destination.setValue(x_coord, y_coord, 1, qGreen(qpixel) );
      destination.setValue(x_coord, y_coord, 2, qBlue(qpixel) );
      destination.setValue(x_coord, y_coord, 3, qAlpha(qpixel) );
    }
  destination.attributes.setRange(ScalarType(0.0),ScalarType(255.0));
}

template<typename ScalarType, bool Safe> 
inline void convert_Y_to_QImage(const Image<1,ScalarType,Safe> &source, QImage &destination)
{
  assert(source.isValid());
  assert(source.attributes.hasRange(0,255));
  if(Safe){
    if(!source.isValid()) throw ImageException("Invalid source image");
    if(!source.attributes.hasRange(0,255)) throw ImageException("Invalid range attribute");
  }
  destination=QImage(source.width(),source.height(),QImage::Format_RGB32);
  for (int y_coord = 0; y_coord < source.height(); ++y_coord)
    for (int x_coord = 0; x_coord < source.width(); ++x_coord){
      int Y = valueAsInt(clampValue(source.getValue(x_coord,y_coord,0)));
  	  destination.setPixel(x_coord,y_coord,qRgb(Y,Y,Y));
    }
}

template<typename ScalarType, bool Safe> 
inline void convert_RGB_to_QImage(const Image<3,ScalarType,Safe> &source, QImage &destination)
{
  assert(source.isValid());
  assert(source.attributes.hasRange(ScalarType(0.0),ScalarType(255.0)));
  if(Safe){
    if(!source.isValid()) throw ImageException("Invalid source image");
    if(!source.attributes.hasRange(ScalarType(0.0),ScalarType(255.0))) throw ImageException("Invalid range attribute");
  }
  destination=QImage(source.width(),source.height(),QImage::Format_RGB32);
  for (int y_coord = 0; y_coord < source.height(); ++y_coord)
    for (int x_coord = 0; x_coord < source.width(); ++x_coord){
  	  destination.setPixel(x_coord,y_coord,qRgb(valueAsInt(clampValue(source.getValue(x_coord,y_coord,0))),
  	                                            valueAsInt(clampValue(source.getValue(x_coord,y_coord,1))),
  	                                            valueAsInt(clampValue(source.getValue(x_coord,y_coord,2))) ));
    }
}

template<typename ScalarType, bool Safe> 
inline void convert_RGBA_to_QImage(const Image<4,ScalarType,Safe> &source, QImage &destination)
{
  assert(source.isValid());
  assert(source.attributes.hasRange(ScalarType(0.0),ScalarType(255.0)));
  if(Safe){
    if(!source.isValid()) throw ImageException("Invalid source image");
    if(!source.attributes.hasRange(ScalarType(0.0),ScalarType(255.0))) throw ImageException("Invalid range attribute");
  }
  destination=QImage(source.width(),source.height(),QImage::Format_ARGB32);
  for (int y_coord = 0; y_coord < source.height(); ++y_coord)
    for (int x_coord = 0; x_coord < source.width(); ++x_coord){
  	  destination.setPixel(x_coord,y_coord,qRgba(valueAsInt(clampValue(source.getValue(x_coord,y_coord,0))),
  	                                             valueAsInt(clampValue(source.getValue(x_coord,y_coord,1))),
  	                                             valueAsInt(clampValue(source.getValue(x_coord,y_coord,2))),
  	                                             valueAsInt(clampValue(source.getValue(x_coord,y_coord,3))) ));
    }
}

} //end namespace img

#endif /*IMG_QT_CONVERT_H_*/
