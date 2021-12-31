#ifndef IMG_QT_IO_H_
#define IMG_QT_IO_H_

#include <fstream>
// input/output using the qt system

#include <QImage>

#include "wrap/qt/img_qt_convert.h"
#include "img/img_io.h"

namespace img {

template<typename ScalarType, bool Safe> 
inline void openQtY(const QString filename, Image<1,ScalarType,Safe> &image)
{
   convert_QImage_to_Y(QImage(filename),image);
}

template<typename ScalarType, bool Safe> 
inline void openQtRGB(const QString filename, Image<3,ScalarType,Safe> &image)
{
   convert_QImage_to_RGB(QImage(filename),image);
}

//// queste le tengo commentate perche` ora non mi servono
//template<typename ScalarType, bool Safe> 
//inline void openQtRGBA(const QString filename, Image<4,ScalarType,Safe> &image)
//{
//   convert_QImage_to_RGBA(QImage(filename),image);
//}
//
//template<typename ScalarType, bool Safe> 
//inline void openQtY(const char *filename, Image<1,ScalarType,Safe> &image)
//{
//  openQtY(QString(filename),image);
//}
//
//template<typename ScalarType, bool Safe> 
//inline void openQtRGB(const char *filename, Image<3,ScalarType,Safe> &image)
//{
//  openQtRGB(QString(filename),image);
//}
//
//template<typename ScalarType, bool Safe> 
//inline void openQtRGBA(const char *filename, Image<4,ScalarType,Safe> &image)
//{
//  openQtRGBA(QString(filename),image);
//}
//
template<typename ScalarType, bool Safe> 
inline bool saveQtY(const Image<1,ScalarType,Safe> &image,const QString filename)
{
  QImage qimage;
  convert_Y_to_QImage(image,qimage);
  bool success = qimage.save(filename);
  assert(success);
  if(Safe){
    if(!success)  throw ImageException("cannot save image");
  }
  return success;
}

template<typename ScalarType, bool Safe> 
inline bool saveQtRGB(const Image<3,ScalarType,Safe> &image,const QString filename)
{
  QImage qimage;
  convert_RGB_to_QImage(image,qimage);
  bool success = qimage.save(filename);
  assert(success);
  if(Safe){
    if(!success)  throw ImageException("cannot save image");
  }
  return success;
}

//// queste le tengo commentate perche` ora non mi servono
//template<typename ScalarType, bool Safe> 
//inline bool saveQtRGBA(const Image<4,ScalarType,Safe> &image,const QString filename)
//{
//  QImage qimage;
//  convert_RGBA_to_QImage(image,qimage);
//  bool success = qimage.save(filename);
//  assert(success);
//  if(Safe){
//    if(!success)  throw ImageException("cannot save image");
//  }
//  return success;
//}
//
//template<typename ScalarType, bool Safe>
//inline bool saveQtY(const Image<1,ScalarType,Safe> &image,const char *filename)
//{
//  return saveQtY(image,QString(filename));
//}
//
//template<typename ScalarType, bool Safe>
//inline bool saveQtRGB(const Image<3,ScalarType,Safe> &image,const char *filename)
//{
//  return saveQtRGB(image,QString(filename));
//}
//
//template<typename ScalarType, bool Safe>
//inline bool saveQtRGBA(const Image<4,ScalarType,Safe> &image,const char *filename)
//{
//  return saveQtRGBA(image,QString(filename));
//}

template<typename ScalarType, bool Safe>
inline bool savePGM(const Image<1,ScalarType,Safe> &image, const QString filename)
{
  assert(image.isValid());
  assert(image.attributes.hasRange(ScalarType(0.0),ScalarType(255.0)));
  assert(filename.endsWith(".pgm",Qt::CaseInsensitive));
  if(Safe){
    if(!image.isValid()) throw ImageException("Invalid image");
    if(!image.attributes.hasRange(ScalarType(0.0),ScalarType(255.0))) throw ImageException("Invalid range attribute");
    if(!filename.endsWith(".pgm",Qt::CaseInsensitive)) throw ImageException("filename is not .pgm");
  }
  using namespace std;
  ofstream file (filename.toStdString().c_str(), ios::out|ios::binary|ios::trunc);
  if (file.is_open()) {
    QString header = QString("P5 %1 %2 255\n").arg(image.width()).arg(image.height());
    file << header.toStdString().c_str();

  for (int y_coord = 0; y_coord < image.height(); ++y_coord)
    for (int x_coord = 0; x_coord < image.width(); ++x_coord){
      unsigned char v = static_cast<unsigned char>(valueAsInt(clampValue(image.getValue(x_coord,y_coord,0))));  
      file << v;
    }

    file.close();
    return true;
  }
  if(Safe)
    throw ImageException("Unable to open file");
  return false;
}

// range and gamma aware I/O

template<typename ScalarType, bool Safe> 
inline void open_and_normalize_range_Y(const QString filename, Image<1,ScalarType,Safe> &range_0_1_image)
{
  Image<1,ScalarType,Safe> range_0_255_image;
  openQtY(filename,range_0_255_image);
  convert_range_0_255_to_0_1(range_0_255_image,range_0_1_image);
}

template<typename ScalarType, bool Safe> 
inline void open_and_normalize_range_RGB(const QString filename, Image<3,ScalarType,Safe> &range_0_1_image)
{
  Image<3,ScalarType,Safe> range_0_255_image;
  openQtRGB(filename,range_0_255_image);
  convert_range_0_255_to_0_1(range_0_255_image,range_0_1_image);
}

template<typename ScalarType, bool Safe> 
inline void open_normalize_range_and_SRGB_linearize_RGB(const QString filename, Image<3,ScalarType,Safe> &linear_image)
{
  Image<3,ScalarType,Safe> range_0_1_image;
  open_and_normalize_range_RGB(filename, range_0_1_image);
  range_0_1_image.attributes.setColorspace(img::SRGB); // assumes sRGB colorspace for gamma compression
  convert_gamma_precompensated_srgb_to_linear_srgb(range_0_1_image,linear_image);
}

template<typename ScalarType, bool Safe>
inline bool adjust_range_and_save_PGM(const Image<1,ScalarType,Safe> &range_0_1_image, const QString filename)
{
  Image<1,ScalarType,Safe> range_0_255_image;
  convert_range_0_1_to_0_255(range_0_1_image,range_0_255_image);
  return savePGM(range_0_255_image,filename);
}

template<typename ScalarType, bool Safe>
inline bool SRGB_compress_adjust_range_and_save_PGM(const Image<1,ScalarType,Safe> &linear_image, const QString filename)
{
  Image<1,ScalarType,Safe> range_0_1_image;
  convert_linear_srgb_to_gamma_precompensated_srgb(linear_image,range_0_1_image);
  return adjust_range_and_save_PGM(range_0_1_image,filename);
}

template<typename ScalarType, bool Safe>
inline bool adjust_range_and_save_RGB(const Image<3,ScalarType,Safe> &range_0_1_image, const QString filename)
{
  Image<3,ScalarType,Safe> range_0_255_image;
  convert_range_0_1_to_0_255(range_0_1_image,range_0_255_image);
  return saveQtRGB(range_0_255_image,filename);
}

template<typename ScalarType, bool Safe>
inline bool SRGB_compress_adjust_range_and_save_RGB(const Image<3,ScalarType,Safe> &linear_image,const QString filename)
{
  Image<3,ScalarType,Safe> range_0_1_image;
  convert_linear_srgb_to_gamma_precompensated_srgb(linear_image,range_0_1_image);
  return adjust_range_and_save_RGB(range_0_1_image,filename);
}

template<typename ScalarType, bool Safe>
inline bool adjust_range_and_save_Y(const Image<1,ScalarType,Safe> &range_0_1_image, const QString filename)
{
  Image<1,ScalarType,Safe> range_0_255_image;
  convert_range_0_1_to_0_255(range_0_1_image,range_0_255_image);
  return saveQtY(range_0_255_image,filename);
}

template<typename ScalarType, bool Safe>
inline bool SRGB_compress_adjust_range_and_save_Y(const Image<1,ScalarType,Safe> &linear_image,const QString filename)
{
  Image<1,ScalarType,Safe> range_0_1_image;
  convert_linear_srgb_to_gamma_precompensated_srgb(linear_image,range_0_1_image);
  return adjust_range_and_save_Y(range_0_1_image,filename);
}

//// queste le tengo commentate perche` ora non mi servono
//template<typename ScalarType, bool Safe> 
//inline void open_normalize_range_and_SRGB_linearize_Y(const QString filename, Image<1,ScalarType,Safe> &linear_image)
//{
//  Image<1,ScalarType,Safe> range_0_255_image, range_0_1_image;
//  openQtY(filename,range_0_255_image);
//  range_0_255_image.attributes.setColorspace(img::SRGB); // assumes sRGB colorspace (for gamma compression)
//  convert_range_0_255_to_0_1(range_0_255_image,range_0_1_image);
//  convert_gamma_precompensated_srgb_to_linear_srgb(range_0_1_image,linear_image);
//}
//
//template<typename ScalarType, bool Safe>
//inline bool SRGB_compress_adjust_range_and_save_Y(const Image<1,ScalarType,Safe> &linear_image,const QString filename)
//{
//  Image<1,ScalarType,Safe> range_0_1_image, range_0_255_image;
//  convert_linear_srgb_to_gamma_precompensated_srgb(linear_image,range_0_1_image);
//  convert_range_0_1_to_0_255(range_0_1_image,range_0_255_image);
//  return aveQtY(range_0_255_image,filename);
//}
//

///// queste chi gli serve se le riimplementa, ora non c'ho tempo
//template <class PIXELTYPE, bool SAFE> 
//inline void dumpNormalized(const Image<PIXELTYPE,SAFE> &image,const QString filename)
//{
//  save(image::getNormalized(image),filename);
//}
//
//template <class PIXELTYPE, bool SAFE> 
//inline void dumpNormalized(const Image<PIXELTYPE,SAFE> &image,const char *filename)
//{
//  saveNormalized(image,QString(filename));
//}
//
//template <class PIXELTYPE, bool SAFE> 
//inline void dumpAsRaw(const Image<PIXELTYPE,SAFE> &image,const QString filename)
//{
//  dumpAsRaw(image,filename.toStdString().c_str());
//}
//
//template <class PIXELTYPE, bool SAFE> 
//inline void dumpAsAscii(const Image<PIXELTYPE,SAFE> &image,const QString filename)
//{
//  dumpAsAscii(image,filename.toStdString().c_str());
//}
//

} //end namespace img

#endif /*IMG_QT_IO_H_*/
