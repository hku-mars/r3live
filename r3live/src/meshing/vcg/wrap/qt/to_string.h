#ifndef TOSTRING_H
#define TOSTRING_H
#include <QString>
#include <cstdlib>

inline QString toString( const vcg::Point4f& p ){
  QString s;
  s.sprintf("%f %f %f %f", p[0], p[1], p[2], p[3]);
  return s;
}
inline QString toString( const vcg::Point3f& p ){
  QString s;
  s.sprintf("%f %f %f", p[0], p[1], p[2]);
  return s;
}
inline QString toString( const vcg::Point2f& p ){
  QString s;
  s.sprintf("%f %f", p[0], p[1]);
  return s;
}
inline QString toString( const vcg::Point2i& p ){
  QString s;
  s.sprintf("%d %d", p[0], p[1]);
  return s;
}
inline QString toString(vcg::Matrix44f& m){
  QString mat;
  for(int i=0; i<3; i++){
      mat.append( toString( m.GetRow4(i) ) );
      mat.append("\n");
  }
  return mat;
}
#endif // TOSTRING_H
