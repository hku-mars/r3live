#ifndef DEVICE_TO_LOGICAL_H
#define DEVICE_TO_LOGICAL_H
#include <QWidget>
#include <QPainter>
#include <vcg/space/point2.h>

template < class ValueType>
inline ValueType QTLogicalToDevice( QWidget *qw, const ValueType &value)
{
#if QT_VERSION >= 0x050000
  return value*qw->devicePixelRatio() ;
#else
  Q_UNUSED(qw);
  return value;
#endif
}

template < class ValueType>
inline ValueType QTLogicalToDevice( QPainter *qp, const ValueType &value)
{
#if QT_VERSION >= 0x050000
  return value*qp->device()->devicePixelRatio() ;
#else
  Q_UNUSED(qp);
  return value;
#endif
}

template < class ValueType>
inline ValueType QTDeviceToLogical( QPainter *qp, const ValueType &value)
{
#if QT_VERSION >= 0x050000
  return value/qp->device()->devicePixelRatio() ;
#else
  Q_UNUSED(qp);
  return value;
#endif
}

inline vcg::Point2f QTLogicalToOpenGL( QWidget *qw, const QPoint &p)
{
  return vcg::Point2f(QTLogicalToDevice(qw,p.x()), QTLogicalToDevice(qw,qw->height()-p.y()));
}


inline int QTDeviceHeight(QWidget *qw) {return QTLogicalToDevice(qw,qw->height());}
inline int QTDeviceWidth(QWidget *qw) {return QTLogicalToDevice(qw,qw->width());}
#endif // DEVICE_TO_LOGICAL_H
