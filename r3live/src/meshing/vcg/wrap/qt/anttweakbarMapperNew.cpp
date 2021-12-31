#include <QtGui>
#include <QWidget>
#include <AntTweakBar.h>
#include <wrap/qt/device_to_logical.h>

void TW_CALL CopyCDStringToClient(char **destPtr, const char *src)
{
    size_t srcLen = (src!=NULL) ? strlen(src) : 0;
    size_t destLen = (*destPtr!=NULL) ? strlen(*destPtr) : 0;

    // Alloc or realloc dest memory block if needed
    if( *destPtr==NULL )
        *destPtr = (char *)malloc(srcLen+1);
    else if( srcLen>destLen )
        *destPtr = (char *)realloc(*destPtr, srcLen+1);

    // Copy src
    if( srcLen>0 )
        strncpy(*destPtr, src, srcLen);
    (*destPtr)[srcLen] = '\0'; // null-terminated string
}

void TW_CALL CopyStdStringToClient(std::string& destClientString, const std::string& srcLibraryString)
{
    destClientString = srcLibraryString;
}

TwMouseButtonID Qt2TwMouseButtonId(QMouseEvent *e)
{
  if(e->button() && Qt::LeftButton) return TW_MOUSE_LEFT;
  if(e->button() && Qt::MidButton) return TW_MOUSE_MIDDLE;
  if(e->button() && Qt::RightButton) return TW_MOUSE_RIGHT;
  return TW_MOUSE_LEFT;
//  assert(0);
}

int TwMousePressQt(QMouseEvent *e)
{
	TwMouseMotion(e->x (), e->y ());
	return TwMouseButton(TW_MOUSE_PRESSED, Qt2TwMouseButtonId(e));
}

int TwMousePressQt(QWidget *qw, QMouseEvent *e)
{
	TwMouseMotion(QTLogicalToDevice(qw, e->x()), QTLogicalToDevice(qw, e->y()));
	return TwMouseButton(TW_MOUSE_PRESSED, Qt2TwMouseButtonId(e));
}

int TwMouseReleaseQt(QMouseEvent *e)
{
	TwMouseMotion(e->x (), e->y ());
	return TwMouseButton(TW_MOUSE_RELEASED, Qt2TwMouseButtonId(e));
}

int TwMouseReleaseQt(QWidget *qw, QMouseEvent *e)
{
	TwMouseMotion(QTLogicalToDevice(qw, e->x()), QTLogicalToDevice(qw, e->y()));
	return TwMouseButton(TW_MOUSE_RELEASED, Qt2TwMouseButtonId(e));
}

int TwKeyPressQt(QKeyEvent *e)
{
  int kmod = 0;
  if(e->modifiers() & Qt::ShiftModifier )  kmod |= TW_KMOD_SHIFT;
  if(e->modifiers() & Qt::ControlModifier )  kmod |= TW_KMOD_CTRL;
  if(e->modifiers() & Qt::AltModifier )  kmod |= TW_KMOD_ALT;
  int key = e->key();
  int k = 0;

  if( key>0 && key<0x7e ) k=key; // plain ascii codes

  if( key>=Qt::Key_F1 && key<=Qt::Key_F15  )
      k = TW_KEY_F1 + (key-Qt::Key_F1 );
  else
  if ( key>=Qt::Key_A && key<=Qt::Key_Z)
      k = (kmod & TW_KMOD_SHIFT) ? key : (int)'a' + (key-(int)'A');
  else
  {
      switch( key )
      {
      case Qt::Key_Left:      k = TW_KEY_LEFT;  break;
      case Qt::Key_Up:        k = TW_KEY_UP; break;
      case Qt::Key_Right:     k = TW_KEY_RIGHT;  break;
      case Qt::Key_Down:      k = TW_KEY_DOWN;   break;
      case Qt::Key_PageUp:    k = TW_KEY_PAGE_UP;  break;
      case Qt::Key_PageDown:  k = TW_KEY_PAGE_DOWN; break;
      case Qt::Key_Home:      k = TW_KEY_HOME; break;
      case Qt::Key_End:       k = TW_KEY_END; break;
      case Qt::Key_Insert:    k = TW_KEY_INSERT; break;
      case Qt::Key_Backspace: k = TW_KEY_BACKSPACE; break;
      case Qt::Key_Delete:    k = TW_KEY_DELETE; break;
      case Qt::Key_Return:    k = TW_KEY_RETURN; break;
      case Qt::Key_Enter:     k = TW_KEY_RETURN; break;
      case Qt::Key_Escape:    k = TW_KEY_ESCAPE; break;
      case Qt::Key_Tab:       k = TW_KEY_TAB; break;
      case Qt::Key_Space:     k = TW_KEY_SPACE; break;
      }
  }

  return TwKeyPressed(k, kmod);
}
