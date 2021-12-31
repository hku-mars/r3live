#ifndef COL_QT_CONVERT_H_
#define COL_QT_CONVERT_H_

#include <QColor>
#include "../../vcg/space/color4.h"

namespace vcg
{
	class ColorConverter
	{
	public:
		inline static vcg::Color4b ToColor4b(const QColor& col)
		{
			return vcg::Color4b(col.red(),col.green(),col.blue(),col.alpha());
		}

		inline static QColor ToQColor(const vcg::Color4b& col) 
		{
			return QColor(col[0],col[1],col[2],col[3]);
		}
	};
}

#endif