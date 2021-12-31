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
/****************************************************************************
Acknowlegments
Portions of this file were based on the original code of the Ply library 
of Greg Turk and on the work of Claudio Rocchini

****************************************************************************/
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.5  2005/11/26 00:12:25  cignoni
added prototype of  interpret_texture_name

Revision 1.4  2005/11/12 07:07:47  cignoni
Changed Offset types to remove warnings

Revision 1.3  2005/01/03 10:35:59  cignoni
Improved the compatibility for ply format for faces having the list size (e.g. number of vertexes of a face) as a char instead of a uchar.
Added a couple of new face descriptors, corrected a bug in error reporting function (and restructured) and translated a few comments.
Thanks to Patrick Min for the careful bug reporting

Revision 1.2  2004/04/27 13:29:19  turini
*** empty log message ***

Revision 1.1  2004/03/03 15:00:51  cignoni
Initial commit

****************************************************************************/

#ifndef __VCG_PLYLIB 
#define __VCG_PLYLIB 

#include <memory.h>
#include <vector>
#include <string>
#include <assert.h>

namespace vcg {
namespace ply {

	// Data types supported by the ply format
enum PlyTypes {
	T_NOTYPE,
	T_CHAR,
	T_SHORT,
	T_INT,
	T_UCHAR,
	T_USHORT,
	T_UINT,
	T_FLOAT,
	T_DOUBLE,
	T_MAXTYPE
};

	// Error codes reported by GetError
enum PlyError {
	E_NOERROR,				// 0
		// Errors of  open(..)
	E_CANTOPEN,				// 1
	E_NOTHEADER,			// 2
	E_UNESPECTEDEOF,		// 3
	E_NOFORMAT,				// 4
	E_SYNTAX,				// 5
	E_PROPOUTOFELEMENT,		// 6
	E_BADTYPENAME,			// 7
		// Errors of addtoread(..)
	E_ELEMNOTFOUND,			// 8
	E_PROPNOTFOUND,			// 9
	E_BADTYPE,				// 10
	E_INCOMPATIBLETYPE,		// 11
	E_BADCAST,				// 12
	E_MAXPLYERRORS
};

		// file formats supported by the ply format
enum PlyFormat {
	F_UNSPECIFIED,
	F_ASCII,
	F_BINLITTLE,
	F_BINBIG
};


#ifdef USE_ZLIB
typedef void * GZFILE;
#else
typedef FILE * GZFILE;
#endif


	// Messaggio di errore
//extern const char * ply_error_msg[];

	// TIPO FILE


// Descrittore esterno di propieta'
class PropDescriptor
{
public:
	const char * elemname;			// Nome dell'elemento
	const char * propname;			// Nome della propieta'
	int	stotype1;				// Tipo dell'elemento su file    (se lista tipo degli elementi della lista)
	int memtype1;				// Tipo dell'elemento in memoria (se lista tipo degli elementi della lista)
	size_t offset1;				// Offset del valore in memoria
	int islist;					// 1 se lista, 0 altrimenti
	int alloclist;		  // 1 se alloca lista, 0 se preallocata
	int stotype2;				// Tipo del numero di elementi della lista su file
	int memtype2;				// Tipo del numero di elementi della lista in memoria
	size_t offset2;				// Offset valore memoria

	int format;					// duplicazione del formato
	
	size_t			stotypesize() const; // per sapere quanto e'grande un dato descrittore sul file
	size_t			memtypesize() const; // per sapere quanto e'grande un dato descrittore in memoria
	const char *memtypename() const; 
	const char *stotypename() const;
};

// Reading Callback (used to copy a data prop)
typedef bool (* readelemcb) ( GZFILE fp, void * mem, PropDescriptor * p );

class PlyProperty
{
public:
	inline PlyProperty()
	{
		tipo		= 0;
		islist		= 0;
		tipoindex	= 0;
		bestored	= 0;
	}

	inline PlyProperty( const char * na, int ti, int isl, int t2 )
	{
		assert(na);
		assert(ti>0);
		assert(ti<T_MAXTYPE);
		assert( t2>0 || (t2==0 && isl==0) );
		assert(t2<T_MAXTYPE);

		name		= std::string(na);
		tipo		= ti;
		islist		= isl;
		tipoindex	= t2;
		bestored	= 0;
	}

	std::string name;				// Nome della propieta'
	int    tipo;				// Tipo di dato
	int    islist;				// Vero se e' una lista
	int    tipoindex;			// Tipo del contatore della lista

	int	   bestored;			// 1 se va storata
	PropDescriptor desc;		// Descrittore di memorizzazione

	readelemcb	cb;				// Callback di lettura
};


class PlyElement
{
public:
	
	inline PlyElement()
	{
		number	= 0;
	}

	inline PlyElement( const char * na, int nu )
	{
		assert(na);
		assert(nu>=0);

		name    = std::string(na);
		number	= nu;
	}

	
	inline void SetName( const char * na )
	{
		name = std::string(na);
	}

	inline void SetNumbert( int nu )
	{
		assert(nu>0);
		number = nu;
	}

	void AddProp( const char * na, int ti, int isl, int t2 );

	int AddToRead(
		const char * propname,
		int	stotype1,
		int memtype1,
		size_t offset1,
		int islist,
		int alloclist,
		int stotype2,
		int memtype2,
		size_t offset2
	);	// Vedi struttura PropDescriptor

	PlyProperty * FindProp( const char * name );

	std::string name;				// Nome dell'elemento
	int    number;				// Numero di elementi di questo tipo

  std::vector<PlyProperty> props;	// Vettore dinamico delle property
};


class PlyFile
{
private:
	void compile( PlyElement * e );
	void compile( PlyProperty * p );

public:
		// Modi di apertura
	enum {
		MODE_READ,
		MODE_WRITE
	};

	 PlyFile();
	~PlyFile();

		// Apre un file ply
	int Open( const char * filename, int mode );
		// Chiude un file e disalloca la memoria
	void Destroy();
		// Ritorna il codice dell'ultimo errore
	inline int GetError() const { return error; }
		// Definizione di lettura (Vedi struttura PropDescriptor)
	int AddToRead(
		const char * elemname,
		const char * propname,
		int	stotype1,
		int memtype1,
		size_t offset1,
		int islist,
		int alloclist,
		int stotype2,
		int memtype2,
		size_t offset2
	);
		// Come sopra ma con descrittore
	inline int AddToRead( const PropDescriptor & p )
	{
		return AddToRead(p.elemname,p.propname,p.stotype1,
			p.memtype1,p.offset1,p.islist,p.alloclist,p.stotype2,
			p.memtype2,p.offset2
		);
	}
	
		// Ritorna il numero di oggetti di un tipo di elemento
	const char * ElemName( int i );

	int ElemNumber( int i ) const;
		// Setta il tipo di elemento corrente per effetture
		// la lettura
	inline void SetCurElement( int i )
	{
		if(i<0 || i>=int(elements.size())) cure = 0;
		else
		{
			cure = &(elements[i]);
			compile(cure);
		}
	}
		// Lettura du un elemento
	int Read( void * mem );

  std::vector<PlyElement>   elements;	// Vettore degli elementi
	std::vector<std::string>  comments;	// Vettore dei commenti
	static const char * typenames[9];
	static const char * newtypenames[9];

  inline const char * GetHeader() const { return header.c_str(); }
protected:

	GZFILE gzfp;

	float  version;				// Versione del file
	int    error;				// Errore corrente (vedi enum)
	int    format;				// Formato del file (vedi enum )

  std::string   header;			// Testo dell'header	

	PlyElement * cure;			// Elemento da leggere

		// Callback di lettura: vale ReadBin o ReadAcii
	int (* ReadCB)( GZFILE fp, const PlyProperty * r, void * mem, int fmt );

	int OpenRead( const char * filename );
	int OpenWrite( const char * filename );
	
	PlyElement * AddElement( const char * name, int number );
	int FindType( const char * name ) const;
	PlyElement * FindElement( const char * name );
};

void interpret_texture_name(const char*a, const char*fn, char*output);

  } // end namespace ply
} // end namespace vcg
#endif
