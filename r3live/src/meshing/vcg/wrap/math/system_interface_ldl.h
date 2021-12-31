extern "C"{
#include <ldl.h>
}
#include <vector>
#include <map>
#include <algorithm>
#include <sparse_matrix.h>

class SystemLDL:SparseMatrix<double>{
private:
	typedef std::map<IndexType,int> mapType;
	std::vector<double> _B;
	std::vector<double> _X;
	
	std::vector<double> Lx,D,Y ;
    std::vector<int> Li,Lp,Parent,Lnz,Flag,Pattern;
	
	mapType _M;
	
public:

///allocate the resources for the system of equations
void Initalize(int dimension)
{
  _dimension=dimension;
  _Ap.resize(_dimension+1);
  _M.clear();
  _B.resize(_dimension);
  _X.resize(_dimension);
}

double &A(int row,int col)
{
	
	IndexType I=IndexType(row,col);
	mapType::const_iterator ci=_M.find(I);
	if (ci==_M.end())
	{
		std::swap<int>(I.first,I.second);
		ci=_M.find(I);
	}
	assert(ci!=_M.end());
	int index=(*ci).second;

	return(_Ax[index]);
}

double &B(int i)
{return(_B[i]);}

double &X(int i)
{return (_X[i]);}


void Solve()
{
	int d,i;

  //   /* factorize A into LDL' (P and Pinv not used) */
    ldl_symbolic (_dimension, &(*_Ap.begin()), &(*_Ai.begin()), &(*Lp.begin()), 
		&(*Parent.begin()), &(*Lnz.begin()), &(*Flag.begin()), NULL, NULL) ;

  //  printf ("Nonzeros in L, excluding diagonal: %d\n", Lp [_dimension]) ;

	d = ldl_numeric (_dimension, &(*_Ap.begin()), &(*_Ai.begin()), &(*_Ax.begin()), &(*Lp.begin())
		, &(*Parent.begin()), &(*Lnz.begin()), &(*Li.begin()), &(*Lx.begin()), 
		&(*D.begin()), &(*Y.begin()), &(*Pattern.begin()),
	&(*Flag.begin()), NULL, NULL) ;

    if (d == _dimension)
    {
	/* solve Ax=b, overwriting b with the solution x */
	ldl_lsolve (_dimension, &(*_B.begin()), &(*Lp.begin()), &(*Li.begin()), &(*Lx.begin())) ;
	ldl_dsolve (_dimension, &(*_B.begin()), (&*D.begin()) );
	ldl_ltsolve (_dimension, &(*_B.begin()), &(*Lp.begin()), &(*Li.begin()), &(*Lx.begin())) ;

	for (i = 0 ; i < _dimension ; i++) 
		_X[i]=_B[i];//printf ("x [%d] = %g\n", i, b [i]) ;
    }
	 else///dl_numeric failed
    {
		assert(0);
    }
}

bool IsSymmetric()
{return true;}

void Zero()
{
	for (int i=0;i<Size();i++)
		_Ax[i]=0;
}

int Size(){return (_dimension);}

///K is symmetric positice definite matrix
void CreateSparse(std::vector<IndexType> Entries)
{
	_Ax.clear();
	_Ai.clear();
	int _nonzero=0;

	///put the index of vertices for each edge 
	///in the right order for simmetry of the sistem
	std::vector<IndexType>::iterator Vi;
	for (Vi=Entries.begin();Vi<Entries.end();Vi++)
	{
		assert((*Vi).first>=0);
		assert((*Vi).second>=0);
		if ((*Vi).first>(*Vi).second)
			std::swap<int>((*Vi).first,(*Vi).second);
	}
	
	///the sort and erase duplicates
	std::sort(Entries.begin(),Entries.end());
	std::vector<IndexType>::iterator Vend=std::unique(Entries.begin(),Entries.end());
	Entries.erase(Vend,Entries.end());
	
	_Ax.resize(Entries.size());
	_Ai.resize(Entries.size());
	_M.clear();

	int col=0;
	int i=0;
	Vi=Entries.begin();
	while (Vi<Entries.end())
	{
		col=(*Vi).first;
		_Ap[i]=_nonzero;
		//go to next colummn
		while ((col==(*Vi).first)&&(Vi<Entries.end()))
		{
			IndexType I=IndexType((*Vi).first,(*Vi).second);
			_M.insert(std::pair<IndexType,int>(I,_nonzero));
			_Ai[_nonzero]=(*Vi).second;
			_nonzero++;
			Vi++;
		}
		i++;
	}

	_Ap[_dimension]=_nonzero;
	Lx.resize(_nonzero);
	D.resize(_dimension);
	Y.resize(_dimension);
    Li.resize(_nonzero);
	Lp.resize(_dimension+1);
	Parent.resize(_dimension);
	Lnz.resize(_dimension);
	Flag.resize(_dimension);
	Pattern.resize(_dimension);
}

};