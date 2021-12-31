#include <vector>
#include <map>
#include <algorithm>


///this class define the interface to use sparse matrix
///you must extend this class and implement the code of each function in order
///to the system solver you're using.
///For details on implementation see system_interface_LDL.h as example
template <class ScalarType>
class SparseMatrix{

public:

	std::vector<int> _Ap;
	std::vector<int> _Ai;
	std::vector<double> _Ax;
	
	typedef typename std::pair<int,int> IndexType;
	
	int _dimension;	

public:

///initilaization of the system
virtual void Initalize(int dimension)
{_dimension=dimension;}

///create a sparse matrix given a set of entries as vector 
///of pair of int
virtual void CreateSparse(std::vector<IndexType> Entries)
{}

///return the value of the matrix 
virtual ScalarType &A(int row,int col)
{return (_Ax[0]);}

///return true if the rapresention of sparse matriz is symmetric
virtual bool IsSymmetric()
{return false;}

virtual void Zero()
{}

///return the dimension of the matrix
virtual int Size(){return _dimension;}

};