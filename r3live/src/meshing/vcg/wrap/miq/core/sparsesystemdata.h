#ifndef __SPARSE_SYSTEM_DATA_H__
#define __SPARSE_SYSTEM_DATA_H__

#include "auxmath.h"
//#include "MatlabInterface.h"

class SparseMatrixData{ 
protected:
  unsigned int m_nrows;
  unsigned int m_ncols;
  unsigned int m_nentries; // total number of (including repeated) (i,j) entries
  unsigned int m_reserved_entries;
  unsigned int *m_rowind;
  unsigned int *m_colind;
  double       *m_vals;
  unsigned int *m_nz;      // nonzeros per row


  public:
  unsigned int   nrows()    { return m_nrows   ; }
  unsigned int   ncols()    { return m_ncols   ; }
  unsigned int   nentries() { return m_nentries; }
  unsigned int*  nz()       { return m_nz      ; }
  unsigned int&  nz(unsigned int i) { assert(i < m_nrows); return m_nz[i]; }
  unsigned int*  rowind()   { return m_rowind  ; }
  unsigned int*  colind()   { return m_colind  ; }
  double*        vals()     { return m_vals    ; }

  // create an empty matrix with a fixed number of rows
  SparseMatrixData(): m_nrows(0), m_ncols(0), m_nentries(0), m_reserved_entries(0), m_rowind(0), m_colind(0), m_vals(0),
    m_nz(NULL){ }

// create an empty matrix with a fixed number of rows
  void initialize(int nr, int nc) {
      assert(nr >= 0 && nc >=0);
      m_nrows = nr;
      m_ncols = nc;
      m_nentries = 0;
      m_reserved_entries = 0;
      m_rowind = NULL;
      m_colind = NULL;
      m_vals   = NULL;
      if(nr > 0) {
        m_nz     = new unsigned   int[m_nrows];
        assert(m_nz); 
      } else m_nz = 0; 
      std::fill( m_nz, m_nz+m_nrows, 0 );
  }
  // allocate space for nnz nonzero entries
  void reserve(int nnz){  
    assert(nnz >= 0);
    cleanup(); 
    if(nnz > 0) {
      m_rowind = new unsigned   int[nnz];
      m_colind = new unsigned   int[nnz];
      m_vals   = new          double[nnz];
      assert( m_rowind && m_colind && m_vals);
    } else { m_rowind = 0; m_colind = 0; m_vals = 0; }
    m_reserved_entries = nnz;
  }
  // extend space for entries
  void extend_entries(int extra_nnz){  
    assert(m_nrows > 0);
    if( extra_nnz <=0) return;
    unsigned int*  old_rowind = m_rowind; 
    unsigned int* old_colind = m_colind;
    double* old_vals =   m_vals; 
    m_rowind = new unsigned   int[m_reserved_entries+extra_nnz];
    m_colind = new unsigned   int[m_reserved_entries+extra_nnz];
    m_vals   = new          double[m_reserved_entries+extra_nnz];
    assert( m_rowind && m_colind && m_vals);
    if(old_rowind) { std::copy(old_rowind, old_rowind+m_reserved_entries, m_rowind); delete[] old_rowind;}
    if(old_colind) { std::copy(old_colind, old_colind+m_reserved_entries, m_colind); delete[] old_colind;}
    if(old_vals)   { std::copy(old_vals, old_vals+m_reserved_entries, m_vals); delete[] old_vals; }
    m_reserved_entries += extra_nnz;
  }

  virtual void extend_rows(int extra_rows){  
    if(extra_rows <= 0) return; 
    unsigned int* old_nz = m_nz; 
    m_nz     = new unsigned   int[m_nrows+extra_rows];
    if(old_nz) { std::copy(old_nz, old_nz+m_nrows, m_nz); delete[] old_nz;}
    std::fill( m_nz+m_nrows,  m_nz+m_nrows+extra_rows, 0);
    m_nrows +=  extra_rows;   
  }

  // reset the matrix to empty, deallocate space
  void cleanup(){ 
      if(m_rowind) delete[] m_rowind; m_rowind = 0; 
      if(m_colind) delete[] m_colind; m_colind = 0;
      if(m_vals)   delete[] m_vals;   m_vals = 0;
      m_nentries = 0; 
      m_reserved_entries = 0;
  }

  // add a nonzero entry to the matrix 
  // no checks are done for coinciding entries
  // the interpretation of the repeated entries (replace or add) 
  // depends on how the actual sparse matrix datastructure is constructed

  void addEntryCmplx(unsigned int i, unsigned int j, Cmplx val) {         
        assert(m_nentries < m_reserved_entries-3);
        m_rowind[m_nentries  ] = 2*i;   m_colind[m_nentries  ] = 2*j;   m_vals[m_nentries]   =  val.real();
        m_rowind[m_nentries+1] = 2*i;   m_colind[m_nentries+1] = 2*j+1; m_vals[m_nentries+1] = -val.imag();
        m_rowind[m_nentries+2] = 2*i+1; m_colind[m_nentries+2] = 2*j;   m_vals[m_nentries+2] =  val.imag();
        m_rowind[m_nentries+3] = 2*i+1; m_colind[m_nentries+3] = 2*j+1; m_vals[m_nentries+3] =  val.real();
        m_nentries += 4;
  }

  void addEntryReal(unsigned int i, unsigned int j, double val) {         
        assert(m_nentries < m_reserved_entries);
        m_rowind[m_nentries] = i;   m_colind[m_nentries] = j;   m_vals[m_nentries] =  val;
        m_nentries++;
  }


  void symmetrize(unsigned int startind, unsigned int endind) { 
    assert( startind <= m_nentries && endind <= m_nentries); 
    for( unsigned int i = startind; i < endind; i++) { 
      addEntryReal( m_colind[i], m_rowind[i], m_vals[i]); 
    }
  }

  /*void sendToMatlab(const char* name) { 
     MatlabInterface matlab;
     if( m_rowind && m_colind && m_vals) 
       matlab.SetEngineSparseRealMatrix(name, m_nentries, &m_rowind[0], &m_colind[0], &m_vals[0], m_nrows, m_ncols);
     else 
       matlab.SetEngineSparseRealMatrix(name, 0, (const int*)0, (const int*)0, (const double*)0, m_nrows, m_ncols);
  }*/


 /* void getFromMatlab(const char* name) { 
     MatlabInterface matlab;
     cleanup(); 
     if (m_nz) delete[] m_nz; 
     matlab.GetEncodedSparseRealMatrix(name,m_rowind,m_colind,m_vals, m_nentries);
     m_reserved_entries = m_nentries; 
     m_nrows = 0; 
     m_ncols = 0; 
     for(int i = 0; i < (int)m_nentries; i++) { 
       m_nrows = std::max(m_rowind[i]+1,m_nrows);
       m_ncols = std::max(m_colind[i]+1,m_ncols);  
     }
     m_nz = new unsigned int[m_nrows];
     std::fill(m_nz, m_nz+m_nrows,0);
     for(int i = 0; i < (int)m_nentries; i++) { 
       m_nz[m_rowind[i]]++;
     }
  }*/

  virtual ~SparseMatrixData() {
    cleanup();
    delete [] m_nz;
  }

};

// a small class to manage storage for matrix data
// not using stl vectors: want to make all memory management
// explicit to avoid hidden automatic reallocation 
// TODO: redo with STL vectors but with explicit mem. management

class SparseSystemData { 
private:
  // matrix representation,  A[rowind[i],colind[i]] = vals[i]
  // right-hand side
  SparseMatrixData m_A;
  double       *m_b;
  double       *m_x;

public:
  SparseMatrixData& A() { return m_A; }
  double*        b()        { return m_b       ; }
  double*        x()        { return m_x       ; }
  unsigned int   nrows()    { return  m_A.nrows(); }

public:
  
  SparseSystemData(): m_A(), m_b(NULL), m_x(NULL){ }

  void initialize(unsigned int nr, unsigned int nc) {
      m_A.initialize(nr,nc); 
      m_b      = new          double[nr];
      m_x      = new          double[nr];     
      assert(m_b); 
      std::fill( m_b,  m_b+nr, 0.);
  }

  void addRHSCmplx(unsigned int i, Cmplx val) { 
      assert( 2*i+1 < m_A.nrows()); 
      m_b[2*i] += val.real(); m_b[2*i+1] += val.imag();
  }

  void setRHSCmplx(unsigned int i, Cmplx val) { 
      assert( 2*i+1 < m_A.nrows()); 
      m_b[2*i] = val.real(); m_b[2*i+1] = val.imag();
  }

  Cmplx getRHSCmplx(unsigned int i) { 
      assert( 2*i+1 < m_A.nrows());
      return Cmplx( m_b[2*i], m_b[2*i+1]);
  }

  double getRHSReal(unsigned int i) { 
      assert( i < m_A.nrows());
      return m_b[i];
  }

  Cmplx getXCmplx(unsigned int i) { 
      assert( 2*i+1 < m_A.nrows());
      return Cmplx( m_x[2*i], m_x[2*i+1]);
  }

  virtual void extend_rows(int extra_rows){  
    if(extra_rows <= 0) return; 
   
    double* old_b = m_b; 
    m_b    = new double[m_A.nrows()+extra_rows];
    if(old_b) { std::copy(old_b, old_b+m_A.nrows(), m_b); delete[] old_b;}
    std::fill( m_b+m_A.nrows(),  m_b+m_A.nrows()+extra_rows, 0.);
    double* old_x = m_x; 
    m_x    = new double[m_A.nrows()+extra_rows];
    if(old_x) { std::copy(old_x, old_x+m_A.nrows(), m_x); delete[] old_x;}
    m_A.extend_rows(extra_rows); 
  }


  /*void getFromMatlab(const char* nameA, const char* nameb, const char* namex, unsigned int n_vars, bool getsol=false) {
    MatlabInterface matlab; 
    m_A.getFromMatlab(nameA);
    assert(n_vars <= m_A.nrows()); 
    if (m_b) delete [] m_b;
    if (m_x) delete [] m_x;
    m_b = new double[m_A.nrows()];
    m_x = new double[n_vars];
    matlab.GetEngineRealMatrix(nameb, m_A.nrows(),1, m_b);
    if(getsol) { 
      if (m_x) delete [] m_x;
      matlab.GetEngineRealMatrix(nameb, n_vars,1, m_x);
    }
  }*/
	
  void cleanMem() {
    m_A.cleanup(); 
    delete [] m_b;
    delete [] m_x;
  }

  virtual ~SparseSystemData() {
    delete [] m_b;
    delete [] m_x;
  }
};

#endif
