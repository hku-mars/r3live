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

#ifndef VCG_SPACE_INDEX_PERFECT_SPATIAL_HASHING_H
#define VCG_SPACE_INDEX_PERFECT_SPATIAL_HASHING_H

#pragma warning(disable : 4996)

#define _USE_GRID_UTIL_PARTIONING_ 1
#define _USE_OCTREE_PARTITIONING_  (1-_USE_GRID_UTIL_PARTIONING_)

#include <vector>
#include <list>
#include <algorithm>

#include <vcg/space/index/base.h>
#include <vcg/space/index/grid_util.h>

#include <vcg/space/point2.h>
#include <vcg/space/point3.h>
#include <vcg/space/box3.h>

namespace vcg
{

	// Compute the greatest common divisor between two integers a and b
	int GreatestCommonDivisor(const int a, const int b)
	{
		int m = a;
		int n = b;

		do 
		{
			if (m<n) std::swap(m, n);
			m = m % n;
			std::swap(m, n);
		}
		while (n!=0);
		return m;
	}
	
	// Doxygen documentation
	/** \addtogroup index */
	/*! @{ */ 

	/*!
	 * This class implements the perfect spatial hashing by S.Lefebvre and H.Hoppe
	 * This is an spatial indexing structure such as the uniform grid, but with lower
	 * memory requirements, since all the empty cells of the uniform grid are removed.
	 * Access to a non-empty cell is performed looking up in two d-dimensional tables,
	 * the offset table and the hash table.
	 * @param OBJECT_TYPE (Template parameter) the type of objects to be indexed
	 * @param SCALAR_TYPE (Template parameter) the scalar type
	 */
	template < class OBJECT_TYPE, class SCALAR_TYPE >
	class PerfectSpatialHashing : public vcg::SpatialIndex< OBJECT_TYPE, SCALAR_TYPE >
	{
		// Given an object or a pointer to an object, return the reference to the object
		template < typename TYPE >
		struct Dereferencer
		{
			static				TYPE& Reference(TYPE				 &t)	{	return  t;	}
			static				TYPE& Reference(TYPE*			 &t)	{ return *t;  }
			static const  TYPE& Reference(const TYPE	 &t)	{ return  t;	}
			static const	TYPE& Reference(const TYPE* &t)	{ return *t;	}
		};

		// Given a type, holds this type in Type 
		template < typename TYPE >
		struct ReferenceType { typedef TYPE Type; };

		// Given as type a "pointer to type", holds the type in Type
		template < typename TYPE >
		struct ReferenceType< TYPE * > { typedef typename ReferenceType<TYPE>::Type Type; };

	public:
		typedef						SCALAR_TYPE													ScalarType;
		typedef						OBJECT_TYPE													ObjectType;
		typedef typename	ReferenceType< ObjectType >::Type * ObjectPointer;
		typedef typename  vcg::Box3< ScalarType >							BoundingBoxType;
		typedef typename	vcg::Point3< ScalarType >						CoordinateType;
		
	protected:
		/*! \struct NeighboringEntryIterator
		* This class provides a convenient way to iterate over the six neighboring cells of a given cell.
		*/
		struct NeighboringEntryIterator
		{
			/*!
			* Default constructor. 
			* @param[in] entry				The index of the cell in the UniformGrid around which iterate.
			* @param[in] table_size  The number of cells in the UniformGrid for each side.
			*/
			NeighboringEntryIterator(const vcg::Point3i &entry, const int table_size)
			{
				m_Center		= entry;
				m_TableSize = table_size;
				m_CurrentNeighbor.X() = (m_Center.X()+m_TableSize-1)%m_TableSize;
				m_CurrentNeighbor.Y() = m_Center.Y();
				m_CurrentNeighbor.Z() = m_Center.Z();
				m_CurrentIteration		= 0;
			}

			/*!
			* Increment the iterator to point to the next neighboring entry in the UniformGrid
			*/
			void operator++(int)
			{
				switch(++m_CurrentIteration)
				{
				case 1: m_CurrentNeighbor.X()=(m_Center.X()+1)%m_TableSize; break;
				case 2: m_CurrentNeighbor.X()=m_Center.X(); m_CurrentNeighbor.Y()=(m_Center.Y()+m_TableSize-1)%m_TableSize; break;
				case 3: m_CurrentNeighbor.Y()=(m_Center.Y()+1)%m_TableSize; break;
				case 4: m_CurrentNeighbor.Y()=m_Center.Y(); m_CurrentNeighbor.Z()=(m_Center.Z()+m_TableSize-1)%m_TableSize; break;
				case 5: m_CurrentNeighbor.Z()=(m_Center.Z()+1)%m_TableSize; break;
				default: m_CurrentNeighbor = vcg::Point3i(-1, -1, -1); break;
				}
			}

			/*!
			* Dereferencing operator
			* \return The neighbor of the given cell at the current iteration
			*/
			vcg::Point3i operator*() { return m_CurrentNeighbor; }

			/*!
			* Assignment operator
			* @param[in] The source neighboring iterator
			* \return		The reference to this iterator
			*/
			NeighboringEntryIterator& operator =(const NeighboringEntryIterator &it)
			{
				m_Center						= it.m_Center						;
				m_CurrentNeighbor		= it.m_CurrentNeighbor	;
				m_CurrentIteration	= it.m_CurrentIteration	;
				m_TableSize					= it.m_TableSize				;
				return *this;
			}

			/*!
			* Less than operator. Since each entry in the UniformGrid has only 6 neighbors, 
			* the iterator over the neighboring entries can be compared with an integer.
			*/
			inline bool operator <(const int value) { return m_CurrentIteration<value; }

		protected:
			vcg::Point3i	m_Center;						/*!< The cell whose neighboring cells are to be looked up.	*/
			vcg::Point3i	m_CurrentNeighbor;	/*!< The neighboring cell at the current iteration.					*/
			int						m_CurrentIteration; /*!< The current iteration.																	*/
			int						m_TableSize;				/*!< The number of cell in the UniformGrid for each side		*/
		}; // end of class NeighboringEntryIterator



		/************************************************************************/
		/*! \class UniformGrid
		 * This class represent the domain U in the original article. It is used 
		 * only during the construction of the offset and hash tables. 
		 */ 
		/************************************************************************/
		class UniformGrid
		{
		public:
			typedef vcg::Point3i CellCoordinate;

			/*! \struct EntryIterator
			 * This class provides a convenient way to iterate over the set of the grid cells.
			 */
			struct EntryIterator
			{
				friend class UniformGrid;

				/*!
				 * Default constructor
				 */
				EntryIterator(UniformGrid *uniform_grid, const CellCoordinate &position)
				{
					m_UniformGrid			= uniform_grid;
					m_CurrentPosition = position;
				}

				
				/*!
				 * Increment operator. Move the iterator to the next cell in the UniformGrid
				 */
				void operator++(int)
				{
					if (++m_CurrentPosition.Z()==m_UniformGrid->GetResolution())
					{
						m_CurrentPosition.Z() = 0;
						if (++m_CurrentPosition.Y()==m_UniformGrid->GetResolution())
						{
							m_CurrentPosition.Y() = 0;
							if (++m_CurrentPosition.X()==m_UniformGrid->GetResolution())
								m_CurrentPosition = CellCoordinate(-1, -1, -1);
						}
					}
				}


				/*!
				 * Copy operator.
				 * @param[in] it The iterator whose value has to be copied.
				 */
				void operator =(const EntryIterator &it)
				{
					m_UniformGrid			= it.m_UniformGrid;
					m_CurrentPosition = it.m_CurrentPosition;
				}

				/*!
				 * Equivalence operator
				 */
				bool operator==(const EntryIterator &it) const
				{
					return m_CurrentPosition==it.m_CurrentPosition;
				}

				/*!
				 * Diversity operator
				 */
				bool operator!=(const EntryIterator &it) const
				{
					return m_CurrentPosition!=it.m_CurrentPosition;
				}

				/*!
				 * Dereferencing operator.
				 * \return The pointer to the vector of the objects contained in the cell pointed to by the iterator.
				 */
				std::vector< ObjectPointer >* operator*()
				{
					return m_UniformGrid->GetObjects(m_CurrentPosition);
				}

				/*!
				 * Return the index of the cell pointed to by the iterator.
				 */
				CellCoordinate GetPosition() const
				{
					return m_CurrentPosition;
				}


			protected:
				UniformGrid			* m_UniformGrid;
				CellCoordinate		m_CurrentPosition;
			}; // end of struct EntryIterator


			/*!
			 * Default constructor
			 */
			UniformGrid() {}

			/*!
			 * Default destructor
			 */
			~UniformGrid() {}


			/*!
			 * These functions return an iterator pointing to the first and the last cell of the grid respectively.
			 */
			EntryIterator Begin() { return EntryIterator(this, CellCoordinate( 0,  0,  0)); }
			EntryIterator End()		{ return EntryIterator(this, CellCoordinate(-1, -1, -1)); }

			
			/*!
			 * Return an iterator that iterates over the six adjacent cells of a given cell.
			 * @param[in] at	The cell around which this iterator takes values.
			 * \return				The iterator over the neighboring cells of <CODE>at</CODE>.
			 */
			NeighboringEntryIterator GetNeighboringEntryIterator(const CellCoordinate &at) { return NeighboringEntryIterator(at, m_CellPerSide); }


			/*!
			 * Allocate the necessary space for the uniform grid.
			 * @param[in] bounding_box	The bounding box enclosing the whole dataset.
			 * @param[in] cell_per_side	The resolution of the grid.
			 */
			void Allocate(const BoundingBoxType &bounding_box, const int cell_per_side)
			{
				m_CellPerSide = cell_per_side;
				m_BoundingBox = bounding_box;
				m_CellSize		= (m_BoundingBox.max - m_BoundingBox.min)/ScalarType(cell_per_side);
				
				m_Grid.resize(m_CellPerSide);
				for (int i=0; i<m_CellPerSide; i++)
				{
					m_Grid[i].resize(m_CellPerSide);
					for (int j=0; j<m_CellPerSide; j++)
						m_Grid[i][j].resize(m_CellPerSide);
				}
			}


			/*!
			 * Removes all the reference to the domain data from the UniformGrid cells.
			 */
			void Finalize()
			{
				m_Grid.clear();
			}


			/*!
			 * Adds a set of elements to the uniform grid.
			 * @param[in] begin The iterator addressing the position of the first element in the range to be added.
			 * @param[in] end		The iterator addressing the position one past the final element in the range to be added.
			 */
			template < class OBJECT_ITERATOR >
			void InsertElements(const OBJECT_ITERATOR &begin, const OBJECT_ITERATOR &end)
			{
				typedef OBJECT_ITERATOR ObjectIterator;
				typedef Dereferencer< typename ReferenceType< typename OBJECT_ITERATOR::value_type >::Type > ObjectDereferencer;

				std::vector< CellCoordinate > cells_occupied;
				for (ObjectIterator iObject=begin; iObject!=end; iObject++)
				{
					ObjectPointer pObject = &ObjectDereferencer::Reference( *iObject );
					GetCellsIndex( pObject, cells_occupied);
					for (std::vector< CellCoordinate >::iterator iCell=cells_occupied.begin(), eCell=cells_occupied.end(); iCell!=eCell; iCell++)
						GetObjects( *iCell )->push_back( pObject );
					cells_occupied.clear();
				}
			}


			/*!
			 * Given a point contained in the UniformGrid, returns the index of the cell where it's contained.
			 * @param[in] query The 3D point.
			 * \return					The index of the UniformGrid entry where this point is contained.
			 */
			inline CellCoordinate Interize(const CoordinateType &query) const
			{
				CellCoordinate result;
				result.X() = (int) floorf( (query.X()-m_BoundingBox.min.X())/m_CellSize.X() );
				result.Y() = (int) floorf( (query.Y()-m_BoundingBox.min.Y())/m_CellSize.Y() );
				result.Z() = (int) floorf( (query.Z()-m_BoundingBox.min.Z())/m_CellSize.Z() );
				return result;
			}

			/*!
			 * Given a bounding box contained in the UniformGrid, returns its integer-equivalent bounding box.
			 * @param[in] bounding_box	The bounding box in the 3D space.
			 * \return									The integer representation of the bounding box.
			 */
			inline vcg::Box3i Interize(const BoundingBoxType &bounding_box) const
			{
				vcg::Box3i result;
				result.min = Interize(bounding_box.min);
				result.max = Interize(bounding_box.max);
				return result;
			}

			
			/*!
			 * Given the pointer to an object, returns the set of cells in the uniform grid containing the object.
			 * @param[in]		pObject					The pointer to the object
			 * @param[out]	cells_occuppied	The set of cell index containing the object
			 */
			void GetCellsIndex(const ObjectPointer pObject, std::vector< CellCoordinate > & cells_occupied)
			{
				BoundingBoxType object_bb;
				(*pObject).GetBBox(object_bb);
				CoordinateType corner = object_bb.min;
				
				while (object_bb.IsIn(corner))
				{
					CellCoordinate cell_index;
					cell_index.X() = (int) floorf( (corner.X()-m_BoundingBox.min.X())/m_CellSize.X() );
					cell_index.Y() = (int) floorf( (corner.Y()-m_BoundingBox.min.Y())/m_CellSize.Y() );
					cell_index.Z() = (int) floorf( (corner.Z()-m_BoundingBox.min.Z())/m_CellSize.Z() );
					cells_occupied.push_back( cell_index );

					if ((corner.X()+=m_CellSize.X())>object_bb.max.X())
					{
						corner.X() = object_bb.min.X();
						if ( (corner.Z()+=m_CellSize.Z())>object_bb.max.Z() )
						{
							corner.Z() = object_bb.min.Z();
							corner.Y() += m_CellSize.Y();
						}
					}
				}
			}


			/*!
			 * Return the number of cells of the uniform grid where there are no item of the input dataset.
			 * \return The number of cells occupied by at least one item.
			 */
			int GetNumberOfNotEmptyCells()
			{
				int number_of_not_empty_cell = 0;
				for (int i=0; i<m_CellPerSide; i++)
					for (int j=0; j<m_CellPerSide; j++)
						for (int k=0; k<m_CellPerSide; k++)
							if (GetObjects(i, j, k)->size()>0)
								number_of_not_empty_cell++;
				return number_of_not_empty_cell;
			}

			/*!
			 * Returns the number of entries for each side of the grid.
			 * \return The resolution of the UniformGrid in each dimension.
			 */
			inline int GetResolution() const { return m_CellPerSide; }


			/*!
			 * Return the pointer to a vector containing pointers to the objects falling in a given domain cell.
			 * @param[in] at	The index of the cell of the uniform grid where looking for
			 * \return				A pointer to a vector of pointers to the objects falling in the cell having index <CODE>at</CODE>.
			 */
			std::vector< ObjectPointer >* GetObjects(const int i, const int j, const int k) { return &m_Grid[i][j][k]; }
			std::vector< ObjectPointer >* GetObjects(const CellCoordinate &at)							{ return &m_Grid[at.X()][at.Y()][at.Z()];}
			std::vector< ObjectPointer >* operator[](const CellCoordinate &at)							{ return &m_Grid[at.X()][at.Y()][at.Z()];}
			
		protected:
			std::vector< std::vector< std::vector< std::vector< ObjectPointer > > > > 
											m_Grid;					/*!< The uniform grid												*/
			BoundingBoxType	m_BoundingBox;	/*!< The bounding box of the uniform grid.	*/
			int							m_CellPerSide;	/*!< The number of cell per side.						*/
			CoordinateType 	m_CellSize;			/*!< The dimension of each cell.						*/
		}; //end of class UniformGrid




		/************************************************************************/
		/*! \class HashTable
		 * This class substitutes the uniform grid. 
		 */
		/************************************************************************/
		class HashTable
		{
		public:
			typedef vcg::Point3i EntryCoordinate;
			
			// We preferred using the Data structure instead of a pointer 
			// to the vector of the domain elements just for extensibility
			struct Data
			{
				/*!
				 * Default constructor
				 */
				Data(std::vector< ObjectPointer > *data) 
				{ 
					domain_data = data; 
				}
				
				std::vector< ObjectPointer >	*domain_data;
			};

			/*!
			 * Default constructor
			 */
			HashTable() {}

			/*!
			 * Default destructor
			 */
			~HashTable() { Clear(true); }


			/*!
			 * 
			 */
			NeighboringEntryIterator GetNeighborintEntryIterator(const EntryCoordinate &at) { return NeighboringEntryIterator(at, m_EntryPerSide); }

			
			/*!
			 * Allocates the space for the hash table; the number of entries created is entry_per_side^3.
			 * @param[in] entry_per_side The number of entries for each size
			 */
			void Allocate(const int entry_per_side)
			{
				m_EntryPerSide = entry_per_side;
				m_Table.resize(m_EntryPerSide);
				for (int i=0; i<m_EntryPerSide; i++)
				{
					m_Table[i].resize(m_EntryPerSide);
					for (int j=0; j<m_EntryPerSide; j++)
						m_Table[i][j].resize(m_EntryPerSide, NULL);
				}
				
				BuildFreeEntryList();	
			}

			
			/*
			 * Once the PerfectSpatialHash has been computed, all the unnecessary data can be eliminated.
			 * This function frees the empyt_list, and substitutes all the pointers to the UniformGrid 
			 * whit brand new pointers to the input objects.
			 */
			void Finalize()
			{
				Data *pData;
				for (int i=0; i<m_EntryPerSide; i++)
					for (int j=0; j<m_EntryPerSide; j++)
						for (int k=0; k<m_EntryPerSide; k++)
							if ((pData=GetData(i, j, k))!=NULL)
							{
								std::vector< ObjectPointer >	*domain_data = pData->domain_data;
								pData->domain_data = new std::vector< ObjectPointer>( *domain_data );
							}


				m_FreeEntries.clear();
			}


			/*!
			 * Inserts each entry in the hash table in the free entry list.
			 * When this function is called, each entry in the hash table must be free.
			 */
			void BuildFreeEntryList()
			{
				m_FreeEntries.clear();
				for (int i=0; i<m_EntryPerSide; i++)
					for (int j=0; j<m_EntryPerSide; j++)
						for (int k=0; k<m_EntryPerSide; k++)
						{
							assert(m_Table[i][j][k]==NULL);
							m_FreeEntries.push_back(EntryCoordinate(i, j, k));
						}
			}

			/*!
			 * Removes all the entries from the table and clears the free entry list
			 */
			void Clear(bool delete_vectors=false)
			{
				for (int i=0; i<m_EntryPerSide; i++)
					for (int j=0; j<m_EntryPerSide; j++)
						for (int k=0; k<m_EntryPerSide; k++)
							if (m_Table[i][j][k]!=NULL)
							{
								if (delete_vectors)
									delete m_Table[i][j][k]->domain_data;

								delete m_Table[i][j][k];
								m_Table[i][j][k] = NULL;
							}
				
				m_FreeEntries.clear();
			}

			/*!
			 * Returns the reference to the free entry list
			 * \return The reference to the free entry list
			 */
			std::list< EntryCoordinate >* GetFreeEntryList() { return &m_FreeEntries; }

			/*!
			 * Maps a given domain entry index into a hash table index.
			 * It corresponds to the \f$f_0\f$ function in the original article.
			 */
			EntryCoordinate DomainToHashTable(const typename UniformGrid::CellCoordinate &p)
			{
				EntryCoordinate result;
				result.X() = p.X()%m_EntryPerSide;
				result.Y() = p.Y()%m_EntryPerSide;
				result.Z() = p.Z()%m_EntryPerSide;
				return result;
			}

			/*!
			 * Inserts a new element in the hash table at the given position.
			 * @param[in] at		The position in the hash table where the new element will be created
			 * @param[in] data	The set of the domain elements contained in this entry
			 */ 
			void SetEntry(const EntryCoordinate &at, std::vector< ObjectPointer > *data)
			{
				assert(IsFree(at));
				m_Table[at.X()][at.Y()][at.Z()] = new Data(data);
				m_FreeEntries.remove(at);
			}

			/*!
			 * Given a hash table entry, this function modifies its coordinates in order to guarantee that
			 * they are in the valid range. Call this function before accessing the hash table.
			 * @param[in, out] entry The entry whose coordinates have to be checked.
			 */ 
			void ValidateEntry(EntryCoordinate &entry)
			{
				while (entry.X()<0) entry.X()+=m_EntryPerSide;
				while (entry.Y()<0) entry.Y()+=m_EntryPerSide;
				while (entry.Z()<0) entry.Z()+=m_EntryPerSide;
			}

			/*!
			 * Check if a given position in the hash table is free.
			 * @param[in] at The position of the hash table to check.
			 * \return			 True if and only if the hash table is free at the given position.
			 */
			inline bool IsFree(const EntryCoordinate &at) const
			{
				return (GetData(at)==NULL);
			}

			/*!
			 */
			inline int GetSize() { return m_EntryPerSide; }

			/*!
			 * Returns the number of free entries.
			 */
			inline int GetNumberOfFreeEntries() 
			{
				return int(m_FreeEntries.size());
			}

			/*!
			 * Return the number of entries where there is some domain data.
			 */
			inline int GetNumberOfNotEmptyEntries()
			{
				return (int(powf(float(m_EntryPerSide), 3.0f))-int(m_FreeEntries.size()));
			}

			/*!
			 * Return the pointer to the data stored in the hash table at the given position.
			 * @param[in] at	The position of the hash table where looks for the data.
			 * \return				A pointer to a valid data only if a valid pointer is stored in the hash 
			 *								table at the given position; otherwise return NULL.
			 */
			inline Data* GetData	 (const int i, const int j, const int k) const { return m_Table[i][j][k]; }
			inline Data* GetData	 (const EntryCoordinate &at) const { return m_Table[at.X()][at.Y()][at.Z()]; }
			inline Data* operator[](const EntryCoordinate &at) const { return m_Table[at.X()][at.Y()][at.Z()]; }

		protected:
			int																									m_EntryPerSide; /*!< The number of entries for each side of the hash-table. */
			std::vector< std::vector< std::vector < Data* > > > m_Table;				/*!< The table. */
			std::list< EntryCoordinate >												m_FreeEntries;  /*!< The list containing the free entries. */
		}; //end of class HashTable

		/************************************************************************/
		/*! \class OffsetTable
		* This class containts the offsets used for shifting the access to the hash table.
		*/
		/************************************************************************/
		class OffsetTable
		{
		public:
			typedef unsigned char						OffsetType;
			typedef vcg::Point3<OffsetType>	Offset;
			typedef Offset								* OffsetPointer;
			typedef vcg::Point3i						EntryCoordinate;
			
			/*! \struct PreImage
			 * This class represents the pre-image for a given entry in the offset table, that is the set
			 * \f$h_1^{-1}(q)={p \in S s.t. h_1(p)=q}\f$
			 */
			struct PreImage
			{
				/*!
				 * Default constructor.
				 * @param[in] at				The entry in the offset table where the cells in the preimage are mapped into.
				 * @param[in] preimage	The set of UniformGrid cells mapping to this entry.
				 */
				PreImage(EntryCoordinate &at, std::vector< typename UniformGrid::CellCoordinate > *preimage)
				{
					entry_index = at;
					pre_image		= preimage;
					cardinality = int(pre_image->size());
				}

				/*!
				 * less-than operator: needed for sorting the preimage slots based on their cardinality.
				 * @param second 
				 * \return <code>true</code> if and only if the cardinality of this preimage slot is greater than that of <code>second</code>.
				 */
				inline bool operator<(const PreImage &second) const { return (cardinality>second.cardinality); }
				

				std::vector< typename UniformGrid::CellCoordinate >	
													*	pre_image;		/*!< The set of entries in the uniform grid whose image through \f$h_1\f$ is this entry.*/
				EntryCoordinate			entry_index;  /*!< The index of the entry inside the offset table.	*/
				int									cardinality;	/*!< The cardinality of the pre-image.								*/
			}; // end of struct PreImage


			/*!
			 * Default constructor
			 */
			OffsetTable() { m_EntryPerSide=-1; m_NumberOfOccupiedEntries=0;}

			/*!
			 * Destructor
			 */
			~OffsetTable() { Clear(); }

			/*!
			 * Clear the entries in the offset table and in the preimage table.
			 */
			void Clear()
			{
				for (int i=0; i<m_EntryPerSide; i++)
					for (int j=0; j<m_EntryPerSide; j++)
						for (int k=0; k<m_EntryPerSide; k++)
							if (m_Table[i][j][k]!=NULL)
							{
								delete m_Table[i][j][k];
								m_Table[i][j][k] = NULL;
							}
				m_EntryPerSide = -1;
				m_H1PreImage.clear();
				m_NumberOfOccupiedEntries = 0;
			}

			/*!
			 * Allocate the space necessary for a offset table containing size entries for 
			 * each dimension and the necessary space for computing the relative anti-image. 
			 * @param[in] size The number of entries per side to allocate. 
			 */
			void Allocate(int size)
			{
				m_NumberOfOccupiedEntries = 0;
				
				m_EntryPerSide = size;
				m_Table.resize(m_EntryPerSide);
				for (int i=0; i<m_EntryPerSide; i++)
				{
					m_Table[i].resize(m_EntryPerSide);
					for (int j=0; j<m_EntryPerSide; j++)
						m_Table[i][j].resize(m_EntryPerSide, NULL);
				}

				m_H1PreImage.resize(m_EntryPerSide);
				for (int i=0; i<m_EntryPerSide; i++)
				{
					m_H1PreImage[i].resize(m_EntryPerSide);
					for (int j=0; j<m_EntryPerSide; j++)
						m_H1PreImage[i][j].resize(m_EntryPerSide);
				}
			}


			/*!
			 *
			 */
			void Finalize()
			{
				m_H1PreImage.clear();
			}


			/*!
			 * Build the pre-image of the \f$h_1\f$ function: the <CODE>m_H1PreImage</CODE> grid contains, for each  
			 * cell (i, j, k) a list of the domain grid (the UniformGrid) that are mapped through \f$h_1\f$ into that cell.
			 */
			void BuildH1PreImage(const typename UniformGrid::EntryIterator &begin, const typename UniformGrid::EntryIterator &end)
			{
				for (typename UniformGrid::EntryIterator iter=begin; iter!=end; iter++)
				{
					if ((*iter)->size()==0)
						continue;

					typename UniformGrid::CellCoordinate cell_index = iter.GetPosition();
					EntryCoordinate at = DomainToOffsetTable(cell_index);
					m_H1PreImage[at.X()][at.Y()][at.Z()].push_back(cell_index);
				}
			}

			/*!
			 * Sorts the entries of the PreImage table based on their cardinality.
			 * @param[out] preimage The list containing the entries of the preimage sorted by cardinality
			 */
			void GetPreImageSortedPerCardinality(std::list< PreImage > &pre_image)
			{
				pre_image.clear();
				for (int i=0; i<m_EntryPerSide; i++)
					for (int j=0; j<m_EntryPerSide; j++)
						for (int k=0; k<m_EntryPerSide; k++)
						{
							std::vector< typedef UniformGrid::CellCoordinate > *preimage = &m_H1PreImage[i][j][k];
							if (preimage->size()>0)
								pre_image.push_back( PreImage(typename UniformGrid::CellCoordinate(i, j, k), preimage) );
						}
				pre_image.sort();
			}
			
				
			/*!
			 * Check if the entries in the offset table near the given entry contain a valid offset.
			 * @param[in]				at The entry of the offset table whose neighboring entries will be checked.
			 * @param[out] offsets The set of consistent offset found by inspecting the neighboring entries.
			 * \return a vector containing possible offsets for the given entry
			 */
			void SuggestConsistentOffsets(const EntryCoordinate &at, std::vector< Offset > &offsets)
			{
				offsets.clear();
				for (int i=-1; i<2; i++)
					for (int j=-1; j<2; j++)
						for (int k=-1; k<2; k++)
						{
							if (i==0 && j==0 && k==0)
								continue;

							int x = (at.X()+i+m_EntryPerSide)%m_EntryPerSide;
							int y = (at.Y()+j+m_EntryPerSide)%m_EntryPerSide;
							int z = (at.Z()+k+m_EntryPerSide)%m_EntryPerSide;
							EntryCoordinate neighboring_entry(x, y, z);
							if (!IsFree(neighboring_entry))
								offsets.push_back( *GetOffset(neighboring_entry) );
						}
			}

			
			/*!
			 * Assures that the given entry can be used to access the offset table without throwing an out-of-bound exception.
			 * @param[in,out] entry The entry to be checked.
			 */
			void ValidateEntryCoordinate(EntryCoordinate &entry)
			{
				while (entry.X()<0) entry.X()+=m_EntryPerSide;
				while (entry.Y()<0) entry.Y()+=m_EntryPerSide;
				while (entry.Z()<0) entry.Z()+=m_EntryPerSide;
			}

			/*!
			 * Converts the coordinate of a given cell in the UniformGrid to a valid entry in the offset table.
			 * This function corresponds to the \f$h_1\f$ function of the article.
			 * @param[in] coord The index of a domain cell in the UniformGrid.
			 * \return					The coordinate of the entry corresponding to <CODE>coord<CODE> through this mapping.
			 */
			EntryCoordinate DomainToOffsetTable(const typename UniformGrid::CellCoordinate &coord)
			{
				EntryCoordinate result;
				result.X() = coord.X()%m_EntryPerSide;
				result.Y() = coord.Y()%m_EntryPerSide;
				result.Z() = coord.Z()%m_EntryPerSide;
				return result;
			}

			/*!
			 * Adds a new element to the offset table.
			 * @param[in] coord		The index of the UniformGrid cell whose offset has to be stored.
			 * @param[in] offset	The offset to associate to the given UniformGrid cell.
			 */
			void SetOffset(const typename UniformGrid::CellCoordinate &coord, const Offset &offset)
			{
				EntryCoordinate entry = DomainToOffsetTable(coord);
				assert(IsFree(entry));
				m_Table[entry.X()][entry.Y()][entry.Z()] = new Offset(offset);
				m_NumberOfOccupiedEntries++;
			}
			
			/*!
			 * Return a random offset: this function is used during the first steps of the creation process,
			 * when the offsets are computed at random.
			 * @param[out] A random offset
			 */
			void GetRandomOffset( Offset &offset )
			{
				offset.X() = OffsetType(rand()%m_MAX_VERSOR_LENGTH);
				offset.Y() = OffsetType(rand()%m_MAX_VERSOR_LENGTH);
				offset.Z() = OffsetType(rand()%m_MAX_VERSOR_LENGTH);
			}

			
			/*!
			* Return the number of entries of the offset table for each dimension.
			* \return The number of entries for each side
			*/
			inline int	GetSize()			const {return m_EntryPerSide;}


			/*!
			* Checks if the given entry in the offset table is free
			* @param[in] at The coordinate of the entry to be checked.
			* \return			 true if and only if the entry with coordinate <CODE>at</CODE> is free.
			*/
			inline bool IsFree(const EntryCoordinate &at) const { return GetOffset(at)==NULL;	} 
			//{ return m_Table[at.X()][at.Y()][at.Z()]==NULL;	}


			/*!
			 * Return the number of entries containing a valid offset.
			 * \return The number of not empty entries.
			 */
			inline int GetNumberOfOccupiedCells() const { return m_NumberOfOccupiedEntries;		}

			/*!
			 * Return the pointer to the offset stored at the given entry. NULL if that entry doesn't contain a offset
			 */
			inline OffsetPointer& GetOffset (const int i, const int j, const int k)				{ return m_Table[i][j][k]; } 
			inline OffsetPointer  GetOffset (const int i, const int j, const int k) const { return m_Table[i][j][k]; } 
			
			inline OffsetPointer& GetOffset (const EntryCoordinate &at)				{ return m_Table[at.X()][at.Y()][at.Z()]; } 
			inline OffsetPointer  GetOffset (const EntryCoordinate &at) const { return m_Table[at.X()][at.Y()][at.Z()]; } 
			
			inline OffsetPointer& operator[](const EntryCoordinate &at)				{ return m_Table[at.X()][at.Y()][at.Z()]; } 
			inline OffsetPointer  operator[](const EntryCoordinate &at) const { return m_Table[at.X()][at.Y()][at.Z()]; } 

		protected:
			const static int	m_MAX_VERSOR_LENGTH = 256;	/*!< The maximal length of the single component of each offset. */ 
			int								m_EntryPerSide;							/*!< The resolution of the offset table.												*/
			int								m_NumberOfOccupiedEntries;	/*!< The number of entries containing a valid offset.						*/
			std::vector< std::vector< std::vector< OffsetPointer > > >																				m_Table;			/*!< The offset table.					*/
			std::vector< std::vector< std::vector< std::vector< typename UniformGrid::CellCoordinate > > > >	m_H1PreImage; /*!< The \f$f1\f$ pre-image.		*/
		}; //end of class OffsetTable


		
		/*******************************************************************************************************************************/
		/*! \class BinaryImage
		* This class is used to encode the sparsity of the dataset. Since the hash table stores data associated with a sparse 
		* subset of the domain, it may be necessary to determine if an arbitrary query point lies in this defined domain.
		*/
		/*******************************************************************************************************************************/
		 class BinaryImage
		 {
		 public:
			 /*!
			  * Default constructor
				*/
			 BinaryImage() 
			 {
				 m_Resolution = -1;
			 }

			 
			 /*!
			  * Destructor
				*/
			 ~BinaryImage() {}


			 /*!
			  * Allocate the space necessary to encode the distribution of the dataset over the domain.
				* @param[in] size The resolution on each dimension of the bitmap.
				*/
			 void Allocate(const int size)
			 {
				 m_Resolution = size;
				 m_Mask.resize(m_Resolution);
				 for (int i=0; i<m_Resolution; i++)
				 {
					 m_Mask[i].resize(m_Resolution);
					 for (int j=0; j<m_Resolution; j++)
						 m_Mask[i][j].resize(m_Resolution, false);
				 }
			 }


			 /*!
			  * Removes all flags from the bitmap. This function is called after a failed attempt to create the offset-table.
				*/
			 void Clear()
			 {
				 for (int i=0; i<m_Resolution; i++)
					 for (int j=0; j<m_Resolution; j++)
						 std::fill(m_Mask[i][j].begin(), m_Mask[i][j].end(), false);
			 }


			 /*!
			  * Checks if a portion of the dataset fall inside the UniformGrid cell having coordinate <CODE>at</CODE>.
				* @param[in] at The index of the UniformGrid cell to check.
				* \return				<code>true</code> if and only if a portion of the dataset in included in this  UniformGrid cell.
				*/
			 inline bool ContainsData(const typename UniformGrid::CellCoordinate &at) const { return GetFlag(at)==true;}

			 
			 /*!
			  * Returns the number of entries in each dimension.
				* \return The resolution of the BinaryImage.
				*/
			 inline int GetResolution() const { return m_Resolution; }

			 
			 /*!
			  * Return the value stored in the d-dimensional bitmap at the given position.
				* @param[in] i 
				* @param[in] j 
				* @param[in] k
				*	\return
				*/
			 inline bool operator()(const int i, const int j, const int k)					{ return m_Mask[i][j][k]; }
			 
			 
			 /*!
			  * Return the value stored at the given position in the d-dimensional bitmap.
				* @param[in] at
				* \return
				*/
			 inline bool operator[](const typename UniformGrid::CellCoordinate &at)	{ return m_Mask[at.X()][at.Y()][at.Z()]; }
			 inline bool& GetFlag(const int i, const int j, const int k)const				{ return m_Mask[i][j][k]; }
			 inline void  SetFlat(const int i, const int j, const int k)						{ m_Mask[i][j][k] = true; }
			 

			 inline bool  GetFlag(const typename UniformGrid::CellCoordinate &at)	const			{ return m_Mask[at.X()][at.Y()][at.Z()]; }
			 inline void  SetFlag(const typename UniformGrid::CellCoordinate &at)						{ m_Mask[at.X()][at.Y()][at.Z()] = true; }

		 protected:
			 std::vector< std::vector< std::vector< bool > > > 
						m_Mask;					/*!< The bitmap image.							*/
			 int	m_Resolution;		/*!< The resolution of the bitmap.	*/
		 }; // end of class BinaryImage



		 /*******************************************************************************************************************************/
		 /*! \struct Neighbor
		 * This class is used to retrieve the neighboring objects in the spatial queries and to sort them.
		 */
		 /*******************************************************************************************************************************/
		 struct Neighbor
		 {
			 /*!
			  * Default constructor
				*/
			 Neighbor() 
			 { 
				 object		= NULL;
				 distance = ScalarType(-1.0);
				 nearest_point.SetZero();
			 }

			 
			 /*!
			  * Constructor
				* @param[in] pObject	The pointer to the object.
				* @param[in] dist			The distance between the object and the query point.
				* @param[in] point		The point on the object having minimal distance from the query point.
				*/
			 Neighbor(ObjectPointer pObject, ScalarType dist, CoordinateType point)
			 {
				 object = pObject;
				 distance = dist;
				 nearest_point(point);
			 }

			 
			 /*!
			  * Less than operator. Needed for sorting a range of neighbor based on their distance from the query object.
				*/
			 inline bool operator<(const Neighbor &second)
			 {
				 return distance<second.distance;
			 }

			 ObjectPointer	object;
			 ScalarType			distance;
			 CoordinateType nearest_point;
		 }; // end of struct Neighbor

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		/************************************************************************/
		/* Functions	                                                          */
		/************************************************************************/
	public:
		/*!
		 * The hash table can be constructed following two different approaches:
		 * the first is more fast, but might allocate a offset table bigger than the necessary
		 * the second try to construct the offset table up to m_MAX_TRIALS_IN_COMPACT_CONSTRUCTION times, and then chooses the minimum size for which the construction succeeded. 
		 */
		enum		ConstructionApproach { FastConstructionApproach=0, CompactConstructionApproach=1 };

		/*!
		 * Default constructor
		 */
		PerfectSpatialHashing() { srand( (unsigned) time(NULL) ); }

		/*!
		 * Destructor
		 */
		~PerfectSpatialHashing() { /* ... I don't remember if there is something to delete! :D */ } 

		template < class OBJECT_ITERATOR >
		void Set(const OBJECT_ITERATOR & bObj, const OBJECT_ITERATOR & eObj) 
		{ Set<OBJECT_ITERATOR>(bObj, eObj, FastConstructionApproach, NULL); }

		template < class OBJECT_ITERATOR >
		void Set(const OBJECT_ITERATOR & bObj, const OBJECT_ITERATOR & eObj, vcg::CallBackPos *callback) 
		{ Set<OBJECT_ITERATOR>(bObj, eObj, FastConstructionApproach, callback); }

		template < class OBJECT_ITERATOR >
		void Set(const OBJECT_ITERATOR & bObj, const OBJECT_ITERATOR & eObj, const ConstructionApproach approach) 
		{ Set<OBJECT_ITERATOR>(bObj, eObj, approach, NULL); }

		/*!
		 * Add the elements to the PerfectSpatialHashing data structure. Since this structure can handle only
		 * static dataset, the elements mustn't be changed while using this structure.
		 * @param[in] bObj			The iterator addressing the first element to be included in the hashing.
		 * @param[in] eObj			The iterator addressing the position after the last element to be included in the hashing.
		 * @param[in] approach	Either <code>FastConstructionApproach</code> or <code>CompactConstructionApproach</code>.
		 * @param[in] callback  The callback to call to provide information about the progress of the computation.
		 */
		template < class OBJECT_ITERATOR >
		void Set(const OBJECT_ITERATOR & bObj, const OBJECT_ITERATOR & eObj, const ConstructionApproach approach, vcg::CallBackPos *callback)
		{
			BoundingBoxType bounding_box;
			BoundingBoxType object_bb;
			bounding_box.SetNull();
			for (OBJECT_ITERATOR iObj=bObj; iObj!=eObj; iObj++)
			{
				(*iObj).GetBBox(object_bb);
				bounding_box.Add(object_bb);
			}	

			//...and expand it a bit more
			BoundingBoxType	resulting_bb(bounding_box);
			CoordinateType	offset = bounding_box.Dim()*float(m_BOUNDING_BOX_EXPANSION_FACTOR);
			CoordinateType	center = bounding_box.Center();
			resulting_bb.Offset(offset);
			float longest_side = vcg::math::Max( resulting_bb.DimX(), vcg::math::Max(resulting_bb.DimY(), resulting_bb.DimZ()) )/2.0f;
			resulting_bb.Set(center);
			resulting_bb.Offset(longest_side);

			int number_of_objects = int(std::distance(bObj, eObj));

			// Try to find a reasonable space partition
#ifdef _USE_GRID_UTIL_PARTIONING_
			vcg::Point3i resolution;
			vcg::BestDim<ScalarType>(number_of_objects, resulting_bb.Dim(), resolution); 
			int cells_per_side = resolution.X();
#else ifdef _USE_OCTREE_PARTITIONING_ // Alternative to find the resolution of the uniform grid:
			int primitives_per_voxel;
			int depth = 4;
			do 
			{
				int		number_of_voxel = 1<<(3*depth); // i.e. 8^depth
				float density					= float(number_of_voxel)/float(depth);
				primitives_per_voxel	= int(float(number_of_objects)/density);
				depth++;
			}
			while (primitives_per_voxel>16 && depth<15);
			int cells_per_side = int(powf(2.0f, float(depth)));
#endif

			m_UniformGrid.Allocate(resulting_bb, cells_per_side);
			m_UniformGrid.InsertElements(bObj, eObj);
			m_Bitmap.Allocate(cells_per_side);
			int number_of_cells_occupied = m_UniformGrid.GetNumberOfNotEmptyCells();

			int hash_table_size = (int) ceilf(powf(float(number_of_cells_occupied), 1.0f/float(m_DIMENSION)));
			if (hash_table_size>256)
				hash_table_size = (int) ceilf(powf(1.01f*float(number_of_cells_occupied), 1.0f/float(m_DIMENSION)));
			m_HashTable.Allocate(hash_table_size);

			switch (approach)
			{
			case FastConstructionApproach		:	PerformFastConstruction(number_of_cells_occupied, callback)		; break;
			case CompactConstructionApproach: PerformCompactConstruction(number_of_cells_occupied, callback); break;
			default: assert(false);
			}
			Finalize();
		} // end of method Set


		/*!
		 * Returns all the objects contained inside a specified sphere
		 * @param[in]  distance_functor
		 * @param[in]	 marker
		 * @param[in]	 sphere_center
		 * @param[in]	 sphere_radius
		 * @param[out] objects
		 * @param[out] distances
		 * @param[out] points
		 * \return
		 */
		template <class OBJECT_POINT_DISTANCE_FUNCTOR, class OBJECT_MARKER, class OBJECT_POINTER_CONTAINER, class DISTANCE_CONTAINER, class POINT_CONTAINER>
		unsigned int GetInSphere
			(
			OBJECT_POINT_DISTANCE_FUNCTOR		&	distance_functor, 
			OBJECT_MARKER										&	marker,
			const CoordType									&	sphere_center, 
			const ScalarType								&	sphere_radius,
			OBJECT_POINTER_CONTAINER				&	objects, 
			DISTANCE_CONTAINER							&	distances, 
			POINT_CONTAINER									&	points,
			bool															sort_per_distance   = true,
			bool															allow_zero_distance = true
			)
		{
			BoundingBoxType query_bb(sphere_center, sphere_radius);
			vcg::Box3i integer_bb = m_UniformGrid.Interize(query_bb);

			vcg::Point3i index;
			std::vector< std::vector< ObjectPointer >* > contained_objects;
			std::vector< ObjectPointer >* tmp;
			for (index.X()=integer_bb.min.X(); index.X()<=integer_bb.max.X(); index.X()++)
				for (index.Y()=integer_bb.min.Y(); index.Y()<=integer_bb.max.Y(); index.Y()++)
					for (index.Z()=integer_bb.min.Z(); index.Z()<=integer_bb.max.Z(); index.Z()++)
						if ((tmp=(*this)[index])!=NULL)
							contained_objects.push_back(tmp);

			std::vector< Neighbor > results;
			for (std::vector< std::vector< ObjectPointer >* >::iterator iVec=contained_objects.begin(), eVec=contained_objects.end(); iVec!=eVec; iVec++)
				for (std::vector< ObjectPointer >::iterator iObj=(*iVec)->begin(), eObj=(*iVec)->end(); iObj!=eObj; iObj++ )
				{
					int r = int(results.size());
					results.push_back(Neighbor());
					results[r].object		= *iObj;
					results[r].distance = sphere_radius;
					if (!distance_functor(*results[r].object, sphere_center, results[r].distance, results[r].nearest_point) || (results[r].distance==ScalarType(0.0) && !allow_zero_distance) )
						results.pop_back();
				}

			if (sort_per_distance)
				std::sort( results.begin(), results.end() );

			int number_of_objects = int(results.size());
			distances.resize(number_of_objects);
			points.resize(number_of_objects);
			objects.resize(number_of_objects);
			for (int i=0, size=int(results.size()); i<size; i++)
			{
				distances[i]	= results[i].distance;
				points[i]			= results[i].nearest_point;
				objects[i]		= results[i].object;
			}
			return number_of_objects;
		} //end of GetInSphere

		
		/*!
		* Once the offset table has been built, this function can be used to access the data.
		* Given a 3D point in the space, this function returns the set of ObjectPointers contained
		* in the same UniformGrid cell where this point is contained.
		* @param[in] query The 3D query point.
		* \return		The pointer to the vector of ObjectPointers contained in the same UG of query. NULL if any.
		*/		
		std::vector< ObjectPointer >* operator[](const CoordinateType &query)
		{
			typename UniformGrid::CellCoordinate ug_index = m_UniformGrid.Interize(query);
			if (!m_Bitmap[ug_index])
				return NULL;

			HashTable::EntryCoordinate ht_index = PerfectHashFunction(ug_index);
			std::vector< ObjectPointer >* result = m_HashTable[ht_index];
			return result;
		}


		std::vector< ObjectPointer >* operator[](const typename UniformGrid::CellCoordinate  &query)
		{
			if(!m_Bitmap[query])
				return NULL;

			HashTable::EntryCoordinate ht_index = PerfectHashFunction(query);
			std::vector< ObjectPointer >* result = m_HashTable[ht_index]->domain_data;
			return result;
		}


	protected:
		/*!
		* The injective mapping from the set of occupied cells to a slot in the hash-table
		* @param[in]	query		The index of a domain cell whose mapping has to be calculated.
		* @param[out]	result	The index of a entry in the hash-table where query is mapped into.
		*/
		typename HashTable::EntryCoordinate PerfectHashFunction(const typename UniformGrid::CellCoordinate &query)
		{
			typename HashTable::EntryCoordinate result;
			OffsetTable::OffsetPointer offset = m_OffsetTable[ m_OffsetTable.DomainToOffsetTable(query) ];
			result = m_HashTable.DomainToHashTable( Shift(query, *offset) );
			return result;
		} 

		
		/*!
		 * Performs the addition between a entry coordinate and an offset.
		 * @param[in] entry		The index of a given cell.
		 * @param[in] offset	The offset that must be applied to the entry.
		 * \return						The entry resulting by the addition of entry and offset.
		 */
		typename HashTable::EntryCoordinate Shift(const vcg::Point3i &entry, const typename OffsetTable::Offset &offset)
		{
			HashTable::EntryCoordinate result;
			result.X() = entry.X() + int(offset.X());
			result.Y() = entry.Y() + int(offset.Y());
			result.Z() = entry.Z() + int(offset.Z());
			return result;
		}


		/*!
		* Finalizes the data structures at the end of the offset-table construction.
		* This function eliminates all unnecessary data, and encodes sparsity.
		* TODO At the moment, the sparsity encoding is implemented thought a bitmap, i.e. a boolean grid 
		*			where each slot tells if the relative UniformGrid has a valid entry in the HashTable.
		*/
		void Finalize()
		{
#ifdef _DEBUG
			for (UniformGrid::EntryIterator iUGEntry=m_UniformGrid.Begin(), eUGEntry=m_UniformGrid.End(); iUGEntry!=eUGEntry; iUGEntry++)
				assert(m_Bitmap.ContainsData(iUGEntry.GetPosition())==((*iUGEntry)->size()>0));
#endif
			m_HashTable.Finalize();
			m_UniformGrid.Finalize();
			m_OffsetTable.Finalize();
		}

		
		/*!
		 * Check if the given offset is valid for a set of domain cell.
		 * @param[in] pre_image
		 * @param[in] offset
		 * \return
		 */
		bool IsAValidOffset(const std::vector< typename UniformGrid::CellCoordinate > *pre_image, const typename OffsetTable::Offset &offset)
		{
			int ht_size			= m_HashTable.GetSize();
			int sqr_ht_size = ht_size*ht_size;
			std::vector< int > involved_entries;
			for (int i=0, pre_image_size=int((*pre_image).size()); i<pre_image_size; i++)
			{
				typename UniformGrid::CellCoordinate domain_entry = (*pre_image)[i];
				typename HashTable::EntryCoordinate  hash_entry		= m_HashTable.DomainToHashTable( Shift(domain_entry, offset) );
				if (!m_HashTable.IsFree(hash_entry))
					return false;
				else
					involved_entries.push_back(hash_entry.X()*sqr_ht_size + hash_entry.Y()*ht_size + hash_entry.Z());
			}

			// In order to guarantee that the PerfectHashFunction is injective, the image of each domain-entry must be unique in the hash-table
			std::sort(involved_entries.begin(), involved_entries.end());
			for (int i=0, j=1, size=int(involved_entries.size()); j<size; i++, j++)
				if (involved_entries[i]==involved_entries[j])
					return false;

			return true;
		}

		
		/*!
		 * Given the size of the hash table and an initial seed for the size of the offset table, returns an appropriate size
		 * for the offset table in order to avoid less effective constructions.
		 * @param[in] hash_table_size		The number of entries for each side of the hash-table.
		 * @param[in] offset_table_size The number of entries for each side of the offset-table.
		 * \return											The next appropriate size for the offset table.
		 */
		int GetUnefectiveOffsetTableSize(const int hash_table_size, const int offset_table_size)
		{
			int result = offset_table_size;
			while (GreatestCommonDivisor(hash_table_size, result)!=1 || hash_table_size%result==0)
				result += (hash_table_size%2==0) ? (result%2)+1 : 1;  //Change the offset table size, otherwise the hash construction should be ineffective
			return result;
		}

		
		/*!
		* Start the construction of the offset table trying to complete as quickly as possible.
		* Sometimes the dimension of the offset table will not be minimal.
		* @param[in] number_of_filled_cells The number of entries in the uniform grid containing some elements of the dataset.
		*/
		void PerformFastConstruction(const int number_of_filled_cells, vcg::CallBackPos *callback)
		{
			int offset_table_size = (int) ceilf(powf(m_SIGMA*float(number_of_filled_cells), 1.0f/float(m_DIMENSION)));
			int hash_table_size   = m_HashTable.GetSize();
			int failed_construction_count = 0;
			do 
			{
				offset_table_size += failed_construction_count++;
				offset_table_size  = GetUnefectiveOffsetTableSize(hash_table_size, offset_table_size);
			} 
			while(!OffsetTableConstructionSucceded(offset_table_size, callback));
		}

			
		/*!
		* Start the construction of the offset table trying to minimize its dimension.
		* The offset table size is chosen by performing a binary search over possible values for the offset table size.
		* For this reason, this method will generally require more time than PerformFastConstruction.
		*/
		void PerformCompactConstruction(const int number_of_filled_cells, vcg::CallBackPos *callback)
		{
			int min_successfully_dimension	= std::numeric_limits<int>::max();
			int hash_table_size							= m_HashTable.GetSize();
			int half_hash_table_size				= int(float(hash_table_size)/2.0f);

			// According to the original article, a maximum number of trials are to be made in order to select the optimal (i.e. minimal) offset table size
			for (int t=0; t<m_MAX_TRIALS_IN_COMPACT_CONSTRUCTION; t++)
			{
				int lower_bound = GetUnefectiveOffsetTableSize(hash_table_size, int(double(rand())/double(RAND_MAX)*half_hash_table_size + 1) );
				int upper_bound = GetUnefectiveOffsetTableSize(hash_table_size, int(((double) rand() / (double) RAND_MAX) * hash_table_size + half_hash_table_size));

				// The construction of the offset table using this pair of values surely succeed for max, but not for min
				// The optimal value for the offset table size is then found via binary search over this pair of values
				int candidate_offset_table_size;
				int last_tried_size = -1;
				while (lower_bound<upper_bound)
				{
					candidate_offset_table_size = GetUnefectiveOffsetTableSize(hash_table_size, int(floorf((lower_bound+upper_bound)/2.0f)));

					// If in the previous iteration the offset table has been successfully constructed with the same size,
					// a minimum for the range [lower_bound, upper_bound] has been found
					if (last_tried_size==candidate_offset_table_size)
						break; 

					if ( OffsetTableConstructionSucceded((last_tried_size=candidate_offset_table_size), callback) )
					{
						upper_bound = candidate_offset_table_size;
						min_successfully_dimension = vcg::math::Min(candidate_offset_table_size, min_successfully_dimension);
					}
					else
						lower_bound = candidate_offset_table_size;

					m_HashTable.Clear();
					m_HashTable.BuildFreeEntryList();
					m_OffsetTable.Clear();
					m_Bitmap.Clear();
				}
#ifdef _DEBUD
				printf("\nPerfectSpatialHashing: minimum offset table size found at the %d-th iteration was %d\n", (t+1), min_successfully_dimension);
#endif
			}

			// Finally the OffsetTable must be constructed using the minimum valid size
			while (!OffsetTableConstructionSucceded(min_successfully_dimension, callback)) 
			{
				m_HashTable.Clear();
				m_HashTable.BuildFreeEntryList();
				m_OffsetTable.Clear();
				m_Bitmap.Clear();
			}
		}



		/*!
		* Try to construct the offset table for a given size
		*	\param[in] offset_table_size  The size of the offset table.
		*	\return												<CODE>true</CODE> if and only if the construction of the offset table succeeds.
		*/
		bool OffsetTableConstructionSucceded(const int offset_table_size, vcg::CallBackPos *callback)
		{
			m_OffsetTable.Allocate(offset_table_size); // Create the Offset table
			m_OffsetTable.BuildH1PreImage(m_UniformGrid.Begin(), m_UniformGrid.End()); // Build the f0 pre-image

			std::list< OffsetTable::PreImage > preimage_slots;
			m_OffsetTable.GetPreImageSortedPerCardinality(preimage_slots);

			char msg[128];
			sprintf(msg, "Building offset table of resolution %d", m_OffsetTable.GetSize());
			int	step = int(preimage_slots.size())/100;
			int number_of_slots = int(preimage_slots.size());
			int perc = 0; 
			int iter = 0;
			for (std::list< OffsetTable::PreImage >::iterator iPreImage=preimage_slots.begin(), ePreImage=preimage_slots.end(); iPreImage!=ePreImage; iPreImage++, iter++)
			{
				if (callback!=NULL && iter%step==0 && (perc=iter*100/number_of_slots)<100) (*callback)(perc, msg);

				bool found_valid_offset = false;
				typename OffsetTable::Offset candidate_offset;

				// Heuristic #1: try to set the offset value to one stored in a neighboring entry of the offset table
				std::vector< typename OffsetTable::Offset > consistent_offsets;
				m_OffsetTable.SuggestConsistentOffsets( (*iPreImage).entry_index, consistent_offsets);
				for (std::vector< typename OffsetTable::Offset >::iterator iOffset=consistent_offsets.begin(), eOffset=consistent_offsets.end(); iOffset!=eOffset && !found_valid_offset; iOffset++)
					if (IsAValidOffset(iPreImage->pre_image, *iOffset))
					{
						found_valid_offset = true;
						candidate_offset = *iOffset;
					}


					// Heuristic #2: 
					if (!found_valid_offset)
					{
						std::vector< typename UniformGrid::CellCoordinate > *pre_image = (*iPreImage).pre_image;
						for (std::vector< typename UniformGrid::CellCoordinate >::iterator iPreImage=pre_image->begin(), ePreImage=pre_image->end(); iPreImage!=ePreImage && !found_valid_offset; iPreImage++)
							for (NeighboringEntryIterator iUGNeighbourhood=m_UniformGrid.GetNeighboringEntryIterator(*iPreImage); iUGNeighbourhood<6 && !found_valid_offset; iUGNeighbourhood++ )
								if (!m_OffsetTable.IsFree( m_OffsetTable.DomainToOffsetTable( *iUGNeighbourhood ) ))
								{
									HashTable::EntryCoordinate ht_entry = PerfectHashFunction(*iUGNeighbourhood);
									for (NeighboringEntryIterator iHTNeighbourhood=m_HashTable.GetNeighborintEntryIterator(ht_entry); iHTNeighbourhood<6 && !found_valid_offset; iHTNeighbourhood++)
										if (m_HashTable.IsFree(*iHTNeighbourhood))
										{
											candidate_offset.Import( *iHTNeighbourhood-m_HashTable.DomainToHashTable(*iPreImage) ) ;
											// m_OffsetTable.ValidateEntry( candidate_offset ); Is'n necessary, becouse the offset type is unsigned char.
											if (IsAValidOffset(pre_image, candidate_offset))
												found_valid_offset = true;
										}
								}
					}

					if (!found_valid_offset)
					{
						// At the beginning, the offset can be found via random searches
						for (int i=0; i<m_MAX_NUM_OF_RANDOM_GENERATED_OFFSET && !found_valid_offset; i++)
						{
							HashTable::EntryCoordinate base_entry = (*iPreImage).pre_image->at(0);
							do 
								m_OffsetTable.GetRandomOffset(candidate_offset);
							while (!m_HashTable.IsFree( m_HashTable.DomainToHashTable( Shift(base_entry, candidate_offset) ) ));

							if (IsAValidOffset( (*iPreImage).pre_image, candidate_offset))
								found_valid_offset = true;
						}

						// The chance to find a valid offset table via random searches decreases toward the end of the offset table construction:
						// So a exhaustive search over all the free hash table entries is performed.
						for (std::list< HashTable::EntryCoordinate >::const_iterator iFreeCell=m_HashTable.GetFreeEntryList()->begin(), eFreeCell=m_HashTable.GetFreeEntryList()->end(); iFreeCell!=eFreeCell && !found_valid_offset; iFreeCell++)
						{
							UniformGrid::CellCoordinate domain_entry		= (*iPreImage).pre_image->at(0);
							OffsetTable::EntryCoordinate offset_entry		= m_OffsetTable.DomainToOffsetTable(domain_entry);
							HashTable::EntryCoordinate hashtable_entry	= m_HashTable.DomainToHashTable(domain_entry);
							candidate_offset.Import(*iFreeCell - hashtable_entry);

							if ( IsAValidOffset(iPreImage->pre_image, candidate_offset) )
								found_valid_offset = true;
						}
				}

					// If a valid offset has been found, the construction of the offset table continues, 
					// otherwise the offset table must be enlarged and the construction repeated
					if (found_valid_offset)
					{
						m_OffsetTable.SetOffset( (*iPreImage->pre_image).at(0), candidate_offset);
						for (int c=0, pre_image_cardinality = iPreImage->cardinality; c<pre_image_cardinality; c++)
						{
							HashTable::EntryCoordinate ht_entry = PerfectHashFunction( (*iPreImage->pre_image).at(c));
							std::vector< ObjectPointer > *domain_data = m_UniformGrid[ (*iPreImage->pre_image).at(c) ];
							m_HashTable.SetEntry(ht_entry, domain_data /*, (*iPreImage->pre_image).at(c)*/); // might be useful for encoding sparsity
							m_Bitmap.SetFlag((*iPreImage->pre_image).at(c));
						}
					}
					else
					{
						m_OffsetTable.Clear();
						m_HashTable.Clear();
						m_HashTable.BuildFreeEntryList();
						m_Bitmap.Clear();
						return false; 
					}
			}

			if (callback!=NULL) (*callback)(100, msg);
			return true;
		} // end of OffsetTableConstructionSucceded


		/************************************************************************/
		/* Data Members                                                         */
		/************************************************************************/
	protected: 
		UniformGrid m_UniformGrid;	/*!< The uniform grid used for partitioning the volume.						*/
		OffsetTable m_OffsetTable;	/*!< The offset table corresponding to \f$\Phi\f$ in the article. */
		HashTable		m_HashTable;		/*!< The hash table that will substitute the uniform grid.				*/
		BinaryImage	m_Bitmap;

		const static float	m_BOUNDING_BOX_EXPANSION_FACTOR;
		const static float	m_SIGMA;
		const static int		m_MAX_TRIALS_IN_COMPACT_CONSTRUCTION;
		const static int		m_MAX_NUM_OF_RANDOM_GENERATED_OFFSET;
		const static int    m_DIMENSION;
	}; //end of class PerfectSpatialHashing

	/*! @} */ 
	//end of Doxygen documentation
}//end of namespace vcg

template < class OBJECT_TYPE, class SCALAR_TYPE >
const int vcg::PerfectSpatialHashing< OBJECT_TYPE, SCALAR_TYPE >::m_MAX_NUM_OF_RANDOM_GENERATED_OFFSET	= 32;

template < class OBJECT_TYPE, class SCALAR_TYPE >
const int vcg::PerfectSpatialHashing< OBJECT_TYPE, SCALAR_TYPE >::m_MAX_TRIALS_IN_COMPACT_CONSTRUCTION	= 5;

template < class OBJECT_TYPE, class SCALAR_TYPE >
const int vcg::PerfectSpatialHashing< OBJECT_TYPE, SCALAR_TYPE >::m_DIMENSION	= 3;

template < class OBJECT_TYPE, class SCALAR_TYPE >
const SCALAR_TYPE vcg::PerfectSpatialHashing< OBJECT_TYPE, SCALAR_TYPE >::m_BOUNDING_BOX_EXPANSION_FACTOR	= SCALAR_TYPE(0.035);

template < class OBJECT_TYPE, class SCALAR_TYPE >
const SCALAR_TYPE vcg::PerfectSpatialHashing< OBJECT_TYPE, SCALAR_TYPE >::m_SIGMA	= SCALAR_TYPE(1.0f/(2.0f*SCALAR_TYPE(m_DIMENSION)));

#endif //VCG_SPACE_INDEX_PERFECT_SPATIAL_HASHING_H