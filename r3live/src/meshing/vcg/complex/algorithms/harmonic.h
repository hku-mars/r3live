#ifndef __VCGLIB_HARMONIC_FIELD
#define __VCGLIB_HARMONIC_FIELD

#include <vcg/complex/complex.h>
#include <utility>
#include <vector>
#include <map>

#include <eigenlib/Eigen/Sparse>

namespace vcg {
namespace tri {

template <class MeshType, typename Scalar = double>
class Harmonic
{
public:
	typedef typename MeshType::VertexType VertexType;
	typedef typename MeshType::FaceType   FaceType;
	typedef typename MeshType::CoordType  CoordType;
	typedef typename MeshType::ScalarType ScalarType;

	typedef double CoeffScalar;

	typedef typename std::pair<VertexType *, Scalar> Constraint;
	typedef typename std::vector<Constraint>         ConstraintVec;
	typedef typename ConstraintVec::const_iterator   ConstraintIt;

	/**
	 * @brief ComputeScalarField
	 * Generates a scalar harmonic field over the mesh.
	 * For more details see:\n Kai Xua, Hao Zhang, Daniel Cohen-Or, Yueshan Xionga,'Dynamic Harmonic Fields for Surface Processing'.\nin Computers & Graphics, 2009
	 * @param m the mesh
	 * @param constraints the Dirichlet boundary conditions in the form of vector of pairs <vertex pointer, value>.
	 * @param field the accessor to use to write the computed per-vertex values (must have the [ ] operator).
	 * @return true if the algorithm succeeds, false otherwise.
	 * @note the algorithm has unexpected behavior if the mesh contains unreferenced vertices.
	 */
	template <typename ACCESSOR>
	static bool ComputeScalarField(MeshType & m, const ConstraintVec & constraints, ACCESSOR field)
	{
		typedef Eigen::SparseMatrix<CoeffScalar> SpMat;  // sparse matrix type
		typedef Eigen::Triplet<CoeffScalar>      Triple; // triplet type to fill the matrix

		RequirePerVertexFlags(m);
		RequireCompactness(m);
		RequireFFAdjacency(m);

		if (constraints.empty())
			return false;

		int n  = m.VN();

		// Generate coefficients
		std::vector<Triple>          coeffs;   // coefficients of the system
		std::map<size_t,CoeffScalar> sums;     // row sum of the coefficient matrix

		vcg::tri::UpdateFlags<MeshType>::FaceClearV(m);
		for (size_t i = 0; i < m.face.size(); ++i)
		{
			FaceType & f = m.face[i];

			assert(!f.IsD());
			assert(!f.IsV());

			f.SetV();

			// Generate coefficients for each edge
			for (int edge = 0; edge < 3; ++edge)
			{
				CoeffScalar weight;
				WeightInfo res = CotangentWeightIfNotVisited(f, edge, weight);

				if (res == EdgeAlreadyVisited) continue;
				assert(res == Success);

				// Add the weight to the coefficients vector for both the vertices of the considered edge
				size_t v0_idx = vcg::tri::Index(m, f.V0(edge));
				size_t v1_idx = vcg::tri::Index(m, f.V1(edge));

				coeffs.push_back(Triple(v0_idx, v1_idx, -weight));
				coeffs.push_back(Triple(v1_idx, v0_idx, -weight));

				// Add the weight to the row sum
				sums[v0_idx] += weight;
				sums[v1_idx] += weight;
			}
		}

		// Setup the system matrix
		SpMat laplaceMat;        // the system to be solved
		laplaceMat.resize(n, n); // eigen initializes it to zero
		laplaceMat.reserve(coeffs.size());
		for (std::map<size_t,CoeffScalar>::const_iterator it = sums.begin(); it != sums.end(); ++it)
		{
			coeffs.push_back(Triple(it->first, it->first, it->second));
		}
		laplaceMat.setFromTriplets(coeffs.begin(), coeffs.end());


		// Setting the constraints
		const CoeffScalar alpha = pow(10, 8); // penalty factor alpha
		Eigen::Matrix<CoeffScalar, Eigen::Dynamic, 1> b, x; // Unknown and known terms vectors
		b.setZero(n);

		for (ConstraintIt it=constraints.begin(); it!=constraints.end(); it++)
		{
			size_t v_idx = vcg::tri::Index(m, it->first);
			b(v_idx) = alpha * it->second;
			laplaceMat.coeffRef(v_idx, v_idx) += alpha;
		}

		// Perform matrix decomposition
		Eigen::SimplicialLDLT<SpMat> solver;
		solver.compute(laplaceMat);
		// TODO eventually use another solver (e.g. CHOLMOD for dynamic setups)
		if(solver.info() != Eigen::Success)
		{
			// decomposition failed
			switch (solver.info())
			{
			// possible errors
			case Eigen::NumericalIssue :
			case Eigen::NoConvergence :
			case Eigen::InvalidInput :
			default : return false;
			}
		}

		// Solve the system: laplacianMat x = b
		x = solver.solve(b);
		if(solver.info() != Eigen::Success)
		{
			return false;
		}

		// Set field value using the provided handle
		for (size_t i = 0; int(i) < n; ++i)
		{
			field[i] = Scalar(x[i]);
		}

		return true;
	}

	enum WeightInfo
	{
		Success            = 0,
		EdgeAlreadyVisited
	};


	/**
	 * @brief CotangentWeightIfNotVisited computes the cotangent weighting for an edge
	 * (if it has not be done yet).
	 * This must be ensured setting the visited flag on the face once all edge weights have been computed.
	 * @param f the face
	 * @param edge the edge of the provided face for which we compute the weight
	 * @param weight the computed weight (output)
	 * @return Success if everything is fine, EdgeAlreadyVisited if the weight
	 * for the considered edge has been already computed.
	 * @note the mesh must have the face-face topology updated
	 */
	template <typename ScalarT>
	static WeightInfo CotangentWeightIfNotVisited(const FaceType &f, int edge, ScalarT & weight)
	{
		const FaceType * fp = f.cFFp(edge);
		if (fp != NULL && fp != &f)
		{
			// not a border edge
			if (fp->IsV()) return EdgeAlreadyVisited;
		}

		weight = CotangentWeight<ScalarT>(f, edge);

		return Success;
	}

	/**
	 * @brief ComputeWeight computes the cotangent weighting for an edge
	 * @param f the face
	 * @param edge the edge of the provided face for which we compute the weight
	 * @return the computed weight
	 * @note the mesh must have the face-face topology updated
	 */
	template <typename ScalarT>
	static ScalarT CotangentWeight(const FaceType &f, int edge)
	{
		assert(edge >= 0 && edge < 3);

		// get the adjacent face
		const FaceType * fp = f.cFFp(edge);

		//       v0
		//      /|\
		//     / | \
		//    /  |  \
		//   /   |   \
		// va\   |   /vb
		//    \  |  /
		//     \ | /
		//      \|/
		//       v1

		ScalarT cotA = 0;
		ScalarT cotB = 0;

		// Get the edge (a pair of vertices)
		VertexType * v0 = f.cV0(edge);
		VertexType * v1 = f.cV1(edge);

		if (fp != NULL &&
		    fp != &f)
		{
			// not a border edge
			VertexType * vb = fp->cV2(f.cFFi(edge));
			ScalarT angleB = ComputeAngle<ScalarT>(v0, vb, v1);
			cotB = vcg::math::Cos(angleB) / vcg::math::Sin(angleB);
		}

		VertexType * va = f.cV2(edge);
		ScalarT angleA = ComputeAngle<ScalarT>(v0, va, v1);
		cotA = vcg::math::Cos(angleA) / vcg::math::Sin(angleA);

		return (cotA + cotB) / 2;
	}

	template <typename ScalarT>
	static ScalarT ComputeAngle(const VertexType * a, const VertexType * b, const VertexType * c)
	{
		//       a
		//      /
		//     /
		//    /
		//   /  ___ compute the angle in b
		// b \
		//    \
		//     \
		//      \
		//       c
		assert(a != NULL && b != NULL && c != NULL);
		Point3<ScalarT> A,B,C;
		A.Import(a->P());
		B.Import(b->P());
		C.Import(c->P());
		ScalarT angle = vcg::Angle(A - B, C - B);
		return angle;
	}
};

}
}
#endif // __VCGLIB_HARMONIC_FIELD
