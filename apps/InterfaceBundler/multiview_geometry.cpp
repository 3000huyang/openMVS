#include "multiview_geometry.h"

#include "eigen/Geometry"
namespace insight
{
	namespace mvs
	{


		void ComputeRelativePose(const float R1[9], const float T1[3],
			const float R2[9], const float T2[3], float R[9],
			float T[3]) {
			const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R1_m(R1);
			const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R2_m(R2);
			const Eigen::Map<const Eigen::Matrix<float, 3, 1>> T1_m(T1);
			const Eigen::Map<const Eigen::Matrix<float, 3, 1>> T2_m(T2);
			Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R_m(R);
			Eigen::Map<Eigen::Vector3f> T_m(T);

			R_m = R2_m * R1_m.transpose();
			T_m = T2_m - R_m * T1_m;
		}

		void ComposeProjectionMatrix(const float K[9], const float R[9],
			const float T[3], float P[12]) {
			Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>> P_m(P);
			P_m.leftCols<3>() =
				Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(R);
			P_m.rightCols<1>() = Eigen::Map<const Eigen::Vector3f>(T);
			P_m = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(K) * P_m;
		}

		void ComposeInverseProjectionMatrix(const float K[9], const float R[9],
			const float T[3], float inv_P[12]) {
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> P;
			ComposeProjectionMatrix(K, R, T, P.data());
			P.row(3) = Eigen::Vector4f(0, 0, 0, 1);
			const Eigen::Matrix4f inv_P_temp = P.inverse();
			Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>> inv_P_m(inv_P);
			inv_P_m = inv_P_temp.topRows<3>();
		}

		void ComputeProjectionCenter(const float R[9], const float T[3], float C[3]) {
			const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R_m(R);
			const Eigen::Map<const Eigen::Matrix<float, 3, 1>> T_m(T);
			Eigen::Map<Eigen::Vector3f> C_m(C);
			C_m = -R_m.transpose() * T_m;
		}

		void RotatePose(const float RR[9], float R[9], float T[3]) {
			Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R_m(R);
			Eigen::Map<Eigen::Matrix<float, 3, 1>> T_m(T);
			const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> RR_m(RR);
			R_m = RR_m * R_m;
			T_m = RR_m * T_m;
		}
	}
}

