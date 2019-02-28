#ifndef multiview_geometry_h__
#define multiview_geometry_h__

namespace insight
{
	namespace mvs
	{


		void ComputeRelativePose(const float R1[9], const float T1[3],
			const float R2[9], const float T2[3], float R[9],
			float T[3]);

		void ComposeProjectionMatrix(const float K[9], const float R[9],
			const float T[3], float P[12]);

		void ComposeInverseProjectionMatrix(const float K[9], const float R[9],
			const float T[3], float inv_P[12]);

		void ComputeProjectionCenter(const float R[9], const float T[3], float C[3]);

		void RotatePose(const float RR[9], float R[9], float T[3]);

	}
}
#endif // multiview_geometry_h__
