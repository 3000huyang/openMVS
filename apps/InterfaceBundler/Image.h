#pragma once

#include <string>
#include <Eigen/core>
namespace insight
{
	namespace mvs
	{
		class Image {
		public:
			Image(const std::string& path, const float* K, const float* R,
				const float* T);

			inline const float* GetR() const;
			inline const float* GetT() const;
			inline const float* GetK() const;
			inline const float* GetP() const;
			inline const float* GetInvP() const;
			inline const float* GetViewingDirection() const;

			std::string GetPath() const;

			void Rescale(const float factor);

			//没有缩放的K
			const float *NoScaleK() const { return NoScaleK_; }

			float Depth(const Eigen::Vector3f &X) const {
				return 	Eigen::Map<const Eigen::Vector3f>(&GetR()[6]).dot(X) +
					GetT()[2];
			}

			Eigen::Vector3f ProjectPointP3(const Eigen::Vector3f &X) const{
				typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> RowMat33;
				return Eigen::Map<const RowMat33>(GetR()) * X + Eigen::Map<const Eigen::Vector3f>(GetT());
			}

			Eigen::Vector3f Center() const {
				typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> RowMat33;
				return -Eigen::Map<const RowMat33>(GetR()).transpose() * Eigen::Map<const Eigen::Vector3f>(GetT());
			}
		private:
			std::string path_;
			float K_[9];
			float R_[9];
			float T_[3];
			float P_[12];
			float inv_P_[12];

			float NoScaleK_[9];//与图像相同分辨率的K的值
		};

		const float* Image::GetR() const { return R_; }

		const float* Image::GetT() const { return T_; }

		const float* Image::GetK() const { return K_; }

		const float* Image::GetP() const { return P_; }

		const float* Image::GetInvP() const { return inv_P_; }

		const float* Image::GetViewingDirection() const { return &R_[6]; }
	}

}
