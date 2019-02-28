/************************************************************************/
/*
@file:Image.cpp
@author: Jones
@date: 2017-10-1
@modify: 2018-08-27
@version:1.1
*/
/************************************************************************/

#include "image.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include "multiview_geometry.h"
namespace insight {
	namespace mvs
	{


		Image::Image(const std::string& path, const float* K, const float* R,
			const float* T)
			: path_(path) {
			memcpy(K_, K, 9 * sizeof(float));
			memcpy(R_, R, 9 * sizeof(float));
			memcpy(T_, T, 3 * sizeof(float));
			ComposeProjectionMatrix(K_, R_, T_, P_);
			ComposeInverseProjectionMatrix(K_, R_, T_, inv_P_);
			memcpy(NoScaleK_, K, 9 * sizeof(float));
		}


		std::string Image::GetPath() const
		{
			return path_;
		}

		void Image::Rescale(const float factor) {
			memcpy(K_, NoScaleK_, sizeof(float) * 9);//恢复K的原始值
			K_[0] *= factor;
			K_[2] *= factor;
			K_[4] *= factor;
			K_[5] *= factor;
			ComposeProjectionMatrix(K_, R_, T_, P_);
			ComposeInverseProjectionMatrix(K_, R_, T_, inv_P_);
		}
	}
}  // namespace insight
