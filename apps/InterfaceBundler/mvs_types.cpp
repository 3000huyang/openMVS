
#include "mvs_types.h"

#include <eigen/core>
#if 0
#include "bin_io.h"
#endif
namespace insight
{
	namespace mvs
	{


		void SimpleModel::Image::toC()
		{
			//t = -RC
			//C = -R`t
			Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::DontAlign | Eigen::RowMajor>> R(R_);
			Eigen::Vector3d C = -R.transpose() * Eigen::Vector3d(T_[0], T_[1], T_[2]);
			C_[0] = C.x();
			C_[1] = C.y();
			C_[2] = C.z();
		}

		void SimpleModel::Image::toT()
		{
			Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::DontAlign | Eigen::RowMajor>> R(R_);
			Eigen::Vector3d t = -R* Eigen::Vector3d(C_[0], C_[1], C_[2]);
			T_[0] = t.x();
			T_[1] = t.y();
			T_[2] = t.z();
		}

#if 0
		void PointsCache::savePoints(const std::string &BLOCKS_DIR)
		{
			int nCPU = omp_get_max_threads();
			nCPU = std::min<int>(nCPU, indexMapPointsCache.size());
#pragma omp parallel num_threads(nCPU)
			for (auto itr = indexMapPointsCache.begin();
				itr != indexMapPointsCache.end(); ++itr)
			{
#pragma omp single nowait
				{
					uint64_t keyInt = itr->first;
					uint8_t *pData = (uint8_t*)&keyInt;
					int keyX = 0;
					int keyY = 0;
					memcpy(&keyX, pData, sizeof(int));
					memcpy(&keyY, pData + 4, sizeof(int));
					std::string key = std::to_string(keyX) + "_" + std::to_string(keyY);
					std::string binptsFile = BLOCKS_DIR + "/" + key + ".binpts";
					std::string idxFile = BLOCKS_DIR + "/" + key + ".binidx";
					std::vector<BlockPointsPtr> &blockPts = itr->second;
					for (int i = 0; i < blockPts.size(); ++i){
						WriteBlock(binptsFile, idxFile, blockPts[i]->iL, blockPts[i]->iR, blockPts[i]->datas);
					}
				}
			}
		}
#endif
	}
}