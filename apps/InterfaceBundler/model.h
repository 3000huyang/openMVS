#ifndef INSIGHT_MVS_MODEL_HS
#define INSIGHT_MVS_MODEL_HS

#include <string>
#include <vector>
#include <unordered_map>
#include "image.h"
#include "mvs_types.h"
#include <map>


namespace insight
{
	namespace mvs
	{
		class Model
		{
		public:
			struct Point {
				float x = 0;
				float y = 0;
				float z = 0;
				std::vector<int> track;
			};

		public:
			//void readFromFMWorkspace(const std::string &workspacePath);
			void readFromPMVSWorkspace(const std::string &workspacePath);
			void readFromSimpleModel(const std::string &path, SimpleModel &simpleModel);
			void saveTrasformFile(Vec3 &center);

			bool readTrasformFile(Vec3 & center);

			// Get the image identifier for the given image name.
			int getImageId(const std::string& name) const;
			std::string getImageName(const int image_id) const;

			// Compute the robust minimum and maximum depths from the sparse point cloud.
			std::vector<std::pair<float, float>> computeDepthRanges() const;

			// Compute the mean gsd of the model
			float computeGSD() const;
			void setCenterCood(bool val) { _bCenterCoord = val; }

			// Compute the number of shared points between all possible pairs of images.
			std::vector<std::map<int, int>> computeSharedPoints() const;

			std::vector<Image> images;
			std::vector<Point> points;
			
		private:

			std::vector<std::string> image_names_;
			std::unordered_map<std::string, int> image_name_to_id_;
			std::string _workSpacePath;
			bool _bCenterCoord = true;//ÖÐÐÄ»¯
		};
	}
}
#endif
