#include "model.h"

#ifdef QT
#include <QDir>
#include <QString>
#endif

#include <glog/logging.h>

#include "file_system.hpp"
#include <iostream>
#include <fstream>
#include <map>
#include "image.h"

#if 0
#include "blocks.h"
#include "imageio/GdalUtils.h"
#endif

#include "mvs_types.h"
#include <string>


namespace insight
{

	static bool split(const std::string &s, const char delim, std::vector<std::string> &tokens) {
		tokens.clear();
		std::stringstream ss(s);
		std::string token;
		while (std::getline(ss, token, delim)) {
			tokens.push_back(token);
		}
		return tokens.size() >= 2;
	}

	static bool split(const std::string &src, const std::string& delim, std::vector<std::string>& vec_value)
	{
		bool bDelimiterExist = false;
		if (!delim.empty())
		{
			vec_value.clear();
			std::string::size_type start = 0;
			std::string::size_type end = std::string::npos - 1;
			while (end != std::string::npos)
			{
				end = src.find(delim, start);
				vec_value.push_back(src.substr(start, end - start));
				start = end + delim.size();
			}
			if (vec_value.size() >= 2)
				bDelimiterExist = true;
		}
		return bDelimiterExist;
	}


	namespace mvs
	{


#ifdef FM_MODEL
		void Model::readFromFMWorkspace(const std::string &fmWorkspace)
		{
			_workSpacePath = fmWorkspace;
			QDir workspaceDir = QDir(QString::fromLocal8Bit(_workSpacePath.c_str()));

			const std::string imageFileStr = tos(workspaceDir.absoluteFilePath("dm_img.txt"));
			const std::string cameraFileStr = tos(workspaceDir.absoluteFilePath("dm_cam.txt"));
			const std::string trackFileStr = tos(workspaceDir.absoluteFilePath("dm_tracks.txt"));

			std::ifstream cameraFile(cameraFileStr);
			std::ifstream imageFile(imageFileStr);
			std::ifstream trackFile(trackFileStr);

			CHECK(cameraFile.is_open()) << cameraFileStr;
			CHECK(imageFile.is_open()) << imageFileStr;
			CHECK(trackFile.is_open()) << trackFileStr;

			struct FMCamera
			{
				int index;
				int width;
				int height;
				float focalMM;
				float ccdWidth;
				float ccdHeight;
				float ppx;
				float ppy;

				void toK(float K[9])
				{
					memset(K, 0, sizeof(float) * 9);
					K[8] = 1.f;
					K[4] = K[0] = width / ccdWidth * focalMM;

					int w = width;
					int h = height;

					K[2] = (w - 1) * 0.5;
					K[5] = (h - 1) * 0.5;
				}
				void print()
				{
					char buffer[256];
					std::sprintf(buffer, "%d %d %d %f %f %f %f %f\n", index, width, height, focalMM, ccdWidth, ccdHeight, ppx, ppy);
					VLOG(2) << buffer;
				}
			};

			struct FMImage
			{
				std::string name;
				std::string path;
				int image_id;
				double R[9];
				double C[3];
				double opk[3];
				double T[3];
				int camIndex;
				void toR()
				{
					//phi omega kappa 
					double *R2 = R;
					R2[0] = cos(opk[1]) * cos(opk[2]) - sin(opk[1])* sin(opk[0]) * sin(opk[2]);
					R2[3] = cos(opk[0]) * sin(opk[2]);
					R2[6] = sin(opk[1]) * cos(opk[2]) + cos(opk[1]) * sin(opk[0]) * sin(opk[2]);

					R2[1] = -cos(opk[1]) * sin(opk[2]) - sin(opk[1]) * sin(opk[0]) * cos(opk[2]);
					R2[4] = cos(opk[0]) * cos(opk[2]);
					R2[7] = -sin(opk[1]) * sin(opk[2]) + cos(opk[1]) * sin(opk[0]) * cos(opk[2]);

					R2[2] = -sin(opk[1]) * cos(opk[0]);
					R2[5] = -sin(opk[0]);
					R2[8] = cos(opk[1]) * cos(opk[0]);

					//transpose
					Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::DontAlign | Eigen::RowMajor>> rotation1(R);
					rotation1.transposeInPlace();
				}

				void rotateX180()
				{
					for (size_t i = 3; i < 9; ++i) {
						R[i] = -R[i];
					}
				}

				void toT()
				{
					Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::DontAlign | Eigen::RowMajor>> rotation(this->R);
					Eigen::Vector3d t = rotation * Eigen::Vector3d(C[0], C[1], C[2]) * -1;//t = -RC;
					T[0] = t(0);
					T[1] = t(1);
					T[2] = t(2);
				}
			};

			struct FMTrack
			{
				double X[3];
				std::vector<int> imageIds;
				std::vector<float> featX;
				std::vector<float> featY;
			};

			//read camera info
			std::map<int, FMCamera> map_fmcams;
			//read camera information
			{
				// Header line.
				std::string buffer;
				int totalCount = 0;
				std::getline(cameraFile, buffer);
				std::stringstream ss(buffer);
				ss >> totalCount;
				LOG(INFO) << "Get " << totalCount << " cameras.";

				for (int i = 0; i < totalCount; ++i)
				{
					FMCamera cam;
					std::getline(cameraFile, buffer);
					ss.clear();
					ss.str(buffer);
					ss >> cam.index;
					std::getline(cameraFile, buffer);
					std::getline(cameraFile, buffer);
					ss.clear();
					ss.str(buffer);

					ss >> cam.width >> cam.height >> cam.focalMM >> cam.ccdWidth >> cam.ccdHeight >>
						cam.ppx >> cam.ppy;
					cam.print();
					map_fmcams[cam.index] = cam;
				}
			}
			//read image list
			std::vector<FMImage> vec_fmimages;
			{
				std::string buffer;

				int image_groups_count = 0;
				std::getline(imageFile, buffer);
				std::stringstream ss;
				ss.str(buffer);
				ss >> image_groups_count;
				std::cout << "Get " << image_groups_count << " groups.\n";

				int image_id = 0;
				for (int i = 0; i < image_groups_count; ++i)
				{
					std::getline(imageFile, buffer);
					ss.clear();
					ss.str(buffer);
					int image_count = 0;
					ss >> image_count;
					std::cout << "Get " << image_count << " image in group " << i << std::endl;
					std::string path;
					std::getline(imageFile, path);

					for (int j = 0; j < image_count; ++j)
					{
						std::getline(imageFile, buffer);
						ss.clear();
						ss.str(buffer);
						FMImage image;
						ss >> image.name
							>> image.C[0]
							>> image.C[1]
							>> image.C[2]
							>> image.opk[0]
							>> image.opk[1]
							>> image.opk[2]
							>> image.camIndex;
						image.toR();
						image.rotateX180();
						image.toT();
						image.image_id = image_id++;
						image.path = path;
						vec_fmimages.push_back(image);
					}
				}
			}
			std::cout << "Get " << vec_fmimages.size() << "total images\n";
			//read tracks
			std::vector<FMTrack> vec_fmtracks;
			{
				std::string buffer;
				std::stringstream ss;
				while (!trackFile.eof() && trackFile.good())
				{
					std::getline(trackFile, buffer);
					if (buffer.empty()) break;
					ss.clear();
					ss.str(buffer);
					//std::cout << buffer << std::endl;
					FMTrack track;
					int img_count = 0;
					ss >> track.X[0] >> track.X[1] >> track.X[2] >> img_count;
					for (int ii = 0; ii < img_count; ++ii)
					{
						std::getline(trackFile, buffer);
						ss.clear();
						ss.str(buffer);
						//std::cout << buffer << std::endl;
						int image_id;
						float x, y;
						ss >> image_id >> x >> y;
						track.imageIds.push_back(image_id);
						track.featX.push_back(x);
						track.featY.push_back(y);
					}
					vec_fmtracks.push_back(track);
				}
			}

			LOG(INFO) << "Get " << vec_fmtracks.size() << "points\n";

			//gravity-centralizing 
			Box box;
			bool init = false;
			for (FMTrack &track : vec_fmtracks)
			{
				if (init)
				{
					box.min.x() = std::min<double>(box.min.x(), track.X[0]);
					box.min.y() = std::min<double>(box.min.y(), track.X[1]);
					box.min.z() = std::min<double>(box.min.z(), track.X[2]);

					box.max.x() = std::max<double>(box.max.x(), track.X[0]);
					box.max.y() = std::max<double>(box.max.y(), track.X[1]);
					box.max.z() = std::max<double>(box.max.z(), track.X[2]);
				}
				else
				{
					box.min.x() = track.X[0];
					box.min.y() = track.X[1];
					box.min.z() = track.X[2];

					box.max.x() = track.X[0];
					box.max.y() = track.X[1];
					box.max.z() = track.X[2];
					init = true;
				}
			}

			Vec3 center = box.center();
			center.x() = floor(center.x());
			center.y() = floor(center.y());
			center.z() = floor(center.z());
			//transform track
			for (FMTrack &track : vec_fmtracks)
			{
				track.X[0] -= center.x();
				track.X[1] -= center.y();
				track.X[2] -= center.z();
			}

			//transform Camera center
			for (FMImage &image : vec_fmimages)
			{
				image.C[0] -= center.x();
				image.C[1] -= center.y();
				image.C[2] -= center.z();
			}
			saveTrasformFile(center);
			images.clear();
			image_names_.clear();
			image_name_to_id_.clear();
			images.reserve(vec_fmimages.size());
			image_names_.reserve(vec_fmimages.size());
			image_name_to_id_.reserve(vec_fmimages.size());

			for (int image_id = 0; image_id < vec_fmimages.size(); ++image_id) {
				auto &image = vec_fmimages[image_id];
				const std::string image_name = image.name;
				const std::string image_path = stlplus::create_filespec(image.path, image.name);
				const std::string base_name = stlplus::basename_part(image_path);

				float K[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
				map_fmcams[image.camIndex].toK(K);
				image.toT();
				float t[3] = { image.T[0], image.T[1], image.T[2] };
				float R[9];
				for (int i = 0; i < 9; ++i)
				{
					R[i] = image.R[i];
				}
				images.emplace_back(image_path, K, R, t);
				image_names_.push_back(base_name);
				image_name_to_id_.emplace(base_name, image_id);
			}

			points.resize(vec_fmtracks.size());
			for (int point_id = 0; point_id < vec_fmtracks.size(); ++point_id) {
				auto& point = points[point_id];
				point.x = vec_fmtracks[point_id].X[0];
				point.y = vec_fmtracks[point_id].X[1];
				point.z = vec_fmtracks[point_id].X[2];

				int track_len = static_cast<int>(vec_fmtracks[point_id].imageIds.size());

				point.track.resize(track_len);


				for (int i = 0; i < track_len; ++i) {
					point.track[i] = vec_fmtracks[point_id].imageIds[i];
					CHECK_LT(point.track[i], images.size());
					//#define CHECK_RT
#ifdef CHECK_RT
					//check read result
					Eigen::Vector3f X(point.x, point.y, point.z);
					auto &image = vec_fmimages[vec_fmtracks[point_id].imageIds[i]];

					Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::DontAlign | Eigen::RowMajor>> R(image.R);
					std::cout << R << std::endl;
					Eigen::Vector3f t(image.T[0], image.T[1], image.T[2]);

					Eigen::Vector3f Xc = R * (X - Eigen::Vector3f(image.C[0], image.C[1], image.C[2]));
					float K[9];
					map_fmcams[image.camIndex].toK(K);
					Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::DontAlign | Eigen::RowMajor>> K1(K);
					Eigen::Vector3f x = K1 * Xc;
					x(0) /= x(2);
					x(1) /= x(2);
					//x(2) = 1.f;
					x(1) = map_fmcams[image.camIndex].height - x(1) - 1;

					//if (point_id == 0)
					{
						Eigen::Vector3f feat(vec_fmtracks[point_id].featX[i], vec_fmtracks[point_id].featY[i], 1);
						std::cout << "------------" << std::endl;
						std::cout << x << std::endl << std::endl;
						std::cout << feat << std::endl << std::endl;
						std::cout << x - feat << std::endl;
						std::cout << "------------" << std::endl;
					}
#endif



				}
			}
		}
#endif//FM_MODEL


		void Model::readFromPMVSWorkspace(const std::string &workspacePath)
		{
			_workSpacePath = workspacePath;
			const std::string bundle_file_path = _workSpacePath + "/bundle.rd.out";
			const std::string list_file_path = _workSpacePath + "/list.txt";
			const std::string image_wh_path = _workSpacePath + "/bundle.wh.txt";
			std::ifstream file(bundle_file_path);

			CHECK(file.is_open()) << bundle_file_path;

			// Header line.
			std::string header;
			std::getline(file, header);

			int num_images, num_points;
			file >> num_images >> num_points;

			std::ifstream ifs(list_file_path);
			std::ifstream wh_ifs(image_wh_path);
			std::string s;
			std::vector<std::string> images_names;
			std::vector<std::pair<int, int>> image_whs;
			while (std::getline(ifs, s)){
				std::string wh_str;
				std::getline(wh_ifs, wh_str);
				if (s.empty())continue;
				std::vector<std::string> vs;
				split(s, ' ', vs);
				if (!vs.empty()){
					images_names.push_back((vs[0]));
				}
				vs.clear();
				split(wh_str, ' ', vs);
				CHECK(vs.size() == 2);
				int w = atoi(vs[0].c_str());
				int h = atoi(vs[1].c_str());
				image_whs.push_back(std::pair<int,int>(w, h));
			}
			CHECK(images_names.size() == num_images);
			CHECK(image_whs.size() == num_images);

			SimpleModel model;

			for (int image_id = 0; image_id < num_images; ++image_id) {
				const std::string image_name = images_names[image_id];
				const std::string image_path = _workSpacePath + "/" + image_name;

				SimpleModel::Image img;
				img.image_path = image_path;

				double K[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
				file >> K[0];
				K[4] = K[0];

				int w = image_whs[image_id].first;
				int h = image_whs[image_id].second;

				//CHECK(GdalUtils::GetWidthHeightPixel(image_path.c_str(), w, h));
				//Jones
				K[2] = (w - 1) * 0.5;
				K[5] = (h - 1) * 0.5;
				double k1, k2;
				file >> k1 >> k2;
				CHECK_EQ(k1, 0.0f);
				CHECK_EQ(k2, 0.0f);

				double R[9];
				for (size_t i = 0; i < 9; ++i) {
					file >> R[i];
				}

				for (size_t i = 3; i < 9; ++i) {
					R[i] = -R[i];
				}

				double T[3];
				file >> T[0] >> T[1] >> T[2];
				T[1] = -T[1];
				T[2] = -T[2];
				memcpy(img.K_, K, sizeof(double) * 9);
				memcpy(img.R_, R, sizeof(double) * 9);
				memcpy(img.T_, T, sizeof(double) * 3);
				model.images.push_back(img);
			}
			//points.resize(num_points);
			for (int point_id = 0; point_id < num_points; ++point_id) {
				SimpleModel::Point point;
				file >> point.x >> point.y >> point.z;

				int color[3];
				file >> color[0] >> color[1] >> color[2];

				int track_len;
				file >> track_len;
				point.track.resize(track_len);

				for (int i = 0; i < track_len; ++i) {
					int feature_idx;
					float imx, imy;
					file >> point.track[i] >> feature_idx >> imx >> imy;
					CHECK_LT(point.track[i], model.images.size());
				}
				model.points.push_back(point);
			}

			readFromSimpleModel(_workSpacePath, model);
		}

		void Model::readFromSimpleModel(const std::string &path, SimpleModel &simpleModel)
		{
			this->images.clear();
			image_names_.clear();
			image_name_to_id_.clear();
			Vec3 center = Vec3(0, 0, 0);
			if (_bCenterCoord)
			{
				//gravity-centralizing 
				Box box;

				bool init = false;
				for (const auto &point : simpleModel.points)
				{
					if (init)
					{
						box.min.x() = std::min<double>(box.min.x(), point.x);
						box.min.y() = std::min<double>(box.min.y(), point.y);
						box.min.z() = std::min<double>(box.min.z(), point.z);

						box.max.x() = std::max<double>(box.max.x(), point.x);
						box.max.y() = std::max<double>(box.max.y(), point.y);
						box.max.z() = std::max<double>(box.max.z(), point.z);
					}
					else
					{
						box.min.x() = point.x;
						box.min.y() = point.y;
						box.min.z() = point.z;

						box.max.x() = point.x;
						box.max.y() = point.y;
						box.max.z() = point.z;
						init = true;
					}
				}

				center = box.center();
				center.x() = floor(center.x());
				center.y() = floor(center.y());
				center.z() = floor(center.z());
				saveTrasformFile(center);
			}
			else {
				//read center
				Vec3 C;
				if (readTrasformFile(C)) {
					center = C;
					std::cout << "read transform " << C << std::endl;
				}
			}

			int num_images = simpleModel.images.size();
			this->images.reserve(num_images);
			//ÖØÐÄ»¯
			for (int image_id = 0; image_id < num_images; ++image_id)
			{
				simpleModel.images[image_id].toC();
				simpleModel.images[image_id].C_[0] -= center.x();
				simpleModel.images[image_id].C_[1] -= center.y();
				simpleModel.images[image_id].C_[2] -= center.z();
				simpleModel.images[image_id].toT();
				float K[9] = { 0 };
				float R[9] = { 0 };
				float T[3] = { 0 };
				for (int ii = 0; ii < 9; ++ii){
					K[ii] = static_cast<float>(simpleModel.images[image_id].K_[ii]);
					R[ii] = static_cast<float>(simpleModel.images[image_id].R_[ii]);
				}
				for (int ii = 0; ii < 3; ++ii){
					T[ii] = static_cast<float>(simpleModel.images[image_id].T_[ii]);
				}
				this->images.emplace_back(simpleModel.images[image_id].image_path,
					K, R, T);

				std::string image_name = stlplus::basename_part(simpleModel.images[image_id].image_path);
				this->image_names_.push_back(image_name);
				this->image_name_to_id_.emplace(image_name, image_id);
			}

			int num_points = simpleModel.points.size();
			this->points.resize(num_points);
			for (int point_id = 0; point_id < num_points; ++point_id) {
				auto& point = this->points[point_id];
				point.x = simpleModel.points[point_id].x - center.x();
				point.y = simpleModel.points[point_id].y - center.y();
				point.z = simpleModel.points[point_id].z - center.z();
				point.track = simpleModel.points[point_id].track;
			}
		}


		void Model::saveTrasformFile(Vec3 &center)
		{
			//QDir workspaceDir = QDir(QString::fromLocal8Bit(_workSpacePath.c_str()));
			std::string transformFile = stlplus::create_filespec(_workSpacePath, "transformFile.txt");

			std::ofstream transformOfs(transformFile);
			transformOfs << std::fixed;
			transformOfs << "The offset of the point cloud coordinate.\n Format : x y z\n"
				<< center.x() << " " << center.y() << " " << center.z();
			transformOfs.close();
		}

		bool Model::readTrasformFile(Vec3 &center)
		{
			std::string transformFile = stlplus::create_filespec(_workSpacePath, "transform.txt");

			std::ifstream transformIfs(transformFile);
			if (!transformIfs) return false;
			std::string s;
			std::getline(transformIfs, s);
			std::cout << s << std::endl;
			std::getline(transformIfs, s);
			std::cout << s << std::endl;
			std::getline(transformIfs, s);
			std::vector<std::string> vs;
			split(s, ' ', vs);
			double x = atof(vs[0].c_str());
			double y = atof(vs[1].c_str());
			double z = atof(vs[2].c_str());
			center.x() = x;
			center.y() = y;
			center.z() = z;
			return true;
		}

		int Model::getImageId(const std::string& name) const {
			CHECK_GT(image_name_to_id_.count(name), 0) << "Image with name `" << name
				<< "` does not exist";
			return image_name_to_id_.at(name);
		}

		std::string Model::getImageName(const int image_id) const {
			CHECK_GE(image_id, 0);
			CHECK_LT(image_id, image_names_.size());
			return image_names_.at(image_id);
		}

		std::vector<std::pair<float, float>> Model::computeDepthRanges() const {
			std::vector<std::vector<float>> depths(images.size());
			for (const auto& point : points) {
				const Eigen::Vector3f X(point.x, point.y, point.z);
				for (const auto& image_id : point.track) {
					const auto& image = images.at(image_id);
					const float depth =
						Eigen::Map<const Eigen::Vector3f>(&image.GetR()[6]).dot(X) +
						image.GetT()[2];
					if (depth > 0) {
						depths[image_id].push_back(depth);
					}
				}
			}

			std::vector<std::pair<float, float>> depth_ranges(depths.size());
			for (size_t image_id = 0; image_id < depth_ranges.size(); ++image_id) {
				auto& depth_range = depth_ranges[image_id];

				auto& image_depths = depths[image_id];

				if (image_depths.empty()) {
					depth_range.first = -1.0f;
					depth_range.second = -1.0f;
					continue;
				}

				std::sort(image_depths.begin(), image_depths.end());

				const float kMinPercentile = 0.01f;
				const float kMaxPercentile = 0.99f;
				const float kMidPercentile = 0.5f;

				depth_range.first = image_depths[image_depths.size() * kMinPercentile];
				depth_range.second = image_depths[image_depths.size() * kMaxPercentile];

				const float kStretchRatio = 0.25f;
				depth_range.first *= (1.0f - kStretchRatio);
				depth_range.second *= (1.0f + kStretchRatio);
			}

			return depth_ranges;
		}

		float Model::computeGSD() const
		{
			std::vector<std::vector<float>> depths(images.size());
			for (const auto& point : points) {
				const Eigen::Vector3f X(point.x, point.y, point.z);
				for (const auto& image_id : point.track) {
					const auto& image = images.at(image_id);
					const float depth =
						Eigen::Map<const Eigen::Vector3f>(&image.GetR()[6]).dot(X) +
						image.GetT()[2];
					if (depth > 0) {
						depths[image_id].push_back(depth);
					}
				}
			}

			std::vector<float> mid_depth(depths.size());
			for (size_t image_id = 0; image_id < mid_depth.size(); ++image_id) {
				auto& depth = mid_depth[image_id];

				auto& image_depths = depths[image_id];

				if (image_depths.empty()) {
					depth = -1.0f;
					continue;
				}

				std::nth_element(image_depths.begin(), image_depths.begin() + image_depths.size() / 2, image_depths.end());
				depth = image_depths[image_depths.size() * 0.5];
			}

			//mean value of gsd
			//gsd = depth / focal
			double sum_gsd = 0.0;
			for (int im = 0; im < images.size(); ++im)
			{
				float focal = images[im].NoScaleK()[0];
				float gsd = mid_depth[im] / focal;
				sum_gsd += gsd;
			}
			return float(sum_gsd / images.size());
		}

		std::vector<std::map<int, int>> Model::computeSharedPoints() const {
			std::vector<std::map<int, int>> shared_points(images.size());
			for (const auto& point : points) {
				for (size_t i = 0; i < point.track.size(); ++i) {
					const int image_id1 = point.track[i];
					for (size_t j = 0; j < i; ++j) {
						const int image_id2 = point.track[j];
						if (image_id1 != image_id2) {
							shared_points.at(image_id1)[image_id2] += 1;
							shared_points.at(image_id2)[image_id1] += 1;
						}
					}
				}
			}
			return shared_points;
		}
	}
}
