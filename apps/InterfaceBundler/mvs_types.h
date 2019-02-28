#ifndef types_h__
#define types_h__

#include <eigen/Core>
#include <cstdint>
#include <string>
#ifdef QT
#include <QString>
#endif
#include <vector>
#include <opencv2/core.hpp>
#include <memory>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace insight
{
	namespace mvs
	{


		typedef Eigen::Matrix3d Mat3;
		typedef Eigen::Vector3d Vec3;

#pragma pack(1)
		struct VPointRGB//visit point with color
		{
			union
			{
				struct{
					float x, y, z, w;
				};
				float xyzw[4];
			};
			int idx[2];
			uint8_t rgb[3];
		};
		struct VPoint//visit point with no color
		{
			union
			{
				struct{
					float x, y, z, w;
				};
				float xyzw[4];
			};
			int idx[2];
		};

		struct XYZ
		{
			float x, y, z;
			void makeRGB(float r, float g, float b){
			}
			void sumRGB(float &sumR, float &sumG, float &sumB){
			}
			void setW(float w){}
			void setID(int i){}
			void setVis(const std::set<int> &vis){}

			std::ostream &write(std::ostream &os) const
			{
				os.write((char*)this, sizeof(XYZ));
				return os;
			}
			std::istream &read(std::istream &is)
			{
				is.read((char*)this, sizeof(XYZ));
				return is;
			}
		};

		struct XYZRGB
		{
			float x, y, z;
			uint8_t r, g, b;
			void makeRGB(float _r, float _g, float _b){
				r = static_cast<uint8_t>(_r);
				g = static_cast<uint8_t>(_g);
				b = static_cast<uint8_t>(_b);
			}
			void sumRGB(float &sumR, float &sumG, float &sumB){
				sumR += r;
				sumG += g;
				sumB += b;
			}
			void setW(float w){}
			void setID(int i){}
			void setVis(const std::set<int> &vis){}

			std::ostream &write(std::ostream &os) const
			{
				os.write((char*)this, sizeof(XYZRGB));
				return os;
			}
			std::istream &read(std::istream &is)
			{
				is.read((char*)this, sizeof(XYZRGB));
				return is;
			}
		};

		struct XYZW{
			union
			{
				struct{
					float x, y, z, w;
				};
				float xyzw[4];
			};
			// do nothing,just make template function OK
			void makeRGB(float r, float g, float b){}
			void sumRGB(float &sumR, float &sumG, float &sumB){}
			void setW(float _w){ w = _w; }
			void setID(int i){}
			void setVis(const std::set<int> &vis){}
			void setL(int l){ }
			void setR(int r){ }
			int getL() const { return -1; }
			int getR() const { return -1; }
		};

		struct XYZWRGB{
			union
			{
				struct{
					float x, y, z, w;
				};
				float xyzw[4];
			};
			uint8_t rgb[3];
			void makeRGB(float r, float g, float b){
				rgb[0] = static_cast<uint8_t>(r);
				rgb[1] = static_cast<uint8_t>(g);
				rgb[2] = static_cast<uint8_t>(b);
			}
			void sumRGB(float &sumR, float &sumG, float &sumB){
				sumR += rgb[0];
				sumG += rgb[1];
				sumB += rgb[2];
			}
			void setW(float _w){ w = _w; }
			void setID(int i){}
			void setVis(const std::set<int> &vis){}
			void setL(int l){ }
			void setR(int r){ }
			int getL() const { return -1; }
			int getR() const { return -1; }
		};

		struct XYZWRGB_LR{
			union
			{
				struct{
					float x, y, z, w;
				};
				float xyzw[4];
			};
			uint8_t rgb[3];
			//可见性
			int L = -1;//
			int R = -1;
			void makeRGB(float r, float g, float b){
				rgb[0] = static_cast<uint8_t>(r);
				rgb[1] = static_cast<uint8_t>(g);
				rgb[2] = static_cast<uint8_t>(b);
			}
			void sumRGB(float &sumR, float &sumG, float &sumB){
				sumR += rgb[0];
				sumG += rgb[1];
				sumB += rgb[2];
			}
			void setW(float _w){ w = _w; }
			void setID(int i){}
			void setVis(const std::set<int> &vis){}
			void setL(int l){ L = l; }
			void setR(int r){ R = r; }
			int getL() const { return L; }
			int getR() const { return R; }
		};

		struct XYZWID : public XYZW{
			int id = 0;
			void setID(int i){
				id = i;
			}
		};

		struct XYZWRGBID : public XYZWRGB{
			int id = 0;
			void setID(int i){
				id = i;
			}
		};

		struct XYZVis : public XYZ
		{
			std::vector<int> vis;
			void setVis(const std::set<int> &setvis){
				vis.clear();
				vis.assign(setvis.begin(), setvis.end());
			}
			
			std::ostream &write(std::ostream &os) const
			{
				os.write((char*)&x, sizeof(float));
				os.write((char*)&y, sizeof(float));
				os.write((char*)&z, sizeof(float));
				int n = vis.size();
				if (n < 0){
					printf("________ERR)R_________\n");
				}
				os.write((char*)&n, sizeof(int));
				if (n > 0){
					os.write((char*)vis.data(), sizeof(int) * n);
				}
				return os;
			}
			std::istream &read(std::istream &is)
			{
				is.read((char*)&x, sizeof(float));
				is.read((char*)&y, sizeof(float));
				is.read((char*)&z, sizeof(float));
				int n = 0;
				is.read((char*)&n, sizeof(int));
				vis.resize(n);
				if (n > 0){
					is.read((char*)vis.data(), sizeof(int) * n);
				}
				return is;
			}
		};

		struct XYZRGBVis : public XYZRGB{
			std::vector<int> vis;

			void setVis(const std::set<int> &setvis){
				vis.clear();
				vis.assign(setvis.begin(), setvis.end());
			}

			std::ostream &write(std::ostream &os) const
			{
				os.write((char*)&x, sizeof(float));
				os.write((char*)&y, sizeof(float));
				os.write((char*)&z, sizeof(float));
				os.write((char*)&r,  1);
				os.write((char*)&g, 1);
				os.write((char*)&b, 1);
				int n = vis.size();
				os.write((char*)&n, sizeof(int));
				if (n > 0){
					os.write((char*)vis.data(), sizeof(int) * n);
				}
				return os;
			}
			std::istream &read(std::istream &is)
			{
				is.read((char*)&x, sizeof(float));
				is.read((char*)&y, sizeof(float));
				is.read((char*)&z, sizeof(float));
				is.read((char*)&r,  1);
				is.read((char*)&g, 1);
				is.read((char*)&b, 1);
				int n = 0;
				is.read((char*)&n, sizeof(int));
				vis.resize(n);
				if (n > 0){
					is.read((char*)vis.data(), sizeof(int) * n);
				}
				return is;
			}
		};

		struct Coor{
			int idxL = 0;
			int  d = 0;
			void makeD(float disparity){
				const int SCALE = 1000;
				d = static_cast<int>(disparity * SCALE);
			}
			void getDisparity(float &disparity){
				const float SCALE = 0.001;
				disparity = d * SCALE;
			}
		};

		typedef Eigen::Vector3d Vec3;
		struct Box//包围盒
		{
			Vec3 min;
			Vec3 max;

			Box(const Vec3 &minPoint, const Vec3 &maxPoint)
				:min(minPoint), max(maxPoint)
			{
			}

			Box()
				:min(1, 1, 1), max(-1, -1, -1)
			{
			}

			bool isValid() const
			{
				return (max.x() >= min.x() && max.y() >= min.y() && max.z() >= min.z());
			}

			Vec3 center() const
			{
				return (max + min) / 2.0;
			}

			double xSize() const { return max.x() - min.x(); }
			double ySize() const { return max.y() - min.y(); }
			double zSize() const { return max.z() - min.z(); }


			void setMinMax(const Vec3 &_min, const Vec3 &_max) { max = _max; min = _min; }

			//包括边界
			bool isPointIn(const Vec3 &p) const {

				if (p.x() <= max.x() &&
					p.y() <= max.y() &&
					p.z() <= max.z() &&
					p.x() >= min.x() &&
					p.y() >= min.y() &&
					p.z() >= min.z())
				{
					return true;
				}
				else
					return false;
			}
			//不包括边界
			bool isAbsolutePointIn(const Vec3 &p) const {

				if (p.x() < max.x() &&
					p.y() < max.y() &&
					p.z() < max.z() &&
					p.x() > min.x() &&
					p.y() > min.y() &&
					p.z() > min.z())
				{
					return true;
				}
				else
					return false;
			}

			void expand(float dx, float dy, float dz)
			{
				min.x() -= dx; max.x() += dx;
				min.y() -= dy; max.y() += dy;
				min.z() -= dz; max.z() += dz;
			}

			bool isPointIn2(float *p) const
			{
				if (p[0] <= max[0] &&
					p[1] <= max[1] &&
					p[0] >= min[0] &&
					p[1] >= min[1])
				{
					return true;
				}
				else
				{
					return false;
				}

			}
			void expand(float dx, float dy)
			{
				min[0] -= dx; min[1] -= dy;
				max[0] += dx; max[1] += dy;
			}

			void print(std::ostream &os)
			{
				os<< "\t" << min[0] << "\t" << min[1] << "\t" << min[2]
					<< "\n\t" << max[0] << "\t" << max[1] << "\t" << max[2] << std::endl;
			}

			template<typename T>
			void update(T begin, T end) {
				CHECK(isValid());
				while (begin != end) {
					update(*begin);
					++begin;
				}
			}

			template<typename T>
			void update(T &pt) {
				min.x() = std::min<double>(pt.x, min.x());
				min.y() = std::min<double>(pt.y, min.y());
				min.z() = std::min<double>(pt.z, min.z());
				max.x() = std::max<double>(pt.x, max.x());
				max.y() = std::max<double>(pt.y, max.y());
				max.z() = std::max<double>(pt.z, max.z());
			}
		};

#pragma pack()

		template<typename Point>
		struct FusePointT
		{
			std::vector<Point*> pts;
			void push_back(Point *pt){
				pts.push_back(pt);
			}
			int count() const { return pts.size(); }
			void clear(){
				pts.clear();
			}

			void median(float &x, float &y, float &z, float &roundXYZ){
				std::nth_element(pts.begin(), pts.begin() + count() / 2, pts.end(), [](Point *first, Point*second){
					return first->x < second->x; });
				x = pts[count() / 2]->x;

				std::nth_element(pts.begin(), pts.begin() + count() / 2, pts.end(), [](Point *first, Point*second)	{
					return first->y < second->y; });
				y = pts[count() / 2]->y;
				std::nth_element(pts.begin(), pts.begin() + count() / 2, pts.end(), [](Point *first, Point*second)	{
					return first->z < second->z;
				});
				z = pts[count() / 2]->z;

#if 0
				std::nth_element(pts.begin(), pts.begin() + count() / 2, pts.end(), [](Point *first, Point*second)
				{
					return first->w < second->w;
				});
				roundXYZ = pts[count() / 2]->w;
#else
				double sumW = 0;
				for (int ii = 0; ii < pts.size(); ++ii)
				{
					sumW += pts[ii]->w;
				}
				roundXYZ = sumW / pts.size();
#endif
			}
		};

		template<typename Point, int N>
		struct FixFusePointT
		{
			Point *pts[N];
			uint8_t n = 0;
			void push_back(Point *pt){
				if (n < N){
					pts[n] = pt;
					++n;
				}
				else{
					printf("f.");
				}
			}
			int count() const { return n; }
			void clear(){
				n = 0;
			}

			void median(float &x, float &y, float &z, float &roundXYZ){
				std::nth_element(pts, pts + count() / 2, pts + n, [](Point *first, Point*second){
					return first->x < second->x; });
				x = pts[count() / 2]->x;

				std::nth_element(pts, pts + count() / 2, pts + n, [](Point *first, Point*second)	{
					return first->y < second->y; });
				y = pts[count() / 2]->y;
				std::nth_element(pts, pts + count() / 2, pts + n, [](Point *first, Point*second)	{
					return first->z < second->z;
				});
				z = pts[count() / 2]->z;

				std::nth_element(pts, pts + count() / 2, pts + n, [](Point *first, Point*second)
				{
					return first->w < second->w;
				});
				roundXYZ = pts[count() / 2]->w;
			}
		};

		struct SimpleModel {
			struct Point {
				double x = 0;
				double y = 0;
				double z = 0;
				std::vector<int> track;
			};
			struct Image {
				double K_[9];
				double R_[9];
				double T_[3];
				double C_[3];
				void toC();
				void toT();
				std::string image_path;
			};
			std::vector<Image> images;
			std::vector<Point> points;
		};


		//block coor cache
		struct BlockPoints{
			int iL = 0;
			int iR = 0;
			std::vector<Coor> datas;
		};

		typedef std::shared_ptr<BlockPoints> BlockPointsPtr;

		//typedef std::map<std::pair<int, int>, BlockPointsPtr> SingleBlock;
		typedef std::unordered_map<uint64_t, BlockPointsPtr> SingleBlock;

#if 0
		struct PointsCache
		{
			int nTotal = 0;
			int nMaxPoints = 134217728;//一GB/8字节
			bool noCache = false;// if nocache ,will write immediately
			//key = blockid
			//std::map<std::pair<int, int>, std::vector<BlockPointsPtr>> indexMapPointsCache;
			std::unordered_map<uint64_t, std::vector<BlockPointsPtr>> indexMapPointsCache;

			bool isFULL() const{
				return nTotal >= nMaxPoints;
			}

			void clear(){
				nTotal = 0;
				indexMapPointsCache.clear();
			}
			void append(SingleBlock &singleBlock){
				for (SingleBlock::iterator itr = singleBlock.begin(); itr != singleBlock.end(); ++itr){
					indexMapPointsCache[itr->first].push_back(itr->second);
					nTotal += itr->second->datas.size();
				}
			}

			void savePoints(const std::string &BLOCKS_DIR);
		};
#endif

		struct MarginCloudHeader
		{
			size_t totalPoints = 0;
			size_t marginCount[9];
			MarginCloudHeader(){
				memset(this, 0, sizeof(MarginCloudHeader));
			}
		};
#ifdef QT
		inline std::string tos(const QString &s){
			return s.toLocal8Bit().constData();
		}
		inline QString toqs(const std::string &s){
			return QString::fromLocal8Bit(s.c_str());
		}
#endif

		template<typename Scalar, int R, int C, int alig, typename cvType>
		inline Eigen::Matrix<Scalar, R, C, alig> cvToEigen(cv::Mat mat){
			Eigen::Matrix<Scalar, R, C, alig> emat;
			for (int r = 0; r < R; ++r){
				for (int c = 0; c < C; ++c){
					emat(r, c) = mat.at<cvType>(r, c);
				}
			}
			return emat;
		}

		template<typename Scalar, int R, int C, int alig, typename cvType>
		inline cv::Mat_<cvType> eigenToCV(const Eigen::Matrix<Scalar, R, C, alig> &eMat){
			cv::Mat_<cvType> mat(R, C);
			for (int r = 0; r < R; ++r){
				for (int c = 0; c < C; ++c){
					mat.at<cvType>(r, c) = eMat(r, c);
				}
			}
			return mat;
		}
	}
}
#endif // types_h__
