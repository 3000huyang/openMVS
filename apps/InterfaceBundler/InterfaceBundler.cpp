/*
 * InterfaceVisualSFM.cpp
 *
 * Copyright (c) 2014-2015 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#define LOG_OUT() GET_LOG()
#define LOG_ERR() GET_LOG()
#include <boost/program_options.hpp>

#include "model.h"

 // D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceBundler")
#define MVS_EXT _T(".mvs")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
	String strBundlerWorkFolder;
	String strOutputFileName;
	//String strOutputImageFolder;
	unsigned nArchiveType;
	int nProcessPriority;
	unsigned nMaxThreads;
	String strConfigFileName;
	boost::program_options::variables_map vm;
} // namespace OPT

// initialize and parse the command line parameters
bool Initialize(size_t argc, LPCTSTR* argv)
{
	// initialize log and console
	OPEN_LOG();
	OPEN_LOGCONSOLE();

	// group of options allowed only on command line
	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("archive-type", boost::program_options::value<unsigned>(&OPT::nArchiveType)->default_value(2), "project archive type: 0-text, 1-binary, 2-compressed binary")
		("process-priority", boost::program_options::value<int>(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
		("max-threads", boost::program_options::value<unsigned>(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
#if TD_VERBOSE != TD_VERBOSE_OFF
		("verbosity,v", boost::program_options::value<int>(&g_nVerbosityLevel)->default_value(
#if TD_VERBOSE == TD_VERBOSE_DEBUG
			3
#else
			2
#endif
		), "verbosity level")
#endif
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-folder,i", boost::program_options::value<std::string>(&OPT::strBundlerWorkFolder), "input bundler folder containing bundle.rd.out bundle.wh.txt list.txt files")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh");
		//("output-image-folder", boost::program_options::value<std::string>(&OPT::strOutputImageFolder)->default_value("undistorted_images"), "output folder to store undistorted images")
		//;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config);

	boost::program_options::positional_options_description p;
	p.add("input-file", -1);

	try {
		// parse command line options
		boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(cmdline_options).positional(p).run(), OPT::vm);
		boost::program_options::notify(OPT::vm);
		INIT_WORKING_FOLDER;
		// parse configuration file
		std::ifstream ifs(MAKE_PATH_SAFE(OPT::strConfigFileName));
		if (ifs) {
			boost::program_options::store(parse_config_file(ifs, config_file_options), OPT::vm);
			boost::program_options::notify(OPT::vm);
		}
	}
	catch (const std::exception& e) {
		LOG(e.what());
		return false;
	}

	// initialize the log file
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-") + Util::getUniqueName(0) + _T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line:%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strBundlerWorkFolder);
	Util::ensureUnifySlash(OPT::strBundlerWorkFolder);
	if (OPT::vm.count("help") || OPT::strBundlerWorkFolder.IsEmpty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (OPT::strBundlerWorkFolder.IsEmpty())
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strBundlerWorkFolder) + MVS_EXT;

	// initialize global options
	Process::setCurrentProcessPriority((Process::Priority)OPT::nProcessPriority);
#ifdef _USE_OPENMP
	if (OPT::nMaxThreads != 0)
		omp_set_num_threads(OPT::nMaxThreads);
#endif

#ifdef _USE_BREAKPAD
	// start memory dumper
	MiniDumper::Create(APPNAME, WORKING_FOLDER);
#endif
	return true;
}

// finalize application instance
void Finalize()
{
#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
#endif

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

namespace MVS {
	// given an undistorted pixel coordinate and one radial-undistortion parameter,
	// compute the corresponding distorted coordinate
	template<typename TYPE>
	inline TPoint2<TYPE> DistortPointR1(const TPoint2<TYPE>& pt, const REAL& k1) {
		if (k1 == 0)
			return pt;
		const REAL y(pt.y == 0 ? REAL(1.e-12) : REAL(pt.y));
		const REAL t2(y*y);
		const REAL t3(t2*t2*t2);
		const REAL t4(pt.x*pt.x);
		const REAL t7(k1*(t2 + t4));
		const REAL t9(1.0 / t7);
		const REAL t10(t2*t9*y*0.5);
		const REAL t11(t3*t9*t9*(0.25 + t9 / 27.0));
#ifndef _RELEASE
		TPoint2<TYPE> upt;
#endif
		if (k1 > 0) {
			const REAL t17(CBRT(t10 + SQRT(t11)));
			const REAL t18(t17 - t2 * t9 / (t17 * 3));
#ifndef _RELEASE
			upt =
#else
			return
#endif
				TPoint2<TYPE>(TYPE(t18*pt.x / y), TYPE(t18));
		}
		else {
			ASSERT(t11 <= 0);
			const std::complex<REAL> t16(t10, SQRT(-t11));
			const std::complex<REAL> t17(pow(t16, 1.0 / 3.0));
			const std::complex<REAL> t14((t2*t9) / (t17*3.0));
			const std::complex<REAL> t18((t17 + t14)*std::complex<REAL>(0.0, SQRT_3));
			const std::complex<REAL> t19(0.5*(t14 - t17 - t18));
#ifndef _RELEASE
			upt =
#else
			return
#endif
				TPoint2<TYPE>(TYPE(t19.real()*pt.x / y), TYPE(t19.real()));
		}
#ifndef _RELEASE
		ASSERT(ABS(TYPE((1.0 + k1 * (upt.x*upt.x + upt.y*upt.y))*upt.x) - pt.x) < TYPE(0.001));
		ASSERT(ABS(TYPE((1.0 + k1 * (upt.x*upt.x + upt.y*upt.y))*upt.y) - pt.y) < TYPE(0.001));
		return upt;
#endif
	}

	void UndistortImage(const Camera& camera, const REAL& k1, const Image8U3 imgIn, Image8U3& imgOut)
	{
		// allocate the undistorted image
		if (imgOut.data == imgIn.data ||
			imgOut.cols != imgIn.cols ||
			imgOut.rows != imgIn.rows ||
			imgOut.type() != imgIn.type())
			imgOut = Image8U3(imgIn.rows, imgIn.cols);

		// compute each pixel
		const int w = imgIn.cols;
		const int h = imgIn.rows;
		const Matrix3x3f K(camera.K);
		const Matrix3x3f invK(camera.GetInvK());
		ASSERT(ISEQUAL(K(0, 2), 0.5f*(w - 1)) && ISEQUAL(K(1, 2), 0.5f*(h - 1)));
		typedef Sampler::Cubic<float> Sampler;
		const Sampler sampler;
		Point2f pt;
		Pixel32F clr;
		for (int v = 0; v < h; ++v) {
			for (int u = 0; u < w; ++u) {
				// compute corresponding coordinates in the distorted image
				pt.x = (float)u; pt.y = (float)v;
				Camera::NormalizeProjection(invK.val, pt.ptr(), pt.ptr());
				pt = DistortPointR1(pt, k1);
				Camera::NormalizeProjection(K.val, pt.ptr(), pt.ptr());

				// if coordinates in range
				Pixel8U& col = imgOut(v, u);
				if (imgIn.isInside(pt)) {
					// get pixel color
					clr = imgIn.sample<Sampler, Pixel32F>(sampler, pt);
					col.r = CLAMP(ROUND2INT(clr.r), 0, 255);
					col.g = CLAMP(ROUND2INT(clr.g), 0, 255);
					col.b = CLAMP(ROUND2INT(clr.b), 0, 255);
				}
				else {
					// set to black
					col = Pixel8U::BLACK;
				}
			}
		}
	}
} // namespace MVS


using namespace insight::mvs;
int main(int argc, LPCTSTR* argv)
{
#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	TD_TIMER_START();

	Model model;
	model.setCenterCood(false);//only read centered
	model.readFromPMVSWorkspace(OPT::strBundlerWorkFolder);
	// read VisualSFM input data
// 	std::vector<PBA::Camera> cameras;
// 	std::vector<PBA::Point3D> vertices;
// 	std::vector<PBA::Point2D> measurements; // the array of 2D projections (only inliers)
// 	std::vector<int> correspondingPoint; // 3D point index corresponding to each 2D projection
// 	std::vector<int> correspondingView; // and camera index
// 	std::vector<std::string> names;
// 	std::vector<int> ptc;
// 	if (!PBA::LoadModelFile(MAKE_PATH_SAFE(OPT::strInputFileName), cameras, vertices, measurements, correspondingPoint, correspondingView, names, ptc))
// 		return EXIT_FAILURE;

	int nImage = model.images.size();
	// convert data from VisualSFM to OpenMVS
	MVS::Scene scene(OPT::nMaxThreads);
	scene.platforms.Reserve((uint32_t)nImage);
	scene.images.Reserve((uint32_t)nImage);
	scene.nCalibratedImages = nImage;
	for (size_t idx = 0; idx < nImage; ++idx) {
		MVS::Image& image = scene.images.AddEmpty();
		image.name = model.images[idx].GetPath();
		//Util::ensureUnifySlash(image.name);
		//image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, image.name);
		if (!image.ReloadImage(0, false)) {
			LOG("error: can not read image %s", image.name.c_str());
			return EXIT_FAILURE;
		}
		// set camera
		image.platformID = idx;//scene.platforms.GetSize();
		MVS::Platform& platform = scene.platforms.AddEmpty();
		MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
		image.cameraID = 0;
		//const PBA::Camera& cameraNVM = cameras[idx];
		float f = model.images[idx].GetK()[0];
		camera.K = MVS::Platform::Camera::ComposeK<REAL, REAL>(f, f, image.width, image.height);
		camera.R = RMatrix::IDENTITY;
		camera.C = CMatrix::ZERO;
		// normalize camera intrinsics
		const REAL fScale(REAL(1) / MVS::Camera::GetNormalizationScale(image.width, image.height));
		camera.K(0, 0) *= fScale;
		camera.K(1, 1) *= fScale;
		camera.K(0, 2) *= fScale;
		camera.K(1, 2) *= fScale;
		// set pose
		image.poseID = platform.poses.GetSize();
		MVS::Platform::Pose& pose = platform.poses.AddEmpty();
		const float *R = model.images[idx].GetR();
		Eigen::Vector3f C = model.images[idx].Center();
		for (int ii = 0; ii < 9; ++ii)
			pose.R.val[ii] = R[ii];
		pose.C.x = C.x();
		pose.C.y = C.y();
		pose.C.z = C.z();
		image.UpdateCamera(scene.platforms);
		++scene.nCalibratedImages;
	}

	scene.pointcloud.points.Reserve(model.points.size());
	scene.pointcloud.pointViews.Resize(model.points.size());
	for (size_t idx = 0; idx < model.points.size(); ++idx) {
		scene.pointcloud.points.AddConstruct(model.points[idx].x, model.points[idx].y, model.points[idx].z);
		MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[idx];
		for (int iImage : model.points[idx].track)
		{
			views.InsertSort(iImage);
		}
	}
	//No color
	// write OpenMVS input data
	scene.SaveInterface(MAKE_PATH_SAFE(OPT::strOutputFileName));

	VERBOSE("Input data imported: %u cameras, %u poses, %u images, %u vertices (%s)", model.images.size(), model.images.size(), 
		model.images.size(), model.points.size(), TD_TIMER_GET_FMT().c_str());

	Finalize();
	return EXIT_SUCCESS;
	}
/*----------------------------------------------------------------*/
