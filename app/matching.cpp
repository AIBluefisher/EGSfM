#include "db.h"
#include "base.h"

using namespace std;
using namespace REC3D;

//#define bCloud

namespace OPT 
{
	string sIp = "";
	string sUser = "";
	string sPass = "";
	string sDbName = "";
	unsigned nPort = 0;
	string sLogName = "";
	string sDataPath = "";
	string sImageFolder = "";
	string sSparseBoundName = "";
	string sDenseBoundName = "";
	string sModelName = "";
	string sVoctreeName = "";
	string sMeshName = "";
	int ID_first = -1;
	int ID_last = -1;
	bool bOverWrite = 0;
	int bTexture = 0;
	float fFocalLength = 0;
	int nfeatures = 20000;
	int nMaxSize = 3200;
	int nFirstOctave = 0;
	int nNumLevels = 3;
	int nMaxNumOrientations = 2;
	int nOctaveLayers = 3;
	float fContrastThreshold = 0.004;
	float fEdgeThreshold = 10;
	float fSigma = 1.6;
	float fNNDR = 0.7;
	unsigned nMinMatch = 15;
	int nQuery = 30;
	int nOverlap = 10;
	int nSpatial = 30;
	float fScale = 0.5;
	unsigned nMaxViews = 15;
	unsigned nMinViews = 2;
	float fNCCThreshold = 0.7;
	unsigned nConsisFilter = 2;
	float fDepthDiffThreshold = 0.01;
	unsigned nConsisFuse = 1;
	unsigned nFuseStep = 1;
	float fTextureScale = 0.5;
	float fDecimateMesh = 1.f;
	float fDistInsert = 2.5f;
	int k_cover = 2048;
	int n_OMST = 100;
	int min_P3P_matches = 24;
	int min_vis_views = 2;
	int ra_conf = 0;
	float min_tri_angle = 1.5;
	float rigister_ratio = 0.3;
	float xMin; // 过滤点云范围
	float xMax;
	float yMin;
	float yMax;
	float zMin;
	float zMax;
	bool bBoundBox = 0;
	unsigned long long nMaxPoints = 5e7;
}

int bCreatDB = 0;
int bSequenceMatch = 0; // 1:CPU提取和匹配 2:GPU提取CPU匹配 3:GPU提取和匹配
int bFeatureExtraction = 0;
int bFeatureExtractionGPU = 0;
int bFeatureExtractionBatch = 0;
int bFeatureExtractionBatchGPU = 0;
int bGroupCheck = 0;
int bExhaustivePairImage = 0;
int bIndexImage = 0;
int bQueryImage = 0;
int bFeatureMatching = 0;
int bFeatureMatchingGPU = 0;
int bFeatureMatchingBatch = 0;
int bFeatureMatchingBatchGPU = 0;
int bSfM = 0;
int bReadFromBundler = 0;
int bBoundSparsePoints = 0;
int bUndistortImage = 0;
int bUndistortImageBatch = 0;
int bNeighborSelection = 0;
int bDepthComputation = 0;
int bDepthComputationBatch = 0;
int bDepthFilter = 0;
int bDepthFilterBatch = 0;
int bDepthFusion = 0;
int bBoundDensePoints = 0;
int bColorEstimation = 0;
int bPointsMeshing = 0; // 1:从稠密点 2:从稀疏点

int main(int argc, char *argv[])
{
	boost::program_options::options_description config("Options");
	config.add_options()
		("ip", boost::program_options::value<string>(&OPT::sIp), "ip address")
		("port", boost::program_options::value<unsigned>(&OPT::nPort), "port")
		("user", boost::program_options::value<string>(&OPT::sUser), "user name")
		("pass", boost::program_options::value<string>(&OPT::sPass), "password")
		("db_name", boost::program_options::value<string>(&OPT::sDbName), "DB name")
		("log_name", boost::program_options::value<string>(&OPT::sLogName), "log name")
		("data_path", boost::program_options::value<string>(&OPT::sDataPath), "data path")
		("image_folder", boost::program_options::value<string>(&OPT::sImageFolder), "image folder")
		("vobtree_name", boost::program_options::value<string>(&OPT::sVoctreeName), "vocabulary tree name")
		("sparse_bound_name", boost::program_options::value<string>(&OPT::sSparseBoundName), "sparse bound file name")
		("dense_bound_name", boost::program_options::value<string>(&OPT::sDenseBoundName), "dense bound file name")
		("model_name", boost::program_options::value<string>(&OPT::sModelName), "model name")
		("mesh_name", boost::program_options::value<string>(&OPT::sMeshName), "mesh name")
		("image_id_first", boost::program_options::value<int>(&OPT::ID_first), "image ID first")
		("image_id_last", boost::program_options::value<int>(&OPT::ID_last), "image ID last")
		("overwrite", boost::program_options::value<bool>(&OPT::bOverWrite), "force to overwrite")
		("texture", boost::program_options::value<int>(&OPT::bTexture), "texture mapping")
		("focal", boost::program_options::value<float>(&OPT::fFocalLength), "focal length")
		("features", boost::program_options::value<int>(&OPT::nfeatures), "max features")
		("max_size", boost::program_options::value<int>(&OPT::nMaxSize), "max image size")
		("first_octave", boost::program_options::value<int>(&OPT::nFirstOctave), "first octave")
		("num_levels", boost::program_options::value<int>(&OPT::nNumLevels), "levels number in each octave")
		("max_num_orientations", boost::program_options::value<int>(&OPT::nMaxNumOrientations), "max orientations number")
		("octave_layers", boost::program_options::value<int>(&OPT::nOctaveLayers), "octave layers")
		("contrast_threshold", boost::program_options::value<float>(&OPT::fContrastThreshold), "contrast threshold")
		("edge_threshold", boost::program_options::value<float>(&OPT::fEdgeThreshold), "edge threshold")
		("sigma", boost::program_options::value<float>(&OPT::fSigma), "sigma")
		("nndr", boost::program_options::value<float>(&OPT::fNNDR), "nndr")
		("num_min_matches", boost::program_options::value<unsigned>(&OPT::nMinMatch), "min matches number")
		("num_query", boost::program_options::value<int>(&OPT::nQuery), "query number")
		("num_overlap", boost::program_options::value<int>(&OPT::nOverlap), "overlap number")
		("num_spatial", boost::program_options::value<int>(&OPT::nSpatial), "spatial number")
		("scale", boost::program_options::value<float>(&OPT::fScale), "MVS scale")
		("num_max_views", boost::program_options::value<unsigned>(&OPT::nMaxViews), "max neighbor views number")
		("num_min_views", boost::program_options::value<unsigned>(&OPT::nMinViews), "min neighbor views number")
		("ncc", boost::program_options::value<float>(&OPT::fNCCThreshold), "ncc")
		("num_consis_filter", boost::program_options::value<unsigned>(&OPT::nConsisFilter), "filter consistency view number")
		("depth_diff_threshold", boost::program_options::value<float>(&OPT::fDepthDiffThreshold), "depth difference threshold")
		("num_consis_fuse", boost::program_options::value<unsigned>(&OPT::nConsisFuse), "fuse consistency view number")
		("fuse_step", boost::program_options::value<unsigned>(&OPT::nFuseStep), "fuse pixel step")
		("max_points", boost::program_options::value<unsigned long long>(&OPT::nMaxPoints), "max points number")
		("texture_scale", boost::program_options::value<float>(&OPT::fTextureScale), "texture scale")
		("decimate", boost::program_options::value<float>(&OPT::fDecimateMesh), "mesh decimate ratio")
		("dist_insert", boost::program_options::value<float>(&OPT::fDistInsert), "distance insert")
		("k_cover", boost::program_options::value<int>(&OPT::k_cover), "k_cover")
		("n_omst", boost::program_options::value<int>(&OPT::n_OMST), "n_OMST")
		("min_p3p_matches", boost::program_options::value<int>(&OPT::min_P3P_matches), "min_P3P_matches")
		("min_vis_views", boost::program_options::value<int>(&OPT::min_vis_views), "min_vis_views")
		("ra_conf", boost::program_options::value<int>(&OPT::ra_conf), "ra_conf")
		("min_tri_angle", boost::program_options::value<float>(&OPT::min_tri_angle), "min_tri_angle")
		("rigister_ratio", boost::program_options::value<float>(&OPT::rigister_ratio), "rigister_ratio")
		("bCreatDB", boost::program_options::value<int>(&bCreatDB), "bCreatDB")
		("bSequenceMatch", boost::program_options::value<int>(&bSequenceMatch), "bSequenceMatch")
		("bFeatureExtraction", boost::program_options::value<int>(&bFeatureExtraction), "bFeatureExtraction")
		("bFeatureExtractionGPU", boost::program_options::value<int>(&bFeatureExtractionGPU), "bFeatureExtractionGPU")
		("bFeatureExtractionBatch", boost::program_options::value<int>(&bFeatureExtractionBatch), "bFeatureExtractionBatch")
		("bFeatureExtractionBatchGPU", boost::program_options::value<int>(&bFeatureExtractionBatchGPU), "bFeatureExtractionBatchGPU")
		("bGroupCheck", boost::program_options::value<int>(&bGroupCheck), "bGroupCheck")
		("bExhaustivePairImage", boost::program_options::value<int>(&bExhaustivePairImage), "bExhaustivePairImage")
		("bIndexImage", boost::program_options::value<int>(&bIndexImage), "bIndexImage")
		("bQueryImage", boost::program_options::value<int>(&bQueryImage), "bQueryImage")
		("bFeatureMatching", boost::program_options::value<int>(&bFeatureMatching), "bFeatureMatching")
		("bFeatureMatchingGPU", boost::program_options::value<int>(&bFeatureMatchingGPU), "bFeatureMatchingGPU")
		("bFeatureMatchingBatch", boost::program_options::value<int>(&bFeatureMatchingBatch), "bFeatureMatchingBatch")
		("bFeatureMatchingBatchGPU", boost::program_options::value<int>(&bFeatureMatchingBatchGPU), "bFeatureMatchingBatchGPU")
		("bSfM", boost::program_options::value<int>(&bSfM), "bSfM")
		("bReadFromBundler", boost::program_options::value<int>(&bReadFromBundler), "bReadFromBundler")
		("bBoundSparsePoints", boost::program_options::value<int>(&bBoundSparsePoints), "bBoundSparsePoints")
		("bUndistortImage", boost::program_options::value<int>(&bUndistortImage), "bUndistortImage")
		("bUndistortImageBatch", boost::program_options::value<int>(&bUndistortImageBatch), "bUndistortImageBatch")
		("bNeighborSelection", boost::program_options::value<int>(&bNeighborSelection), "bNeighborSelection")
		("bDepthComputation", boost::program_options::value<int>(&bDepthComputation), "bDepthComputation")
		("bDepthComputationBatch", boost::program_options::value<int>(&bDepthComputationBatch), "bDepthComputationBatch")
		("bDepthFilter", boost::program_options::value<int>(&bDepthFilter), "bDepthFilter")
		("bDepthFilterBatch", boost::program_options::value<int>(&bDepthFilterBatch), "bDepthFilterBatch")
		("bDepthFusion", boost::program_options::value<int>(&bDepthFusion), "bDepthFusion")
		("bBoundDensePoints", boost::program_options::value<int>(&bBoundDensePoints), "bBoundDensePoints")
		("bColorEstimation", boost::program_options::value<int>(&bColorEstimation), "bColorEstimation")
		("bPointsMeshing", boost::program_options::value<int>(&bPointsMeshing), "bPointsMeshing");
	boost::program_options::variables_map vm;

	if (argc == 2)
	{
		std::string configFile = argv[1];
		std::ifstream ifs(configFile);
		if (!ifs)
		{
			cout << "Cannot open configfile: " << configFile << endl;
			return 1;
		}
		try
		{
			boost::program_options::store(boost::program_options::parse_config_file(ifs, config), vm);
			boost::program_options::notify(vm);
		}
		catch (const std::exception& e) {
			cout << e.what() << endl;
			return 1;
		}
	}
	else
	{
		boost::program_options::positional_options_description p;
		p.add("ip", -1);
		try
		{
			boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(config).positional(p).run(), vm);
			boost::program_options::notify(vm);
		}
		catch (const std::exception& e) {
			cout << e.what() << endl;
			return 1;
		}
	}

	if (OPT::sIp.empty() || OPT::sUser.empty() || OPT::sDbName.empty())
	{
		cout << "Not enought input args\n";
		return 1;
	}
	
	if (bCreatDB && (OPT::sDataPath.empty() || OPT::sImageFolder.empty()))
	{
		cout << "Not enought input args\n";
		return 1;
	}

	if (bBoundSparsePoints && OPT::sSparseBoundName.empty())
	{
		cout << "Not enought input args\n";
		return 1;
	}
	
	if (bIndexImage && OPT::sVoctreeName.empty())
	{
		cout << "Not enought input args\n";
		return 1;
	}
	
	if (bBoundDensePoints && OPT::sDenseBoundName.empty())
	{
		cout << "Not enought input args\n";
		return 1;
	}

	if ((bFeatureExtraction || bFeatureExtractionGPU || bFeatureMatching || bFeatureMatchingGPU || bUndistortImage || bDepthComputation || bDepthFilter) && (OPT::ID_first < 0 || OPT::ID_last < 0))
	{
		cout << "Not enought input args\n";
		return 1;
	}

	if (bPointsMeshing && OPT::sModelName.empty())
	{
		cout << "Not enought input args\n";
		return 1;
	}

	OPT::sImageFolder = OPT::sDataPath + "/" + OPT::sImageFolder;
	OPT::sSparseBoundName = OPT::sDataPath + "/" + OPT::sSparseBoundName;
	OPT::sDenseBoundName = OPT::sDataPath + "/" + OPT::sDenseBoundName;
	OPT::sModelName = OPT::sDataPath + "/" + OPT::sModelName;
	OPT::sMeshName = OPT::sDataPath + "/" + OPT::sMeshName;

	google::InitGoogleLogging("");
	google::SetLogDestination(google::GLOG_INFO, OPT::sLogName.c_str());
	google::SetStderrLogging(google::GLOG_INFO);
	FLAGS_logbufsecs = 0;

	LOG(INFO) << "IP: " << OPT::sIp;
	LOG(INFO) << "Port: " << OPT::nPort;
	LOG(INFO) << "User: " << OPT::sUser;
	LOG(INFO) << "Pass: " << OPT::sPass;
	LOG(INFO) << "DbName: " << OPT::sDbName;
	LOG(INFO) << "LogName: " << OPT::sLogName;
	LOG(INFO) << "Overwrite: " << OPT::bOverWrite;
	if (bFeatureExtraction || bFeatureExtractionGPU || bFeatureMatching || bFeatureMatchingGPU || bUndistortImage || bDepthComputation || bDepthFilter)
		LOG(INFO) << "ID_first: " << OPT::ID_first << ", ID_last: " << OPT::ID_last;

	double timer = (double)cv::getTickCount();
	
	// 创建数据库，写入全局参数，存储图像
	if (bCreatDB)
	{
		LOG(INFO) << "============== Creat DB Begin ==============";

		Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);

		// 创建数据库
		if (db.CreatDB())
			LOG(INFO) << "Creat DB " << OPT::sDbName;
		else
		{
			LOG(ERROR) << "Failde to creat DB " << OPT::sDbName;
			db.Close();
			return 1;
		}
		db.Close();

		// 创建数据表
		db.Open();
		if (OPT::bOverWrite)
			db.DeleteTables();

		if (db.CreatTables())
			LOG(INFO) << "Creat tables successful";
		else
		{
			LOG(ERROR) << "Failed to creat tables";
			db.Close();
			return 1;
		}

		if (!bSequenceMatch)
		{
			// 存储图像名
			LOG(INFO) << "Image path: " << OPT::sImageFolder;

			if (!boost::filesystem::exists(OPT::sImageFolder))
			{
				LOG(ERROR) << "Can't find the image folder: " << OPT::sImageFolder;
				db.Close();
				return 1;
			}

			vector<string> imageDirs;			
			IterateImageDirs(OPT::sImageFolder, imageDirs);
			imageDirs.push_back("");
			sort(imageDirs.begin(), imageDirs.end(), std::less<std::string>());
			int groupId = 0;
			for (string imageDir : imageDirs)
			{
				vector<string> imageNames;
				IterateImageFiles(OPT::sImageFolder + "/" + imageDir, imageNames);
				if (imageNames.empty())
				{
					if (imageDir.empty())
						groupId++;
					continue;
				}					
				
				string prePath = "";
				if (!imageDir.empty())
					prePath = imageDir + "/";
				
				LOG(INFO) << "Image group " << groupId << ":";

				for (string imageName : imageNames)
				{
					imageName = prePath + imageName;					
					if (db.ExistImageName(imageName))
						LOG(INFO) << "Image " << imageName << " is exist in DB";
					else
					{
						if (db.WriteImageName(imageName, groupId))
							LOG(INFO) << "Write image " << imageName << " into DB";
						else
							LOG(ERROR) << "Failed to write image " << imageName << " into DB";
					}
				}
				groupId++;
			}
		}

		db.Close();
		LOG(INFO) << "============== Creat DB Finished ==============";

	} // 创建数据库，写入全局参数，存储图像		

	// 特征检测
	if (bFeatureExtraction || bFeatureExtractionBatch || bFeatureExtractionGPU || bFeatureExtractionBatchGPU)
	{
		LOG(INFO) << "============== Feature extraction Begin ==============";		
		LOG(INFO) << "Feature extraction parameters: max features:" << OPT::nfeatures << ", max size:" << OPT::nMaxSize << ", first layers:" << OPT::nFirstOctave << ", levels number:" << OPT::nNumLevels;
		LOG(INFO) << "Feature extraction parameters: orientations number:" << OPT::nMaxNumOrientations << ", octave layers:" << OPT::nOctaveLayers << ", contrast threshold:" << OPT::fContrastThreshold;
		LOG(INFO) << "Feature extraction parameters: edge threshold:" << OPT::fEdgeThreshold << ", sigma:" << OPT::fSigma;

		if (!boost::filesystem::exists(OPT::sImageFolder))
		{
			LOG(ERROR) << "Can't find the image folder: " << OPT::sImageFolder;
			return 1;
		}
		else
			LOG(INFO) << "Image path: " << OPT::sImageFolder;
		
		if ((bFeatureExtractionGPU || bFeatureExtractionBatchGPU) && !SiftGPUExtractionInilization())
		{
			LOG(ERROR) << "SiftGPU extraction initialization failed";
			return 1;
		}
		
		Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);
		db.Open();

		vector<unsigned> IDs;
		if (bFeatureExtractionBatch || bFeatureExtractionBatchGPU)
		{
			if (db.ReadAllImageID(IDs))
				LOG(INFO) << "Found " << IDs.size() << " images in DB";
			else
			{
				LOG(ERROR) << "Found NO images in DB";
				db.Close();
				return 1;
			}
		}
		else
		{
			for (int i = OPT::ID_first; i <= OPT::ID_last; i++)
				IDs.push_back(i);
		}

#if !defined(bCloud)
		db.Close();

#pragma omp parallel for
#endif
		for (int i = 0; i < IDs.size(); i++)
		{
#if !defined(bCloud)
			Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);
			db.Open();
#endif

			int ID = IDs[i];

			if (!db.ExistImageID(ID))
			{
				LOG(ERROR) << "Image " << ID << " is not exist in DB";
#if !defined(bCloud)
				db.Close();
#endif
				continue;
			}

			if (!OPT::bOverWrite && db.ExistKeypoints(ID))
			{
				LOG(INFO) << "Keypoints for image " << ID << " is exist in DB";
#if !defined(bCloud)
				db.Close();
#endif
				continue;
			}

			string imageName = OPT::sImageFolder + "/" + db.ReadImageName(ID);
			cv::Mat img = cv::imread(imageName);
			cv::Mat keys, descriptors;
			bool flag;

			if (bFeatureExtractionGPU || bFeatureExtractionBatchGPU)
				flag = FeatureExtractionGPU(img, keys, descriptors);
			else
				flag = FeatureExtraction(img, keys, descriptors);

			if (flag)
			{
				LOG(INFO) << "Extract " << keys.rows << " features for image " << ID;

				double focalLength, lon, lat, alt, gps_x, gps_y, gps_z;
				GetExifInfo(imageName, focalLength, lon, lat, alt);
				GPS2XYZ(lon, lat, alt, gps_x, gps_y, gps_z);

				if (OPT::fFocalLength > 0)
					focalLength = OPT::fFocalLength;
				else if (focalLength <=0)
					focalLength = MAX(img.cols, img.rows) * 1.2;

				if (lon != -1 && lat != -1 && alt != -1)
				{					
					if (db.WriteImageGPS(ID, lon, lat, alt, gps_x, gps_y, gps_z))
						LOG(INFO) << "Write GPS infomation for image " << ID << " into DB (lon:" << setprecision(10) << lon << ", lat:" << lat << ", alt:" << alt << ")";
					else
						LOG(ERROR) << "Failed to write GPS infomation for image " << ID << " into DB";
				}

				if (db.WriteImageSize(ID, 1, img.cols, img.rows) && db.WriteImageIntParam(ID, focalLength, focalLength, img.cols / 2.f, img.rows / 2.f))
					LOG(INFO) << "Write image size and initial interial parameters for image " << ID << " in DB";
				else
					LOG(ERROR) << "Failed to write image size and initial interial parameters for image " << ID << " in DB";

				if (db.WriteKeypoints(ID, keys, descriptors))
					LOG(INFO) << "Write keypoints for image " << ID << " into DB (" << keys.rows << " features)";
				else
					LOG(ERROR) << "Failed to write keypoints for image " << ID << " into DB";
			}
			else
			{
				if (db.WriteImageSize(ID, 0, 0, 0))
					LOG(INFO) << "Write image size (empty) and initial interial parameters (empty) for image " << ID << " in DB";
				else
					LOG(ERROR) << "Failed to write image size (empty) and initial interial parameters (empty) for image " << ID << " in DB";
			}

#if !defined(bCloud)
			db.Close();
#endif
		}

#if defined(bCloud)
		db.Close();
#endif

		LOG(INFO) << "============== Feature extraction Finished ==============";
	} // 特征检测

	// 修正分组焦距参数
	if (bGroupCheck)
	{
		LOG(INFO) << "============== Group check begin ==============";

		Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);
		db.Open();

		if (db.GroupCheck())
			LOG(INFO) << "Group check done";
		else
			LOG(ERROR) << "Group check failed";

		db.Close();
		LOG(INFO) << "============== Group check finished ==============";
	} // 修正分组焦距参数

	// Cn2构造图像对
	if (bExhaustivePairImage)
	{
		LOG(INFO) << "============== Exhaustive pair image begin ==============";

		Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);
		db.Open();

		vector<unsigned> IDs;
		if (db.ReadAllImageID(IDs))
			LOG(INFO) << "Found " << IDs.size() << " images in DB";
		else
		{
			LOG(ERROR) << "Found NO images in DB";
			db.Close();
			return 1;
		}

		// 构造匹配图像对ID
		ImagePairIds imagePairIds;
		for (int i = 0; i < IDs.size(); i++)
		{
			for (int j = i + 1; j < IDs.size(); j++)
			{
				int imageId1 = IDs[i];
				int imageId2 = IDs[j];
				imagePairIds.push_back(std::pair<int, int>(imageId1, imageId2));
			}
		}

		if (OPT::bOverWrite)
		{
			if (db.CleanImagePair())
				LOG(INFO) << "Clean matches table done";
			else
				LOG(ERROR) << "Failed to clean matches table";
		}

		for (ImagePairId imagePairId : imagePairIds)
		{
			if (db.WriteImagePairs(imagePairId.first, imagePairId.second))
				LOG(INFO) << "Write image pairId " << imagePairId.first << " and " << imagePairId.second << " into DB";
			else
			{
				LOG(ERROR) << "Failed to write image pairId " << imagePairId.first << " and " << imagePairId.second << " into DB";
				return 1;
			}
		}

		db.Close();
		LOG(INFO) << "============== Exhaustive pair image finished ==============";
	} // Cn2构造图像对

	// 语义描述构建
	if (bIndexImage)
	{
		LOG(INFO) << "============== Index image begin ==============";		
		
		string visualIndexName = OPT::sDataPath + "/index.bin";
		if(IndexImage(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort, OPT::sVoctreeName, visualIndexName))
			LOG(INFO) << "Index images done";
		else
		{
			LOG(ERROR) << "Failed to index images";
			return 1;
		}
		
		LOG(INFO) << "============== Index image finished ==============";
	} // 语义描述构建

	// 语义检索
	if (bQueryImage)
	{
		LOG(INFO) << "============== Query image begin ==============";		
		
		string visualIndexName = OPT::sDataPath + "/index.bin";
		if (QueryImage(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort, OPT::nQuery, OPT::nOverlap, OPT::nSpatial, visualIndexName))
			LOG(INFO) << "Query and write image pairIds into DB";
		else
		{
			LOG(ERROR) << "Failed to query and write image pairIds into DB";
			return 1;
		}

		LOG(INFO) << "============== Query image finished ==============";
	} // 语义检索

	// 特征匹配
	if (bFeatureMatching || bFeatureMatchingBatch || bFeatureMatchingGPU || bFeatureMatchingBatchGPU)
	{
		LOG(INFO) << "============== Feature matching begin ==============";
		LOG(INFO) << "Image matching parameters, nndr:" << OPT::fNNDR << ", min matches:" << OPT::nMinMatch;

		if ((bFeatureMatchingGPU || bFeatureMatchingBatchGPU) && !SiftGPUMatchingInilization())
		{
			LOG(ERROR) << "SiftGPU matching initialization failed";
			return 1;
		}
		
		Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);
		db.Open();
		
		vector<unsigned> IDs;
		if (bFeatureMatchingBatch || bFeatureMatchingBatchGPU)
		{
			if (db.ReadAllImageID(IDs))
				LOG(INFO) << "Found " << IDs.size() << " images in DB";
			else
			{
				LOG(ERROR) << "Found NO images in DB";
				db.Close();
				return 1;
			}			
		}
		else
		{
			for (int i = OPT::ID_first; i <= OPT::ID_last; i++)
				IDs.push_back(i);
		}

		ImagePairIds imagePairIds;
		if (db.ReadImagePairsByIds(imagePairIds, IDs))
			LOG(INFO) << "Load " << imagePairIds.size() << " image pair Ids from DB";
		else
		{
			LOG(ERROR) << "Failed to load image pair Ids from DB";
			db.Close();
			return 1;
		}
		std::sort(imagePairIds.begin(), imagePairIds.end());

#if !defined(bCloud)
		db.Close();		

#pragma omp parallel for
#endif
		for (int i = 0; i < IDs.size(); i++)
		{
#if !defined(bCloud)
			Db db(OPT::sIp, OPT::sUser, OPT::sPass, OPT::sDbName, OPT::nPort);
			db.Open();
#endif

			int imageId1 = IDs[i];
			vector<int> imageId2s;
			bool flag = 1;
			for (ImagePairId imagePairId : imagePairIds)
			{
				if (imageId1 == imagePairId.first)
				{
					if (flag)
					{
						if (!db.ExistKeypoints(imageId1))
						{
							LOG(ERROR) << "Keypoint for image " << imageId1 << " is not exist in DB";
							continue;
						}
					}
					flag = 0;
					int imageId2 = imagePairId.second;
					if (!db.ExistKeypoints(imageId2))
					{
						LOG(ERROR) << "Keypoint for image " << imageId2 << " is not exist in DB";
						continue;
					}
					imageId2s.push_back(imagePairId.second);
				}
				else if (imageId1 < imagePairId.first)
					break;
			}

			int imageId2Num = imageId2s.size();
			if (imageId2Num == 0)
			{
				LOG(INFO) << "Image " << imageId1 << " has no matched images";
#if !defined(bCloud)
				db.Close();
#endif
				continue;
			}

			cv::Mat keys1, keys2, descriptors1, descriptors2, inlierMatchesData;

			if (db.ReadKeypoints(imageId1, keys1, descriptors1))
				LOG(INFO) << "Load keypoints for image " << imageId1 << " from DB (" << keys1.rows << " features)";
			else
			{
				LOG(ERROR) << "Failed to load keypoints for image " << imageId1 << " from DB";
#if !defined(bCloud)
				db.Close();
#endif
				continue;
			}

			for (int imageId2 : imageId2s)
			{
				if (db.ReadKeypoints(imageId2, keys2, descriptors2))
				{
					LOG(INFO) << "Load keypoints for image " << imageId2 << " from DB (" << keys2.rows << " features)";

					if (bFeatureMatchingGPU || bFeatureMatchingBatchGPU)
						flag = FeatureMatchingGPU(keys1, descriptors1, keys2, descriptors2, inlierMatchesData);
					else
						flag = FeatureMatching(keys1, descriptors1, keys2, descriptors2, inlierMatchesData);

					if (flag)
					{
						LOG(INFO) << "Match image " << imageId1 << " and image " << imageId2 << " :" << inlierMatchesData.rows << " matches";
					}
					else
					{
						inlierMatchesData.release();
						LOG(INFO) << "Not enough matches between image " << imageId1 << " and image " << imageId2;
					}

					if (db.WriteMatches(imageId1, imageId2, inlierMatchesData))
						LOG(INFO) << "Write matches for image " << imageId1 << " and " << imageId2 << ": " << inlierMatchesData.rows << " matches";
					else
						LOG(ERROR) << "Failed to write matches for image " << imageId1 << " and " << imageId2;
				}
				else
				{
					LOG(ERROR) << "Failed to load keypoints for image " << imageId2 << " from DB";
					continue;
				}
			}

#if !defined(bCloud)
			db.Close();
#endif
		}

#if defined(bCloud)
		db.Close();
#endif

		LOG(INFO) << "============== Feature matching finished ==============";
	} // 特征匹配

	timer = (double)cv::getTickCount() - timer;
	LOG(INFO) << "Time: " << timer / cv::getTickFrequency() << "s";
	google::ShutdownGoogleLogging();

	return 0;    
}