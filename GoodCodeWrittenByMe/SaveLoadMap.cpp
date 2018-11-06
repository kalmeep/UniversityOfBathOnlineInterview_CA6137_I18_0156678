#include "..\include\SaveLoadMap.h"
#include <set>
void ORB_SLAM2::SaveLoadMap::SetCurrentMapFields(ORB_SLAM2::Map & map)
{
	std::vector<ORB_SLAM2::KeyFrame*> keyframes = map.GetAllKeyFrames();
	std::vector<ORB_SLAM2::MapPoint*> mappoints = map.GetAllMapPoints();
	unique_lock<mutex> lock(mMutexMap);
	for (auto keyframe : keyframes)
	{
		//mspKeyFrames.insert(keyframes.begin(), keyframes.end());
		msKeyFrames.insert(keyframes.begin(), keyframes.end());
	}

	//unique_lock<mutex> lock(mMutexMap);
	for (MapPoint * mapPoint : mappoints)
	{
		//mspMapPoints.insert(mappoints.begin(), mappoints.end());
		msMapPoints.insert(mappoints.begin(), mappoints.end());
	}
}

bool ORB_SLAM2::SaveLoadMap::SaveMapPoint(std::ofstream & ofs, ORB_SLAM2::MapPoint& mP)
{
	bool mappointsaved = false;
	cv::Mat worldPos = mP.GetWorldPos();

#ifdef SAVE_LOAD_TXT
	savemapstream << mP.mnId;
	savemapstream << worldPos.at<float>(0);
	savemapstream << worldPos.at<float>(1);
	savemapstream << worldPos.at<float>(2);

#else

	try {

		ofs.write(reinterpret_cast<char*>(&mP.mnId), sizeof(mP.mnId)); // long unsigned int
		
		ofs.write(reinterpret_cast<char*>(&(worldPos.at<float>(0))), sizeof(float));
		ofs.write(reinterpret_cast<char*>(&(worldPos.at<float>(1))), sizeof(float));
		ofs.write(reinterpret_cast<char*>(&(worldPos.at<float>(2))), sizeof(float));
		//	ofs.write(reinterpret_cast<char*>(&mP.GetReferenceKeyFrame()->mnId), (sizeof(mP.GetReferenceKeyFrame()->mnId)));

	}
	catch (std::exception ex)
	{
		typeid(ex);
		//throw ;
		return mappointsaved;
	}
#endif
#ifdef DEBUG_LOG
	dbgstr << "mP.mnId " << mP.mnId << "\n";
	dbgstr << "worldPos.at<float>(0) " << worldPos.at<float>(0) << "\n";
	dbgstr << "worldPos.at<float>(1) " << worldPos.at<float>(1) << "\n";
	dbgstr << "worldPos.at<float>(2) " << worldPos.at<float>(2) << "\n";

	std::cout << worldPos.at<float>(0) << "," << worldPos.at<float>(1) << "," << worldPos.at<float>(2) << "\n";

#endif

	mappointsaved = true;
	return mappointsaved;
}

bool ORB_SLAM2::SaveLoadMap::SaveKeyFrame(std::ofstream & ofs, ORB_SLAM2::KeyFrame& kf, std::map<ORB_SLAM2::MapPoint *, unsigned long int> & mmMapPointWithID)
{
	//kf 
	//tranformation from camera to world
	// intrinsics
	// ORB ftr
	bool savedKeyFrame = false;
	//kfid//save mnid
	ofs.write(reinterpret_cast<char*>(&kf.mnId), sizeof(kf.mnId)); // long unsigned int

	//kfsf
	//ofs.write(reinterpret_cast<char*>(&kf.mfScaleFactor),sizeof(float));
	double kfTimestamp = kf.mTimeStamp;
	ofs.write(reinterpret_cast<char*>(&kfTimestamp), sizeof(kfTimestamp));

	//camera localizaion
	cv::Mat kfPose = kf.GetPose();

	//save pose tr
	float tx = kfPose.at<float>(0, 3),
		ty = kfPose.at<float>(1, 3),
		tz = kfPose.at<float>(2, 3);

	// save pose quat from rot
	std::vector<float> quat = ORB_SLAM2::Converter::toQuaternion(kfPose.rowRange(0, 3).colRange(0, 3));

	try {
		ofs.write(reinterpret_cast<char*>(&tx), sizeof(float));
		ofs.write(reinterpret_cast<char*>(&ty), sizeof(float));
		ofs.write(reinterpret_cast<char*>(&tz), sizeof(float));
		ofs.write(reinterpret_cast<char*> (&quat[0]), sizeof(float));
		ofs.write(reinterpret_cast<char*> (&quat[1]), sizeof(float));
		ofs.write(reinterpret_cast<char*>(&quat[2]), sizeof(float));
		ofs.write(reinterpret_cast<char*> (&quat[3]), sizeof(float));


#ifdef DEBUG_LOG
		dbgstr << "kf.mnId" << kf.mnId << "\n";
		dbgstr << "kfTimestamp" << kfTimestamp;
		dbgstr << "tx" << tx << "\n";
		dbgstr << "ty" << ty << "\n";
		dbgstr << "tz" << tz << "\n";
		dbgstr << "quat[0]" << quat[0] << "\n";;
		dbgstr << "quat[1]" << quat[1] << "\n";
		dbgstr << "quat[2]" << quat[2] << "\n";
		dbgstr << "quat[3]" << quat[3] << "\n";
#endif
	}
	catch (std::exception ex)
	{
		typeid(ex);
		throw;
		return savedKeyFrame;
	}
	// total features keypoints
	int nFeatKeyPoints = kf.N;
	ofs.write(reinterpret_cast<char*>(&nFeatKeyPoints), sizeof(nFeatKeyPoints));

	//for each kpt in N {
	std::vector<cv::KeyPoint> mvKPt = kf.mvKeys;

#ifdef DEBUG_LOG
	dbgstr << "nFeatKeyPoints " << nFeatKeyPoints << "\n";
#endif
	//save keypoint {x,y,octave...descriptor}
	for (int keyPtindex = 0; keyPtindex < kf.N; ++keyPtindex)
	{
		ofs.write(reinterpret_cast<char*>(&mvKPt[keyPtindex].pt.x), sizeof(mvKPt[keyPtindex].pt.x));
		ofs.write(reinterpret_cast<char*>(&mvKPt[keyPtindex].pt.y), sizeof(mvKPt[keyPtindex].pt.y));
		ofs.write(reinterpret_cast<char*>(&mvKPt[keyPtindex].size), sizeof(mvKPt[keyPtindex].size));
		ofs.write(reinterpret_cast<char*>(&mvKPt[keyPtindex].angle), sizeof(mvKPt[keyPtindex].angle));
		//	ofs.write(reinterpret_cast<char*>(&mvKPt[keyPtindex].octave), sizeof(mvKPt[keyPtindex].octave));
			//ofs.write(reinterpret_cast<char*>(&mvKPt[keyPtindex].response), sizeof(mvKPt[keyPtindex].response));

#ifdef DEBUG_LOG
		dbgstr << " for keyPtindex - " << keyPtindex << "pt x -" << mvKPt[keyPtindex].pt.x << "\n";
		dbgstr << " for keyPtindex - " << keyPtindex << "pt y -" << mvKPt[keyPtindex].pt.y << "\n";
		dbgstr << " for keyPtindex - " << keyPtindex << "size -" << mvKPt[keyPtindex].size << "\n";
		dbgstr << " for keyPtindex - " << keyPtindex << "angle -" << mvKPt[keyPtindex].angle << "\n";
#endif

		//save representative desriptor(length 32)
		for (int indexOfDescriptor = 0; indexOfDescriptor < 32; indexOfDescriptor++) {
			char descvalue = kf.mDescriptors.at<char>(keyPtindex, indexOfDescriptor);
			ofs.write(reinterpret_cast<char*>(&descvalue), sizeof(char));
#ifdef DEBUG_LOG
			dbgstr << " for indexOfDescriptor - " << indexOfDescriptor << "descvalue is -" << descvalue << "\n";
#endif

		}
#ifdef DEBUG_LOG
		dbgstr << "\n\n eodesc";
#endif

		// for each key pt , map point  based on id 
		MapPoint * mapPoint = NULL;
		unsigned long int currentKPtId = LONG_MAX;
		if (kf.GetMapPoint(keyPtindex) == NULL)
		{
			currentKPtId = currentKPtId;
		}
		else {
			mapPoint = kf.GetMapPoint(keyPtindex);
			if (!mapPoint)
			{
				std::cout << "no matchinf id found" << "\n";
			}
			else {
				try {
					//std::cout << "id for mp" << mmMapPointWithID.at(mapPoint);
					currentKPtId = mmMapPointWithID.at(mapPoint);
				}
				catch (std::exception & ex)
				{
					typeid(ex);
				}
			}
		}

		//	currentKPtId = (!((mmMapPointWithID[mapPoint] = kf.GetMapPoint(keyPtindex) == NULL))) ?  mmMapPointWithID.at(kf.GetMapPoint(keyPtindex)) : currentKPtId ;
		ofs.write(reinterpret_cast<char*>(&currentKPtId), sizeof(currentKPtId));

#ifdef DEBUG_LOG
		dbgstr << " for keypt index - " << keyPtindex << "currentKPtId is -" << currentKPtId << "\n";
#endif
	}

	// graph
	unsigned long int currentID = LONG_MAX;
	KeyFrame* parentKeyFrame = kf.GetParent();
	//	if (!parentKeyFrame) {
		//	ofs.write(reinterpret_cast<char*>(&currentID), sizeof(currentID));
		//}
	//	else {
	if (parentKeyFrame) {
		currentID = parentKeyFrame->mnId;
	}
	ofs.write(reinterpret_cast<char*>(&currentID), sizeof(currentID));

#ifdef DEBUG_LOG
	dbgstr << " parent kf id currentID - " << currentID << "\n";
#endif

	/*if (!parentKeyFrame)
	{
		kf.ChangeParent(&kf);
	}
		//parentKeyFrame->mnId = currentID;
	else */

	//tree
	for (auto connectedkf : kf.GetConnectedKeyFrames()) {
		int connectedkfWeight = connectedkf->GetWeight(parentKeyFrame);
		long unsigned connectedkfid = connectedkf->mnId;

		ofs.write(reinterpret_cast<char*>(&connectedkfid), sizeof(connectedkfid));
		ofs.write(reinterpret_cast<char*>(&connectedkfWeight), sizeof(connectedkfWeight));	//

#ifdef DEBUG_LOG
		dbgstr << " connectedkfid, wt = " << connectedkfid << "," << connectedkfWeight << "\n" << "eoconntected kf " << "\n" ;
#endif			
	}

	//if (!mORBvocabulary)

	//	mORBvocabulary = ;

	savedKeyFrame = true;
	return savedKeyFrame;
}

bool ORB_SLAM2::SaveLoadMap::SaveMap(std::string &  filename)
{
	//map 
	//mappoint {worldpos} , id
	//kf {//tranformation from camera to world
	// intrinsics ?(from frame)
	// ORB ftr}
	//CovsGraph
	//EssGraph qualified by spannign tree edges

	bool mapsaved = false;
	bool mapPointSaved = false;
	size_t mapPtCount = msMapPoints.size();

	savemapstream.open(filename, std::ios::out);

#ifdef SAVE_LOAD_TXT
	savemapstream << mapPtCount;

#ifdef DEBUG_LOG
	dbgstr << "mapct " << mapPtCount << "\n";
#endif

	unsigned long int id = 0;
	std::map<ORB_SLAM2::MapPoint*, unsigned long int> mmMapPointWithID;
	//for (ORB_SLAM2::MapPoint * mspMP : mspMapPoints) {
	for (ORB_SLAM2::MapPoint * mspMP : msMapPoints) {
		if (mspMP)
		{
			mapPointSaved = SaveMapPoint(savemapstream, *mspMP);
		}
	}
	savemapstream.close();
#else 

	std::ofstream ofs(filename, std::ios::out | std::ios::binary | std::ios::app);

	//total map pts in map
	//size_t mapPtCount = mspMapPoints.size();
	
	ofs.write(reinterpret_cast<char*>(&mapPtCount), (sizeof(mapPtCount)));

#ifdef DEBUG_LOG
	dbgstr << "mapct " << mapPtCount << "\n";
#endif


	// each map pt
	//loop thru mspMapPoints &
	// prepare map of map pts by id
	unsigned long int id = 0;
	std::map<ORB_SLAM2::MapPoint*, unsigned long int> mmMapPointWithID;
	//for (ORB_SLAM2::MapPoint * mspMP : mspMapPoints) {
	for (ORB_SLAM2::MapPoint * mspMP : msMapPoints) {
		if (mspMP)
		{
			mapPointSaved = SaveMapPoint(ofs, *mspMP);
		}

	}
	if (mapPointSaved)
	{
		//	for (ORB_SLAM2::MapPoint * mspMP : mspMapPoints) {
		for (ORB_SLAM2::MapPoint * mspMP : msMapPoints) {
			if (mspMP) {
				mmMapPointWithID[mspMP] = id;
				++id;
			}
		}
	}
	else {
		//break;
#ifdef DEBUG_LOG
		dbgstr << "error saving map point for id" << (id + 1) << "\n";
#endif
		std::cerr << "error saving map point for id" << (id + 1) << "\n";
		return mapsaved;
	}	 
	ofs.close();

#endif

	
	//saveKF
	mapsaved = true;
	

#ifdef DEBUG_LOG
	if(dbgstr.is_open())
	dbgstr.close();
#endif

	return mapsaved;
}

bool ORB_SLAM2::SaveLoadMap::LoadVocabulary(std::string& filename)
{
	bool vocabularyLoaded = false;
	mORBvocabulary = new ORBVocabulary();
	vocabularyLoaded = mORBvocabulary->loadFromTextFile2(filename);

	return vocabularyLoaded;
}

bool ORB_SLAM2::SaveLoadMap::LoadMapPoint(std::ifstream & ifs, ORB_SLAM2::MapPoint & mapPt)
//bool ORB_SLAM2::SaveLoadMap::LoadMapPoint(std::ifstream & ifs , unsigned long int & mpPtId , cv::Mat & wolrdPos , long unsigned int & mptrefKFid)

{
	bool mapPointLoaded = false;
	long unsigned int mapid = 0;
	float x, y, z;
	long unsigned int mfirstKFid = 0, mRefKFid = 0,
		mFirstFrameid = 0;
	
#ifdef SAVE_LOAD_TXT

	ifs >> mapid;
	ifs >> x;
	ifs >> y;
	ifs >> z;

#else 
	//load mappt id
	try {

		ifs.read(reinterpret_cast<char*>(&mapid), (sizeof(mapid)));

		//load worldpos
		ifs.read(reinterpret_cast<char*>(&x), (sizeof(x)));
		ifs.read(reinterpret_cast<char*>(&y), (sizeof(y)));
		ifs.read(reinterpret_cast<char*>(&z), (sizeof(z))); 

		cv::Mat worldPos = (cv::Mat_<float>(3, 1) << x, y, z);

		//add mappts

		mapPt.mnId = mapid;
		mapPt.SetWorldPos(worldPos);
		mapPt.mnFirstKFid = mfirstKFid;
		mapPt.mnFirstFrame = mFirstFrameid;

}
	catch (std::exception ex)
	{
		typeid(ex);
		return mapPointLoaded;
	}


#endif
#ifdef DEBUG_LOG_LOAD_MAP
	dbgstrrd << "mapid-" << mapid << "\n";
	dbgstrrd << "worldPos.at<float>(0) " << x << "\n";
	dbgstrrd << "worldPos.at<float>(1) " << y << "\n";
	dbgstrrd << "worldPos.at<float>(2) " << z << "\n";
 
#endif
	mapPointLoaded = true;
	return mapPointLoaded;
}

bool ORB_SLAM2::SaveLoadMap::LoadKeyFrame(std::ifstream & ifs, std::vector<ORB_SLAM2::MapPoint*>&mvMapPointsLoaded, ORB_SLAM2::ORBextractor &extractorORB, ORB_SLAM2::Frame& frame) 
{
	bool kfLoaded = false;
	//assign a fr to kf
	//Frame frame;


	ifs.read(reinterpret_cast<char*>(&frame.mnId), sizeof(frame.mnId));
	ifs.read(reinterpret_cast<char*>(&frame.mTimeStamp), sizeof(frame.mTimeStamp));

	//load camera localization info 
	//tr
	//rot from quat
	//pose
	float tx = 0.0f, ty = 0.0f, tz = 0.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 0.0f;
	ifs.read(reinterpret_cast<char*>(&tx), sizeof(float));
	ifs.read(reinterpret_cast<char*>(&ty), sizeof(float));
	ifs.read(reinterpret_cast<char*>(&tz), sizeof(float));
	ifs.read(reinterpret_cast<char*> (&qx), sizeof(float));
	ifs.read(reinterpret_cast<char*> (&qy), sizeof(float));
	ifs.read(reinterpret_cast<char*>(&qz), sizeof(float));
	ifs.read(reinterpret_cast<char*> (&qw), sizeof(float));
#ifdef DEBUG_LOG_LOAD_MAP
	dbgstrrd << " LoadKeyFrame -frame mnid -" << frame.mnId << ",tstmp-" << frame.mTimeStamp << ",tx-"
		<< tx << ",ty-" << ty << ",tz- " << tz << ",qx-" << qx << ",qy-" << qy << ",qz-" << qz << ",qw-" << qw << "\n";
#endif
	cv::Mat translation = (cv::Mat_<float>(4, 1) << tx, ty, tz, 1.0f);
	cv::Mat quat = (cv::Mat_<float>(4, 1) << qx, qy, qz, qw);
	float quatarray[] = { qw,qx,qy,qz };

	//cv::Rodrigues(quat.colRange(1,1).rowRange(1,3), rotationMatrix);
	//add using eigen ont working (check)
	Eigen::Quaternionf quatfloat(quatarray);
	Eigen::Matrix<float, 3, 3> rotationMatrixEigen = quatfloat.toRotationMatrix(); \

		cv::Mat rotationMatrix = (cv::Mat_<float>(3, 3) << rotationMatrixEigen(0, 0), rotationMatrixEigen(0, 1), rotationMatrixEigen(0, 2),
			rotationMatrixEigen(1, 0), rotationMatrixEigen(1, 1), rotationMatrixEigen(1, 2),
			rotationMatrixEigen(2, 0), rotationMatrixEigen(2, 1), rotationMatrixEigen(2, 2));

	cv::Mat TcameraToWorld = (cv::Mat_<float>(4, 4) << rotationMatrix.at<float>(0, 0), rotationMatrix.at<float>(0, 1), rotationMatrix.at<float>(0, 2), translation.at<float>(0),
		rotationMatrix.at<float>(1, 0), rotationMatrix.at<float>(1, 1), rotationMatrix.at<float>(1, 2), translation.at<float>(1),
		rotationMatrix.at<float>(2, 0), rotationMatrix.at<float>(2, 1), rotationMatrix.at<float>(2, 2), translation.at<float>(2),
		0.0, 0.0, 0.0, translation.at<float>(3));

	frame.SetPose(TcameraToWorld);

	int totalKeyPointCount = 0;
	ifs.read(reinterpret_cast<char*>(&totalKeyPointCount), sizeof(totalKeyPointCount));
#ifdef DEBUG_LOG_LOAD_MAP
	dbgstrrd << "kptct " << totalKeyPointCount << "\n";
#endif
	
	//const int assignment check
	frame.N = totalKeyPointCount;
	// loop thru keypts
	// load keypt info 
	frame.mvKeys.reserve(frame.N);
	frame.mDescriptors.create(frame.N, 32, CV_8UC1);
	frame.mvpMapPoints = vector<ORB_SLAM2::MapPoint*>(frame.N, static_cast<ORB_SLAM2::MapPoint*>(NULL));
	for (int keyPtIndex = 0; keyPtIndex < frame.N; keyPtIndex++)
	{
		cv::KeyPoint currentKeyPoint;
		ifs.read(reinterpret_cast<char*>(&currentKeyPoint.pt.x), sizeof(currentKeyPoint.pt.x));
		ifs.read(reinterpret_cast<char*>(&currentKeyPoint.pt.y), sizeof(currentKeyPoint.pt.y));
		ifs.read(reinterpret_cast<char*>(&currentKeyPoint.size), sizeof(currentKeyPoint.size));
		ifs.read(reinterpret_cast<char*>(&currentKeyPoint.angle), sizeof(currentKeyPoint.angle));
		//ifs.read(reinterpret_cast<char*>(&currentKeyPoint.octave), sizeof(currentKeyPoint.octave));
		//ifs.read(reinterpret_cast<char*>(&currentKeyPoint.response), sizeof(currentKeyPoint.response));
#ifdef DEBUG_LOG_LOAD_MAP
		dbgstrrd << "x - " << currentKeyPoint.pt.x << ",y-" << currentKeyPoint.pt.y << ",size-" << currentKeyPoint.size << ",angle-" << currentKeyPoint.angle << "\n";
#endif
		//AddKeyPoint

		frame.mvKeys.push_back(currentKeyPoint);
		long unsigned int mapid = 0;
		for (int idx = 0; idx < 32; idx++)
		{
			ifs.read(reinterpret_cast<char*>(&frame.mDescriptors.at<char>(keyPtIndex, idx)), sizeof(char));
#ifdef DEBUG_LOG_LOAD_MAP
			dbgstrrd << "desc val" << frame.mDescriptors.at<char>(keyPtIndex, idx) << "\n";
#endif
		}
		unsigned long int currentKPtId = 0;
		ifs.read(reinterpret_cast<char*>(&currentKPtId), sizeof(currentKPtId));
#ifdef DEBUG_LOG_LOAD_MAP
		dbgstrrd << "currentKPtId"  << currentKPtId << "\n";
#endif
		try {
			if (currentKPtId == LONG_MAX)
				frame.mvpMapPoints[currentKPtId] = NULL;
			else
				frame.mvpMapPoints[currentKPtId] = mvMapPointsLoaded[currentKPtId];
		}
		catch (std::exception& ex)
		{
#ifdef DEBUG_LOG_LOAD_MAP
			dbgstrrd.close();
#endif
			typeid(ex);
		}
	}

	//  // Set no stereo information
	frame.mvuRight = vector<float>(frame.N, -1.0f);
	frame.mvDepth = vector<float>(frame.N, -1.0f);
	frame.mpORBextractorLeft = &extractorORB;

	//scale info for mono wrt pyramid
	frame.mnScaleLevels = frame.mpORBextractorLeft->GetLevels();
	frame.mfScaleFactor = frame.mpORBextractorLeft->GetScaleFactor();
	frame.mfLogScaleFactor = log(frame.mfScaleFactor);
	frame.mvScaleFactors = frame.mpORBextractorLeft->GetScaleFactors();
	frame.mvInvScaleFactors = frame.mpORBextractorLeft->GetInverseScaleFactors();
	frame.mvLevelSigma2 = frame.mpORBextractorLeft->GetScaleSigmaSquares();
	frame.mvInvLevelSigma2 = frame.mpORBextractorLeft->GetInverseScaleSigmaSquares();

	//undistort 
	if (frame.mDistCoef.at<float>(0) == 0.0)
	{
		frame.mvKeysUn = frame.mvKeys;
	}
	else {
		cv::Mat mat(frame.N, 2, CV_32F);
		for (int i = 0; i < frame.N; i++)
		{
			mat.at<float>(i, 0) = frame.mvKeys[i].pt.x;
			mat.at<float>(i, 1) = frame.mvKeys[i].pt.y;
		}
		// Undistort points
		mat = mat.reshape(2);
		cv::undistortPoints(mat, mat, frame.mK, frame.mDistCoef, cv::Mat(), frame.mK);
		mat = mat.reshape(1);

		// Fill undistorted keypoint vector
		frame.mvKeysUn.resize(frame.N);
		for (int i = 0; i < frame.N; i++)
		{
			cv::KeyPoint kp = frame.mvKeys[i];
			kp.pt.x = mat.at<float>(i, 0);
			kp.pt.y = mat.at<float>(i, 1);
			frame.mvKeysUn[i] = kp;
		}
	}

	//features to grid
	int nReserve = 0.5f*frame.N / (FRAME_GRID_COLS*FRAME_GRID_ROWS);
	for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
		for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
			frame.mGrid[i][j].reserve(nReserve);

	for (int i = 0; i < frame.N; i++)
	{
		const cv::KeyPoint &kp = frame.mvKeysUn[i];

		int nGridPosX, nGridPosY;
		if (frame.PosInGrid(kp, nGridPosX, nGridPosY))
			frame.mGrid[nGridPosX][nGridPosY].push_back(i);
	}

	//update bow vector upto scale levels wrt pyramid
	//std::vector<cv::Mat> vCurrentDesc = ORB_SLAM2::Converter::toDescriptorVector(frame.mDescriptors);
	//mORBvocabulary->transform(vCurrentDesc, frame.mBowVec, frame.mFeatVec, 4);	
	frame.ComputeBoW();

	kfLoaded = true;
	return kfLoaded;
}

ORB_SLAM2::Map * ORB_SLAM2::SaveLoadMap::LoadMap(std::string & filename)
{
#ifdef SAVE_LOAD_TXT
	
	loadmapstream.open(filename,std::ios::in);
	ORB_SLAM2::Map * loadedMap = new ORB_SLAM2::Map();
	cv::FileStorage fsSettings(morbSettingsFile.c_str(), cv::FileStorage::READ);
	int nfeatures = (int)fsSettings["ORBextractor.nFeatures"];
	float scaleFactor = (float)fsSettings["ORBextractor.scaleFactor"];
	int nlevels = (int)fsSettings["ORBextractor.nLevels"],
		iniThFAST = (int)fsSettings["ORBextractor.iniThFAST"],
		minThFAST = (int)fsSettings["ORBextractor.minThFAST"];

		ORB_SLAM2::ORBextractor extractorORB(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
		// map point count 
		int mspMapPointsCount;
		loadmapstream >> mspMapPointsCount;
		
#ifdef DEBUG_LOG_LOAD_MAP
		dbgstrrd << "mspmap pts count" << mspMapPointsCount << "\n";
#endif

		unsigned long int idMapPoint = 0;

		for (int idx = 0; idx < mspMapPointsCount; idx++)
		{
			//loadMapPt		
			unsigned long int mapPtId = 0;
			unsigned long int mapPtRefKFId = 0;
			cv::Mat worldPos;


			ORB_SLAM2::MapPoint * mspMP = new ORB_SLAM2::MapPoint();
					bool mapPointLoaded = LoadMapPoint(loadmapstream, *mspMP);
			if (mapPointLoaded) 		//AddMapPoint 
			{
				loadedMap->AddMapPoint(mspMP);
				idMapPoint = (mspMP->mnId >= idMapPoint) ? mspMP->mnId : idMapPoint;
			}
		}
		nNextId = ++idMapPoint;

#ifdef DEBUG_LOG_LOAD_MAP
		if (dbgstrrd.is_open())
			dbgstrrd.close();
#endif

		loadmapstream.close();
#else


	std::ifstream ifs(filename, std::ios::in | std::ios::binary);
	ORB_SLAM2::Map * loadedMap = NULL;
	loadedMap = new ORB_SLAM2::Map();
	//retrieve ORB SLAM parameters {intrinsics , features , voc }
	//ORB_SLAM2::Frame  frame;
	cv::FileStorage fsSettings(morbSettingsFile.c_str(), cv::FileStorage::READ);
	int nfeatures = (int)fsSettings["ORBextractor.nFeatures"];
	float scaleFactor = (float)fsSettings["ORBextractor.scaleFactor"];
	int nlevels = (int)fsSettings["ORBextractor.nLevels"],
		iniThFAST = (int)fsSettings["ORBextractor.iniThFAST"],
		minThFAST = (int)fsSettings["ORBextractor.minThFAST"];

	//extract feature descriptor 
	ORB_SLAM2::ORBextractor extractorORB(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);

	// map point count 
	int mspMapPointsCount;
	//		char * mspMapPtCount;

	//std:map<ORB_SLAM2::MapPoint * , unsigned long int > mapPointWithRefKFId;
	//std:map<ORB_SLAM2::SaveLoadMap*, unsigned long int > mapPointWithRefKFId;

	//start
	ifs.read(reinterpret_cast<char*>(&mspMapPointsCount), sizeof(int));

#ifdef DEBUG_LOG_LOAD_MAP
	dbgstrrd << "mspmap pts count" << mspMapPointsCount << "\n";
#endif

	unsigned long int idMapPoint = 0;
	//loop thru mspMapPoints & retrive mappts and ids
	//for (auto mspMP : mspMapPoints) {

	//	ORB_SLAM2::MapPoint * mapPt = new ORB_SLAM2::MapPoint();

	for (int idx = 0; idx < mspMapPointsCount; idx++)
	{
		//loadMapPt		
		unsigned long int mapPtId = 0;
		unsigned long int mapPtRefKFId = 0;
		cv::Mat worldPos;


		ORB_SLAM2::MapPoint * mspMP = new ORB_SLAM2::MapPoint(); 
		bool mapPointLoaded = LoadMapPoint(ifs, *mspMP);
		if (mapPointLoaded) 		//AddMapPoint 
		{
			loadedMap->AddMapPoint(mspMP);
			idMapPoint = (mspMP->mnId >= idMapPoint) ? mspMP->mnId : idMapPoint;
		} 
	} 
	nNextId = ++idMapPoint;

	//read mappts loaded
	std::vector<MapPoint*> mvMapPoint = loadedMap->GetAllMapPoints();
	//std::vector<SaveLoadMap *> mvMapPoint = loadedMap->GetAllMapPoints();
	std::vector<KeyFrame*> mvKeyFramesLocation;

	//kf count

	int mspKeyFramesCount = 0;
	ifs.read(reinterpret_cast<char*>(&mspKeyFramesCount), sizeof(int));//garbage
#ifdef DEBUG_LOG_LOAD_MAP
	dbgstrrd << "mspKeyFramesCount 1st read" << mspKeyFramesCount << "\n";
#endif
	ifs.read(reinterpret_cast<char*>(&mspKeyFramesCount), sizeof(int));
#ifdef DEBUG_LOG_LOAD_MAP
	dbgstrrd << "mspKeyFramesCount 2nd read" << mspKeyFramesCount << "\n";
#endif
	//reorder kfs as per ifs 	//add keframes as written
	for (int index = 0; index < mspKeyFramesCount; index++) {


		Frame frame, frameWithRefKF;
		//extractorORB 	//assign frame to keyframe
		frame.mpORBvocabulary = mORBvocabulary;

		//kptcount
		//int keyPointCount = 0;
		//	bool kfLoaded = LoadKeyFrame(ifs, mvMapPoint, mapPointWithRefKFId, extractorORB, frame , mvKeyFramesLocation);
		bool kfLoaded = LoadKeyFrame(ifs, mvMapPoint, extractorORB, frame);
		if (kfLoaded) {
			//associate kf to frame via orb vocab , timestamp , kfkeyPointCount , mvMapPoint

			std::vector<SaveLoadMap *> mvMapPointsWithKFRef = vector<ORB_SLAM2::SaveLoadMap*>(frame.N, static_cast<ORB_SLAM2::SaveLoadMap*>(NULL));

			KeyFrame * pKF = new KeyFrame(frame, this, NULL);
			loadedMap->AddKeyFrame(pKF);
			mvKeyFramesLocation.push_back(pKF);

			for (int idx = 0; idx < frame.N; idx++)
			{
				ORB_SLAM2::SaveLoadMap * mspMPSLM = new ORB_SLAM2::SaveLoadMap();
				if (frame.mvpMapPoints[idx])
				{
					mspMPSLM->mnId = frame.mvpMapPoints[idx]->mnId;
					mspMPSLM->SetWorldPos(frame.mvpMapPoints[idx]->GetWorldPos());
					mspMPSLM->mnFirstKFid = frame.mvpMapPoints[idx]->mnFirstKFid;
					mspMPSLM->mnFirstFrame = frame.mvpMapPoints[idx]->mnFirstFrame;

					frame.mvpMapPoints[idx]->AddObservation(pKF, idx);
					mspMPSLM->AddObservation(pKF, idx);
					if (!(frame.mvpMapPoints[idx]->GetReferenceKeyFrame())) {
						std::cout << "no ref kf" << "\n";
						mspMPSLM->SetReferenceKeyFrame(pKF);
					}
					mvMapPointsWithKFRef[idx] = mspMPSLM;
				}
				else
				{
					mvMapPointsWithKFRef[idx] = NULL;
				}
			}

#ifdef DEBUG_LOG_LOAD_MAP
			dbgstrrd << "in load map frame.mvpMapPoints count assoc to kpt " << frame.mvpMapPoints.size() << "\n";
			dbgstrrd << " in loadmap mvMapPointsWithKFRef count assoc to kpt " << mvMapPointsWithKFRef.size() << "\n";

#endif
			frame.mvpMapPoints.clear();
			frame.mvpMapPoints.assign(mvMapPointsWithKFRef.begin(), mvMapPointsWithKFRef.end());
		}
	}

	std::map<long unsigned int, KeyFrame*> mmKeyFrames;
	//add kfs 
	//for (auto mspKF : mspKeyFrames) {
	for (auto mspKF : msKeyFrames) {
		mmKeyFrames.at(mspKF->mnId) = mspKF;
	}

	// spanning tree parent , node , weight
	//repopulate map by kfid
	//add kfs wrt ids
	// repopulate map by mappt id
	// add mappts wrt ids

	//	for (int index = 0; index < mspKeyFramesCount; index++)
	for (KeyFrame * kf : mvKeyFramesLocation)
	{
		long unsigned int parentId = 0;
		ifs.read(reinterpret_cast<char*>(&parentId), sizeof(parentId));
#ifdef DEBUG_LOG_LOAD_MAP
		dbgstrrd << "parent id" << parentId << "\n";
#endif
		if (parentId == LONG_MAX)
		{
			kf->ChangeParent(mmKeyFrames.at(kf->mnId));
		}

		int connectedKeyFrameCount = 0, connectedKFweight = 0;
		unsigned long int connectedKFId = 0;
		ifs.read(reinterpret_cast<char *>(&connectedKeyFrameCount), sizeof(connectedKeyFrameCount));
#ifdef DEBUG_LOG_LOAD_MAP
		dbgstrrd << "connectedKeyFrameCount" << connectedKeyFrameCount << "\n";
#endif
		for (int ckfIndex = 0; ckfIndex < connectedKeyFrameCount; ckfIndex++)
		{
			ifs.read(reinterpret_cast<char *>(&connectedKFId), sizeof(connectedKFId));
			ifs.read(reinterpret_cast<char *>(&connectedKFweight), sizeof(connectedKFweight));
#ifdef DEBUG_LOG_LOAD_MAP
			dbgstrrd << "connectedKFId" << connectedKFId << "\n";
			dbgstrrd << "connectedKFweight" << connectedKFweight << "\n";
#endif
			kf->AddConnection(mmKeyFrames.at(connectedKFId), connectedKFweight);
		}

	}

	//pop all mappts amd kfs into map
	//std::vector loadedmap<ORB_SLAM2::MapPoint *, unsigned long int id>;
	//std::vector loadedkeyframe<ORB_SLAM2::KeyFrame *, unsigned long int id>;


	//AddKeyFrame

	//graph + tree stuff

	//order kfs as retrieved by file ifs

	//order kfs by id


	// add to Map


	// Create MapPoints and asscoiate to keyframes via dsecriptors , normal

	for (auto mapPoint : mvMapPoint)
	{
		mapPoint->ComputeDistinctiveDescriptors();
		mapPoint->UpdateNormalAndDepth();
	}


	ifs.close();
#ifdef DEBUG_LOG_LOAD_MAP
	if (dbgstrrd.is_open())
		dbgstrrd.close();
#endif

#endif


	mORBvocabulary = 0;
	return loadedMap;
}
