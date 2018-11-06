#pragma once

#include "Map.h"
#include <vector>
#include <queue>
#include <map>
#include <iostream>
#include <fstream>

#include <opencv2\opencv.hpp>
#include "Converter.h"
#include <ORBextractor.h>
#include <ORBVocabulary.h>

//#define DEBUG_LOG 

#define DEBUG_LOG_LOAD_MAP

#define SAVE_LOAD_TXT

namespace ORB_SLAM2 {

	class SaveLoadMap : public ORB_SLAM2::Map, public ORB_SLAM2::MapPoint
	{
		std::string morbSettingsFile;
		ORB_SLAM2::Map * mcurrentmap = 0;

		std::set<MapPoint * > msMapPoints;
		std::set<KeyFrame *> msKeyFrames;


	public:
		ORB_SLAM2::ORBVocabulary * mORBvocabulary = 0;
		SaveLoadMap() {}

		SaveLoadMap(const std::string & filename) :morbSettingsFile(filename) {

#ifdef DEBUG_LOG 
			dbgstr.open("C://Dataset/TUM_Sequences/dbgstrmapsave.txt", std::ios::out | std::ios::app);
#endif
#ifdef DEBUG_LOG_LOAD_MAP
			dbgstrrd.open("C://Dataset/TUM_Sequences/dbgstrreadmapload.txt",std::ios::out | std::ios::app);
#endif


		}
		bool SaveMapPoint(std::ofstream &, ORB_SLAM2::MapPoint &);


		void SetCurrentMapFields(ORB_SLAM2::Map & mapfromORB);
		bool SaveKeyFrame(std::ofstream & ofs, ORB_SLAM2::KeyFrame& kf, std::map<ORB_SLAM2::MapPoint *, unsigned long int> & mmMapPointWithID);
		bool LoadVocabulary(std::string &);
		bool LoadMapPoint(std::ifstream &, ORB_SLAM2::MapPoint & mapPt);
		//bool LoadMapPoint(std::ifstream & ifs, unsigned long int & mpPtId, cv::Mat & wolrdPos, long unsigned int & mptrefKFid);

		bool LoadKeyFrame(std::ifstream &, std::vector<MapPoint*>&, ORB_SLAM2::ORBextractor &, ORB_SLAM2::Frame&);

		bool SaveMap(std::string &);
		Map * LoadMap(std::string &);

#ifdef DEBUG_LOG
		std::ofstream dbgstr;
#endif
#ifdef DEBUG_LOG_LOAD_MAP
		std::ofstream dbgstrrd;
#endif
#ifdef SAVE_LOAD_TXT
		std::ofstream savemapstream;
		std::ifstream loadmapstream;
#endif
		inline void SetReferenceKeyFrame(ORB_SLAM2::KeyFrame* mpKF) { mpRefKF = mpKF; }

	};

}

