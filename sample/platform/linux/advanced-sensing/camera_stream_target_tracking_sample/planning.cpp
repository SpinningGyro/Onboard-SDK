#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "planning.hpp"


const float64_t earthRadius =  6378137.0;

//计算两个GPS点的距离
float64_t calculateDistance(float64_t lon1,float64_t lat1,float64_t alt1, float64_t lon2,float64_t lat2, float64_t alt2){
  // return acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon2-lon1))*earthRadius;
  // waypointV2Internal.positionX = (waypointV2.longitude - mission[0].longitude) * EARTH_RADIUS *cos(mission[0].latitude);
  // waypointV2Internal.positionY = (waypointV2.latitude - mission[0].latitude) * EARTH_RADIUS;
  // waypointV2Internal.positionZ = waypointV2.relativeHeight;
  float64_t positionX = (lon2 - lon1) * earthRadius * cos(lat1);
  float64_t positionY = (lat2 - lat1) * earthRadius;
  float64_t positionZ = alt2 - alt1;
  std::cout << "positionX: " << positionX << "positionY: " << positionY << "positionZ: " << positionZ << std::endl;
  float64_t dist = sqrt(positionX*positionX + positionY*positionY ); //+ positionZ*positionZ);
  return dist;
  
}

// 从文件中读取航线规划点
std::vector<planningWayPoint> getPlaningPoint(std::string wpFile){
  std::vector<planningWayPoint> planningWPs;
  std::ifstream fread(wpFile);
	char	line[1024];
  int i = 0;
	if(fread.is_open()){    
		while (!fread.eof())
		{      
      fread.getline(line, 1024);
      std::cout << line << std::endl;
      if (*line != 0) {       
        float64_t lon,lat,alt;
        float32_t yaw,cameraYaw,cameraPitch;
        int isPhoto;
        int ret = sscanf(line,"%lf %lf %lf %f %f %f %d",&lon,&lat,&alt,&yaw,&cameraYaw,&cameraPitch,&isPhoto);
        if(ret){
          planningWayPoint planningWP;
          planningWP.longitude = lon;
          planningWP.latitude = lat;
          planningWP.altitude = alt;
          planningWP.droneYaw = yaw;
          planningWP.cameraYaw = cameraYaw;
          planningWP.cameraPitch = cameraPitch;
          planningWP.isPhoto = isPhoto;
          planningWPs.push_back(planningWP);
        }  
      }
      
    }
  }
  fread.close();
  return planningWPs;     
}

// 判断是否在航线规划点的范围内
bool isInPlanningRang(float64_t lon,float64_t lat, float64_t alt,std::vector<planningWayPoint> pWps,float32_t distanceLimit){
  for(auto &pWp:pWps){
    if(pWp.isPhoto){      
      float64_t dist = calculateDistance(lon,lat,alt,pWp.longitude,pWp.latitude,pWp.altitude);
      if(dist < distanceLimit){
        return true;
      }
    }
  }
  return false;
}