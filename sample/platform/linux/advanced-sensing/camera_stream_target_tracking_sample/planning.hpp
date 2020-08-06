#include <vector>

typedef float  float32_t;
typedef double float64_t;

// 航线规划点
typedef struct planningWayPoint{
  float64_t longitude;
  float64_t latitude;
  float64_t altitude;
  float32_t droneYaw;
  float32_t cameraYaw;
  float32_t cameraPitch;
  int isPhoto;
}planningWayPoint;

//计算两个GPS点的距离
float64_t calculateDistance(float64_t lon1,float64_t lat1,float64_t alt1, float64_t lon2,float64_t lat2, float64_t alt2);

// 从文件中读取航线规划点
std::vector<planningWayPoint> getPlaningPoint(std::string wpFile);

// 判断是否在航线规划点的范围内
bool isInPlanningRang(float64_t lon, float64_t lat, float64_t alt, std::vector<planningWayPoint> pWps,float32_t distanceLimit);