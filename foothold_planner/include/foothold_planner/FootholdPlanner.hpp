
#pragma once

//! STL
#include <thread>

//! ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

//! grid map
//#include <grid_map_msgs/GridMap.h>
//#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

//! 自定义
#include "foothold_planner_msgs/GlobalFootholds.h"
#include "foothold_planner/GlobalFootholdPlan.h"


namespace foothold_planner
{

class FootholdPlanner
{
public:
    FootholdPlanner(ros::NodeHandle& node);
    virtual ~FootholdPlanner();

public:
    /********************************************
        进行全局落脚点规划的ROS服务。
        输入：服务请求，参数（规划的步态周期数）
        返回：服务应答（全局落脚点信息）
    *******************************************/
//    bool globalFootholdPlan(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);                                     
bool globalFootholdPlan(foothold_planner::GlobalFootholdPlan::Request& request, 
                        foothold_planner::GlobalFootholdPlan::Response& response);

    /********************************************
        单个足端落脚点规划，包括默认点检查、候选点检查、计算落脚点高度。封装成一个单独线程。
        输入：；
        返回：
    *******************************************/
    // geometry_msgs::PointStamped checkFoothold();


    /********************************************
        输入：grid map；
             圆形区域中心；
             圆形足端区域半径；
             圆形搜索区域半径；
             四边形区域；
        返回：
    *******************************************/
    geometry_msgs::PointStamped checkFoothold(  grid_map::GridMap gridmap, 
                                                grid_map::Position center,
                                                float footRadius,
                                                float searchRadius,
                                                grid_map::Polygon polygon,
                                                geometry_msgs::PointStamped& footholdResult,
                                                bool& footholdValidation);


    /********************************************
        检查默认落脚点（即圆形足端区域）的可通行性
        输入：grid map；
             圆形区域中心；
             圆形区域半径；
        返回：是否可通行
    *******************************************/
    bool checkDefaultFoothold(  grid_map::GridMap gridmap, 
                                grid_map::Position center,
                                float footRadius);


    /********************************************
        检查候选落脚点的可通行性，当默认落脚点检查不通过的前提下才执行
        输入：grid map；
             圆形区域中心；
             圆形足端区域半径；
             圆形搜索区域半径；
             四边形区域；
        返回：是否可通行
    *******************************************/
    bool checkCandidateFoothold(grid_map::GridMap gridmap, 
                                grid_map::Position spiralCenter, 
                                float footRadius,
                                float searchRadius,
                                grid_map::Polygon polygon,
                                geometry_msgs::PointStamped& footholdResult);


    /********************************************
        检查同时落在设定的圆形及矩形区域的落脚点的可通行性
        输入：grid map；
             圆形区域中心；
             圆形区域半径；
             四边形区域；
        返回：是否可通行
    *******************************************/
    bool checkCirclePolygonFoothold(grid_map::GridMap gridmap, 
                                    grid_map::Position center,
                                    float searchRadius,
                                    grid_map::Polygon polygon);


    /********************************************
        获取当前中心对应的足端位置
        输入：当前中心位置；
             相对于中心的位置偏差量；
        返回：相对于中心的足端位置
    *******************************************/
    geometry_msgs::PointStamped getDefaultFootholdNext( geometry_msgs::Point feetCenterNext,
                                                        geometry_msgs::PointStamped defaultBias);


    /********************************************
        获取四边形几何中心，这里指机器人4个足端位置构成的四边形
        输入：4个足端位置；
        返回：四边形几何中心
    *******************************************/   
    geometry_msgs::Point getPolygonCenter(  geometry_msgs::PointStamped RF_point,
                                            geometry_msgs::PointStamped RH_point,
                                            geometry_msgs::PointStamped LH_point,
                                            geometry_msgs::PointStamped LF_point);


    /********************************************
        获取落脚点搜索矩形区域
        输入：圆形搜索区域半径；
             圆形搜索区域中心
        返回：矩形搜索区域
    *******************************************/
    grid_map::Polygon getSearchPolygon( geometry_msgs::PointStamped center,
                                        float radius);

    geometry_msgs::PolygonStamped getSearchPolygon( geometry_msgs::Vector3 center,
                                                    float radius);                                                    


    /********************************************
        获取足端位置平均高度（假设足端为一个圆形，计算该圆形区域平均高度作为该足端高度）
        输入：grid map
             足端位置
             足端半径
        返回：足端平均高度
    *******************************************/
    float getFootholdMeanHeight(grid_map::GridMap gridmap, 
                                grid_map::Position center,
                                float radius);


    /********************************************
        获取足端当前位置
        输入：足端当前位置
        返回：足端当前位置
    *******************************************/
    inline geometry_msgs::Point getFootCurrentPosition(geometry_msgs::Point point){
        geometry_msgs::Point p;
        p = point;

        return p;
    }


    /********************************************
        发布4个足端落脚点矩形搜索区域的markers
        输入：
        返回：
    *******************************************/
    // void publishSearchPolygonMarker(grid_map::GridMap gridmap,					  
    //                                 geometry_msgs::Vector3 RF_DefaultFootholdPosition, 
    //                                 geometry_msgs::Vector3 RH_DefaultFootholdPosition, 
    //                                 geometry_msgs::Vector3 LH_DefaultFootholdPosition, 
    //                                 geometry_msgs::Vector3 LF_DefaultFootholdPosition)
    void publishSearchPolygonMarker(geometry_msgs::Vector3 center,
                                    float radius);    


    /********************************************
        发布规划得到的全局落脚点markers
        输入：4个落脚点位置
        返回：无
    *******************************************/
    void publishGlobalFootholdMarkers(  geometry_msgs::PointStamped rfFootholdResult, 
                                        geometry_msgs::PointStamped rhFootholdResult, 
                                        geometry_msgs::PointStamped lhFootholdResult, 
                                        geometry_msgs::PointStamped lfFootholdResult);



private:
    bool readParameters();
    bool initialize();

    void gridmapCallback(grid_map_msgs::GridMap msg);


    ros::NodeHandle& node_;

    ros::Subscriber gridmapSub_;
    ros::Subscriber requestSub_;

    ros::Publisher resultsPub_;
    ros::Publisher resultFootholdsMarkersPub_;    
    ros::Publisher resultSearchRegionMarkersPub_;    
    ros::Publisher RF_searchPolygonPub_, RH_searchPolygonPub_, LH_searchPolygonPub_, LF_searchPolygonPub_;

    ros::Publisher globalFootholdsPub_;
    ros::Publisher globalFootholdsMarkerPub_;

    ros::ServiceServer globalFootholdPlanService_;

    grid_map::GridMap gridmap_;

    //! **************** 全局变量 *********************
    bool defaultFootholdIsvalid_;
    bool candidateFootholdIsvalid_;
    bool RF_footholdValidation_;
    bool RH_footholdValidation_;
    bool LH_footholdValidation_;
    bool LF_footholdValidation_;
    bool footholdValidation_;

    foothold_planner_msgs::GlobalFootholds globalFootholds_;

    grid_map::Position spiralCenter_;


    //! 当前4个足端位置，由全局定位模块提供，位于世界坐标系。目前由本体里程计提供
    geometry_msgs::PointStamped RF_currentPosition_;
    geometry_msgs::PointStamped RH_currentPosition_;
    geometry_msgs::PointStamped LH_currentPosition_;
    geometry_msgs::PointStamped LF_currentPosition_;

    //! 下一个步态周期对应的4个足端默认落脚点位置
    geometry_msgs::PointStamped RF_nextPosition_;
    geometry_msgs::PointStamped RH_nextPosition_;
    geometry_msgs::PointStamped LH_nextPosition_;
    geometry_msgs::PointStamped LF_nextPosition_;    

    geometry_msgs::Vector3 RF_DefaultFootholdPosition_;
    geometry_msgs::Vector3 RH_DefaultFootholdPosition_;
    geometry_msgs::Vector3 LH_DefaultFootholdPosition_;
    geometry_msgs::Vector3 LF_DefaultFootholdPosition_;

    //! 4个足端默认落脚点相对于其构成的几何中心的位置偏差量（即相对于本体坐标系下的默认落脚点位置）
    geometry_msgs::PointStamped RF_defaultBias_;
    geometry_msgs::PointStamped RH_defaultBias_;
    geometry_msgs::PointStamped LH_defaultBias_;
    geometry_msgs::PointStamped LF_defaultBias_;

    //! 线程
    std::thread RF_planThread_;
    std::thread RH_planThread_;
    std::thread LH_planThread_;
    std::thread LF_planThread_;

    //! **************** 从参数服务器加载的参数parameters ***********
    std::string test_;
    //! 是否开启debug模式
    bool debug_;
    bool checkDefaultFoothold_Debug_;

    float defaultFootholdThreshold_;
    float candidateFootholdThreshold_;

    //! 步态周期个数(一个步态周期包含4个落脚点)，用于决定一次性规划多少个全局落脚点
    int gaitCycleNum_;

    //! 圆形搜索区域半径
    float searchRadius_;
    float stepLength_;
    //! 圆形足端半径
    float footRadius_;


    //! 4个足端默认落脚点相对于其构成的几何中心的位置偏差量（即相对于本体坐标系下的默认落脚点位置）
    float RF_defaultBiasX_;
    float RF_defaultBiasY_;

    float RH_defaultBiasX_;
    float RH_defaultBiasY_;

    float LH_defaultBiasX_;
    float LH_defaultBiasY_;

    float LF_defaultBiasX_;
    float LF_defaultBiasY_;

    //! laikago运动学参数
    struct LaikagoKinematics
    {
        float length;
        float width;
        float l1;
        float l2;
        float l3;

        float lengthBase;
        float widthBase;
    };
    LaikagoKinematics laikagoKinematics_;

    //! （等腰梯形长边-短边）/2
    float skewLength_;
    


    //! 来自其他节点的参数
    std::string mapFrame_;


};

}
