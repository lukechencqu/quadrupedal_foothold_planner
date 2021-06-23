
#pragma once

//! STL
#include <thread>

// system information
#include <unistd.h>
#include <pwd.h>
#include <fstream>
#include <time.h>       /* time_t, struct tm, time, localtime, asctime */ //http://www.cplusplus.com/reference/ctime/asctime/

//! ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

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
        输入：默认落脚点坐标
        输出：最终落脚点坐标
        返回：/
    *******************************************/
    // bool checkFootholdUseCentroidMethod(geometry_msgs::PointStamped defaultFoothold,
    //                                     geometry_msgs::PointStamped& resultFoothold);


    /********************************************
     * 遍历检查足端矩形搜索gridmap子图可通行性，获取完全可通行区域（即全部为1的行），并计算此完全可通行区域几何中心
     * 输入：默认落脚点坐标
     *      gridmap
     * 输出：完全可通行区域几何中心坐标、起始与结束行索引
     * 返回：成功标志            
    *******************************************/
    bool checkFootholdUseCentroidMethod(grid_map::GridMap& gridmap,
                                        geometry_msgs::PointStamped defaultFoothold,
                                        geometry_msgs::PointStamped& resultFoothold,
                                        int& beginRow,
                                        int& endRow);


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
     * 获取当前/下一个步态周期的默认足端位置
     * 输入：4个足端当前世界坐标系下位置
     * 输出：4个足端下一个步态周期的世界坐标系下默认位置
     * 返回：成功标志
    *******************************************/
    bool getDefaultFootholds( geometry_msgs::PointStamped RF_position,
                                geometry_msgs::PointStamped RH_position,
                                geometry_msgs::PointStamped LH_position,
                                geometry_msgs::PointStamped LF_position,
                                geometry_msgs::PointStamped& RF_nextPosition,
                                geometry_msgs::PointStamped& RH_nextPosition,
                                geometry_msgs::PointStamped& LH_nextPosition,
                                geometry_msgs::PointStamped& LF_nextPosition);  


    /********************************************
     * 获取当前/下一个步态周期的四个足端矩形搜索区域、当前/下一个步态周期四个足端几何中心连线轨迹
     * 输入：4个足端当前世界坐标系下位置
     * 输出：4个足端支撑面中心轨迹
     *      4个足端下一个步态周期的世界坐标系下位置
     * 返回：成功标志
    *******************************************/
    bool getFootholdSearchGridMap(  geometry_msgs::PointStamped RF_position,
                                    geometry_msgs::PointStamped RH_position,
                                    geometry_msgs::PointStamped LH_position,
                                    geometry_msgs::PointStamped LF_position,
                                    nav_msgs::Path& feetCenterPath,
                                    geometry_msgs::PointStamped& RF_nextPosition,
                                    geometry_msgs::PointStamped& RH_nextPosition,
                                    geometry_msgs::PointStamped& LH_nextPosition,
                                    geometry_msgs::PointStamped& LF_nextPosition,
                                    grid_map::Polygon& RF_polygon,
                                    grid_map::Polygon& RH_polygon,
                                    grid_map::Polygon& LH_polygon,
                                    grid_map::Polygon& LF_polygon,
                                    ros::Publisher RF_searchPolygonPub,
                                    ros::Publisher RH_searchPolygonPub,
                                    ros::Publisher LH_searchPolygonPub,
                                    ros::Publisher LF_searchPolygonPub,                                                
                                    ros::Publisher RF_nextSearchPolygonPub,
                                    ros::Publisher RH_nextSearchPolygonPub,
                                    ros::Publisher LH_nextSearchPolygonPub,
                                    ros::Publisher LF_nextSearchPolygonPub);


    /********************************************
     * 获取一个步态周期对应的gridmap矩形子图
     * 输入：4个足端当前世界坐标系下位置
     * 输出：4个足端支撑面中心轨迹
     *      gridmap
     * 返回：成功标志
    *******************************************/
    bool getGaitCycleSearchGridMap( geometry_msgs::PointStamped RF_position,
                                                    geometry_msgs::PointStamped RH_position,
                                                    geometry_msgs::PointStamped LH_position,
                                                    geometry_msgs::PointStamped LF_position,
                                                    grid_map::GridMap& gaitMap,
                                                    nav_msgs::Path& feetCenterPath,
                                                    geometry_msgs::Point feetCenter,
                                                geometry_msgs::PointStamped& RF_nextPosition,
                                                geometry_msgs::PointStamped& RH_nextPosition,
                                                geometry_msgs::PointStamped& LH_nextPosition,
                                                geometry_msgs::PointStamped& LF_nextPosition);                                                    


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
        获取落脚点搜索矩形区域对应的gridmap
        输入：矩形长和宽
        返回：矩形gridmap
    *******************************************/
    // grid_map::GridMap getSubPo


    /********************************************
        获取足端位置平均高度（假设足端为一个圆形，计算该圆形区域平均高度作为该足端高度）
        输入：grid map
             足端位置
             足端半径
             修正值（在平均高度上加上修正值用来将落脚点显示在地面以上）
        返回：足端平均高度
    *******************************************/
    float getFootholdMeanHeight(grid_map::GridMap gridmap, 
                                grid_map::Position center,
                                float radius,
                                double h);


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
     * 获取gridmap地图指定坐标位置对应的索引值
     * 输入：gridmap
     *      坐标位置
     * 输出：索引值
     * 返回：成功标志
    *******************************************/
    bool getMapIndex(   grid_map::GridMap map,
                        geometry_msgs::Point position,                                                
                        grid_map::Index& index);


    /********************************************
     * 计算一个步态周期内的对脚足端间距
     * 输入：四个足端落脚点坐标
     * 输出：两个对脚足端间距
     * 返回：成功标志
    *******************************************/
    bool getHipDistance(geometry_msgs::PointStamped rfFootholdResult, 
                        geometry_msgs::PointStamped rhFootholdResult, 
                        geometry_msgs::PointStamped lhFootholdResult, 
                        geometry_msgs::PointStamped lfFootholdResult,
                        std::vector<double>& feetDistance);


    /********************************************
     * 估算一个步态周期内的前后2次对脚移动cog线速度
     * 输入：四个足端下一个步态周期落脚点坐标
     *      四个足端当前步态周期落脚点坐标
     * 输出：cog线速度
     * 返回：成功标志
    *******************************************/
    bool getCogSpeed(geometry_msgs::PointStamped rfFootholdResult, 
                    geometry_msgs::PointStamped rhFootholdResult, 
                    geometry_msgs::PointStamped lhFootholdResult, 
                    geometry_msgs::PointStamped lfFootholdResult,
                    geometry_msgs::PointStamped rfCurrentFoothold, 
                    geometry_msgs::PointStamped rhCurrentFoothold, 
                    geometry_msgs::PointStamped lhCurrentFoothold, 
                    geometry_msgs::PointStamped lfCurrentFoothold,                        
                    std::vector<double>& cogSpeed);


    /********************************************
     * 设置当前步态周期四个落脚点位置
     * 输入：左前脚还是右前脚先迈步标志位
     *      开始搜索的起始点位置（到map坐标系原点的偏差量）
     * 输出：落脚点坐标
     * 返回：无
    *******************************************/
    void setCurrentFootholdsPosition(bool RF_FIRST,
                                Eigen::Vector3d initialPose,
                                geometry_msgs::PointStamped &RF_currentPosition,
                                geometry_msgs::PointStamped &RH_currentPosition,
                                geometry_msgs::PointStamped &LH_currentPosition,
                                geometry_msgs::PointStamped &LF_currentPosition);


    /********************************************
     * 设置第一个步态周期的落脚点
     * 输入：机器狗原始站立时四个足端位置
     * 输出：第一个步态周期的落脚点位置
     * 返回：成功标志
    *******************************************/
    bool setFirstGait( geometry_msgs::PointStamped RF_initialPosition, 
                                        geometry_msgs::PointStamped RH_initialPosition, 
                                        geometry_msgs::PointStamped LH_initialPosition, 
                                        geometry_msgs::PointStamped LF_initialPosition, 
                                        geometry_msgs::PointStamped& RF_currentPosition,
                                        geometry_msgs::PointStamped& RH_currentPosition,
                                        geometry_msgs::PointStamped& LH_currentPosition,
                                        geometry_msgs::PointStamped& LF_currentPosition);


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
             落脚点尺寸(直径，建议与球型足端直径相同)
        返回：无
    *******************************************/
    void publishGlobalFootholdPointMarkers(  geometry_msgs::PointStamped rfFootholdResult, 
                                        geometry_msgs::PointStamped rhFootholdResult, 
                                        geometry_msgs::PointStamped lhFootholdResult, 
                                        geometry_msgs::PointStamped lfFootholdResult,
                                        float scale,
                                        double alpha);


    /********************************************
        发布规划得到的全局落脚点markers
        输入：4个落脚点位置
             落脚点尺寸(直径，建议与球型足端直径相同)
        返回：无
    *******************************************/
    void publishGlobalFootholdSphereMarkers(  geometry_msgs::PointStamped rfFootholdResult, 
                                        geometry_msgs::PointStamped rhFootholdResult, 
                                        geometry_msgs::PointStamped lhFootholdResult, 
                                        geometry_msgs::PointStamped lfFootholdResult,
                                        float scale,
                                        double alpha);


    /********************************************
        发布规划得到的全局落脚点markers(立方体)
        输入：4个落脚点位置
             落脚点尺寸(直径，建议与球型足端直径相同)
        返回：无
    *******************************************/
    void publishGlobalFootholdCubeMarkers(  geometry_msgs::PointStamped rfFootholdResult, 
                                            geometry_msgs::PointStamped rhFootholdResult, 
                                            geometry_msgs::PointStamped lhFootholdResult, 
                                            geometry_msgs::PointStamped lfFootholdResult,
                                            float scale,
                                            double alpha);


    /********************************************
        发布规划得到的全局落脚点markers(圆柱体)
        输入：4个落脚点位置
             落脚点尺寸(直径，建议与球型足端直径相同)
             透明度
             步态周期的4倍
        返回：无
    *******************************************/
    void publishGlobalFootholdCylinderMarkers(  geometry_msgs::PointStamped rfFootholdResult, 
                                            geometry_msgs::PointStamped rhFootholdResult, 
                                            geometry_msgs::PointStamped lhFootholdResult, 
                                            geometry_msgs::PointStamped lfFootholdResult,
                                            float scale,
                                            double alpha,
                                            int count);


    /********************************************
        发布初始站立的4个足端位置markers
        输入：4个落脚点位置
             落脚点尺寸(直径，建议与球型足端直径相同)
        返回：无
    *******************************************/
    void publishInitialFootholdMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                        geometry_msgs::PointStamped rhFootholdResult, 
                                        geometry_msgs::PointStamped lhFootholdResult, 
                                        geometry_msgs::PointStamped lfFootholdResult,
                                        float scale,
                                        double alpha);


    /********************************************
     *保存全局落脚点规划结果到本体txt文件
     * 输入：
     * 输出：txt文件
     * 返回：是否成功标志
    *******************************************/
   bool saveLog();


    /********************************************
     *获取系统用户名
     * 输入：/
     * 输出：/
     * 返回：系统用户名
    *******************************************/
    inline std::string getUserName(){
        struct passwd* pwd;
        uid_t userid;
        userid = getuid();
        pwd = getpwuid(userid);
        // cout<<pwd->pw_name<<endl;
        return pwd->pw_name;
    }   


    // // 目标函数
    // double nloptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);
    
    // // 约束函数
    // double nloptConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

public:



    // 约束条件
    // struct ConstraintData{
    //     double a;
    // };
    // ConstraintData constraintData_;

    // 自变量
    // std::vector<double> x(8); // 为gridmap cell的index值（非负整数）


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
    // 当前步态周期足端搜索区域
    ros::Publisher RF_searchPolygonPub_, RH_searchPolygonPub_, LH_searchPolygonPub_, LF_searchPolygonPub_;
    ros::Publisher RF_centroidSearchPolygonPub_, RH_centroidSearchPolygonPub_, LH_centroidSearchPolygonPub_, LF_centroidSearchPolygonPub_;
    ros::Publisher RF_optSearchPolygonPub_, RH_optSearchPolygonPub_, LH_optSearchPolygonPub_, LF_optSearchPolygonPub_;
    // 下一个步态周期足端搜索区域
    ros::Publisher RF_nextSearchPolygonPub_, RH_nextSearchPolygonPub_, LH_nextSearchPolygonPub_, LF_nextSearchPolygonPub_;   
    ros::Publisher RF_centroidNextSearchPolygonPub_, RH_centroidNextSearchPolygonPub_, LH_centroidNextSearchPolygonPub_, LF_centroidNextSearchPolygonPub_;
    ros::Publisher RF_optNextSearchPolygonPub_, RH_optNextSearchPolygonPub_, LH_optNextSearchPolygonPub_, LF_optNextSearchPolygonPub_;
    ros::Publisher gaitMapPolygonPub_;

    // 全局落脚点规划结果话题发布器
    ros::Publisher nominalGlobalFootholdsPub_;
    ros::Publisher centroidGlobalFootholdsPub_;
    ros::Publisher optGlobalFootholdsPub_;
    // 全局落脚点规划结果markers话题发布器
    ros::Publisher globalFootholdsPointMarkerPub_;
    ros::Publisher globalFootholdsSphereMarkerPub_;
    ros::Publisher globalFootholdsCubeMarkerPub_;
    ros::Publisher globalFootholdsCylinderMarkerPub_;

    ros::Publisher initialFootholdMarkerPub_;

    // 足端支撑面几何中心轨迹
    ros::Publisher nominalFeetCenterPathPub_;
    ros::Publisher centroidFeetCenterPathPub_;

    // 一个步态周期对应的gridmap子图
    ros::Publisher gaitMapPub_;
    

    ros::ServiceServer globalFootholdPlanService_;

    // ************* grid map
    grid_map::GridMap gridmap_; // 全局地图
    grid_map::GridMap gaitMap_; // 子地图，一个步态周期默认等腰梯形对应的gridmap


    //! **************** 全局变量 *********************
    bool defaultFootholdIsvalid_;
    bool candidateFootholdIsvalid_;
    bool RF_footholdValidation_;
    bool RH_footholdValidation_;
    bool LH_footholdValidation_;
    bool LF_footholdValidation_;
    bool footholdValidation_;

    foothold_planner_msgs::GlobalFootholds nominalGlobalFootholdsMsg_;
    foothold_planner_msgs::GlobalFootholds centroidGlobalFootholdsMsg_;
    foothold_planner_msgs::GlobalFootholds optGlobalFootholdsMsg_;

    grid_map::Position spiralCenter_;


    // 起始站立位置
    geometry_msgs::PointStamped RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_;
    // 用户设定的起始站立位置（cog在map坐标系下的位置）
    Eigen::Vector3d initialPose_;
    Eigen::Vector3d ajustedPose_; // 落脚点人工修正偏移量（每一个步态周期修正一次，即累加此修正值）

    //! 当前4个足端位置，由全局定位模块提供，位于世界坐标系。目前由本体里程计提供
    geometry_msgs::PointStamped RF_currentPosition_, RH_currentPosition_, LH_currentPosition_, LF_currentPosition_;
    geometry_msgs::PointStamped RF_defaultCurrentPosition_, RH_defaultCurrentPosition_, LH_defaultCurrentPosition_, LF_defaultCurrentPosition_;
    geometry_msgs::PointStamped RF_nominalCurrentPosition_, RH_nominalCurrentPosition_, LH_nominalCurrentPosition_, LF_nominalCurrentPosition_;
    geometry_msgs::PointStamped RF_centroidCurrentPosition_, RH_centroidCurrentPosition_, LH_centroidCurrentPosition_, LF_centroidCurrentPosition_;
    geometry_msgs::PointStamped RF_optCurrentPosition_, RH_optCurrentPosition_, LH_optCurrentPosition_, LF_optCurrentPosition_;

    
    //! 下一个步态周期对应的4个足端默认落脚点位置
    geometry_msgs::PointStamped RF_defaultNextPosition_, RH_defaultNextPosition_, LH_defaultNextPosition_, LF_defaultNextPosition_;
    geometry_msgs::PointStamped RF_nominalNextPosition_, RH_nominalNextPosition_, LH_nominalNextPosition_, LF_nominalNextPosition_;
    geometry_msgs::PointStamped RF_centroidNextPosition_, RH_centroidNextPosition_, LH_centroidNextPosition_, LF_centroidNextPosition_;
    geometry_msgs::PointStamped RF_optNextPosition_, RH_optNextPosition_, LH_optNextPosition_, LF_optNextPosition_;

    geometry_msgs::Vector3 RF_DefaultFootholdPosition_;
    geometry_msgs::Vector3 RH_DefaultFootholdPosition_;
    geometry_msgs::Vector3 LH_DefaultFootholdPosition_;
    geometry_msgs::Vector3 LF_DefaultFootholdPosition_;

    // 足端搜索区域对应的矩形
    grid_map::Polygon RF_polygon_, RH_polygon_, LH_polygon_, LF_polygon_;
    grid_map::Polygon RF_nominalPolygon_, RH_nominalPolygon_, LH_nominalPolygon_, LF_nominalPolygon_;
    grid_map::Polygon RF_centroidPolygon_, RH_centroidPolygon_, LH_centroidPolygon_, LF_centroidPolygon_;
    grid_map::Polygon RF_optPolygon_, RH_optPolygon_, LH_optPolygon_, LF_optPolygon_;


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
    bool debug_; // 1级调试标志(只打印必要信息)
    bool debug2_; // 2级调试标志（打印部分信息）
    bool debug3_; // 3级调试标志（打印所有信息）
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

    // 落脚点marker透明度
    double alpha_;

    // 落脚点高度修正值，用于将落脚点在地面以上显示
    double h_;


    //! 4个足端默认落脚点相对于其构成的几何中心的位置偏差量（即相对于本体坐标系下的默认落脚点位置）
    float RF_defaultBiasX_;
    float RF_defaultBiasY_;

    float RH_defaultBiasX_;
    float RH_defaultBiasY_;

    float LH_defaultBiasX_;
    float LH_defaultBiasY_;

    float LF_defaultBiasX_;
    float LF_defaultBiasY_;

    // ************* gridmap信息
    struct GridMapInfo
    {
        double x;
        double y;
        double length; // x
        double width; // y
        int row; // x
        int col; // y
        double resolution;
        std::string frameID;
    };
    GridMapInfo gridMapInfo_;


    //! laikago运动学参数
    struct LaikagoKinematics
    {
        float length; // 机身外围长
        float width; // 机身外围宽
        float l1; // hip到thigh长
        float l2; // 大腿长
        float l3; // 小腿长

        float lengthBase; // 前后hip间距
        float widthBase; // 左右hip间距
    };
    LaikagoKinematics laikagoKinematics_;

    //! laikago步态参数
    double gaitCycle_;

    bool RF_FIRST_; //决定了默认trot落脚点等腰梯形的形状，true-右腿为底边，false-左腿为底边


    // ************* 等腰梯形几何参数
    struct ISOS{ //isosceles trapezoid
        // 长短边
        double longEdge;
        double shortEdge;

        //（等腰梯形长边-短边）/2
        float skew;   

        // 几何中心
        Eigen::Vector2d center;

        // 最小外包围矩形
        double length;
        double width;      

        /* 一些几何关系结论：
            (1) skew = steplength/4;
            (2) 最优等腰梯形：一次对脚移动的cog距离 = 2×skew = steplength/2;
        */   
    };
    ISOS isos_;


    // ************* 足端矩形搜索区域几何参数
    struct FootSearchRect{
        double length;
        double width;
        double row; // 对应length
        double col; // 对应width
        Eigen::Vector2d center;
    };
    FootSearchRect footSearchRect_;



    // 全局落脚点规划结果
    struct GlobalFootholds
    {
        std::vector<Eigen::VectorXd> defaultFootholds;
        std::vector<Eigen::VectorXd> nominalFootholds;
        std::vector<Eigen::VectorXd> centroidFootholds;
        std::vector<Eigen::VectorXd> optFootholds;
    };
    GlobalFootholds globalFootholdsResult_;


    // ************* 落脚点规划算法
    enum class PlanMethod{
        NominalBased,
        CentroidBased,
        EnergyBased
    };


    // ************* 落脚点效果量化评估指标
    struct FootholdsKPI{
        // 基于默认落脚点约束的落脚点
        std::vector<double> cogSpeed_nominal; // cog前进方向线速度
        std::vector<double> feetDistance_nominal;    // 对脚间距

        // 基于地形中心约束的落脚点
        std::vector<double> cogSpeed_centroid;
        std::vector<double> feetDistance_centroid;       

        // 最优化的落脚点
        std::vector<double> cogSpeed_opt;
        std::vector<double> feetDistance_opt;            
    };
    FootholdsKPI footholdsKPI_;

    bool manulStepSize_; // nlopt优化是否进行人工设置步长
    double stepSize_; // 步长
    bool useInequalityConstraits_; // nlopt优化是否包含非等式约束



    //! 来自其他节点的参数
    std::string mapFrame_;


};

}
