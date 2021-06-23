/*
    Author: chenlu
    Version: v3.0 
    Update: 2020-0802
    Email: chenlucqu@gmail.com
    Institute: AIRS
*/

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>

#include <cmath>
#include <math.h>
#include <nlopt.hpp>

#include "foothold_planner/FootholdPlanner.hpp"

using namespace std;

/* ******************
多目标优化：
https://www.sohu.com/a/193401702_744423

 *******************/
// ************************ NLopt ************************
/* NLopt parameters */
// 各子目标函数权重
double w1, w2, w3, w4;
// gridmap行、列方向权重
double wr, wc;
std::string nloptMethod = "LN_BOBYQA";
double lengthBase, skew;
int count_; // nlopt迭代次数
double ctol = 1e-2, xtol = 1e-4;
double mapResolution = 0.02; // gridmap分辨率
double lfCurrentRow, rhCurrentRow;

// 约束条件
// struct ConstraintData{
//     double a;
// };
// ConstraintData constraintData_;

// 约束区间范围
double t1, t2, t3, t4; // 单位：地图行列索引值
double t11 = t1, t22 = t2, t33 = t3, t44 =t4; // 单位：欧式距离m
double hip_lower_scale = 0.9, hip_upper_scale = 1.1; // 对脚hip间距约束区间
double skew_lower_scale = 0.8, skew_upper_scale = 1.2; // 一次对脚迈步的cog移动距离约束区间

std::vector<int> nominalIndex(8); // 顺序：LF,RH,RF,LH
std::vector<int> centroidIndex(8); // 顺序：LF,RH,RF,LH

// double FootholdPlanner::nloptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
double nloptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    // https://nlopt.readthedocs.io/en/latest/NLopt_Tutorial/#example-in-cc

    ++count_;

    return (
            w1*( wr*(pow(x[0]-nominalIndex[0] ,2.0)) + wc*(pow(x[1]-nominalIndex[1] ,2.0)) + 
                 wr*(pow(x[2]-nominalIndex[2] ,2.0)) + wc*(pow(x[3]-nominalIndex[3] ,2.0)) +
                 wr*(pow(x[4]-nominalIndex[4] ,2.0)) + wc*(pow(x[5]-nominalIndex[5] ,2.0)) +
                 wr*(pow(x[6]-nominalIndex[6] ,2.0)) + wc*(pow(x[7]-nominalIndex[7] ,2.0)) ) +
            w2*( wr*(pow(x[0]-centroidIndex[0] ,2.0)) + wc*(pow(x[1]-centroidIndex[1] ,2.0)) + 
                 wr*(pow(x[2]-centroidIndex[2] ,2.0)) + wc*(pow(x[3]-centroidIndex[3] ,2.0)) +
                 wr*(pow(x[4]-centroidIndex[4] ,2.0)) + wc*(pow(x[5]-centroidIndex[5] ,2.0)) +
                 wr*(pow(x[6]-centroidIndex[6] ,2.0)) + wc*(pow(x[7]-centroidIndex[7] ,2.0)) ) +  
            w3*( pow((x[0]-x[2]) - lengthBase/mapResolution ,2.0) +
                 pow((x[4]-x[6]) - lengthBase/mapResolution ,2.0) ) +     
            w4*( pow(abs(0.5*(x[0]-x[2]) - 0.5*(x[4]-x[6])) - 2*skew/mapResolution ,2.0) +
                 pow(abs(0.5*(x[4]-x[6]) - 0.5*(lfCurrentRow - rhCurrentRow)) - 2*skew/mapResolution ,2.0) )              
            );      
 
/*     return (
            w1*( wr*(abs(x[0]-nominalIndex[0])) + wc*(abs(x[1]-nominalIndex[1])) + 
                 wr*(abs(x[2]-nominalIndex[2])) + wc*(abs(x[3]-nominalIndex[3])) +
                 wr*(abs(x[4]-nominalIndex[4])) + wc*(abs(x[5]-nominalIndex[5])) +
                 wr*(abs(x[6]-nominalIndex[6])) + wc*(abs(x[7]-nominalIndex[7])) ) +
            w2*( wr*(abs(x[0]-centroidIndex[0])) + wc*(abs(x[1]-centroidIndex[1])) + 
                 wr*(abs(x[2]-centroidIndex[2])) + wc*(abs(x[3]-centroidIndex[3])) +
                 wr*(abs(x[4]-centroidIndex[4])) + wc*(abs(x[5]-centroidIndex[5])) +
                 wr*(abs(x[6]-centroidIndex[6])) + wc*(abs(x[7]-centroidIndex[7])) ) +  
            w3*( abs(abs(x[0]-x[2]) - lengthBase/mapResolution) +
                 abs(abs(x[4]-x[6]) - lengthBase/mapResolution) ) +     
            w4*( abs(abs(0.5*abs(x[0]-x[2]) - 0.5*abs(x[4]-x[6])) - 2*skew/mapResolution) +
                 abs(abs(0.5*abs(x[4]-x[6]) - 0.5*abs(lfCurrentRow - rhCurrentRow)) - 2*skew/mapResolution) )              
            );   */     
}

/* NLopt inequality constraints */
// (LF, RH)对脚hip间距约束下限
double nloptConstraint1(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    // constraintData_ *d = reinterpret_cast<constraintData_*>(data);

    // double a = d->a, b = d->b;

    // if (!grad.empty()) {
    //     grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
    //     grad[1] = -1.0;
    // }

    // return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);

    return ( t1 - (x[0] - x[2]) );
}

// (LF, RH)对脚hip间距约束上限
double nloptConstraint2(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( (x[0] - x[2]) - t2 );
}

// (RF, LH)对脚hip间距约束下限
double nloptConstraint3(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( t1 - (x[4] - x[6]) );
}

// (RF, LH)对脚hip间距约束上限
double nloptConstraint4(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( (x[4] - x[6]) - t2 );
}

// 一次对脚迈步的cog移动距离约束下限
double nloptConstraint5(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( t3 - 0.5*abs( (x[0] - x[2]) - (x[4] - x[6]) ) );
}

// 一次对脚迈步的cog移动距离约束上限
double nloptConstraint6(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( 0.5*abs( (x[0] - x[2]) - (x[4] - x[6]) ) - t4 );
}

// 一次对脚迈步的cog移动距离约束下限
double nloptConstraint7(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( t3 - 0.5*abs( (x[4] - x[6]) - (lfCurrentRow - rhCurrentRow) ) );
}

// 一次对脚迈步的cog移动距离约束上限
double nloptConstraint8(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    return ( 0.5*abs( (x[4] - x[6]) - (lfCurrentRow - rhCurrentRow) ) - t4 );
}



namespace foothold_planner
{
// 落脚点marks
visualization_msgs::Marker RF_points, RH_points, LH_points, LF_points; // default footholds
visualization_msgs::Marker RF_sphere_list, RH_sphere_list, LH_sphere_list, LF_sphere_list; // nominal footholds
visualization_msgs::Marker RF_cube_list, RH_cube_list, LH_cube_list, LF_cube_list; // centroid footholds
visualization_msgs::MarkerArray RF_cylinderArray, RH_cylinderArray, LH_cylinderArray, LF_cylinderArray; // opt footholds
visualization_msgs::Marker initial_RF_cube_list, initial_RH_cube_list, initial_LH_cube_list, initial_LF_cube_list; // initial stance footholds

geometry_msgs::Point RF_currentPosition, RH_currentPosition, LH_currentPosition, LF_currentPosition;

// foothold_planner_msgs::Foothold footholdResult;
foothold_planner_msgs::Foothold footholdResult_nominal, footholdResult_centroid, footholdResult_opt;

// 每个步态周期内4个足端落脚点几何中心位置对应的轨迹
nav_msgs::Path nominalFeetCenterPath;
nav_msgs::Path centroidFeetCenterPath;
nav_msgs::Path optFeetCenterPath;



// // NLopt优化器：算法，自变量数目
// nlopt::opt opt(nlopt::LD_MMA, 8);

            // static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) 
            // {
            //     return (*reinterpret_cast<FootholdPlanner*>(data))(x, grad); 
            // }

FootholdPlanner::FootholdPlanner(ros::NodeHandle& node)
    : node_(node)
{
    ROS_INFO("Node foothold_planner started.");

    readParameters();

    gridmapSub_ = node_.subscribe("/traversability_estimation/traversability_map", 20, &FootholdPlanner::gridmapCallback, this);

    // resultsPub_ - node_advertise<>("footholds", 20);
    RF_searchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RF_search_polygon", 10);
    RH_searchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RH_search_polygon", 10);
    LH_searchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LH_search_polygon", 10);
    LF_searchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LF_search_polygon", 10);
    RF_nextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RF_next_search_polygon", 10);
    RH_nextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RH_next_search_polygon", 10);
    LH_nextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LH_next_search_polygon", 10);
    LF_nextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LF_next_search_polygon", 10);

    RF_centroidSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RF_centroid_search_polygon", 10);
    RH_centroidSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RH_centroid_search_polygon", 10);
    LH_centroidSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LH_centroid_search_polygon", 10);
    LF_centroidSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LF_centroid_search_polygon", 10);
    RF_centroidNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RF_centroid_next_search_polygon", 10);
    RH_centroidNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RH_centroid_next_search_polygon", 10);
    LH_centroidNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LH_centroid_next_search_polygon", 10);
    LF_centroidNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LF_centroid_next_search_polygon", 10);

    RF_optSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RF_opt_search_polygon", 10);
    RH_optSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RH_opt_search_polygon", 10);
    LH_optSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LH_opt_search_polygon", 10);
    LF_optSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LF_opt_search_polygon", 10);
    RF_optNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RF_opt_next_search_polygon", 10);
    RH_optNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("RH_opt_next_search_polygon", 10);
    LH_optNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LH_opt_next_search_polygon", 10);
    LF_optNextSearchPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("LF_opt_next_search_polygon", 10);

    gaitMapPolygonPub_ = node_.advertise<geometry_msgs::PolygonStamped>("gait_map_polygon", 10);
    nominalGlobalFootholdsPub_ = node_.advertise<foothold_planner_msgs::GlobalFootholds>("global_footholds", 1);
    centroidGlobalFootholdsPub_ = node_.advertise<foothold_planner_msgs::GlobalFootholds>("global_footholds_centroid", 1);
    optGlobalFootholdsPub_ = node_.advertise<foothold_planner_msgs::GlobalFootholds>("global_footholds_opt", 1);


    globalFootholdsPointMarkerPub_ = node_.advertise<visualization_msgs::Marker>("global_footholds_default_marker", 10);
    globalFootholdsSphereMarkerPub_ = node_.advertise<visualization_msgs::Marker>("global_footholds_marker", 10);
    globalFootholdsCubeMarkerPub_ = node_.advertise<visualization_msgs::Marker>("global_footholds_centroid_marker", 10);
    globalFootholdsCylinderMarkerPub_ = node_.advertise<visualization_msgs::MarkerArray>("global_footholds_opt_marker", 10);
    initialFootholdMarkerPub_ = node_.advertise<visualization_msgs::Marker>("initial_footholds_marker", 10);

    // 轨迹
    nominalFeetCenterPathPub_ = node_.advertise<nav_msgs::Path>("nominal_feet_center_path", 1);
    centroidFeetCenterPathPub_ = node_.advertise<nav_msgs::Path>("centroid_feet_center_path", 1);
    //
    gaitMapPub_ = node_.advertise<grid_map_msgs::GridMap>("gait_map", 1);

    
    globalFootholdPlanService_ = node_.advertiseService("plan_global_footholds", &FootholdPlanner::globalFootholdPlan, this);

    initialize();
}


FootholdPlanner::~FootholdPlanner(){
    node_.shutdown();
}


bool FootholdPlanner::readParameters(){
    node_.param("debug", debug_, true);
    node_.param("debug2", debug2_, false);
    node_.param("debug3", debug3_, false);

    node_.param("checkDefaultFoothold_Debug", checkDefaultFoothold_Debug_, false);

    node_.param("footRadius", footRadius_, float(0.03));

    node_.param("defaultFootholdThreshold", defaultFootholdThreshold_, float(0.7));
    node_.param("candidateFootholdThreshold", candidateFootholdThreshold_, float(0.7));

    node_.param("gaitCycleNum", gaitCycleNum_, 1);
    node_.param("searchRadius", searchRadius_, float(0.1));
    node_.param("stepLength", stepLength_, float(0.2));

    node_.param("RF_FIRST", RF_FIRST_, false);


    node_.param("RF_defaultBiasX", RF_defaultBiasX_, float(0.3));
    node_.param("RF_defaultBiasY", RF_defaultBiasY_, float(-0.2));
    RF_defaultBias_.point.x = RF_defaultBiasX_;
    RF_defaultBias_.point.y = RF_defaultBiasY_;
    node_.param("RH_defaultBiasX", RH_defaultBiasX_, float(-0.3));
    node_.param("RH_defaultBiasY", RH_defaultBiasY_, float(-0.2));
    RH_defaultBias_.point.x = RH_defaultBiasX_;
    RH_defaultBias_.point.y = RH_defaultBiasY_;    
    node_.param("LH_defaultBiasX", LH_defaultBiasX_, float(-0.3));
    node_.param("LH_defaultBiasY", LH_defaultBiasY_, float(0.2));
    LH_defaultBias_.point.x = LH_defaultBiasX_;
    LH_defaultBias_.point.y = LH_defaultBiasY_;      
    node_.param("LF_defaultBiasX", LF_defaultBiasX_, float(0.3));
    node_.param("LF_defaultBiasY", LF_defaultBiasY_, float(0.2));
    LF_defaultBias_.point.x = LF_defaultBiasX_;
    LF_defaultBias_.point.y = LF_defaultBiasY_;   

    //! laikago_kinematics
    node_.param("laikago_kinematics/length", laikagoKinematics_.length, float(0.4387));
    node_.param("laikago_kinematics/width", laikagoKinematics_.width, float(0.175));
    node_.param("laikago_kinematics/l1", laikagoKinematics_.l1, float(0.037));
    node_.param("laikago_kinematics/l2", laikagoKinematics_.l2, float(0.25));
    node_.param("laikago_kinematics/l3", laikagoKinematics_.l3, float(0.25));
    node_.param("laikago_kinematics/skewLength", isos_.skew, float(0.1));  

    // robot initial standce position(map frame).
    // node_.param("initial_position/x", initialPose_.point.x, float(0.0));  
    // node_.param("initial_position/y", initialPose_.point.y, float(0.0));  
    // node_.param("initial_position/z", initialPose_.point.z, float(0.0));  
    node_.param("initial_position/x", initialPose_[0], double(0.0));  
    node_.param("initial_position/y", initialPose_[1], double(0.0));  
    node_.param("initial_position/z", initialPose_[2], double(0.0));  

    // nlopt优化参数
    node_.param("nlopt/w1", w1, double(1));
    node_.param("nlopt/w2", w2, double(1));
    node_.param("nlopt/w3", w3, double(1));
    node_.param("nlopt/w4", w4, double(1));
    node_.param("nlopt/wr", wr, double(1));
    node_.param("nlopt/wc", wc, double(1));
    node_.param("nlopt/method", nloptMethod, std::string("LN_COBYLA"));
    node_.param("nlopt/manulStepSize", manulStepSize_, bool(false));
    node_.param("nlopt/useInequalityConstraits", useInequalityConstraits_, bool(false));
    node_.param("nlopt/stepSize", stepSize_, double(1.0));
    

    //! 从其他节点获取的参数
    node_.param("/elevation_mapping/map_frame_id", mapFrame_, string("odom"));
    
    return true;
}


bool FootholdPlanner::initialize(){
    ROS_INFO("Node foothold_planner initilizing ...");

    //! ************* 初始化变量
    defaultFootholdIsvalid_ = false;
    candidateFootholdIsvalid_ = false;
    RF_footholdValidation_ = false;
    RH_footholdValidation_ = false;
    LH_footholdValidation_ = false;
    LF_footholdValidation_ = false;

    nominalGlobalFootholdsMsg_.header.frame_id = mapFrame_;
    nominalGlobalFootholdsMsg_.gait_cycles = gaitCycleNum_;

    //
    gaitCycle_ = 1.0;

    alpha_ = 0.7;

    h_ = 0.01;


    //! 计算轴距(前后hip间距)、轮距(左右hip间距)
    laikagoKinematics_.lengthBase = laikagoKinematics_.length;
    laikagoKinematics_.widthBase = laikagoKinematics_.width + laikagoKinematics_.l1*2;


    /* 初始位置 */
    // 机器人四个足端初始站立位置（世界坐标系），呈一个矩形，假设COG初始位置在世界坐标系原点，4个足端在对应HIP正下方
    RF_initialPosition_.header.frame_id = RH_initialPosition_.header.frame_id 
                                                = LH_initialPosition_.header.frame_id 
                                                = LF_initialPosition_.header.frame_id 
                                                = mapFrame_;
    RF_initialPosition_.point.x = laikagoKinematics_.lengthBase*0.5;
    RF_initialPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
    RF_initialPosition_.point.z = 0;

    RH_initialPosition_.point.x = -laikagoKinematics_.lengthBase*0.5;
    RH_initialPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
    RH_initialPosition_.point.z = 0;    

    LH_initialPosition_.point.x = -laikagoKinematics_.lengthBase*0.5;
    LH_initialPosition_.point.y = laikagoKinematics_.widthBase*0.5;
    LH_initialPosition_.point.z = 0;    

    LF_initialPosition_.point.x = laikagoKinematics_.lengthBase*0.5;
    LF_initialPosition_.point.y = laikagoKinematics_.widthBase*0.5;
    LF_initialPosition_.point.z = 0;      

    // 加上用户设定的起始位置（相对于地图原点的偏移量）
    RF_initialPosition_.point.x += initialPose_[0];
    RF_initialPosition_.point.y += initialPose_[1];
    RF_initialPosition_.point.z += initialPose_[2];
    RH_initialPosition_.point.x += initialPose_[0];
    RH_initialPosition_.point.y += initialPose_[1];
    RH_initialPosition_.point.z += initialPose_[2];
    LH_initialPosition_.point.x += initialPose_[0];
    LH_initialPosition_.point.y += initialPose_[1];
    LH_initialPosition_.point.z += initialPose_[2];  
    LF_initialPosition_.point.x += initialPose_[0];
    LF_initialPosition_.point.y += initialPose_[1];
    LF_initialPosition_.point.z += initialPose_[2];
    // publishInitialFootholdMarkers(RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_, footRadius_*1, alpha_);


    // 获取足端矩形搜索区域参数
    footSearchRect_.length = searchRadius_ * 2;
    footSearchRect_.width = searchRadius_;

    
    
    // trot行走过程：获取等腰梯形几何参数
    // isos_.longEdge = laikagoKinematics_.lengthBase + isos_.skew; //BUG!
    isos_.longEdge = laikagoKinematics_.lengthBase + isos_.skew*2;
    isos_.shortEdge = laikagoKinematics_.widthBase;
    isos_.length = isos_.longEdge + footSearchRect_.length;
    isos_.width = isos_.shortEdge + footSearchRect_.width;
    if(debug_){
        cout<<"ISOS parameters:  "
            <<"longedge="<<isos_.longEdge<<", shortedge="<<isos_.shortEdge
            <<", length="<<isos_.length<<", width="<<isos_.width<<endl;
    }
    
    
    //! 计算默认站立梯形的4个顶点坐标（相对于梯形几何中心）
    if(RF_FIRST_){     // 如果右前腿先迈出
        RF_defaultBias_.point.x = 0.5*laikagoKinematics_.lengthBase + isos_.skew;
        RF_defaultBias_.point.y = -0.5*laikagoKinematics_.widthBase;
        RH_defaultBias_.point.x = -0.5*laikagoKinematics_.lengthBase - isos_.skew;
        RH_defaultBias_.point.y = -0.5*laikagoKinematics_.widthBase;
        LH_defaultBias_.point.x = -0.5*laikagoKinematics_.lengthBase + isos_.skew;
        LH_defaultBias_.point.y = 0.5*laikagoKinematics_.widthBase;    
        LF_defaultBias_.point.x = 0.5*laikagoKinematics_.lengthBase - isos_.skew;
        LF_defaultBias_.point.y = 0.5*laikagoKinematics_.widthBase;   
    }else{      // 如果左前腿先迈 
        RF_defaultBias_.point.x = 0.5*laikagoKinematics_.lengthBase - isos_.skew;
        RF_defaultBias_.point.y = -0.5*laikagoKinematics_.widthBase;
        RH_defaultBias_.point.x = -0.5*laikagoKinematics_.lengthBase + isos_.skew;
        RH_defaultBias_.point.y = -0.5*laikagoKinematics_.widthBase;
        LH_defaultBias_.point.x = -0.5*laikagoKinematics_.lengthBase - isos_.skew;
        LH_defaultBias_.point.y = 0.5*laikagoKinematics_.widthBase;    
        LF_defaultBias_.point.x = 0.5*laikagoKinematics_.lengthBase + isos_.skew;
        LF_defaultBias_.point.y = 0.5*laikagoKinematics_.widthBase;               
    }   

    if(RF_FIRST_){
        if(debug_) cout<<"RF and LH first move."<<endl;
        RF_centroidCurrentPosition_.header.frame_id = RH_centroidCurrentPosition_.header.frame_id 
                                                    = LH_centroidCurrentPosition_.header.frame_id 
                                                    = LF_centroidCurrentPosition_.header.frame_id 
                                                    = mapFrame_;
        RF_centroidCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5 + isos_.skew;
        RF_centroidCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RF_centroidCurrentPosition_.point.z = 0;

        RH_centroidCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5 - isos_.skew;
        RH_centroidCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RH_centroidCurrentPosition_.point.z = 0;    

        LH_centroidCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5 + isos_.skew;
        LH_centroidCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LH_centroidCurrentPosition_.point.z = 0;    

        LF_centroidCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5 - isos_.skew ;
        LF_centroidCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LF_centroidCurrentPosition_.point.z = 0;         

        // -----------
        RF_nominalCurrentPosition_.header.frame_id = RH_nominalCurrentPosition_.header.frame_id 
                                                    = LH_nominalCurrentPosition_.header.frame_id 
                                                    = LF_nominalCurrentPosition_.header.frame_id 
                                                    = mapFrame_;
        RF_nominalCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5;
        RF_nominalCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RF_nominalCurrentPosition_.point.z = 0;

        RH_nominalCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5;
        RH_nominalCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RH_nominalCurrentPosition_.point.z = 0;    

        LH_nominalCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5;
        LH_nominalCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LH_nominalCurrentPosition_.point.z = 0;    

        LF_nominalCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5;
        LF_nominalCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LF_nominalCurrentPosition_.point.z = 0;           
    }else{
        if(debug_) cout<<"LF and RH first move."<<endl;
        RF_centroidCurrentPosition_.header.frame_id = RH_centroidCurrentPosition_.header.frame_id 
                                                    = LH_centroidCurrentPosition_.header.frame_id 
                                                    = LF_centroidCurrentPosition_.header.frame_id 
                                                    = mapFrame_;
        RF_centroidCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5 - isos_.skew;
        RF_centroidCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RF_centroidCurrentPosition_.point.z = 0;

        RH_centroidCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5 + isos_.skew;
        RH_centroidCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RH_centroidCurrentPosition_.point.z = 0;    

        LH_centroidCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5 - isos_.skew;
        LH_centroidCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LH_centroidCurrentPosition_.point.z = 0;    

        LF_centroidCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5 + isos_.skew ;
        LF_centroidCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LF_centroidCurrentPosition_.point.z = 0;   

        // -----------
        RF_nominalCurrentPosition_.header.frame_id = RH_nominalCurrentPosition_.header.frame_id 
                                                    = LH_nominalCurrentPosition_.header.frame_id 
                                                    = LF_nominalCurrentPosition_.header.frame_id 
                                                    = mapFrame_;
        RF_nominalCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5;
        RF_nominalCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RF_nominalCurrentPosition_.point.z = 0;

        RH_nominalCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5;
        RH_nominalCurrentPosition_.point.y = -laikagoKinematics_.widthBase*0.5;
        RH_nominalCurrentPosition_.point.z = 0;    

        LH_nominalCurrentPosition_.point.x = -laikagoKinematics_.lengthBase*0.5;
        LH_nominalCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LH_nominalCurrentPosition_.point.z = 0;    

        LF_nominalCurrentPosition_.point.x = laikagoKinematics_.lengthBase*0.5;
        LF_nominalCurrentPosition_.point.y = laikagoKinematics_.widthBase*0.5;
        LF_nominalCurrentPosition_.point.z = 0;           
    }



    // -----------
    RF_currentPosition_.header.frame_id = RH_currentPosition_.header.frame_id 
                                        = LH_currentPosition_.header.frame_id 
                                        = LF_currentPosition_.header.frame_id 
                                        = mapFrame_;    
    RF_currentPosition_.point.x = 0.3;
    RF_currentPosition_.point.y = -0.2;
    RF_currentPosition_.point.z = 0;

    RH_currentPosition_.point.x = -0.3;
    RH_currentPosition_.point.y = -0.2;
    RH_currentPosition_.point.z = 0;    

    LH_currentPosition_.point.x = -0.3;
    LH_currentPosition_.point.y = 0.2;
    LH_currentPosition_.point.z = 0;    

    LF_currentPosition_.point.x = 0.5;
    LF_currentPosition_.point.y = 0.2;
    LF_currentPosition_.point.z = 0;       



    //! trot步态四个足端默认位置（世界坐标系）
    RF_DefaultFootholdPosition_.x = 0.3;
    RF_DefaultFootholdPosition_.y = -0.2;
    RF_DefaultFootholdPosition_.z = 0;

    RH_DefaultFootholdPosition_.x = -0.3;
    RH_DefaultFootholdPosition_.y = -0.2;
    RH_DefaultFootholdPosition_.z = 0;

    LH_DefaultFootholdPosition_.x = -0.3;
    LH_DefaultFootholdPosition_.y = 0.2;
    LH_DefaultFootholdPosition_.z = 0;

    LF_DefaultFootholdPosition_.x = 0.3;
    LF_DefaultFootholdPosition_.y = 0.2;
    LF_DefaultFootholdPosition_.z = 0;    

    // 
    nominalFeetCenterPath.header.frame_id = mapFrame_;
    centroidFeetCenterPath.header.frame_id = mapFrame_;     

    // NLopt ***********************************
    lengthBase = laikagoKinematics_.lengthBase;
    skew = isos_.skew;
    count_ = 0;

}


void FootholdPlanner::gridmapCallback(grid_map_msgs::GridMap msg)
{
    grid_map::GridMapRosConverter::fromMessage(msg, gridmap_);

    gridMapInfo_.x = gridmap_.getPosition()(0);
    gridMapInfo_.y = gridmap_.getPosition()(1);
    gridMapInfo_.length = gridmap_.getLength().x();
    gridMapInfo_.width = gridmap_.getLength().y();
    gridMapInfo_.row = gridmap_.getSize()(0);
    gridMapInfo_.col = gridmap_.getSize()(1);
    mapResolution = gridMapInfo_.resolution = gridmap_.getResolution();
    gridMapInfo_.frameID = gridmap_.getFrameId();    

    cout<<endl<<"Traversability GridMap info: "<<endl;
    printf("  size = %f x %f m (%i x %i cells)\n  resolution = %f m/cell\n  position = (%f, %f)m\n  frame = %s\n", 
        gridMapInfo_.length, gridMapInfo_.width,
        gridMapInfo_.row, gridMapInfo_.col,
        gridMapInfo_.resolution,
        gridMapInfo_.x, gridMapInfo_.y,
        gridMapInfo_.frameID.c_str());
    cout<<endl;


    // 更新足端矩形搜索区域信息
    footSearchRect_.row = footSearchRect_.length/mapResolution;
    footSearchRect_.col = footSearchRect_.width/mapResolution;
    if(debug_){
        cout<<"Foot search rectangle:  "
            <<"l="<<footSearchRect_.length<<", w="<<footSearchRect_.width
            <<", row="<<footSearchRect_.row
            <<", col="<<footSearchRect_.col<<endl;
    }
}


bool FootholdPlanner::globalFootholdPlan(   foothold_planner::GlobalFootholdPlan::Request& request, 
                                            foothold_planner::GlobalFootholdPlan::Response& response){
    cout<<endl<<endl;
    cout<<"===================================================================================="<<endl;
    ROS_INFO("Global footholds planning ......");

    //! 获取4个足端当前实时位置
/*     RF_currentPosition_.point = getFootCurrentPosition(RF_currentPosition);
    RH_currentPosition_.point = getFootCurrentPosition(RH_currentPosition);
    LH_currentPosition_.point = getFootCurrentPosition(LH_currentPosition);
    LF_currentPosition_.point = getFootCurrentPosition(LF_currentPosition); */

/*     RF_centroidCurrentPosition_.point = getFootCurrentPosition(RF_initialPosition_.point);
    RH_centroidCurrentPosition_.point = getFootCurrentPosition(RH_initialPosition_.point);
    LH_centroidCurrentPosition_.point = getFootCurrentPosition(LH_initialPosition_.point);
    LF_centroidCurrentPosition_.point = getFootCurrentPosition(LF_initialPosition_.point); */
    
// {0.219	,-0.124	,0.000	,-0.219	,-0.124	,0.000	,-0.219	,0.124	,0.000	,0.219	,0.124	,0.000},    


    /* 第一个步态周期要特殊处理 */
    cout<<endl<<"Setting the first gait cycle footholds ......"<<endl<<endl;
    // default footholds
    if(!setFirstGait(RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_,
                     RF_defaultCurrentPosition_, RH_defaultCurrentPosition_, LH_defaultCurrentPosition_, LF_defaultCurrentPosition_))
    {
        ROS_ERROR("Failed to set the DEFAULT first gait cycle footholds.");
        return false;
    }
    // centroid-based
    if(!setFirstGait(RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_,
                     RF_centroidCurrentPosition_, RH_centroidCurrentPosition_, LH_centroidCurrentPosition_, LF_centroidCurrentPosition_))
    {
        ROS_ERROR("Failed to set the CENTROID first gait cycle footholds.");
        return false;
    }
    // nominal-based
    if(!setFirstGait(RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_,
                     RF_nominalCurrentPosition_, RH_nominalCurrentPosition_, LH_nominalCurrentPosition_, LF_nominalCurrentPosition_))
    {
        ROS_ERROR("Failed to set the OPT first gait cycle footholds.");
        return false;
    }
    // optimization-based
    if(!setFirstGait(RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_,
                     RF_optCurrentPosition_, RH_optCurrentPosition_, LH_optCurrentPosition_, LF_optCurrentPosition_))
    {
        ROS_ERROR("Failed to set the OPT first gait cycle footholds.");
        return false;
    }


    nominalGlobalFootholdsMsg_.footholds.clear();
    gaitCycleNum_ = request.gait_cycles;
    nominalGlobalFootholdsMsg_.gait_cycles = gaitCycleNum_;


    // ################ 预处理工作
    // 1、清空存储变量
    cout<<endl<<"Clearing variables memory space ......"<<endl<<endl;
    count_ = 0;
    // (1) 清空保存全局落脚点结果的容器，即该容器只允许记录本次全局落脚点规划请求的结果
    globalFootholdsResult_.nominalFootholds.clear();
    globalFootholdsResult_.centroidFootholds.clear();
    globalFootholdsResult_.optFootholds.clear();

    footholdsKPI_.cogSpeed_nominal.clear();
    footholdsKPI_.cogSpeed_centroid.clear();
    footholdsKPI_.cogSpeed_opt.clear();
    footholdsKPI_.feetDistance_nominal.clear();
    footholdsKPI_.feetDistance_centroid.clear();
    footholdsKPI_.feetDistance_opt.clear();

    // (2) 必须清空，否则会将以前规划的落脚点也绘制出来
    RF_points.points.clear();  
    RH_points.points.clear();    
    LH_points.points.clear();   
    LF_points.points.clear();      
    RF_sphere_list.points.clear();  
    RH_sphere_list.points.clear();    
    LH_sphere_list.points.clear();   
    LF_sphere_list.points.clear();      
    RF_cube_list.points.clear();  
    RH_cube_list.points.clear();    
    LH_cube_list.points.clear();   
    LF_cube_list.points.clear(); 
    RF_cylinderArray.markers.clear();  
    RH_cylinderArray.markers.clear();    
    LH_cylinderArray.markers.clear();   
    LF_cylinderArray.markers.clear();   
    initial_RF_cube_list.points.clear();
    initial_RH_cube_list.points.clear();          
    initial_LH_cube_list.points.clear();          
    initial_LF_cube_list.points.clear();          
    // (3) 
    nominalFeetCenterPath.poses.clear();
    centroidFeetCenterPath.poses.clear();
    // (4) 让每次调用落脚点服务都从世界坐标系原点开始规划
    RF_centroidNextPosition_.point.x = RF_centroidNextPosition_.point.y 
                                     = RF_centroidNextPosition_.point.z
                                     = RH_centroidNextPosition_.point.x
                                     = RH_centroidNextPosition_.point.y
                                     = RH_centroidNextPosition_.point.z
                                     = LH_centroidNextPosition_.point.x
                                     = LH_centroidNextPosition_.point.y
                                     = LH_centroidNextPosition_.point.z
                                     = LF_centroidNextPosition_.point.x
                                     = LF_centroidNextPosition_.point.y
                                     = LF_centroidNextPosition_.point.z
                                     = 0;     
    RF_nominalNextPosition_.point.x = RF_nominalNextPosition_.point.y 
                                     = RF_nominalNextPosition_.point.z
                                     = RH_nominalNextPosition_.point.x
                                     = RH_nominalNextPosition_.point.y
                                     = RH_nominalNextPosition_.point.z
                                     = LH_nominalNextPosition_.point.x
                                     = LH_nominalNextPosition_.point.y
                                     = LH_nominalNextPosition_.point.z
                                     = LF_nominalNextPosition_.point.x
                                     = LF_nominalNextPosition_.point.y
                                     = LF_nominalNextPosition_.point.z
                                     = 0;                                                                                                        

    // 2、
    cout<<endl<<"Saving the initial standce footholds to global results as the first gait cycle ......"<<endl<<endl;
    // 将初始位置落脚点作为第一组数据保存到全局落脚点中
    Eigen::VectorXd nominalFootholds = Eigen::VectorXd(12);
    nominalFootholds << RF_initialPosition_.point.x, RF_initialPosition_.point.y, RF_initialPosition_.point.z,
                            RH_initialPosition_.point.x, RH_initialPosition_.point.y, RH_initialPosition_.point.z,
                            LH_initialPosition_.point.x, LH_initialPosition_.point.y, LH_initialPosition_.point.z,
                            LF_initialPosition_.point.x, LF_initialPosition_.point.y, LF_initialPosition_.point.z;
    globalFootholdsResult_.nominalFootholds.push_back(nominalFootholds);  

    footholdResult_nominal.gait_cycle_id = 0;
    nominalGlobalFootholdsMsg_.gait_cycles_succeed = 0;
    nominalGlobalFootholdsMsg_.success = false;             

    footholdResult_nominal.point = RF_initialPosition_.point;
    footholdResult_nominal.foot_id = 0;
    nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);

    footholdResult_nominal.point = RH_initialPosition_.point;
    footholdResult_nominal.foot_id = 1;
    nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);      

    footholdResult_nominal.point = LH_initialPosition_.point;
    footholdResult_nominal.foot_id = 2;
    nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);    

    footholdResult_nominal.point = LF_initialPosition_.point;
    footholdResult_nominal.foot_id = 3;
    nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal); 

    // 将初始位置落脚点作为第一组数据保存到全局落脚点中
    Eigen::VectorXd centroidFootholds = Eigen::VectorXd(12);
    centroidFootholds << RF_initialPosition_.point.x, RF_initialPosition_.point.y, RF_initialPosition_.point.z,
                            RH_initialPosition_.point.x, RH_initialPosition_.point.y, RH_initialPosition_.point.z,
                            LH_initialPosition_.point.x, LH_initialPosition_.point.y, LH_initialPosition_.point.z,
                            LF_initialPosition_.point.x, LF_initialPosition_.point.y, LF_initialPosition_.point.z;
    globalFootholdsResult_.centroidFootholds.push_back(centroidFootholds);  

    footholdResult_centroid.gait_cycle_id = 0;
    centroidGlobalFootholdsMsg_.gait_cycles_succeed = 0;
    centroidGlobalFootholdsMsg_.success = false;             

    footholdResult_centroid.point = RF_initialPosition_.point;
    footholdResult_centroid.foot_id = 0;
    centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);

    footholdResult_centroid.point = RH_initialPosition_.point;
    footholdResult_centroid.foot_id = 1;
    centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);      

    footholdResult_centroid.point = LH_initialPosition_.point;
    footholdResult_centroid.foot_id = 2;
    centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);    

    footholdResult_centroid.point = LF_initialPosition_.point;
    footholdResult_centroid.foot_id = 3;
    centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);    

    // 将初始位置落脚点作为第一组数据保存到全局落脚点中
    Eigen::VectorXd optFootholds = Eigen::VectorXd(12);
    optFootholds << RF_initialPosition_.point.x, RF_initialPosition_.point.y, RF_initialPosition_.point.z,
                            RH_initialPosition_.point.x, RH_initialPosition_.point.y, RH_initialPosition_.point.z,
                            LH_initialPosition_.point.x, LH_initialPosition_.point.y, LH_initialPosition_.point.z,
                            LF_initialPosition_.point.x, LF_initialPosition_.point.y, LF_initialPosition_.point.z;
    globalFootholdsResult_.optFootholds.push_back(optFootholds);  

    footholdResult_opt.gait_cycle_id = 0;
    optGlobalFootholdsMsg_.gait_cycles_succeed = 0;
    optGlobalFootholdsMsg_.success = false;             

    footholdResult_opt.point = RF_initialPosition_.point;
    footholdResult_opt.foot_id = 0;
    optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);

    footholdResult_opt.point = RH_initialPosition_.point;
    footholdResult_opt.foot_id = 1;
    optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);      

    footholdResult_opt.point = LH_initialPosition_.point;
    footholdResult_opt.foot_id = 2;
    optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);    

    footholdResult_opt.point = LF_initialPosition_.point;
    footholdResult_opt.foot_id = 3;
    optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);    
    // 3、
    publishInitialFootholdMarkers(RF_initialPosition_, RH_initialPosition_, LH_initialPosition_, LF_initialPosition_, footRadius_*1, alpha_);

    
    int count = 0; // 用于opt global footholds cylinder marker array显示，给每个落脚点标记不同的ID！（否则不能正确显示）
    //! 循环执行落脚点规划任务，直到达到设定的步态周期数
    for(int gaitCycleIndex = 0; gaitCycleIndex < gaitCycleNum_; ++gaitCycleIndex)
    {
        cout<<endl<<"---------------------------------------------------------------------------"<<endl;
        ROS_INFO_STREAM("NO."<<(gaitCycleIndex+1)<<" gait cyle foothold planning ......");                               


        // ------------------------------------------------ 获取默认落脚点 -----------------------------------------
        if(debug_) cout<<endl<<"################## Getting default footholds ...... ##################"<<endl;
        // geometry_msgs::PointStamped RF_footholdResult_default;
        // geometry_msgs::PointStamped RH_footholdResult_default;
        // geometry_msgs::PointStamped LH_footholdResult_default;
        // geometry_msgs::PointStamped LF_footholdResult_default;          
        getDefaultFootholds(RF_defaultCurrentPosition_, 
                            RH_defaultCurrentPosition_,
                            LH_defaultCurrentPosition_,
                            LF_defaultCurrentPosition_,
                            RF_defaultNextPosition_,
                            RH_defaultNextPosition_,
                            LH_defaultNextPosition_,
                            LF_defaultNextPosition_);


        // ------------------------------------------------ 基于地形中心约束的落脚点规划算法 -----------------------------------------
        // 获取当前步态周期4个足端对应的矩形搜索区域
        if(debug_) cout<<endl<<"################## [1].Centroid-based foothold searching ...... ############"<<endl;
        if(debug_) cout<<"Get centroid foothold search region sub grid map."<<endl;
        getFootholdSearchGridMap(   RF_centroidCurrentPosition_, 
                                    RH_centroidCurrentPosition_,
                                    LH_centroidCurrentPosition_,
                                    LF_centroidCurrentPosition_,
                                    centroidFeetCenterPath,
                                    RF_centroidNextPosition_,
                                    RH_centroidNextPosition_,
                                    LH_centroidNextPosition_,
                                    LF_centroidNextPosition_,
                                    RF_centroidPolygon_,
                                    RH_centroidPolygon_,
                                    LH_centroidPolygon_,
                                    LF_centroidPolygon_,
                                    RF_centroidSearchPolygonPub_,
                                    RH_centroidSearchPolygonPub_,
                                    LH_centroidSearchPolygonPub_,
                                    LF_centroidSearchPolygonPub_,
                                    RF_centroidNextSearchPolygonPub_,
                                    RH_centroidNextSearchPolygonPub_,
                                    LH_centroidNextSearchPolygonPub_,
                                    LF_centroidNextSearchPolygonPub_);                                      

        geometry_msgs::PointStamped RF_footholdResult_centroid;
        geometry_msgs::PointStamped RH_footholdResult_centroid;
        geometry_msgs::PointStamped LH_footholdResult_centroid;
        geometry_msgs::PointStamped LF_footholdResult_centroid;        
        if(1)
        {
            cout<<endl;
            int beginRow1, endRow1;     
            checkFootholdUseCentroidMethod(gridmap_, RF_centroidNextPosition_, RF_footholdResult_centroid, beginRow1, endRow1);
            checkFootholdUseCentroidMethod(gridmap_, RH_centroidNextPosition_, RH_footholdResult_centroid, beginRow1, endRow1);
            checkFootholdUseCentroidMethod(gridmap_, LH_centroidNextPosition_, LH_footholdResult_centroid, beginRow1, endRow1);
            checkFootholdUseCentroidMethod(gridmap_, LF_centroidNextPosition_, LF_footholdResult_centroid, beginRow1, endRow1);                                             
        }


        // ------------------------------------------------ 传统落脚点规划算法：到默认落脚点距离最近的约束 -----------------------------------------
        if(debug_) cout<<endl<<"################## [2]. Nominal-based foothold searching ...... ##################"<<endl;

        // 获取当前步态周期4个足端对应的矩形搜索区域
        if(debug_) cout<<"Get nominal foothold search region sub grid map."<<endl;
        getFootholdSearchGridMap(   RF_nominalCurrentPosition_, 
                                    RH_nominalCurrentPosition_,
                                    LH_nominalCurrentPosition_,
                                    LF_nominalCurrentPosition_,
                                    nominalFeetCenterPath,
                                    RF_nominalNextPosition_,
                                    RH_nominalNextPosition_,
                                    LH_nominalNextPosition_,
                                    LF_nominalNextPosition_,
                                    RF_nominalPolygon_,
                                    RH_nominalPolygon_,
                                    LH_nominalPolygon_,
                                    LF_nominalPolygon_,
                                    RF_searchPolygonPub_,
                                    RH_searchPolygonPub_,
                                    LH_searchPolygonPub_,
                                    LF_searchPolygonPub_,
                                    RF_nextSearchPolygonPub_,
                                    RH_nextSearchPolygonPub_,
                                    LH_nextSearchPolygonPub_,
                                    LF_nextSearchPolygonPub_);  

        geometry_msgs::PointStamped RF_footholdResult_nominal;
        geometry_msgs::PointStamped RH_footholdResult_nominal;
        geometry_msgs::PointStamped LH_footholdResult_nominal;
        geometry_msgs::PointStamped LF_footholdResult_nominal;      
        if(1)
        {
            //! 开始落脚点规划......
            cout<<endl;
            grid_map::Position center;
            center.x() = RF_centroidNextPosition_.point.x;
            center.y() = RF_centroidNextPosition_.point.y;    
            RF_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                        gridmap_, 
                                        center, 
                                        footRadius_, 
                                        searchRadius_,
                                        RF_nominalPolygon_, 
                                        std::ref(RF_footholdResult_nominal), 
                                        std::ref(RF_footholdValidation_));

            center.x() = RH_centroidNextPosition_.point.x;
            center.y() = RH_centroidNextPosition_.point.y;     
            RH_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                        gridmap_, 
                                        center, 
                                        footRadius_, 
                                        searchRadius_,
                                        RH_nominalPolygon_, 
                                        std::ref(RH_footholdResult_nominal), 
                                        std::ref(RH_footholdValidation_));

            center.x() = LH_centroidNextPosition_.point.x;
            center.y() = LH_centroidNextPosition_.point.y;  
            LH_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                        gridmap_, 
                                        center, 
                                        footRadius_, 
                                        searchRadius_,
                                        LH_nominalPolygon_, 
                                        std::ref(LH_footholdResult_nominal), 
                                        std::ref(LH_footholdValidation_));

            center.x() = LF_centroidNextPosition_.point.x;
            center.y() = LF_centroidNextPosition_.point.y;   
            LF_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                        gridmap_, 
                                        center, 
                                        footRadius_, 
                                        searchRadius_,
                                        LF_nominalPolygon_, 
                                        std::ref(LF_footholdResult_nominal), 
                                        std::ref(LF_footholdValidation_));

            //! 在此阻塞，直到4个线程都跑完了...
            RF_planThread_.join();
            RH_planThread_.join();
            LH_planThread_.join();
            LF_planThread_.join();      
        }  


        // ------------------------------------------------ 组合最优化落脚点规划算法：将各个约束加权组合构建最优化问题 -----------------------------------------
        if(debug_) cout<<endl<<"################## [3]. Optimization foothold searching ...... ##################"<<endl;        
        geometry_msgs::PointStamped RF_footholdResult_opt, RH_footholdResult_opt, LH_footholdResult_opt, LF_footholdResult_opt;
        if(1)
        {
            // STEP(1). 获取下一个步态周期gridmap子图 ==============================================================
            geometry_msgs::Point optFeetCenter;
            if(!getGaitCycleSearchGridMap(  RF_optCurrentPosition_,
                                            RH_optCurrentPosition_,
                                            LH_optCurrentPosition_,
                                            LF_optCurrentPosition_,
                                            gaitMap_,
                                            optFeetCenterPath,
                                            optFeetCenter,
                                    RF_optNextPosition_,
                                    RH_optNextPosition_,
                                    LH_optNextPosition_,
                                    LF_optNextPosition_) )  
            {
                ROS_ERROR("Failed to get gait-cycle search gridmap.");
                return false;
            }

            // 发布gridmap子图消息
            grid_map_msgs::GridMap gaitMapMsg;
            grid_map::GridMapRosConverter::toMessage(gaitMap_, gaitMapMsg);
            gaitMapPub_.publish(gaitMapMsg);

            // 发布四个足端矩形搜索区域
            getFootholdSearchGridMap(   RF_optCurrentPosition_, 
                                        RH_optCurrentPosition_,
                                        LH_optCurrentPosition_,
                                        LF_optCurrentPosition_,
                                        centroidFeetCenterPath,
                                        RF_optNextPosition_,
                                        RH_optNextPosition_,
                                        LH_optNextPosition_,
                                        LF_optNextPosition_,
                                        RF_optPolygon_,
                                        RH_optPolygon_,
                                        LH_optPolygon_,
                                        LF_optPolygon_,
                                        RF_optSearchPolygonPub_,
                                        RH_optSearchPolygonPub_,
                                        LH_optSearchPolygonPub_,
                                        LF_optSearchPolygonPub_,
                                        RF_optNextSearchPolygonPub_,
                                        RH_optNextSearchPolygonPub_,
                                        LH_optNextSearchPolygonPub_,
                                        LF_optNextSearchPolygonPub_);              

            // 获取默认落脚点对应的地图索引值
            grid_map::Index index;
            nominalIndex.clear();
            getMapIndex(gaitMap_, LF_optNextPosition_.point, index);
            nominalIndex.push_back(index.x());
            nominalIndex.push_back(index.y());
            getMapIndex(gaitMap_, RH_optNextPosition_.point, index);
            nominalIndex.push_back(index.x());
            nominalIndex.push_back(index.y());
            getMapIndex(gaitMap_, RF_optNextPosition_.point, index);
            nominalIndex.push_back(index.x());
            nominalIndex.push_back(index.y());    
            getMapIndex(gaitMap_, LH_optNextPosition_.point, index);
            nominalIndex.push_back(index.x());
            nominalIndex.push_back(index.y());    
            // if(debug2_){
            //     cout<<"Nominal foothold index: "<<endl;
            //     for(std::vector<int>::iterator it = nominalIndex.begin(); it!=nominalIndex.end(); ++it){
            //         printf("%i ", *it);
            //     }
            //     cout<<endl;
            // }               


            // STEP(2). 计算足端支撑面相关几何参数，并保存到其结构体中 ==============================================================

            // 当前足端支撑面的几何中心保存到轨迹中
            geometry_msgs::PoseStamped feetCenter;
            feetCenter.pose.position = optFeetCenter;
            optFeetCenterPath.poses.push_back(feetCenter);


            // STEP(3). 获取四个足端各自可通行区域的几何中心 ======================================================================
            geometry_msgs::PointStamped RF_footholdResult_centroid_of_opt;
            geometry_msgs::PointStamped RH_footholdResult_centroid_of_opt;
            geometry_msgs::PointStamped LH_footholdResult_centroid_of_opt;
            geometry_msgs::PointStamped LF_footholdResult_centroid_of_opt;          

            // int beginRow2, endRow2;
            Eigen::MatrixXi traversableRow = Eigen::MatrixXi(2,4); // 第1行为可通行区域首行，第2行为可通行区域尾行
            checkFootholdUseCentroidMethod(gaitMap_, RF_optNextPosition_, RF_footholdResult_centroid_of_opt, traversableRow(0,0), traversableRow(1,0));
            checkFootholdUseCentroidMethod(gaitMap_, RH_optNextPosition_, RH_footholdResult_centroid_of_opt, traversableRow(0,1), traversableRow(1,1));
            checkFootholdUseCentroidMethod(gaitMap_, LH_optNextPosition_, LH_footholdResult_centroid_of_opt, traversableRow(0,2), traversableRow(1,2));
            checkFootholdUseCentroidMethod(gaitMap_, LF_optNextPosition_, LF_footholdResult_centroid_of_opt, traversableRow(0,3), traversableRow(1,3));             
            if(debug2_){
                cout<<"Default nominal footholds: "<<endl
                    <<"--RF:"<<endl<<RF_optNextPosition_.point<<endl
                    <<"--RH:"<<endl<<RH_optNextPosition_.point<<endl       
                    <<"--LH:"<<endl<<LH_optNextPosition_.point<<endl                    
                    <<"--LF:"<<endl<<LF_optNextPosition_.point<<endl;     
                cout<<"Adjuested centroid footholds: "<<endl
                    <<"--RF:"<<endl<<RF_footholdResult_centroid_of_opt.point<<endl
                    <<"--RH:"<<endl<<RH_footholdResult_centroid_of_opt.point<<endl       
                    <<"--LH:"<<endl<<LH_footholdResult_centroid_of_opt.point<<endl                    
                    <<"--LF:"<<endl<<LF_footholdResult_centroid_of_opt.point<<endl;    

                cout<<"Traversable begin_row and end_row: "<<endl;
                for(int i=0; i<2; ++i){
                    for(int j=0; j<4; ++j){
                        printf("  %i", traversableRow(i,j));
                    }
                    cout<<endl;
                }
                                                      
            }      

            // 由几何中心坐标获取其索引
            centroidIndex.clear();
            getMapIndex(gaitMap_, LF_footholdResult_centroid_of_opt.point, index);
            centroidIndex.push_back(index.x());
            centroidIndex.push_back(index.y());
            getMapIndex(gaitMap_, RH_footholdResult_centroid_of_opt.point, index);
            centroidIndex.push_back(index.x());
            centroidIndex.push_back(index.y());
            getMapIndex(gaitMap_, RF_footholdResult_centroid_of_opt.point, index);
            centroidIndex.push_back(index.x());
            centroidIndex.push_back(index.y());    
            getMapIndex(gaitMap_, LH_footholdResult_centroid_of_opt.point, index);
            centroidIndex.push_back(index.x());
            centroidIndex.push_back(index.y());    
            if(debug2_){
                cout<<"Centroid foothold index: "<<endl;
                for(std::vector<int>::iterator it = centroidIndex.begin(); it!=centroidIndex.end(); ++it){
                    printf("%i ", *it);
                }
                cout<<endl;

                cout<<"Nominal foothold index: "<<endl;
                for(std::vector<int>::iterator it = nominalIndex.begin(); it!=nominalIndex.end(); ++it){
                    printf("%i ", *it);   
                }
                cout<<endl;
            }  

            // 获取四个足端搜索区域之可通行区域的index范围，作为优化问题的自变量有效区间范围
            Eigen::MatrixXi xBounds = Eigen::MatrixXi(2,8); // 第1行为low值，第2行为up值
            // xBounds顺序：LF(X1,X2),RH(X3,X4),RF(X5,X6),LH(X7,X8) // X为优化目标函数自变量
            // traversableRow顺序：RF(X5_begin, X5_end),RH(X3_begin, X3_end),LH(X7_begin, X7_end),LF(X1_begin, X1_end)
            // 列约束x2,x4,x6,x8
            xBounds(0,1) = xBounds(0,7) = 0; // x2=x8
            xBounds(1,1) = xBounds(1,7) = footSearchRect_.col;
            xBounds(0,3) = xBounds(0,5) = isos_.width/mapResolution - footSearchRect_.col; // x4=x6
            xBounds(1,3) = xBounds(1,5) = isos_.width/mapResolution;  
            // 行约束x1,x3,x5,x7
            xBounds(0,0) = traversableRow(0,3); // x1
            xBounds(1,0) = traversableRow(1,3);
            xBounds(0,2) = traversableRow(0,1); // x3
            xBounds(1,2) = traversableRow(1,1);
            xBounds(0,4) = traversableRow(0,0); // x5
            xBounds(1,4) = traversableRow(1,0);       
            xBounds(0,6) = traversableRow(0,2); // x7
            xBounds(1,6) = traversableRow(1,2);                 
            if(debug2_){
                cout<<"x bounds: "<<endl;
                for(int i=0; i<2; ++i){
                    for(int j=0; j<8; ++j){
                        printf("  %i", xBounds(i,j));
                    }
                    cout<<endl;
                }
            }          

              
                    
            // STEP(4). NLOPT优化 ================================================================================

            // NLopt优化器：算法，自变量数目
            // nlopt::opt opt(nlopt::LD_MMA, 8);     

            /* ****************
            https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/#local-derivative-free-optimization
            Local derivative-free optimization: 
            Of these algorithms, only COBYLA currently supports arbitrary nonlinear inequality and equality constraints; 
            the rest of them support bound-constrained or unconstrained problems only. 
            (1) COBYLA(LN_COBYLA) (Constrained Optimization BY Linear Approximations) 
            (2) BOBYQA(LN_BOBYQA) performs derivative-free bound-constrained optimization. 
                may perform poorly for objective functions that are not twice-differentiable.
            (3) NEWUOA + bound constraints(NLOPT_LN_NEWUOA, NLOPT_LN_NEWUOA_BOUND) The original algorithm 
                is specified in NLopt as NLOPT_LN_NEWUOA, and only supports unconstrained problems. 
                For bound constraints, my variant is specified as NLOPT_LN_NEWUOA_BOUND.
            
            Global optimization:
            (1) ESCH (evolutionary algorithm)(NLOPT_GN_ESCH) supports bound constraints only (no nonlinear constraints)
              **************** */     
            const char* method = nloptMethod.c_str(); // c++ string to c char 
            if(debug2_){
                cout<<endl<<"NLopt parameters:"<<endl;
                printf("NLopt method: %s\n", method);
            }         

            // nlopt::opt opt(nlopt::LN_BOBYQA, 8);     
            // nlopt::opt opt(nlopt_algorithm_from_string(method), 8);   
            nlopt::opt opt(method, 8);   
              
            // 自变量区间
            std::vector<double> lb(8); // lower
            for(int i=0; i<8; ++i){
                lb[i] = xBounds(0,i);
            }
            opt.set_lower_bounds(lb);
            std::vector<double> up(8); // upper
            for(int i=0; i<8; ++i){
                up[i] = xBounds(1,i);
            }
            opt.set_upper_bounds(up);
            if(debug2_){
                cout<<"X lower bounds: "<<endl;
                for(int i=0; i<8; ++i){
                    printf(" %f", lb[i]);
                }
                cout<<endl;

                cout<<"X upper bounds: "<<endl;
                for(int i=0; i<8; ++i){
                    printf(" %f", up[i]);
                }
                cout<<endl;        
            }

            // 目标函数
            // wc = 1 - wr;
            if(debug2_){
                printf("weight: w1=%f, w2=%f, w3=%f, w4=%f, wr=%f, wc=%f\n",
                        w1,w2,w3,w4,wr,wc);
            }
            opt.set_min_objective(nloptFunc, NULL);

            // 约束条件
            t11 = lengthBase * hip_lower_scale;
            t22 = lengthBase * hip_upper_scale;
            t33 = 2* skew * skew_lower_scale;
            t44 = 2* skew * skew_upper_scale;
            t1 = lengthBase * hip_lower_scale/mapResolution;
            t2 = lengthBase * hip_upper_scale/mapResolution;
            t3 = 2* skew * skew_lower_scale/mapResolution;
            t4 = 2* skew * skew_upper_scale/mapResolution;             
            if(debug2_){
                cout<<"lfCurrentRow="<<lfCurrentRow<<"  rhCurrentRow="<<rhCurrentRow<<endl;
                cout<<"mapResolution="<<mapResolution<<endl;
                printf("constraits: t1(hip_distance_lower)=%f, t2(hip_distance_upper)=%f, t3(cog_distance_lower)=%f, t4(cog_distance_upper)=%f\n",
                        t1,t2,t3,t4);
            }                       
            // opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
            opt.add_inequality_constraint(nloptConstraint1, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint2, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint3, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint4, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint5, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint6, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint7, NULL, ctol);
            opt.add_inequality_constraint(nloptConstraint8, NULL, ctol);            
            if(!useInequalityConstraits_){
                opt.remove_inequality_constraints();
            }

            opt.set_xtol_rel(xtol);

            // 初始值(估计),此处设置为四个足端可通行区域几何中心
            std::vector<double> x(8);
            for(int i=0; i<8; ++i){
                x[i] = centroidIndex[i];
            }

            double minf;

            // 设置步长
            if(manulStepSize_){
                cout<<"Manually set step size."<<endl;
                std::vector<double> dx(8);
                opt.get_initial_step(x, dx); // 获取算法设置的默认步长（可能不合理，需要人工重新设置）
                cout<<"dx before:"<<endl;
                for(std::vector<double>::iterator it=dx.begin(); it!=dx.end(); ++it){
                    cout<<"  "<<*it<<flush;
                }
                for(int i=0; i<8; ++i){ // 重新设置步长
                    dx[i] = stepSize_;
                }
                cout<<endl<<"dx after:"<<endl;
                for(std::vector<double>::iterator it=dx.begin(); it!=dx.end(); ++it){
                    cout<<"  "<<*it<<flush;
                }            
                opt.set_initial_step(dx);
            }


            // 开始优化
            try{
                nlopt::result result = opt.optimize(x, minf);
                if(debug_){
                    cout<<endl<<"NLopt optimization begin ..."<<endl;
                    cout<<"Found minimum at: "<<endl;
                    for(int i=0; i<8; ++i)
                    {
                        printf(" %f", x[i]);
                    }  
                    cout<<endl;
                    cout<<"minf = "<<minf<<endl;
                    cout<<"count_ = "<<count_<<endl;
                }
            }
            catch(std::exception &e) {
                std::cout << "nlopt failed: " << e.what() << std::endl;
            }       
            cout<<endl;     


            // STEP(5). 量化评估优化结果：以下各偏差值越小，说明优化结果越好 =======================================
            // 优化结果变量到各默认落脚点的距离偏差
            std::vector<double> biasNorminal(8);
            for(int i=0; i<8; ++i){
                biasNorminal[i] = x[i] - nominalIndex[i];
            }
            if(debug2_){
                cout<<"biasNorminal: "<<endl;
                for(std::vector<double>::iterator it = biasNorminal.begin(); it!=biasNorminal.end(); ++it){                    
                    cout<<setprecision(3)<<"  "<<*it<<flush;
                }
                cout<<endl;
            }

            // 到可通行区域中心的距离偏差
            std::vector<double> biasCentroid(8);
            for(int i=0; i<8; ++i){
                biasCentroid[i] = x[i] - centroidIndex[i];
            }
            if(debug2_){
                cout<<"biasCentroid: "<<endl;
                for(std::vector<double>::iterator it = biasCentroid.begin(); it!=biasCentroid.end(); ++it){
                    cout<<setprecision(3)<<"  "<<*it<<flush;
                }
                cout<<endl;
            }

            // 与前进方向hip固定间距的偏差
            std::vector<double> biasHip(2);
            biasHip[0] = abs(x[0]-x[2]) - lengthBase/mapResolution;
            biasHip[1] = abs(x[4]-x[6]) - lengthBase/mapResolution;
            if(debug2_){
                cout<<"biasHip: "<<endl;
                for(std::vector<double>::iterator it = biasHip.begin(); it!=biasHip.end(); ++it){
                    cout<<setprecision(3)<<"  "<<*it<<flush;
                }
                cout<<endl;
            }

            // 与cog移动距离的偏差
            std::vector<double> biasCog(1);
            biasCog[0] = abs(0.5*abs(x[0]-x[2]) - 0.5*abs(x[4]-x[6])) - 2*skew/mapResolution;
            if(debug2_){
                cout<<"biasCog: "<<endl;
                for(std::vector<double>::iterator it = biasCog.begin(); it!=biasCog.end(); ++it){
                    cout<<setprecision(3)<<"  "<<*it<<flush;
                }
                cout<<endl;
            }
            cout<<endl;



            // STEP(6). 提取优化后的落脚点坐标 ==============================================================
            // 从gait gridmap获取索引对应的坐标
            grid_map::Position p;
            grid_map::Index i;

            i.x() = x[0];
            i.y() = x[1];
            gaitMap_.getPosition(i, p);
            LF_footholdResult_opt.point.z = getFootholdMeanHeight(gaitMap_, p, footRadius_, h_);
            LF_footholdResult_opt.point.x = p.x();
            LF_footholdResult_opt.point.y = p.y();    
            // cout<<LF_footholdResult_opt.point<<endl;  

            i.x() = x[2];
            i.y() = x[3];
            gaitMap_.getPosition(i, p);
            RH_footholdResult_opt.point.z = getFootholdMeanHeight(gaitMap_, p, footRadius_, h_);
            RH_footholdResult_opt.point.x = p.x();
            RH_footholdResult_opt.point.y = p.y();    
            // cout<<RH_footholdResult_opt.point<<endl;   

            i.x() = x[4];
            i.y() = x[5];
            gaitMap_.getPosition(i, p);
            RF_footholdResult_opt.point.z = getFootholdMeanHeight(gaitMap_, p, footRadius_, h_);
            RF_footholdResult_opt.point.x = p.x();
            RF_footholdResult_opt.point.y = p.y();    
            // cout<<RF_footholdResult_opt.point<<endl; 

            i.x() = x[6];
            i.y() = x[7];
            gaitMap_.getPosition(i, p);
            LH_footholdResult_opt.point.z = getFootholdMeanHeight(gaitMap_, p, footRadius_, h_);
            LH_footholdResult_opt.point.x = p.x();
            LH_footholdResult_opt.point.y = p.y();    
            // cout<<LH_footholdResult_opt.point<<endl;                                                                                  
        }


        //! 规划结果，只有4个足端都可通行，才算规划成功
        footholdValidation_ = RF_footholdValidation_ & RH_footholdValidation_ & LH_footholdValidation_ & LF_footholdValidation_;
        if(debug3_){
            cout<<"footholdValidation_ = "<<footholdValidation_<<"  RF_footholdValidation_ = "<<RF_footholdValidation_<<
                "  RH_footholdValidation_ = "<<RH_footholdValidation_<<"  LH_footholdValidation_ = "<<LH_footholdValidation_<<
                "  LF_footholdValidation_ = "<<LF_footholdValidation_<<endl;
        }
        

        /* 当规划成功以后的后续处理 */
        if(footholdValidation_){
            if(debug_) cout<<"All planned next 4 footholds are traversable."<<endl;

            cout<<"**************** Global footholds evaluation metrics: ****************"<<endl;
            // ----------------------- 默认落脚点 -------------------------
            //! 本次步态周期的规划结果作为下一个步态周期的起始落脚点位置
            RF_defaultCurrentPosition_ = RF_defaultNextPosition_;
            RH_defaultCurrentPosition_ = RH_defaultNextPosition_;
            LH_defaultCurrentPosition_ = LH_defaultNextPosition_;
            LF_defaultCurrentPosition_ = LF_defaultNextPosition_;

            //! 发布全局落脚点markders           
            publishGlobalFootholdPointMarkers(RF_defaultNextPosition_, RH_defaultNextPosition_, LH_defaultNextPosition_, LF_defaultNextPosition_, 
                                              footRadius_*0.5, alpha_);


            // ----------------------- 传统落脚点规划算法结果 -------------------------
            // 计算对脚间距
            getHipDistance(RF_footholdResult_nominal, RH_footholdResult_nominal, LH_footholdResult_nominal, LF_footholdResult_nominal,
                footholdsKPI_.feetDistance_nominal);
            if(debug2_){
                cout<<"Nominal feed distance:"<<endl; 
                for(std::vector<double>::iterator it=footholdsKPI_.feetDistance_nominal.begin(); it!=footholdsKPI_.feetDistance_nominal.end(); ++it){
                    cout<<" "<<*it<<flush;
                }
            }
            // 计算cog线速度
            getCogSpeed(RF_footholdResult_nominal, RH_footholdResult_nominal, LH_footholdResult_nominal, LF_footholdResult_nominal,
                RF_nominalCurrentPosition_, RH_nominalCurrentPosition_, LH_nominalCurrentPosition_, LF_nominalCurrentPosition_,
                footholdsKPI_.cogSpeed_nominal);               
            if(debug2_){
                cout<<endl<<"Nominal cog speed:"<<endl; 
                for(std::vector<double>::iterator it=footholdsKPI_.cogSpeed_nominal.begin(); it!=footholdsKPI_.cogSpeed_nominal.end(); ++it){
                    cout<<" "<<*it<<flush;
                }
                cout<<endl;
            }    

            //! 保存到全局落脚点数组中
            footholdResult_nominal.gait_cycle_id = gaitCycleIndex;
            nominalGlobalFootholdsMsg_.gait_cycles_succeed = gaitCycleIndex + 1;
            nominalGlobalFootholdsMsg_.success = true;             

            footholdResult_nominal.point = RF_footholdResult_nominal.point;
            footholdResult_nominal.foot_id = 0;
            nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);

            footholdResult_nominal.point = RH_footholdResult_nominal.point;
            footholdResult_nominal.foot_id = 1;
            nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);      

            footholdResult_nominal.point = LH_footholdResult_nominal.point;
            footholdResult_nominal.foot_id = 2;
            nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);    

            footholdResult_nominal.point = LF_footholdResult_nominal.point;
            footholdResult_nominal.foot_id = 3;
            nominalGlobalFootholdsMsg_.footholds.push_back(footholdResult_nominal);  

            // 保存全局落脚点到本地txt
            nominalFootholds << RF_footholdResult_nominal.point.x, RF_footholdResult_nominal.point.y, RF_footholdResult_nominal.point.z,
                                 RH_footholdResult_nominal.point.x, RH_footholdResult_nominal.point.y, RH_footholdResult_nominal.point.z,
                                 LH_footholdResult_nominal.point.x, LH_footholdResult_nominal.point.y, LH_footholdResult_nominal.point.z,
                                 LF_footholdResult_nominal.point.x, LF_footholdResult_nominal.point.y, LF_footholdResult_nominal.point.z;
            globalFootholdsResult_.nominalFootholds.push_back(nominalFootholds);                                                          

            //! 发布全局落脚点markders           
            publishGlobalFootholdSphereMarkers(RF_footholdResult_nominal, RH_footholdResult_nominal, LH_footholdResult_nominal, LF_footholdResult_nominal, 
                                                footRadius_*1, alpha_);

            // 发布足端支撑面中心轨迹
            nominalFeetCenterPathPub_.publish(nominalFeetCenterPath);

            //! 本次步态周期的规划结果作为下一个步态周期的起始落脚点位置
            RF_nominalCurrentPosition_ = RF_footholdResult_nominal;       
            RH_nominalCurrentPosition_ = RH_footholdResult_nominal;       
            LH_nominalCurrentPosition_ = LH_footholdResult_nominal;       
            LF_nominalCurrentPosition_ = LF_footholdResult_nominal;                


            // ----------------------- 地形中心脚点规划算法结果 -------------------------
            // #### 计算每个步态周期的COG前进方向线速度、对脚间距，用于评估落脚点对运动稳定性影响大小 #####
            // 计算对脚间距
            getHipDistance(RF_footholdResult_centroid, RH_footholdResult_centroid, LH_footholdResult_centroid, LF_footholdResult_centroid,
                footholdsKPI_.feetDistance_centroid);
            if(debug2_){
                cout<<"Centroid feed distance:"<<endl; 
                for(std::vector<double>::iterator it=footholdsKPI_.feetDistance_centroid.begin(); it!=footholdsKPI_.feetDistance_centroid.end(); ++it){
                    cout<<" "<<*it<<flush;
                }
            }
            // 计算cog线速度
            getCogSpeed(RF_footholdResult_centroid, RH_footholdResult_centroid, LH_footholdResult_centroid, LF_footholdResult_centroid,
                RF_centroidCurrentPosition_, RH_centroidCurrentPosition_, LH_centroidCurrentPosition_, LF_centroidCurrentPosition_,
                footholdsKPI_.cogSpeed_centroid);               
            if(debug2_){
                cout<<endl<<"Centroid cog speed:"<<endl; 
                for(std::vector<double>::iterator it=footholdsKPI_.cogSpeed_centroid.begin(); it!=footholdsKPI_.cogSpeed_centroid.end(); ++it){
                    cout<<" "<<*it<<flush;
                }
                cout<<endl;
            }     

            // #### 保存结果 #####
            //! 保存到全局落脚点数组中
            footholdResult_centroid.gait_cycle_id = gaitCycleIndex;
            centroidGlobalFootholdsMsg_.gait_cycles_succeed = gaitCycleIndex + 1;
            centroidGlobalFootholdsMsg_.success = true;             

            footholdResult_centroid.point = RF_footholdResult_centroid.point;
            footholdResult_centroid.foot_id = 0;
            centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);

            footholdResult_centroid.point = RH_footholdResult_centroid.point;
            footholdResult_centroid.foot_id = 1;
            centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);      

            footholdResult_centroid.point = LH_footholdResult_centroid.point;
            footholdResult_centroid.foot_id = 2;
            centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);    

            footholdResult_centroid.point = LF_footholdResult_centroid.point;
            footholdResult_centroid.foot_id = 3;
            centroidGlobalFootholdsMsg_.footholds.push_back(footholdResult_centroid);    

            // 保存全局落脚点到本地txt
            centroidFootholds << RF_footholdResult_centroid.point.x, RF_footholdResult_centroid.point.y, RF_footholdResult_centroid.point.z,
                                 RH_footholdResult_centroid.point.x, RH_footholdResult_centroid.point.y, RH_footholdResult_centroid.point.z,
                                 LH_footholdResult_centroid.point.x, LH_footholdResult_centroid.point.y, LH_footholdResult_centroid.point.z,
                                 LF_footholdResult_centroid.point.x, LF_footholdResult_centroid.point.y, LF_footholdResult_centroid.point.z;
            globalFootholdsResult_.centroidFootholds.push_back(centroidFootholds);    


            // #### 发布结果 #####
            //! 发布全局落脚点markders           
            publishGlobalFootholdCubeMarkers(RF_footholdResult_centroid, RH_footholdResult_centroid, LH_footholdResult_centroid, LF_footholdResult_centroid, footRadius_*1, alpha_);

            // 发布足端支撑面中心轨迹
            centroidFeetCenterPathPub_.publish(centroidFeetCenterPath);

            //! 本次步态周期的规划结果作为下一个步态周期的起始落脚点位置
            RF_centroidCurrentPosition_ = RF_footholdResult_centroid;
            RH_centroidCurrentPosition_ = RH_footholdResult_centroid;
            LH_centroidCurrentPosition_ = LH_footholdResult_centroid;
            LF_centroidCurrentPosition_ = LF_footholdResult_centroid;


            // ----------------------- 最优化的脚点规划算法结果 -------------------------
            // #### 计算每个步态周期的COG前进方向线速度、对脚间距，用于评估落脚点对运动稳定性影响大小 #####
            // 计算对脚间距
            getHipDistance(RF_footholdResult_opt, RH_footholdResult_opt, LH_footholdResult_opt, LF_footholdResult_opt,
                footholdsKPI_.feetDistance_opt);
            if(debug2_){
                printf("Opt feed distance: [%f, %f]m\n", t11, t22); 
                for(std::vector<double>::iterator it=footholdsKPI_.feetDistance_opt.begin(); it!=footholdsKPI_.feetDistance_opt.end(); ++it){
                    cout<<" "<<*it<<flush;
                }
            }
            // 计算cog线速度
            getCogSpeed(RF_footholdResult_opt, RH_footholdResult_opt, LH_footholdResult_opt, LF_footholdResult_opt,
                RF_optCurrentPosition_, RH_optCurrentPosition_, LH_optCurrentPosition_, LF_optCurrentPosition_,
                footholdsKPI_.cogSpeed_opt);          
            if(debug2_){
                printf("\nOpt cog speed: [%f, %f]m/s\n", t33/(0.5*gaitCycle_), t44/(0.5*gaitCycle_)); 
                for(std::vector<double>::iterator it=footholdsKPI_.cogSpeed_opt.begin(); it!=footholdsKPI_.cogSpeed_opt.end(); ++it){
                    cout<<" "<<*it<<flush;
                }
                cout<<endl;
            }  
            cout<<"**********************************************************"<<endl<<endl;


            // #### 保存结果 #####
            cout<<"**************** Save results to log file: *****************"<<endl;
            //! 保存到全局落脚点数组中
            footholdResult_opt.gait_cycle_id = gaitCycleIndex;
            optGlobalFootholdsMsg_.gait_cycles_succeed = gaitCycleIndex + 1;
            optGlobalFootholdsMsg_.success = true;             

            footholdResult_opt.point = RF_footholdResult_opt.point;
            footholdResult_opt.foot_id = 0;
            optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);

            footholdResult_opt.point = RH_footholdResult_opt.point;
            footholdResult_opt.foot_id = 1;
            optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);      

            footholdResult_opt.point = LH_footholdResult_opt.point;
            footholdResult_opt.foot_id = 2;
            optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);    

            footholdResult_opt.point = LF_footholdResult_opt.point;
            footholdResult_opt.foot_id = 3;
            optGlobalFootholdsMsg_.footholds.push_back(footholdResult_opt);  

    
            // 保存全局落脚点到本地txt
            optFootholds << RF_footholdResult_opt.point.x, RF_footholdResult_opt.point.y, RF_footholdResult_opt.point.z,
                                 RH_footholdResult_opt.point.x, RH_footholdResult_opt.point.y, RH_footholdResult_opt.point.z,
                                 LH_footholdResult_opt.point.x, LH_footholdResult_opt.point.y, LH_footholdResult_opt.point.z,
                                 LF_footholdResult_opt.point.x, LF_footholdResult_opt.point.y, LF_footholdResult_opt.point.z;
            globalFootholdsResult_.optFootholds.push_back(optFootholds);    


            // #### 发布结果 #####
            //! 发布全局落脚点markders
            publishGlobalFootholdCylinderMarkers(RF_footholdResult_opt, RH_footholdResult_opt, LH_footholdResult_opt, LF_footholdResult_opt, 
                                                 footRadius_*1, alpha_, count);
            count = count + 4;                                  

            // 发布足端支撑面中心轨迹
            // centroidFeetCenterPathPub_.publish(centroidFeetCenterPath);

            //! 本次步态周期的规划结果作为下一个步态周期的起始落脚点位置
            RF_optCurrentPosition_ = RF_footholdResult_opt;
            RH_optCurrentPosition_ = RH_footholdResult_opt;
            LH_optCurrentPosition_ = LH_footholdResult_opt;
            LF_optCurrentPosition_ = LF_footholdResult_opt;

            // 获取左前脚与右后脚当前时刻在地图中的索引值(优化目标函数需要)
            grid_map::Position pp;
            grid_map::Index ii;
            pp.x() = LF_optCurrentPosition_.point.x; // LF
            pp.y() = LF_optCurrentPosition_.point.y;
            gaitMap_.getIndex(pp, ii);
            lfCurrentRow = ii.x();
            pp.x() = RH_optCurrentPosition_.point.x; // RH
            pp.y() = RH_optCurrentPosition_.point.y;
            gaitMap_.getIndex(pp, ii);
            rhCurrentRow = ii.x();
            
        }
        else
        {
            if(debug_) ROS_ERROR("No traversable footholds are found!");   
            nominalGlobalFootholdsMsg_.success = false;         
            // break; // 若本次步态周期规划失败，那么就停止后面其他步态的继续规划    
        }   

    } // for, 所有步态周期规划结束

    // 保存全局落脚点到本地txt
    cout<<"Saving global footholds results in log file ......"<<endl;
    if(!saveLog())
        ROS_ERROR("Failed to save centroid global footholds to txt.");         
    cout<<"**********************************************************"<<endl<<endl;

    //! 服务response.
    response.footholds = nominalGlobalFootholdsMsg_;

    //! 发布全局落脚点规划结果
    nominalGlobalFootholdsPub_.publish(nominalGlobalFootholdsMsg_);
    centroidGlobalFootholdsPub_.publish(centroidGlobalFootholdsMsg_);
    optGlobalFootholdsPub_.publish(optGlobalFootholdsMsg_);



    ROS_INFO("Global footholds plan Done.");
    cout<<"===================================================================================="<<endl;
    cout<<endl<<endl;

    return true;
}


bool FootholdPlanner::checkFootholdUseCentroidMethod(   grid_map::GridMap& gridmap,
                                                        geometry_msgs::PointStamped defaultFoothold,
                                                        geometry_msgs::PointStamped& resultFoothold,
                                                        int& traversableBeginRow,
                                                        int& traversableEndRow)
{
    grid_map::GridMap map;

    // 足端矩形搜索区域，长=搜索圆的直径，宽=搜索圆的半径
    cout<<endl<<"Get sub gridmap of the searching region ..."<<endl;
    grid_map::Length rect;
    rect.x() = searchRadius_*2;
    rect.y() = searchRadius_;

    bool isSuccess;
    grid_map::Position p;


    // STEP(1). 获取足端搜索区域对应的gridmap
    // http://docs.ros.org/kinetic/api/grid_map_core/html/classgrid__map_1_1GridMap.html#abc25d137c21d60f93b34b7b921b781a7
    p.x() = defaultFoothold.point.x;
    p.y() = defaultFoothold.point.y;
    map = gridmap.getSubmap(p, rect, isSuccess); 
    if(!isSuccess){
        ROS_ERROR("Can not get map.");
        return false;
    }
    if(debug2_){
        printf("sub gridmap length: %f * %f m.\n",
            map.getLength().x(), map.getLength().y());
    }
    // 测试用
    if(debug2_){
        grid_map::Index index;
        if(map.getIndex(p, index))
            printf("Index of sub gridmap: (%i, %i)\n", index(0), index(1));
        if(gridmap.getIndex(p, index))
            printf("Index of gridmap: (%i, %i)\n", index(0), index(1));
    }

    
    // STEP(2). 遍历矩形搜索区域
            // 2.1 先整体遍历矩形搜索区域，若所有cell均可通行，则矩形中心就是最终落脚点
    cout<<endl<<"Checking sub gridmap and get the centroid ..."<<endl;
    bool wholeRegionValid = false;
    grid_map::Matrix& data = map["traversability"];
    for(grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it){
        const int i = it.getLinearIndex();
        if(data(i) < defaultFootholdThreshold_){
            wholeRegionValid = false;
            break;
        }
        wholeRegionValid = true;    
    }

    // 定义足端矩形搜索区域行列区间范围
    /* 
        -------------<--topRow
        |XXXXXXXXXXX|<--minRow
        |XXXXXXXXXXX|<--maxRow
        |     *     |
        |     o     |
        |           |
        -------------<--bottomRow
        */
    int minRow, maxRow; // 台阶边缘对应的2行的index（从0开始计数）
    int topRow, bottomRow; // 矩形搜索区域的最顶行index和最底行index
    int leftCol, rightCol; // 矩形搜索区域的最左列index和最右列index
    // 获取矩形搜索区域尺寸
    grid_map::Size size;
    size = map.getSize();    
    if(debug2_)
        cout<<"Foot Search region size: "<<size(0) <<"  "<<size(1)<<endl;      
    // 针对每一个足端矩形搜索区域，设置其遍历行列区间范围
    topRow = 0; // gridmap行列起点(0,0)在左上角
    bottomRow = size(0) - 1;
    leftCol = 0;
    rightCol = size(1) - 1;

    if(wholeRegionValid) // 如果整个搜索区域全部可通行，那么默认落脚点就是最终落脚点
    {
        printf("The whold foot search region sub gridmap is traversable.");
        resultFoothold.point.z = getFootholdMeanHeight(gridmap_, p, footRadius_, h_);
        resultFoothold.point.x = p.x();
        resultFoothold.point.y = p.y();

        // 保存可通行区域的区间索引（首行，尾行）
        traversableBeginRow = topRow;
        traversableEndRow = bottomRow; 
        if(debug3_) printf("Traversable bigen-row and end-row on foot search gridmap: %i  %i\n", traversableBeginRow, traversableEndRow); 

        // 将足端gridmap子图索引转换为对应的步态周期gridmap图索引
        grid_map::Position p;
        grid_map::Index i, i2;

        i.x() = traversableBeginRow;
        i.y() = 1;
        map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
        gridmap.getIndex(p, i2);
        traversableBeginRow = i2.x();

        i.x() = traversableEndRow;
        i.y() = 1;
        map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
        gridmap.getIndex(p, i2);
        traversableEndRow = i2.x();        
        if(debug3_) printf("Traversable bigen-row and end-row on gait cycle gridmap: %i  %i\n", traversableBeginRow, traversableEndRow); 

    }
    else    // 2.2 如果搜索区域存在不可通行的行，那么进行以下处理，找到可通行区域（遍历4个足端搜索区域对应的grid map，获取新的所有cell全部可通行的搜索区域）
    {            
        // 逐行遍历矩形搜索区域
        minRow = 0; 
        maxRow = 0;
        grid_map::Index startIndex(0,0);
        grid_map::Index endIndex(0,size(1));        
        int k = 0; // 不可通行行数
        for(int j=0; j<(bottomRow+1) ; ++j){ // 行
            // 转到下一行
            if(debug3_) cout<<endl<<"Row j "<<j<<endl;
            startIndex(0,0) = j;
            endIndex(0,0) = j;    
            if(debug3_){
                printf("Line start index: %i %i   end index: %i %i", startIndex(0,0), startIndex(1,0), endIndex(0,0), endIndex(1,0));
            }          

            // 检查指定行
            int i = 0; // 列
            for(grid_map::LineIterator it(map, startIndex, endIndex); !it.isPastEnd(); ++it){
                // bool 1stNonvalidRow = false;
                // 如果某条水平线上的不可通行cell数量超过这条线长度的一半，则认为其为台阶边缘
                if(map.at("traversability", *it) < defaultFootholdThreshold_){
                    ++i;
                    if(debug3_) ROS_ERROR_STREAM("Column i="<<i);
                }                
            }

            // 单行线条遍历完成后进行判定：是否为全不可通行行？
            if(i > ((rightCol + 1) * 0.5)){
                if(k==0){
                    minRow = startIndex(0,0); // 记录第一次出现的全不可通行的行（台阶边缘对应的不可通行区域的顶边）
                }
                maxRow = startIndex(0,0); // 记录最后一次出现的全不可通行的行（台阶边缘对应的不可通行区域的底边）
                ++k;
            }            
        }
        if(debug3_){
            cout<<endl<<"Foot Search region gridmap checking results:"<<endl;
            cout<<"     Non traversable rows = "<<k<<endl;
            cout<<"     Non traversable begin row index= "<<minRow<<"  "<<"end row index= "<<maxRow<<endl;            
        }


        // STEP(3). 计算可通行区域几何中心
        // 根据对矩形搜索区域的遍历检查结果情况，分成以下3种情形来分别处理计算新的全可通行矩形区域，并获取其几何中心（即为最终落脚点）
        grid_map::Position newRegionCentroid; // 可通行区域几何中心坐标
        grid_map::Index newIndex; // 可通行区域几何中心索引

        // case 1: 台阶边缘出现在搜索区域上部分（从最顶边开始）
        /*
            X 不可通行区域
            * nominal foothold 
            o new region centroid foothold
            -------------<--topRow
            |XXXXXXXXXXX|<--minRow
            |XXXXXXXXXXX|<--maxRow
            |     *     |
            |     o     |
            |           |
            -------------<--bottomRow
        */
        cout<<endl;
        if(minRow == topRow && maxRow != bottomRow){
            if(debug2_) cout<<"Case 1."<<endl;

            // 计算新矩形的几何中心index
            // int newRow = ceil((minRow + bottomRow + 1) * 0.5); // bug!
            int newRow = floor((maxRow + bottomRow + 1) * 0.5);
            int newCol = floor((rightCol + 1) * 0.5);
            if(debug3_) printf("Centroid index: %i %i\n", newRow, newCol);
            newIndex.x() = newRow;
            newIndex.y() = newCol;
            if(debug2_){
                printf("Nominal index: %f %f, New centroid index: %i %i\n", 
                floor(bottomRow*0.5), floor(rightCol*0.5),
                newIndex.x(), newIndex.y());
            }      

            // 保存可通行区域的区间索引（首行，尾行）
            traversableBeginRow = maxRow + 1;
            traversableEndRow = bottomRow;     
            if(debug3_) printf("Traversable bigen-row and end-row on foot search gridmap: %i  %i\n", traversableBeginRow, traversableEndRow); 

            // 将足端gridmap子图索引转换为对应的步态周期gridmap图索引
            grid_map::Position p;
            grid_map::Index i, i2;

            i.x() = traversableBeginRow;
            i.y() = 1;
            map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
            gridmap.getIndex(p, i2);
            traversableBeginRow = i2.x();

            i.x() = traversableEndRow;
            i.y() = 1;
            map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
            gridmap.getIndex(p, i2);
            traversableEndRow = i2.x();        
            if(debug3_) printf("Traversable bigen-row and end-row on gait cycle gridmap: %i  %i\n", traversableBeginRow, traversableEndRow);              

            // 计算新矩形的几何中心的坐标
            if(!map.getPosition(newIndex, newRegionCentroid)){
                ROS_ERROR("Can not get position.");                
            }

            resultFoothold.point.z = getFootholdMeanHeight(gridmap_, newRegionCentroid, footRadius_, h_);
            resultFoothold.point.x = newRegionCentroid.x();
            resultFoothold.point.y = newRegionCentroid.y();                

            if(debug2_){
                printf("Nominal foothold: %f %f, New centroid foothold: %f %f\n", 
                    defaultFoothold.point.x, defaultFoothold.point.y,
                    newRegionCentroid.x(), newRegionCentroid.y());
            }
        }
        // case 2: 台阶边缘出现在搜索区域中间，此种情况需要特殊处理，注意！
        /*
            X 不可通行区域
            * nominal foothold 
            o new region centroid foothold
            -------------
            |           |
            |XXXXXXXXXXX|
            |XXXXX*XXXXX|
            |     o     |
            |           |
            -------------
        */        
        else if(minRow!=topRow && maxRow!=bottomRow){
            if(debug2_) cout<<"Case 2."<<endl;

            // 判断是上半部分大还是下半部分大？
            if((minRow-topRow) >= (bottomRow-maxRow)) //上半部分大，选择上半部分
            {
                // 计算新矩形的几何中心index
                int newRow = ceil(minRow * 0.5);
                // int newCol = ceil((rightCol + 1) * 0.5);
                int newCol = floor((rightCol + 0) * 0.5);
                if(debug3_) printf("Centroid index: %i %i\n", newRow, newCol);
                newIndex.x() = newRow;
                newIndex.y() = newCol;

                // 保存可通行区域的区间索引（首行，尾行）
                traversableBeginRow = topRow;
                traversableEndRow = minRow - 1;  
                if(debug3_) printf("Traversable bigen-row and end-row on foot search gridmap: %i  %i\n", traversableBeginRow, traversableEndRow); 

                // 将足端gridmap子图索引转换为对应的步态周期gridmap图索引
                grid_map::Position p;
                grid_map::Index i, i2;

                i.x() = traversableBeginRow;
                i.y() = 1;
                map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
                gridmap.getIndex(p, i2);
                traversableBeginRow = i2.x();

                i.x() = traversableEndRow;
                i.y() = 1;
                map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
                gridmap.getIndex(p, i2);
                traversableEndRow = i2.x();        
                if(debug3_) printf("Traversable bigen-row and end-row on gait cycle gridmap: %i  %i\n", traversableBeginRow, traversableEndRow);                               
            }
            else //下半部分大，选择下半部分
            {
                // 计算新矩形的几何中心index
                int newRow = floor((maxRow + bottomRow) * 0.5);
                int newCol = floor((rightCol + 0) * 0.5);
                if(debug3_) printf("Centroid index: %i %i\n", newRow, newCol);
                newIndex.x() = newRow;
                newIndex.y() = newCol;  

                // 保存可通行区域的区间索引（首行，尾行）
                traversableBeginRow = maxRow + 1;
                traversableEndRow = bottomRow;   
                if(debug3_) printf("Traversable bigen-row and end-row on foot search gridmap: %i  %i\n", traversableBeginRow, traversableEndRow); 

                // 将足端gridmap子图索引转换为对应的步态周期gridmap图索引
                grid_map::Position p;
                grid_map::Index i, i2;

                i.x() = traversableBeginRow;
                i.y() = 1;
                map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
                gridmap.getIndex(p, i2);
                traversableBeginRow = i2.x();

                i.x() = traversableEndRow;
                i.y() = 1;
                map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
                gridmap.getIndex(p, i2);
                traversableEndRow = i2.x();        
                if(debug3_) printf("Traversable bigen-row and end-row on gait cycle gridmap: %i  %i\n", traversableBeginRow, traversableEndRow);                                                    
            }
            if(debug2_){
                printf("Nominal index: %f %f, New centroid index: %i %i\n", 
                floor(bottomRow*0.5), floor(rightCol*0.5),
                newIndex.x(), newIndex.y());
            }  

            // 计算新矩形的几何中心的坐标
            if(!map.getPosition(newIndex, newRegionCentroid)){
                ROS_ERROR("Can not get position.");                
            }

            resultFoothold.point.z = getFootholdMeanHeight(gridmap_, newRegionCentroid, footRadius_, h_);
            resultFoothold.point.x = newRegionCentroid.x();
            resultFoothold.point.y = newRegionCentroid.y();                

            if(debug_){
                printf("Nominal foothold: %f %f, New centroid foothold: %f %f\n", 
                    defaultFoothold.point.x, defaultFoothold.point.y,
                    newRegionCentroid.x(), newRegionCentroid.y());
            }
        }
        // case 3: 台阶边缘出现在搜索区域下部分（从底边开始）
        /*
            X 不可通行区域
            * nominal foothold 
            o new region centroid foothold
            -------------
            |           |
            |     o     |
            |     *     |
            |XXXXXXXXXXX|
            |XXXXXXXXXXX|            
            -------------
        */        
        else if(minRow!=topRow && maxRow==bottomRow){
            if(debug2_) cout<<"Case 3."<<endl;

            // 计算新矩形的几何中心index
            int newRow = ceil(minRow * 0.5);
            int newCol = floor((rightCol + 0) * 0.5);
            if(debug3_) printf("Centroid index: %i %i\n", newRow, newCol);
            newIndex.x() = newRow;
            newIndex.y() = newCol;
            if(debug2_){
                printf("Nominal index: %f %f, New centroid index: %i %i\n", 
                floor(bottomRow*0.5), floor(rightCol*0.5),
                newIndex.x(), newIndex.y());
            }  

            // 保存可通行区域的区间索引（首行，尾行）
            traversableBeginRow = topRow;
            traversableEndRow = minRow - 1; 
            if(debug3_) printf("Traversable bigen-row and end-row on foot search gridmap: %i  %i\n", traversableBeginRow, traversableEndRow); 

            // 将足端gridmap子图索引转换为对应的步态周期gridmap图索引
            grid_map::Position p;
            grid_map::Index i, i2;

            i.x() = traversableBeginRow;
            i.y() = 1;
            map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
            gridmap.getIndex(p, i2);
            traversableBeginRow = i2.x();

            i.x() = traversableEndRow;
            i.y() = 1;
            map.getPosition(i, p);  // 先获取足端gridmap与步态周期gridmap指定索引对应的共同位置坐标
            gridmap.getIndex(p, i2);
            traversableEndRow = i2.x();        
            if(debug3_) printf("Traversable bigen-row and end-row on gait cycle gridmap: %i  %i\n", traversableBeginRow, traversableEndRow);            

            // 计算新矩形的几何中心的坐标
            if(!map.getPosition(newIndex, newRegionCentroid)){
                ROS_ERROR("Can not get position.");                
            }

            resultFoothold.point.z = getFootholdMeanHeight(gridmap_, newRegionCentroid, footRadius_, h_);
            resultFoothold.point.x = newRegionCentroid.x();
            resultFoothold.point.y = newRegionCentroid.y();                

            if(debug2_){
                printf("Nominal foothold: %f %f, New centroid foothold: %f %f\n", 
                    defaultFoothold.point.x, defaultFoothold.point.y,
                    newRegionCentroid.x(), newRegionCentroid.y());
            }
        }
    }
}



geometry_msgs::PointStamped FootholdPlanner::checkFoothold( grid_map::GridMap gridmap, 
                                                            grid_map::Position center,
                                                            float footRadius,
                                                            float searchRadius,
                                                            grid_map::Polygon polygon,
                                                            geometry_msgs::PointStamped& footholdResult,
                                                            bool& footholdValidation){
    geometry_msgs::PointStamped foothold;

    //! 检查默认落脚点
    bool defaultFootholdIsvalid = false;
    defaultFootholdIsvalid = checkDefaultFoothold(gridmap, center, footRadius);
    footholdValidation = defaultFootholdIsvalid;
    if(debug_) ROS_INFO_STREAM("Default foothold validation: "<<defaultFootholdIsvalid);

    footholdResult.point.x = center.x();
    footholdResult.point.y = center.y();

    //! 检查候选落脚点
    bool candidateFootholdIsvalid = false;
    if(!defaultFootholdIsvalid){
        candidateFootholdIsvalid = checkCandidateFoothold(gridmap, center, footRadius, searchRadius, polygon, footholdResult);
        footholdValidation = candidateFootholdIsvalid;
        if(debug_) ROS_INFO_STREAM("Candidate foothold validation: "<<candidateFootholdIsvalid);
    }

    //! 计算最终落脚点高度
    if(defaultFootholdIsvalid | candidateFootholdIsvalid){
        footholdResult.point.z = getFootholdMeanHeight(gridmap, center, footRadius, h_);
        if(debug_) ROS_INFO_STREAM("Foothold height: "<<footholdResult.point.z);
    }else{
        ROS_ERROR("Failed to plan traversable footholds!");
    }

    return foothold;
}


bool FootholdPlanner::checkDefaultFoothold( grid_map::GridMap gridmap, 
                                            grid_map::Position center,
                                            float footRadius){
  if(checkDefaultFoothold_Debug_) ROS_INFO("Checking default foothold validation ......");

  bool validation = false;  
  bool isNAN = false;
  //grid_map::Position default_foothold_center(spiralCenter.x(), spiralCenter.y()); 
      
  for(grid_map::CircleIterator iterator(gridmap, center, footRadius); !iterator.isPastEnd(); ++iterator)
  {
    if(checkDefaultFoothold_Debug_){
        ROS_INFO_STREAM( "Value at index " << (*iterator).transpose() << " = " << gridmap.at("traversability", *iterator ) );
    }      

    //if(gridmap.at("traversability", *iterator) < defaultFootholdThreshold && true == gridmap.isValid((*iterator).transpose(), "traversability"))
    if(gridmap.isValid((*iterator).transpose(), "traversability"))
    {
      if(gridmap.at("traversability", *iterator) < defaultFootholdThreshold_) 
      {
        validation = false;
        isNAN = true;

        if(checkDefaultFoothold_Debug_){
            ROS_WARN_STREAM( "Value at index " << (*iterator).transpose() << " = " << gridmap.at("traversability", *iterator ) );
            ROS_WARN_STREAM("Default cell validation = " << validation);
        }
        break;
      }

      if(false == isNAN) validation = true;      
    }
    else
    {
        if(checkDefaultFoothold_Debug_){
            ROS_WARN( "Cell value not valid." );
        }         
    }

    validation = true; // all cells value is NAN        
  }
      
  return validation;
}


bool FootholdPlanner::checkCandidateFoothold(   grid_map::GridMap gridmap, 
                                                grid_map::Position spiralCenter,
                                                float footRadius, 
                                                float searchRadius,
                                                grid_map::Polygon polygon,
                                                geometry_msgs::PointStamped& footholdResult){
    if(checkDefaultFoothold_Debug_) ROS_INFO("Checking candidate foothold validation ......");
    
    bool validation = false;
    
    for(grid_map::SpiralIterator iterator(gridmap, spiralCenter, searchRadius); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position footCenter;
        gridmap.getPosition(*iterator, footCenter);

        validation = checkCirclePolygonFoothold(gridmap, footCenter, footRadius, polygon);

        if(validation){
            grid_map::Index index((*iterator).transpose());
            grid_map::Position p;
            gridmap.getPosition(index, p);
            footholdResult.point.x = p.x();
            footholdResult.point.y = p.y();       

            break;
        }
    }   

    return validation;
}


bool FootholdPlanner::checkCirclePolygonFoothold(   grid_map::GridMap gridmap, 
                                                    grid_map::Position center,
                                                    float footRadius,
                                                    grid_map::Polygon polygon){
  if(checkDefaultFoothold_Debug_) ROS_INFO("Checking circle and polygon foothold validation ......");

  bool validation = false;  
  bool isNAN = false;
      
  for(grid_map::CircleIterator iterator(gridmap, center, footRadius); !iterator.isPastEnd(); ++iterator)
  {
    if(checkDefaultFoothold_Debug_){
        ROS_INFO_STREAM( "Value at index " << (*iterator).transpose() << " = " << gridmap.at("traversability", *iterator ) );
    }      

    if(gridmap.isValid((*iterator).transpose(), "traversability"))
    {
        grid_map::Index index((*iterator).transpose());
        grid_map::Position p;
        gridmap.getPosition(index, p);      

        if(gridmap.at("traversability", *iterator) < candidateFootholdThreshold_ || false == polygon.isInside(p)) 
        {
            validation = false;
            isNAN = true;

            if(checkDefaultFoothold_Debug_){
                ROS_INFO_STREAM( "Value at index " << (*iterator).transpose() << " = " << gridmap.at("traversability", *iterator ) );
                ROS_INFO_STREAM("Default cell validation = " << validation);
            }
            break;
        }

        if(false == isNAN) validation = true;      
    }
    else
    {
        if(checkDefaultFoothold_Debug_){
            ROS_WARN( "Cell value not valid." );
        }         
    }

    validation = true; // all cells value is NAN        
  }
      
  return validation;
}


bool FootholdPlanner::getFootholdSearchGridMap( geometry_msgs::PointStamped RF_position,
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
                                                ros::Publisher LF_nextSearchPolygonPub)                                              
{
    //! 计算下一个步态周期4个足端构成的四边形几何中心
    // 当前步态周期的足端支撑面几何中心    
    geometry_msgs::Point polygonCenter;
    polygonCenter = getPolygonCenter(RF_position, RH_position, LH_position, LF_position);
    if(debug2_) cout<<"Feet center in current gait-cyle: "<<endl<<polygonCenter<<endl;
    // 当前足端支撑面的几何中心保存到轨迹中
    geometry_msgs::PoseStamped feetCenter;
    feetCenter.pose.position = polygonCenter;
    feetCenterPath.poses.push_back(feetCenter);
    // 下一个步态周期的足端支撑面几何中心
    geometry_msgs::Point polygonCenterNext;
    polygonCenterNext.x = polygonCenter.x + stepLength_;
    if(debug2_) cout<<"Feet center in next gait-cyle: "<<endl<<polygonCenterNext<<endl;


    //! 计算下一个步态周期4个足端默认位置
    RF_nextPosition = getDefaultFootholdNext(polygonCenterNext, RF_defaultBias_);
    if(debug2_) cout<<"Next RF default foothold: "<<endl<<RF_nextPosition.point<<endl;
    RH_nextPosition = getDefaultFootholdNext(polygonCenterNext, RH_defaultBias_);
    if(debug2_) cout<<"Next RH default foothold: "<<endl<<RH_nextPosition.point<<endl;
    LH_nextPosition = getDefaultFootholdNext(polygonCenterNext, LH_defaultBias_);
    if(debug2_) cout<<"Next LH default foothold: "<<endl<<LH_nextPosition.point<<endl;
    LF_nextPosition = getDefaultFootholdNext(polygonCenterNext, LF_defaultBias_);
    if(debug2_) cout<<"Next LF default foothold: "<<endl<<LF_nextPosition.point<<endl;


    //! 获取矩形搜索区域，并发布markers
    // 当前步态周期的足端搜索区域
    geometry_msgs::PolygonStamped RF_currentPolygonMsg, RH_currentPolygonMsg, LH_currentPolygonMsg, LF_currentPolygonMsg;
    grid_map::Polygon RF_currentPolygon, RH_currentPolygon, LH_currentPolygon, LF_currentPolygon;
    RF_currentPolygon = getSearchPolygon(RF_position, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(RF_currentPolygon, RF_currentPolygonMsg);
    RF_searchPolygonPub.publish(RF_currentPolygonMsg);
    RH_currentPolygon = getSearchPolygon(RH_position, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(RH_currentPolygon, RH_currentPolygonMsg);
    RH_searchPolygonPub.publish(RH_currentPolygonMsg);
    LH_currentPolygon = getSearchPolygon(LH_position, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(LH_currentPolygon, LH_currentPolygonMsg);
    LH_searchPolygonPub.publish(LH_currentPolygonMsg);
    LF_currentPolygon = getSearchPolygon(LF_position, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(LF_currentPolygon, LF_currentPolygonMsg);
    LF_searchPolygonPub.publish(LF_currentPolygonMsg);    
    // 下一个步态周期的足端搜索区域
    geometry_msgs::PolygonStamped RF_polygonMsg, RH_polygonMsg, LH_polygonMsg, LF_polygonMsg;
    RF_polygon = getSearchPolygon(RF_nextPosition, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(RF_polygon, RF_polygonMsg);
    RF_nextSearchPolygonPub.publish(RF_polygonMsg);
    RH_polygon = getSearchPolygon(RH_nextPosition, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(RH_polygon, RH_polygonMsg);
    RH_nextSearchPolygonPub.publish(RH_polygonMsg);
    LH_polygon = getSearchPolygon(LH_nextPosition, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(LH_polygon, LH_polygonMsg);
    LH_nextSearchPolygonPub.publish(LH_polygonMsg);
    LF_polygon = getSearchPolygon(LF_nextPosition, searchRadius_);
    grid_map::PolygonRosConverter::toMessage(LF_polygon, LF_polygonMsg);
    LF_nextSearchPolygonPub.publish(LF_polygonMsg); 

           
    return true;
}


bool FootholdPlanner::getDefaultFootholds( geometry_msgs::PointStamped RF_position,
                                                geometry_msgs::PointStamped RH_position,
                                                geometry_msgs::PointStamped LH_position,
                                                geometry_msgs::PointStamped LF_position,
                                                geometry_msgs::PointStamped& RF_nextPosition,
                                                geometry_msgs::PointStamped& RH_nextPosition,
                                                geometry_msgs::PointStamped& LH_nextPosition,
                                                geometry_msgs::PointStamped& LF_nextPosition)                                              
{
    //! 计算下一个步态周期4个足端构成的四边形几何中心
    // 当前步态周期的足端支撑面几何中心    
    geometry_msgs::Point polygonCenter;
    polygonCenter = getPolygonCenter(RF_position, RH_position, LH_position, LF_position);
    if(debug2_) cout<<"Feet center in current gait-cyle: "<<endl<<polygonCenter<<endl;

    // 下一个步态周期的足端支撑面几何中心
    geometry_msgs::Point polygonCenterNext;
    polygonCenterNext.x = polygonCenter.x + stepLength_;
    if(debug2_) cout<<"Feet center in next gait-cyle: "<<endl<<polygonCenterNext<<endl;


    //! 计算下一个步态周期4个足端默认位置
    RF_nextPosition = getDefaultFootholdNext(polygonCenterNext, RF_defaultBias_);
    if(debug2_) cout<<"Next RF default foothold: "<<endl<<RF_nextPosition.point<<endl;
    RH_nextPosition = getDefaultFootholdNext(polygonCenterNext, RH_defaultBias_);
    if(debug2_) cout<<"Next RH default foothold: "<<endl<<RH_nextPosition.point<<endl;
    LH_nextPosition = getDefaultFootholdNext(polygonCenterNext, LH_defaultBias_);
    if(debug2_) cout<<"Next LH default foothold: "<<endl<<LH_nextPosition.point<<endl;
    LF_nextPosition = getDefaultFootholdNext(polygonCenterNext, LF_defaultBias_);
    if(debug2_) cout<<"Next LF default foothold: "<<endl<<LF_nextPosition.point<<endl;


    //! 计算落脚点高度
    grid_map::Position center;
    center.x() = RF_nextPosition.point.x;
    center.y() = RF_nextPosition.point.y;      
    RF_nextPosition.point.z = getFootholdMeanHeight(gridmap_, center, footRadius_, h_);
    center.x() = RH_nextPosition.point.x;
    center.y() = RH_nextPosition.point.y;      
    RH_nextPosition.point.z = getFootholdMeanHeight(gridmap_, center, footRadius_, h_);
    center.x() = LH_nextPosition.point.x;
    center.y() = LH_nextPosition.point.y;      
    LH_nextPosition.point.z = getFootholdMeanHeight(gridmap_, center, footRadius_, h_);    
    center.x() = LF_nextPosition.point.x;
    center.y() = LF_nextPosition.point.y;      
    LF_nextPosition.point.z = getFootholdMeanHeight(gridmap_, center, footRadius_, h_);    
           
    return true;
}


bool FootholdPlanner::getGaitCycleSearchGridMap( geometry_msgs::PointStamped RF_position,
                                                geometry_msgs::PointStamped RH_position,
                                                geometry_msgs::PointStamped LH_position,
                                                geometry_msgs::PointStamped LF_position,
                                                grid_map::GridMap& gaitMap,
                                                nav_msgs::Path& feetCenterPath,
                                                geometry_msgs::Point feetCenter,
                                                geometry_msgs::PointStamped& RF_nextPosition,
                                                geometry_msgs::PointStamped& RH_nextPosition,
                                                geometry_msgs::PointStamped& LH_nextPosition,
                                                geometry_msgs::PointStamped& LF_nextPosition)                                                     
{
    // STEP(1). 计算下一个步态周期4个足端构成的四边形几何中心
    // 当前步态周期的足端支撑面几何中心    
    feetCenter = getPolygonCenter(RF_position, RH_position, LH_position, LF_position);
    if(debug2_) cout<<"Feet center in current gait-cyle: "<<endl<<feetCenter<<endl;


    // STEP(2). 获取子gridmap
    // http://docs.ros.org/kinetic/api/grid_map_core/html/classgrid__map_1_1GridMap.html#abc25d137c21d60f93b34b7b921b781a7
    
    // 下一个步态周期的足端支撑面几何中心
    geometry_msgs::Point nextFeetCenter;
    nextFeetCenter.x = feetCenter.x + stepLength_;
    if(debug2_) cout<<"Feet center in next gait-cyle: "<<endl<<nextFeetCenter<<endl;     
    grid_map::Position p; // gridmap原点(几何中心)
    p.x() = nextFeetCenter.x;
    p.y() = nextFeetCenter.y;

    grid_map::Length rect; // gridmap尺寸
    rect.x() = isos_.length;
    rect.y() = isos_.width;

    bool isSuccess = false;

    gaitMap = gridmap_.getSubmap(p, rect, isSuccess); 

    if(!isSuccess){
        return false;
    }

    if(debug_){
        ROS_INFO("Next gait cycle sub gridmap: length %f * %f m.",
            gaitMap.getLength().x(), gaitMap.getLength().y());
    }

    // 测试用
    if(1){
        grid_map::Index index;
        if(gaitMap.getIndex(p, index))
            ROS_INFO("Index of gaitMap: (%i, %i)", index(0), index(1));
        if(gridmap_.getIndex(p, index))
            ROS_INFO("Index of gridmap_: (%i, %i)", index(0), index(1));
    }


    // STEP(3). 发布子地图矩形框到rviz显示
    grid_map::Position leftup_vertex, rightup_vertex, rightdown_vertex, leftdown_vertex;
    leftup_vertex.x() = p.x() + isos_.length*0.5;
    leftup_vertex.y() = p.y() + isos_.width*0.5;
    rightup_vertex.x() = p.x() + isos_.length*0.5;
    rightup_vertex.y() = p.y() - isos_.width*0.5; 
    rightdown_vertex.x() = p.x() - isos_.length*0.5;
    rightdown_vertex.y() = p.y() - isos_.width*0.5; 
    leftdown_vertex.x() = p.x() - isos_.length*0.5;
    leftdown_vertex.y() = p.y() + isos_.width*0.5;

    grid_map::Polygon polygon;
    polygon.addVertex(leftup_vertex);
    polygon.addVertex(rightup_vertex);
    polygon.addVertex(rightdown_vertex);
    polygon.addVertex(leftdown_vertex);  
    polygon.setFrameId(mapFrame_); 

    geometry_msgs::PolygonStamped polygonMsg;
    if(debug2_) cout<<"Polygon centroid: "<<polygon.getCentroid()<<endl;
    grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);
    gaitMapPolygonPub_.publish(polygonMsg);    


    // STEP(4). 计算下一个步态周期4个足端默认位置
    RF_nextPosition = getDefaultFootholdNext(nextFeetCenter, RF_defaultBias_);
    // if(debug2_) cout<<"Next RF default foothold: "<<endl<<RF_nextPosition<<endl;
    RH_nextPosition = getDefaultFootholdNext(nextFeetCenter, RH_defaultBias_);
    // if(debug2_) cout<<"Next RH default foothold: "<<endl<<RH_nextPosition<<endl;
    LH_nextPosition = getDefaultFootholdNext(nextFeetCenter, LH_defaultBias_);
    // if(debug2_) cout<<"Next LH default foothold: "<<endl<<LH_nextPosition<<endl;
    LF_nextPosition = getDefaultFootholdNext(nextFeetCenter, LF_defaultBias_);
    // if(debug2_) cout<<"Next LF default foothold: "<<endl<<LF_nextPosition<<endl;


    // //! 获取矩形搜索区域
    // RF_polygon = getSearchPolygon(RF_nextPosition, searchRadius_);
    // RH_polygon = getSearchPolygon(RH_nextPosition, searchRadius_);
    // LH_polygon = getSearchPolygon(LH_nextPosition, searchRadius_);
    // LF_polygon = getSearchPolygon(LF_nextPosition, searchRadius_); 

    return true;
}


geometry_msgs::PointStamped FootholdPlanner::getDefaultFootholdNext(geometry_msgs::Point feetCenterNext,
                                                                    geometry_msgs::PointStamped defaultBias){
    geometry_msgs::PointStamped foothold;
    foothold.point.x = feetCenterNext.x + defaultBias.point.x;
    foothold.point.y = feetCenterNext.y + defaultBias.point.y;

    return foothold;
}


geometry_msgs::Point FootholdPlanner::getPolygonCenter( geometry_msgs::PointStamped RF_point,
                                                        geometry_msgs::PointStamped RH_point,
                                                        geometry_msgs::PointStamped LH_point,
                                                        geometry_msgs::PointStamped LF_point){
    int n = 4; 
    double x1,y1,x2,y2,x3,y3;
    double sum_x=0,sum_y=0,sum_s=0;
        
    x1 = RF_point.point.x;
    y1 = RF_point.point.y;
    x2 = RH_point.point.x;
    y2 = RH_point.point.y;
        
    for(int i=1; i<=n-2; i++)
    {
        switch(i)
        {
        case 1:
        x3 = LH_point.point.x;
        y3 = LH_point.point.y;
        break;
        case 2:
        x3 = LF_point.point.x;
        y3 = LF_point.point.y;
        break;			
        }
                
        double s=((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2.0;
        sum_x+=(x1+x2+x3)*s;
        sum_y+=(y1+y2+y3)*s;
        sum_s+=s;
        x2=x3;
        y2=y3;
    }
    
    geometry_msgs::Point polygonCenter;
    polygonCenter.x = sum_x/sum_s/3.0;
    polygonCenter.y = sum_y/sum_s/3.0;  
    // polygonCenter.z = 0.0;
    polygonCenter.z = (RF_point.point.z+RH_point.point.z+LH_point.point.z+LF_point.point.z)/4.0;
    
    return polygonCenter;
};


/*
    暂停使用
*/
geometry_msgs::PolygonStamped FootholdPlanner::getSearchPolygon(geometry_msgs::Vector3 center,
                                                                float radius){
    grid_map::Polygon polygon; 
    grid_map::Position leftup_vertex, rightup_vertex, rightdown_vertex, leftdown_vertex;
    geometry_msgs::PolygonStamped polygonMsg;      

    leftup_vertex.x() = center.x + radius;
    leftup_vertex.y() = center.y + 0.5*radius; 
    rightup_vertex.x() = center.x + radius;
    rightup_vertex.y() = center.y - 0.5*radius; 
    rightdown_vertex.x() = center.x - radius;
    rightdown_vertex.y() = center.y - 0.5*radius; 
    leftdown_vertex.x() = center.x - radius;
    leftdown_vertex.y() = center.y + 0.5*radius; 

    polygon.addVertex(leftup_vertex);
    polygon.addVertex(rightup_vertex);
    polygon.addVertex(rightdown_vertex);
    polygon.addVertex(leftdown_vertex);  
    polygon.setFrameId(mapFrame_); 
    
    grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);

    return(polygonMsg);
}


grid_map::Polygon FootholdPlanner::getSearchPolygon(geometry_msgs::PointStamped center,
                                                    float radius){
    grid_map::Polygon polygon; 
    grid_map::Position leftup_vertex, rightup_vertex, rightdown_vertex, leftdown_vertex;

    leftup_vertex.x() = center.point.x + radius;
    leftup_vertex.y() = center.point.y + 0.5*radius; 
    rightup_vertex.x() = center.point.x + radius;
    rightup_vertex.y() = center.point.y - 0.5*radius; 
    rightdown_vertex.x() = center.point.x - radius;
    rightdown_vertex.y() = center.point.y - 0.5*radius; 
    leftdown_vertex.x() = center.point.x - radius;
    leftdown_vertex.y() = center.point.y + 0.5*radius; 

    polygon.addVertex(leftup_vertex);
    polygon.addVertex(rightup_vertex);
    polygon.addVertex(rightdown_vertex);
    polygon.addVertex(leftdown_vertex);  
    polygon.setFrameId(mapFrame_); 
    
    return(polygon);
}


float FootholdPlanner::getFootholdMeanHeight(   grid_map::GridMap gridmap, 
                                                grid_map::Position center,
                                                float radius,
                                                double h)
{
    // ROS_INFO_STREAM("Foot center height = " << gridmap.atPosition("elevation", center)); // 若所查询的坐标超出TM地图大小，将导致TM节点挂掉！
  
    float iHeight = 0.0, meanHeight = 0.0;
    int i = 0;	
    for(grid_map::CircleIterator iterator(gridmap, center, radius); !iterator.isPastEnd(); ++iterator)
    {
        //! 必须对cell = nan情况进行处理(否则将导致后续程序挂掉))
        if(gridmap.isValid(*iterator, "elevation")){
            iHeight = gridmap.at("elevation", *iterator);
        }else{
            ROS_WARN("Cell height is NAN!");
            iHeight = 0.0;            
        }
        
        if(iHeight < 10 ) 
        {
            i++;
            meanHeight = meanHeight + iHeight;
            if(debug3_) printf("Cell i %i foot height = %f", i, iHeight);
        }       
    }    
    
    if(i != 0){
        meanHeight = meanHeight / i;
    }else{
        meanHeight = iHeight;
    }
    
    return (meanHeight + h);
}


bool FootholdPlanner::getMapIndex(  grid_map::GridMap map,
                                    geometry_msgs::Point position,                                                
                                    grid_map::Index& index)
{
    grid_map::Position p;
    p.x() = position.x;
    p.y() = position.y;

    if(!map.getIndex(p, index)) return false;

    return true;
}


bool FootholdPlanner::getHipDistance(geometry_msgs::PointStamped rfFootholdResult, 
                    geometry_msgs::PointStamped rhFootholdResult, 
                    geometry_msgs::PointStamped lhFootholdResult, 
                    geometry_msgs::PointStamped lfFootholdResult,
                    std::vector<double>& feetDistance)
{
    double feetDistance1, feetDistance2; // 一个步态周期包含2次对脚迈步
    feetDistance1 = (rfFootholdResult.point.x - lhFootholdResult.point.x);
    feetDistance.push_back(feetDistance1);
    feetDistance2 = (lfFootholdResult.point.x - rhFootholdResult.point.x);
    feetDistance.push_back(feetDistance2);      

    return true;          
}


bool FootholdPlanner::getCogSpeed(geometry_msgs::PointStamped rfFootholdResult, 
                                    geometry_msgs::PointStamped rhFootholdResult, 
                                    geometry_msgs::PointStamped lhFootholdResult, 
                                    geometry_msgs::PointStamped lfFootholdResult,
                                    geometry_msgs::PointStamped rfCurrentFoothold, 
                                    geometry_msgs::PointStamped rhCurrentFoothold, 
                                    geometry_msgs::PointStamped lhCurrentFoothold, 
                                    geometry_msgs::PointStamped lfCurrentFoothold,                        
                                    std::vector<double>& cogSpeed)
{
    double cogSpeed1, cogSpeed2;

    // 计算对脚中心
    double feetCenter1, feetCenter2, feetCenter3;
    if(RF_FIRST_){
        feetCenter1 = (rfCurrentFoothold.point.x + lhCurrentFoothold.point.x)/2;
        feetCenter2 = (lfFootholdResult.point.x + rhFootholdResult.point.x)/2;
        feetCenter3 = (rfFootholdResult.point.x + lhFootholdResult.point.x)/2;
    }else{
        feetCenter1 = (lfCurrentFoothold.point.x + rhCurrentFoothold.point.x)/2;
        feetCenter2 = (rfFootholdResult.point.x + lhFootholdResult.point.x)/2;
        feetCenter3 = (lfFootholdResult.point.x + rhFootholdResult.point.x)/2;
    }

    // 计算前后对脚迈步的cog移动距离
    double cogMovedDistance1, cogMovedDistance2;
    cogMovedDistance1 = feetCenter2 - feetCenter1;
    cogMovedDistance2 = feetCenter3 - feetCenter2;

    // 计算速度
    cogSpeed1 = cogMovedDistance1 / (0.5*gaitCycle_);
    cogSpeed.push_back(cogSpeed1);
    cogSpeed2 = cogMovedDistance2 / (0.5*gaitCycle_);
    cogSpeed.push_back(cogSpeed2);           

    return true;                  
}   


bool FootholdPlanner::setFirstGait( geometry_msgs::PointStamped RF_initialPosition, 
                                    geometry_msgs::PointStamped RH_initialPosition, 
                                    geometry_msgs::PointStamped LH_initialPosition, 
                                    geometry_msgs::PointStamped LF_initialPosition, 
                                    geometry_msgs::PointStamped& RF_currentPosition,
                                    geometry_msgs::PointStamped& RH_currentPosition,
                                    geometry_msgs::PointStamped& LH_currentPosition,
                                    geometry_msgs::PointStamped& LF_currentPosition)
{
    RF_currentPosition.point = RF_initialPosition.point;
    RH_currentPosition.point = RH_initialPosition.point;
    LH_currentPosition.point = LH_initialPosition.point;
    LF_currentPosition.point = LF_initialPosition.point;

    RF_currentPosition.point.x = RF_initialPosition.point.x - stepLength_/2;
    RH_currentPosition.point.x = RH_initialPosition.point.x - stepLength_/2;
    LH_currentPosition.point.x = LH_initialPosition.point.x - stepLength_/2;
    LF_currentPosition.point.x = LF_initialPosition.point.x - stepLength_/2;      

    return true;  
}


/*
    暂停使用
*/
void FootholdPlanner::publishSearchPolygonMarker(   geometry_msgs::Vector3 center,
                                                    float radius){
    grid_map::Polygon polygon; 
    grid_map::Position leftup_vertex, rightup_vertex, rightdown_vertex, leftdown_vertex;
    geometry_msgs::PolygonStamped polygonMsg;      

    leftup_vertex.x() = center.x + radius;
    leftup_vertex.y() = center.y + 0.5*radius; 
    rightup_vertex.x() = center.x + radius;
    rightup_vertex.y() = center.y - 0.5*radius; 
    rightdown_vertex.x() = center.x - radius;
    rightdown_vertex.y() = center.y - 0.5*radius; 
    leftdown_vertex.x() = center.x - radius;
    leftdown_vertex.y() = center.y + 0.5*radius; 

    polygon.addVertex(leftup_vertex);
    polygon.addVertex(rightup_vertex);
    polygon.addVertex(rightdown_vertex);
    polygon.addVertex(leftdown_vertex);  
    polygon.setFrameId(mapFrame_); 
    
    grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);
    // searchPolygonPub_.publish(polygonMsg);	    	    
}


void FootholdPlanner::publishGlobalFootholdSphereMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                                    geometry_msgs::PointStamped rhFootholdResult, 
                                                    geometry_msgs::PointStamped lhFootholdResult, 
                                                    geometry_msgs::PointStamped lfFootholdResult,
                                                    float scale,
                                                    double alpha)
{    
    RF_sphere_list.type = RH_sphere_list.type = LH_sphere_list.type = LF_sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;   
    RF_sphere_list.id = 0;
    RH_sphere_list.id = 1;
    LH_sphere_list.id = 2;
    LF_sphere_list.id = 3;    
    
    RF_sphere_list.action = RH_sphere_list.action = LH_sphere_list.action = LF_sphere_list.action = visualization_msgs::Marker::ADD;
    RF_sphere_list.lifetime = RH_sphere_list.lifetime = LH_sphere_list.lifetime = LF_sphere_list.lifetime = ros::Duration(0);
    
    RF_sphere_list.header.frame_id = RH_sphere_list.header.frame_id = LH_sphere_list.header.frame_id = LF_sphere_list.header.frame_id = mapFrame_;
    RF_sphere_list.header.stamp = RH_sphere_list.header.stamp = LH_sphere_list.header.stamp = LF_sphere_list.header.stamp = ros::Time::now();    

    RF_sphere_list.scale.x = RF_sphere_list.scale.y = RF_sphere_list.scale.z = scale;
    RH_sphere_list.scale.x = RH_sphere_list.scale.y = RH_sphere_list.scale.z = scale;
    LH_sphere_list.scale.x = LH_sphere_list.scale.y = LH_sphere_list.scale.z = scale;
    LF_sphere_list.scale.x = LF_sphere_list.scale.y = LF_sphere_list.scale.z = scale;	
    
    RF_sphere_list.color.a = RH_sphere_list.color.a = LH_sphere_list.color.a = LF_sphere_list.color.a = alpha;
    RF_sphere_list.color.r = 1.0;
    RH_sphere_list.color.g = 1.0;
    LH_sphere_list.color.b = 1.0;
    LF_sphere_list.color.r = 1.0;
    LF_sphere_list.color.g = 1.0;	
    
    RF_sphere_list.pose.orientation.w = RH_sphere_list.pose.orientation.w = LH_sphere_list.pose.orientation.w = LF_sphere_list.pose.orientation.w = 1.0;    

    geometry_msgs::Point rfPoint, rhPoint, lhPoint, lfPoint;
    rfPoint.x = rfFootholdResult.point.x;
    rfPoint.y = rfFootholdResult.point.y;
    rfPoint.z = rfFootholdResult.point.z;
    
    rhPoint.x = rhFootholdResult.point.x;
    rhPoint.y = rhFootholdResult.point.y;
    rhPoint.z = rhFootholdResult.point.z;

    lhPoint.x = lhFootholdResult.point.x;
    lhPoint.y = lhFootholdResult.point.y;
    lhPoint.z = lhFootholdResult.point.z;

    lfPoint.x = lfFootholdResult.point.x;
    lfPoint.y = lfFootholdResult.point.y;
    lfPoint.z = lfFootholdResult.point.z;    
       
    RF_sphere_list.points.push_back(rfPoint);
    RH_sphere_list.points.push_back(rhPoint);
    LH_sphere_list.points.push_back(lhPoint);
    LF_sphere_list.points.push_back(lfPoint);	  

    globalFootholdsSphereMarkerPub_.publish(RF_sphere_list);
    globalFootholdsSphereMarkerPub_.publish(RH_sphere_list);
    globalFootholdsSphereMarkerPub_.publish(LH_sphere_list);
    globalFootholdsSphereMarkerPub_.publish(LF_sphere_list);	   	
}  


void FootholdPlanner::publishGlobalFootholdPointMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                                    geometry_msgs::PointStamped rhFootholdResult, 
                                                    geometry_msgs::PointStamped lhFootholdResult, 
                                                    geometry_msgs::PointStamped lfFootholdResult,
                                                    float scale,
                                                    double alpha)
{    
    RF_points.type = RH_points.type = LH_points.type = LF_points.type = visualization_msgs::Marker::POINTS;   
    RF_points.id = 0;
    RH_points.id = 1;
    LH_points.id = 2;
    LF_points.id = 3;    
    
    RF_points.action = RH_points.action = LH_points.action = LF_points.action = visualization_msgs::Marker::ADD;
    RF_points.lifetime = RH_points.lifetime = LH_points.lifetime = LF_points.lifetime = ros::Duration(0);
    
    RF_points.header.frame_id = RH_points.header.frame_id = LH_points.header.frame_id = LF_points.header.frame_id = mapFrame_;
    RF_points.header.stamp = RH_points.header.stamp = LH_points.header.stamp = LF_points.header.stamp = ros::Time::now();    

    RF_points.scale.x = RF_points.scale.y = RF_points.scale.z = scale;
    RH_points.scale.x = RH_points.scale.y = RH_points.scale.z = scale;
    LH_points.scale.x = LH_points.scale.y = LH_points.scale.z = scale;
    LF_points.scale.x = LF_points.scale.y = LF_points.scale.z = scale;	
    
    RF_points.color.a = RH_points.color.a = LH_points.color.a = LF_points.color.a = alpha;
    // RF_points.color.r = 1.0;
    // RH_points.color.g = 1.0;
    // LH_points.color.b = 1.0;
    // LF_points.color.r = 1.0;
    // LF_points.color.g = 1.0;	
    
    RF_points.pose.orientation.w = RH_points.pose.orientation.w = LH_points.pose.orientation.w = LF_points.pose.orientation.w = 1.0;    

    geometry_msgs::Point rfPoint, rhPoint, lhPoint, lfPoint;
    rfPoint.x = rfFootholdResult.point.x;
    rfPoint.y = rfFootholdResult.point.y;
    rfPoint.z = rfFootholdResult.point.z;
    
    rhPoint.x = rhFootholdResult.point.x;
    rhPoint.y = rhFootholdResult.point.y;
    rhPoint.z = rhFootholdResult.point.z;

    lhPoint.x = lhFootholdResult.point.x;
    lhPoint.y = lhFootholdResult.point.y;
    lhPoint.z = lhFootholdResult.point.z;

    lfPoint.x = lfFootholdResult.point.x;
    lfPoint.y = lfFootholdResult.point.y;
    lfPoint.z = lfFootholdResult.point.z;    
       
    RF_points.points.push_back(rfPoint);
    RH_points.points.push_back(rhPoint);
    LH_points.points.push_back(lhPoint);
    LF_points.points.push_back(lfPoint);	  

    globalFootholdsPointMarkerPub_.publish(RF_points);
    globalFootholdsPointMarkerPub_.publish(RH_points);
    globalFootholdsPointMarkerPub_.publish(LH_points);
    globalFootholdsPointMarkerPub_.publish(LF_points);	   	
}  


void FootholdPlanner::publishGlobalFootholdCubeMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                                        geometry_msgs::PointStamped rhFootholdResult, 
                                                        geometry_msgs::PointStamped lhFootholdResult, 
                                                        geometry_msgs::PointStamped lfFootholdResult,
                                                        float scale,
                                                        double alpha)
{    
    RF_cube_list.type = RH_cube_list.type = LH_cube_list.type = LF_cube_list.type = visualization_msgs::Marker::CUBE_LIST;   
    RF_cube_list.id = 0;
    RH_cube_list.id = 1;
    LH_cube_list.id = 2;
    LF_cube_list.id = 3;    
    
    RF_cube_list.action = RH_cube_list.action = LH_cube_list.action = LF_cube_list.action = visualization_msgs::Marker::ADD;
    RF_cube_list.lifetime = RH_cube_list.lifetime = LH_cube_list.lifetime = LF_cube_list.lifetime = ros::Duration(0);
    
    RF_cube_list.header.frame_id = RH_cube_list.header.frame_id = LH_cube_list.header.frame_id = LF_cube_list.header.frame_id = mapFrame_;
    RF_cube_list.header.stamp = RH_cube_list.header.stamp = LH_cube_list.header.stamp = LF_cube_list.header.stamp = ros::Time::now();    

    RF_cube_list.scale.x = RF_cube_list.scale.y = RF_cube_list.scale.z = scale;
    RH_cube_list.scale.x = RH_cube_list.scale.y = RH_cube_list.scale.z = scale;
    LH_cube_list.scale.x = LH_cube_list.scale.y = LH_cube_list.scale.z = scale;
    LF_cube_list.scale.x = LF_cube_list.scale.y = LF_cube_list.scale.z = scale;	
    
    RF_cube_list.color.a = RH_cube_list.color.a = LH_cube_list.color.a = LF_cube_list.color.a = alpha;
    RF_cube_list.color.r = 1.0;
    RH_cube_list.color.g = 1.0;
    LH_cube_list.color.b = 1.0;
    LF_cube_list.color.r = 1.0;
    LF_cube_list.color.g = 1.0;	
    
    RF_cube_list.pose.orientation.w = RH_cube_list.pose.orientation.w = LH_cube_list.pose.orientation.w = LF_cube_list.pose.orientation.w = 1.0;    

    geometry_msgs::Point rfPoint, rhPoint, lhPoint, lfPoint;
    rfPoint.x = rfFootholdResult.point.x;
    rfPoint.y = rfFootholdResult.point.y;
    rfPoint.z = rfFootholdResult.point.z;
    
    rhPoint.x = rhFootholdResult.point.x;
    rhPoint.y = rhFootholdResult.point.y;
    rhPoint.z = rhFootholdResult.point.z;

    lhPoint.x = lhFootholdResult.point.x;
    lhPoint.y = lhFootholdResult.point.y;
    lhPoint.z = lhFootholdResult.point.z;

    lfPoint.x = lfFootholdResult.point.x;
    lfPoint.y = lfFootholdResult.point.y;
    lfPoint.z = lfFootholdResult.point.z;    
       
    RF_cube_list.points.push_back(rfPoint);
    RH_cube_list.points.push_back(rhPoint);
    LH_cube_list.points.push_back(lhPoint);
    LF_cube_list.points.push_back(lfPoint);	  

    globalFootholdsCubeMarkerPub_.publish(RF_cube_list);
    globalFootholdsCubeMarkerPub_.publish(RH_cube_list);
    globalFootholdsCubeMarkerPub_.publish(LH_cube_list);
    globalFootholdsCubeMarkerPub_.publish(LF_cube_list);	   	
}  


/* 
    存在的问题（待解决）：如果后一次落脚点请求的数量少于上一次数量，那么多出来的那些在上一次结果中显示的落脚点markers中后一次中将继续显示出来（不会被覆盖掉）
 */
void FootholdPlanner::publishGlobalFootholdCylinderMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                                        geometry_msgs::PointStamped rhFootholdResult, 
                                                        geometry_msgs::PointStamped lhFootholdResult, 
                                                        geometry_msgs::PointStamped lfFootholdResult,
                                                        float scale,
                                                        double alpha,
                                                        int count)
{    
    visualization_msgs::Marker RF_cylinder, RH_cylinder, LH_cylinder, LF_cylinder;
    RF_cylinder.type = RH_cylinder.type = LH_cylinder.type = LF_cylinder.type = visualization_msgs::Marker::CYLINDER;   
    RF_cylinder.id = 0 + count;
    RH_cylinder.id = 1 + count;
    LH_cylinder.id = 2 + count;
    LF_cylinder.id = 3 + count;    
    
    RF_cylinder.action = RH_cylinder.action = LH_cylinder.action = LF_cylinder.action = visualization_msgs::Marker::ADD;
    RF_cylinder.lifetime = RH_cylinder.lifetime = LH_cylinder.lifetime = LF_cylinder.lifetime = ros::Duration(0);
    
    RF_cylinder.header.frame_id = RH_cylinder.header.frame_id = LH_cylinder.header.frame_id = LF_cylinder.header.frame_id = mapFrame_;
    RF_cylinder.header.stamp = RH_cylinder.header.stamp = LH_cylinder.header.stamp = LF_cylinder.header.stamp = ros::Time::now();    

/*     RF_cylinder.scale.x = RF_cylinder.scale.y = RF_cylinder.scale.z = scale;
    RH_cylinder.scale.x = RH_cylinder.scale.y = RH_cylinder.scale.z = scale;
    LH_cylinder.scale.x = LH_cylinder.scale.y = LH_cylinder.scale.z = scale;
    LF_cylinder.scale.x = LF_cylinder.scale.y = LF_cylinder.scale.z = scale;	 */
    RF_cylinder.scale.x = scale;
    RH_cylinder.scale.x = scale;
    LH_cylinder.scale.x = scale;
    LF_cylinder.scale.x = scale;	
    RF_cylinder.scale.y = scale*0.5;
    RH_cylinder.scale.y = scale*0.5;
    LH_cylinder.scale.y = scale*0.5;
    LF_cylinder.scale.y = scale*0.5;	
    RF_cylinder.scale.z = scale*1;
    RH_cylinder.scale.z = scale*1;
    LH_cylinder.scale.z = scale*1;
    LF_cylinder.scale.z = scale*1;	

    RF_cylinder.color.a = RH_cylinder.color.a = LH_cylinder.color.a = LF_cylinder.color.a = alpha;
    RF_cylinder.color.r = 1.0;
    RH_cylinder.color.g = 1.0;
    LH_cylinder.color.b = 1.0;
    LF_cylinder.color.r = 1.0;
    LF_cylinder.color.g = 1.0;	
    
    RF_cylinder.pose.orientation.w = RH_cylinder.pose.orientation.w = LH_cylinder.pose.orientation.w = LF_cylinder.pose.orientation.w = 1.0;    

    geometry_msgs::Point rfPoint, rhPoint, lhPoint, lfPoint;
    rfPoint.x = rfFootholdResult.point.x;
    rfPoint.y = rfFootholdResult.point.y;
    rfPoint.z = rfFootholdResult.point.z;
    
    rhPoint.x = rhFootholdResult.point.x;
    rhPoint.y = rhFootholdResult.point.y;
    rhPoint.z = rhFootholdResult.point.z;

    lhPoint.x = lhFootholdResult.point.x;
    lhPoint.y = lhFootholdResult.point.y;
    lhPoint.z = lhFootholdResult.point.z;

    lfPoint.x = lfFootholdResult.point.x;
    lfPoint.y = lfFootholdResult.point.y;
    lfPoint.z = lfFootholdResult.point.z;    

    RF_cylinder.pose.position = rfPoint;
    RH_cylinder.pose.position = rhPoint;
    LH_cylinder.pose.position = lhPoint;
    LF_cylinder.pose.position = lfPoint;

    RF_cylinderArray.markers.push_back(RF_cylinder);
    RH_cylinderArray.markers.push_back(RH_cylinder);
    LH_cylinderArray.markers.push_back(LH_cylinder);
    LF_cylinderArray.markers.push_back(LF_cylinder);
        
    globalFootholdsCylinderMarkerPub_.publish(RF_cylinderArray);
    globalFootholdsCylinderMarkerPub_.publish(RH_cylinderArray);
    globalFootholdsCylinderMarkerPub_.publish(LH_cylinderArray);
    globalFootholdsCylinderMarkerPub_.publish(LF_cylinderArray);	   	
}  


void FootholdPlanner::publishInitialFootholdMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                                        geometry_msgs::PointStamped rhFootholdResult, 
                                                        geometry_msgs::PointStamped lhFootholdResult, 
                                                        geometry_msgs::PointStamped lfFootholdResult,
                                                        float scale,
                                                        double alpha)
{    
    initial_RF_cube_list.type = initial_RH_cube_list.type = initial_LH_cube_list.type = initial_LF_cube_list.type = visualization_msgs::Marker::CUBE_LIST;   
    initial_RF_cube_list.id = 0;
    initial_RH_cube_list.id = 1;
    initial_LH_cube_list.id = 2;
    initial_LF_cube_list.id = 3;    
    
    initial_RF_cube_list.action = initial_RH_cube_list.action = initial_LH_cube_list.action = initial_LF_cube_list.action = visualization_msgs::Marker::ADD;
    initial_RF_cube_list.lifetime = initial_RH_cube_list.lifetime = initial_LH_cube_list.lifetime = initial_LF_cube_list.lifetime = ros::Duration(0);
    
    initial_RF_cube_list.header.frame_id = initial_RH_cube_list.header.frame_id = initial_LH_cube_list.header.frame_id = initial_LF_cube_list.header.frame_id = mapFrame_;
    initial_RF_cube_list.header.stamp = initial_RH_cube_list.header.stamp = initial_LH_cube_list.header.stamp = initial_LF_cube_list.header.stamp = ros::Time::now();    

    initial_RF_cube_list.scale.x = initial_RF_cube_list.scale.y = initial_RF_cube_list.scale.z = scale;
    initial_RH_cube_list.scale.x = initial_RH_cube_list.scale.y = initial_RH_cube_list.scale.z = scale;
    initial_LH_cube_list.scale.x = initial_LH_cube_list.scale.y = initial_LH_cube_list.scale.z = scale;
    initial_LF_cube_list.scale.x = initial_LF_cube_list.scale.y = initial_LF_cube_list.scale.z = scale;	
    
    initial_RF_cube_list.color.a = initial_RH_cube_list.color.a = initial_LH_cube_list.color.a = initial_LF_cube_list.color.a = alpha;
    initial_RF_cube_list.color.r = 1.0;
    initial_RF_cube_list.color.g = 1.0;
    initial_RF_cube_list.color.b = 1;
    initial_RH_cube_list.color.r = 1.0;
    initial_RH_cube_list.color.g = 1.0;
    initial_RH_cube_list.color.b = 1;
    initial_LH_cube_list.color.r = 1.0;
    initial_LH_cube_list.color.g = 1.0;
    initial_LH_cube_list.color.b = 1;
    initial_LF_cube_list.color.r = 1.0;
    initial_LF_cube_list.color.g = 1.0;
    initial_LF_cube_list.color.b = 1;	
    
    initial_RF_cube_list.pose.orientation.w = initial_RH_cube_list.pose.orientation.w = initial_LH_cube_list.pose.orientation.w = initial_LF_cube_list.pose.orientation.w = 1.0;    

    geometry_msgs::Point rfPoint, rhPoint, lhPoint, lfPoint;
    rfPoint.x = rfFootholdResult.point.x;
    rfPoint.y = rfFootholdResult.point.y;
    rfPoint.z = rfFootholdResult.point.z;
    
    rhPoint.x = rhFootholdResult.point.x;
    rhPoint.y = rhFootholdResult.point.y;
    rhPoint.z = rhFootholdResult.point.z;

    lhPoint.x = lhFootholdResult.point.x;
    lhPoint.y = lhFootholdResult.point.y;
    lhPoint.z = lhFootholdResult.point.z;

    lfPoint.x = lfFootholdResult.point.x;
    lfPoint.y = lfFootholdResult.point.y;
    lfPoint.z = lfFootholdResult.point.z;    
       
    initial_RF_cube_list.points.push_back(rfPoint);
    initial_RH_cube_list.points.push_back(rhPoint);
    initial_LH_cube_list.points.push_back(lhPoint);
    initial_LF_cube_list.points.push_back(lfPoint);	  

    initialFootholdMarkerPub_.publish(initial_RF_cube_list);
    initialFootholdMarkerPub_.publish(initial_RH_cube_list);
    initialFootholdMarkerPub_.publish(initial_LH_cube_list);
    initialFootholdMarkerPub_.publish(initial_LF_cube_list);	   	
}  


bool FootholdPlanner::saveLog()
{
    // 获取系统当前时间
    time_t rawTime;
    time(&rawTime);
    // cout<<"Time: "<<rawTime<<endl;
    // 方式1 http://www.cplusplus.com/reference/ctime/ctime/ 
    // std::string currentTime = ctime(&rawTime);
    // std::string currentTime = ctime(&rawTime);
    // cout<<"Current time: "<<currentTime<<endl;
    // 方式2 http://www.cplusplus.com/reference/ctime/localtime/
    struct tm* timeInfo;
    timeInfo = localtime(&rawTime);
    std::string currentTime = asctime(timeInfo);
    // cout<<"Current time: "<<currentTime<<endl;
    // 方式3 
    //https://blog.csdn.net/weixin_44032878/article/details/88035449?utm_medium=
    //distribute.pc_aggpage_search_result.none-task-blog-2~all~first_rank_v2~rank
    //_v25-1-88035449.nonecase&utm_term=c++%E7%94%A8%E5%BD%93%E5%89%8D%E6%97%B6%E9%97%B4%E5%91%BD%E5%90%8D%E6%96%87%E4%BB%B6
    char fileName[256] = {0};
    strftime(fileName, sizeof(fileName), "%Y.%m.%d %H-%M-%S.txt", localtime(&rawTime));


    // 写数据到文件
    std::string username = getUserName();


 
    /* 1、传统落脚点规划算法 ------------------------------------------------------------------ */
    std::string filePath;
    std::ofstream cogSpeedFile;    
    std::ofstream feetDistanceFile;       
    if(1)
    {
        // 1.1 全局落脚点
        // std::string filePath = "/home/"+username+"/laika_ws/log/nominal_global_footholds_" + currentTime + ".txt";      
        filePath = "/home/"+username+"/laika_ws/log/nominal_global_footholds_" + fileName;      
        std::ofstream outFile1;    
        outFile1.open(filePath, std::ios::app|std::ios::out);
        outFile1<<std::fixed;
        outFile1.precision(3);
        if(outFile1.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<globalFootholdsResult_.nominalFootholds.size(); ++i){
            outFile1
            <<globalFootholdsResult_.nominalFootholds[i][0]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][1]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][2]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][3]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][4]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][5]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][6]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][7]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][8]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][9]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][10]<<"\t"
            <<globalFootholdsResult_.nominalFootholds[i][11]<<endl;                
        }

        outFile1.close();
        if(debug_) cout<<"Nominal global footholds saved in file : "<<filePath<<endl;         

        // 1.2 cog前进方向线速度
        filePath = "/home/"+username+"/laika_ws/log/nominal_cog_speed_" + fileName;      
        cogSpeedFile.open(filePath, std::ios::app|std::ios::out);
        cogSpeedFile<<std::fixed;
        cogSpeedFile.precision(3);    
        if(cogSpeedFile.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<footholdsKPI_.cogSpeed_nominal.size(); ++i){
            cogSpeedFile<<footholdsKPI_.cogSpeed_nominal[i]<<endl;                
        }

        cogSpeedFile.close();
        if(debug_) cout<<"Opt cog speed saved in file : "<<filePath<<endl;      


        // 1.3 对脚间距
        filePath = "/home/"+username+"/laika_ws/log/nominal_feet_distance_" + fileName;      
        feetDistanceFile.open(filePath, std::ios::app|std::ios::out);
        feetDistanceFile<<std::fixed;
        feetDistanceFile.precision(3);    
        if(feetDistanceFile.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<footholdsKPI_.feetDistance_nominal.size(); ++i){
            feetDistanceFile<<footholdsKPI_.feetDistance_nominal[i]<<endl;                
        }

        feetDistanceFile.close();
        if(debug_) cout<<"Opt feet distance saved in file : "<<filePath<<endl;            
    }
 

    /* 2、保存可通行地形中心落脚点规划结果 ------------------------------------------------------------------ */
    if(1)
    {
        // 2.1 全局落脚点位置
        // filePath = "/home/"+username+"/laika_ws/log/centroid_global_footholds_" + currentTime + ".txt";      
        filePath = "/home/"+username+"/laika_ws/log/centroid_global_footholds_" + fileName;      
        std::ofstream outFile2;    
        outFile2.open(filePath, std::ios::app|std::ios::out);
        outFile2<<std::fixed;
        outFile2.precision(3);    
        if(outFile2.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<globalFootholdsResult_.centroidFootholds.size(); ++i){
            outFile2
            <<"{"
            <<globalFootholdsResult_.centroidFootholds[i][0]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][1]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][2]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][3]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][4]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][5]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][6]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][7]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][8]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][9]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][10]<<"\t,"
            <<globalFootholdsResult_.centroidFootholds[i][11]
            <<"},"
            <<endl;                
        }

        outFile2.close();
        if(debug_) cout<<"Centroid global footholds saved in file : "<<filePath<<endl;      

    /* 
        // filePath = "/home/"+username+"/laika_ws/log/centroid_global_footholds_" + currentTime + ".txt";      
        filePath = "/home/"+username+"/laika_ws/log/centroid_global_footholds_" + fileName;      
        std::ofstream outFile2;    
        outFile2.open(filePath, std::ios::app|std::ios::out);
        outFile2<<std::fixed;
        outFile2.precision(3);    
        if(outFile2.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<globalFootholdsResult_.centroidFootholds.size(); ++i){
            outFile2
            <<globalFootholdsResult_.centroidFootholds[i][0]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][1]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][2]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][3]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][4]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][5]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][6]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][7]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][8]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][9]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][10]<<"\t"
            <<globalFootholdsResult_.centroidFootholds[i][11]<<endl;                
        }

        outFile2.close();
        if(debug_) cout<<"Centroid global footholds saved in file : "<<filePath<<endl; */  


        // 2.2 cog前进方向线速度
        filePath = "/home/"+username+"/laika_ws/log/centroid_cog_speed_" + fileName;      
        cogSpeedFile.open(filePath, std::ios::app|std::ios::out);
        cogSpeedFile<<std::fixed;
        cogSpeedFile.precision(3);    
        if(cogSpeedFile.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<footholdsKPI_.cogSpeed_centroid.size(); ++i){
            cogSpeedFile<<footholdsKPI_.cogSpeed_centroid[i]<<endl;                
        }

        cogSpeedFile.close();
        if(debug_) cout<<"Centroid cog speed saved in file : "<<filePath<<endl;      


        // 2.3 对脚间距
        filePath = "/home/"+username+"/laika_ws/log/centroid_feet_distance_" + fileName;      
        feetDistanceFile.open(filePath, std::ios::app|std::ios::out);
        feetDistanceFile<<std::fixed;
        feetDistanceFile.precision(3);    
        if(feetDistanceFile.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<footholdsKPI_.feetDistance_centroid.size(); ++i){
            feetDistanceFile<<footholdsKPI_.feetDistance_centroid[i]<<endl;                
        }

        feetDistanceFile.close();
        if(debug_) cout<<"Centroid feet distance saved in file : "<<filePath<<endl;      
    }


    /* 3、保存最优化落脚点规划结果 ------------------------------------------------------------------ */
    if(1)
    {
        // 2.1 全局落脚点位置
        filePath = "/home/"+username+"/laika_ws/log/opt_global_footholds_" + fileName;      
        std::ofstream outFile3;    
        outFile3.open(filePath, std::ios::app|std::ios::out);
        outFile3<<std::fixed;
        outFile3.precision(3);    
        if(outFile3.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<globalFootholdsResult_.optFootholds.size(); ++i){
            outFile3
            <<"{"
            <<globalFootholdsResult_.optFootholds[i][0]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][1]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][2]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][3]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][4]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][5]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][6]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][7]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][8]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][9]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][10]<<"\t,"
            <<globalFootholdsResult_.optFootholds[i][11]
            <<"},"
            <<endl;                
        }

        outFile3.close();
        if(debug_) cout<<"Optimized global footholds saved in file : "<<filePath<<endl;      


        // 2.2 cog前进方向线速度
        filePath = "/home/"+username+"/laika_ws/log/opt_cog_speed_" + fileName;      
        // std::ofstream cogSpeedFile;    
        cogSpeedFile.open(filePath, std::ios::app|std::ios::out);
        cogSpeedFile<<std::fixed;
        cogSpeedFile.precision(3);    
        if(cogSpeedFile.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<footholdsKPI_.cogSpeed_opt.size(); ++i){
            cogSpeedFile<<footholdsKPI_.cogSpeed_opt[i]<<endl;                
        }

        cogSpeedFile.close();
        if(debug_) cout<<"Optimized cog speed saved in file : "<<filePath<<endl;      


        // 2.3 对脚间距
        filePath = "/home/"+username+"/laika_ws/log/opt_feet_distance_" + fileName;      
        // std::ofstream feetDistanceFile;    
        feetDistanceFile.open(filePath, std::ios::app|std::ios::out);
        feetDistanceFile<<std::fixed;
        feetDistanceFile.precision(3);    
        if(feetDistanceFile.bad()){
            cout<<"Cannot create file: "<<filePath<<endl;
            return false;
        }

        for(int i=0; i<footholdsKPI_.feetDistance_opt.size(); ++i){
            feetDistanceFile<<footholdsKPI_.feetDistance_opt[i]<<endl;                
        }

        feetDistanceFile.close();
        if(debug_) cout<<"Optimized feet distance saved in file : "<<filePath<<endl;      
    }

    return true;
}

}/* namespace foothold_planner */