/*
    Author: chenlu
    Version: v3.0 
    Update: 2020-0802
    Email: chenlucqu@gmail.com
    Institute: AIRS
*/

#include <ros/ros.h>

#include "foothold_planner/FootholdPlanner.hpp"

using namespace std;

namespace foothold_planner
{
visualization_msgs::Marker RF_sphere_list, RH_sphere_list, LH_sphere_list, LF_sphere_list;
geometry_msgs::Point RF_currentPosition, RH_currentPosition, LH_currentPosition, LF_currentPosition;

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
    globalFootholdsPub_ = node_.advertise<foothold_planner_msgs::GlobalFootholds>("global_footholds", 1);
    globalFootholdsMarkerPub_ = node_.advertise<visualization_msgs::Marker>("global_footholds_marker", 10);

    globalFootholdPlanService_ = node_.advertiseService("plan_global_footholds", &FootholdPlanner::globalFootholdPlan, this);



    initialize();
}


FootholdPlanner::~FootholdPlanner(){
    node_.shutdown();
}


bool FootholdPlanner::readParameters(){
    node_.param("debug", debug_, false);
    node_.param("checkDefaultFoothold_Debug", checkDefaultFoothold_Debug_, false);

    node_.param("footRadius", footRadius_, float(0.03));

    node_.param("defaultFootholdThreshold", defaultFootholdThreshold_, float(0.7));
    node_.param("candidateFootholdThreshold", candidateFootholdThreshold_, float(0.7));

    node_.param("gaitCycleNum", gaitCycleNum_, 1);
    node_.param("searchRadius", searchRadius_, float(0.1));
    node_.param("stepLength", stepLength_, float(0.2));

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
    node_.param("laikago_kinematics/skewLength", skewLength_, float(0.1));
    //! 计算轴距、轮距
    laikagoKinematics_.lengthBase = laikagoKinematics_.length;
    laikagoKinematics_.widthBase = laikagoKinematics_.width + laikagoKinematics_.l1*2;
    //! 计算默认站立梯形的4个顶点坐标（相对于梯形几何中心）
    RF_defaultBias_.point.x = 0.5*laikagoKinematics_.lengthBase - skewLength_;
    RF_defaultBias_.point.y = -0.5*laikagoKinematics_.widthBase;
    RH_defaultBias_.point.x = -0.5*laikagoKinematics_.lengthBase + skewLength_;
    RH_defaultBias_.point.y = -0.5*laikagoKinematics_.widthBase;
    LH_defaultBias_.point.x = -0.5*laikagoKinematics_.lengthBase;
    LH_defaultBias_.point.y = 0.5*laikagoKinematics_.widthBase;    
    LF_defaultBias_.point.x = 0.5*laikagoKinematics_.lengthBase;
    LF_defaultBias_.point.y = 0.5*laikagoKinematics_.widthBase;      

    //! 从其他节点获取的参数
    node_.param("/elevation_mapping/map_frame_id", mapFrame_, string("world"));
    


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

    globalFootholds_.header.frame_id = mapFrame_;
    globalFootholds_.gait_cycles = gaitCycleNum_;


    //! 机器人四个足端初始位置（世界坐标系），假设初始位置在世界坐标系原点
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
    RF_currentPosition_.header.frame_id = 
        RH_currentPosition_.header.frame_id = 
        LH_currentPosition_.header.frame_id = 
        LF_currentPosition_.header.frame_id = mapFrame_;

    RF_currentPosition.x = 0.3;
    RF_currentPosition.y = -0.2;
    RF_currentPosition.z = 0;

    RH_currentPosition.x = -0.3;
    RH_currentPosition.y = -0.2;
    RH_currentPosition.z = 0;    

    LH_currentPosition.x = -0.3;
    LH_currentPosition.y = 0.2;
    LH_currentPosition.z = 0;    

    LF_currentPosition.x = 0.5;
    LF_currentPosition.y = 0.2;
    LF_currentPosition.z = 0;    

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


    ROS_INFO("Done.");
}


void FootholdPlanner::gridmapCallback(grid_map_msgs::GridMap msg){
    ROS_INFO("Grid map called.");

    grid_map::GridMapRosConverter::fromMessage(msg, gridmap_);

    ROS_INFO("******** traversability GridMap info: size = %f x %f m (%i x %i cells), resolution = %f m/cell, pose = %f %f m, frame = %s", 
        gridmap_.getLength().x(), gridmap_.getLength().y(), 
        gridmap_.getSize()(0), gridmap_.getSize()(1), 
        gridmap_.getResolution(), 
        gridmap_.getPosition(), 
        gridmap_.getFrameId());   

}


// bool FootholdPlanner::globalFootholdPlan(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
bool FootholdPlanner::globalFootholdPlan(   foothold_planner::GlobalFootholdPlan::Request& request, 
                                            foothold_planner::GlobalFootholdPlan::Response& response){
    cout<<endl<<endl;
    cout<<"===================================================================================="<<endl;
    ROS_INFO("Global footholds planning ......");

    //! 获取4个足端当前实时位置
    RF_currentPosition_.point = getFootCurrentPosition(RF_currentPosition);
    RH_currentPosition_.point = getFootCurrentPosition(RH_currentPosition);
    LH_currentPosition_.point = getFootCurrentPosition(LH_currentPosition);
    LF_currentPosition_.point = getFootCurrentPosition(LF_currentPosition);

    globalFootholds_.footholds.clear();
    gaitCycleNum_ = request.gait_cycles;
    globalFootholds_.gait_cycles = gaitCycleNum_;

    //! 必须清空，否则会将以前规划的落脚点也绘制出来
    RF_sphere_list.points.clear();  
    RH_sphere_list.points.clear();    
    LH_sphere_list.points.clear();   
    LF_sphere_list.points.clear();      
    
    //! 循环执行落脚点规划任务，直到达到设定的步态周期数
    for(int gaitCycleIndex = 0; gaitCycleIndex < gaitCycleNum_; ++gaitCycleIndex){

        ROS_INFO_STREAM("This is NO."<<gaitCycleIndex<<" gait cyle foothold planning ......");

        //! 根据4只脚当前足端位置，计算其几何中心位置。
        // 之后，在此位置基础上再向前增加一个步长，即可得到下一个步态周期4个足端构成的几何中心位置。
        // 最后，根据4个足端默认落脚点位置，推算出下一个步态周期默认落脚点在地图上的位置。
        // 如果是规划全局多个步态周期的落脚点，那么上述步骤循环执行即可，注意的是之后的几何中心推算以上一次规划出的落脚点为输入（而不是本体里程计给出的实时的！）

        //! 计算下一个步态周期4个足端构成的四边形几何中心
        geometry_msgs::Point polygonCenter;
        polygonCenter = getPolygonCenter(RF_currentPosition_, RH_currentPosition_, LH_currentPosition_, LF_currentPosition_);
        ROS_INFO_STREAM("Feet center in current gait-cyle: "<<polygonCenter);

        geometry_msgs::Point polygonCenterNext;
        polygonCenterNext.x = polygonCenter.x + stepLength_;
        ROS_INFO_STREAM("Feet center in next gait-cyle: "<<polygonCenterNext);

        //! 计算下一个步态周期4个足端默认位置
        RF_nextPosition_ = getDefaultFootholdNext(polygonCenterNext, RF_defaultBias_);
        if(debug_) ROS_INFO_STREAM("Next RF default foothold: "<<RF_nextPosition_);
        RH_nextPosition_ = getDefaultFootholdNext(polygonCenterNext, RH_defaultBias_);
        if(debug_) ROS_INFO_STREAM("Next RH default foothold: "<<RH_nextPosition_);
        LH_nextPosition_ = getDefaultFootholdNext(polygonCenterNext, LH_defaultBias_);
        if(debug_) ROS_INFO_STREAM("Next LH default foothold: "<<LH_nextPosition_);
        LF_nextPosition_ = getDefaultFootholdNext(polygonCenterNext, LF_defaultBias_);
        if(debug_) ROS_INFO_STREAM("Next LF default foothold: "<<LF_nextPosition_);

        //! 获取矩形搜索区域，并发布markers
        grid_map::Polygon RF_polygon, RH_polygon, LH_polygon, LF_polygon;
        geometry_msgs::PolygonStamped RF_polygonMsg, RH_polygonMsg, LH_polygonMsg, LF_polygonMsg;

        RF_polygon = getSearchPolygon(RF_nextPosition_, searchRadius_);
        grid_map::PolygonRosConverter::toMessage(RF_polygon, RF_polygonMsg);
        RF_searchPolygonPub_.publish(RF_polygonMsg);

        RH_polygon = getSearchPolygon(RH_nextPosition_, searchRadius_);
        grid_map::PolygonRosConverter::toMessage(RH_polygon, RH_polygonMsg);
        RH_searchPolygonPub_.publish(RH_polygonMsg);
    
        LH_polygon = getSearchPolygon(LH_nextPosition_, searchRadius_);
        grid_map::PolygonRosConverter::toMessage(LH_polygon, LH_polygonMsg);
        LH_searchPolygonPub_.publish(LH_polygonMsg);
        
        LF_polygon = getSearchPolygon(LF_nextPosition_, searchRadius_);
        grid_map::PolygonRosConverter::toMessage(LF_polygon, LF_polygonMsg);
        LF_searchPolygonPub_.publish(LF_polygonMsg);    

        //! 开始落脚点规划......
        // RF_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, RF_nextPosition_, footRadius_);
        grid_map::Position center;
        center.x() = RF_nextPosition_.point.x;
        center.y() = RF_nextPosition_.point.y;    
        geometry_msgs::PointStamped RF_footholdResult;
        RF_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                    gridmap_, 
                                    center, 
                                    footRadius_, 
                                    searchRadius_,
                                    RF_polygon, 
                                    std::ref(RF_footholdResult), 
                                    std::ref(RF_footholdValidation_));

        center.x() = RH_nextPosition_.point.x;
        center.y() = RH_nextPosition_.point.y;     
        geometry_msgs::PointStamped RH_footholdResult;
        RH_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                    gridmap_, 
                                    center, 
                                    footRadius_, 
                                    searchRadius_,
                                    RH_polygon, 
                                    std::ref(RH_footholdResult), 
                                    std::ref(RH_footholdValidation_));

        center.x() = LH_nextPosition_.point.x;
        center.y() = LH_nextPosition_.point.y;  
        geometry_msgs::PointStamped LH_footholdResult;
        LH_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                    gridmap_, 
                                    center, 
                                    footRadius_, 
                                    searchRadius_,
                                    LH_polygon, 
                                    std::ref(LH_footholdResult), 
                                    std::ref(LH_footholdValidation_));

        center.x() = LF_nextPosition_.point.x;
        center.y() = LF_nextPosition_.point.y;   
        geometry_msgs::PointStamped LF_footholdResult;
        LF_planThread_ = std::thread(&FootholdPlanner::checkFoothold, this, 
                                    gridmap_, 
                                    center, 
                                    footRadius_, 
                                    searchRadius_,
                                    LF_polygon, 
                                    std::ref(LF_footholdResult), 
                                    std::ref(LF_footholdValidation_));

        //! 在此阻塞，直到4个线程都跑完了...
        RF_planThread_.join();
        RH_planThread_.join();
        LH_planThread_.join();
        LF_planThread_.join();      

        //! 规划结果，只有4个足端都可通行，才算规划成功
        footholdValidation_ = RF_footholdValidation_ & RH_footholdValidation_ & LH_footholdValidation_ & LF_footholdValidation_;
        if(debug_){
            ROS_INFO_STREAM("footholdValidation_ = "<<footholdValidation_<<"  RF_footholdValidation_ = "<<RF_footholdValidation_<<
                "  RH_footholdValidation_ = "<<RH_footholdValidation_<<"  LH_footholdValidation_ = "<<LH_footholdValidation_<<
                "  LF_footholdValidation_ = "<<LF_footholdValidation_);
        }
        
        //! 当规划成功以后的后续处理
        foothold_planner_msgs::Foothold footholdResult;
        if(footholdValidation_){
            if(debug_) ROS_INFO("All planned next 4 footholds are traversable.");
            //! 本次循环规划出来的4个落脚点  
            if(debug_) ROS_INFO_STREAM("RF_footholdResult: "<<RF_footholdResult);
            if(debug_) ROS_INFO_STREAM("RH_footholdResult: "<<RH_footholdResult);
            if(debug_) ROS_INFO_STREAM("LH_footholdResult: "<<LH_footholdResult);
            if(debug_) ROS_INFO_STREAM("LF_footholdResult: "<<LF_footholdResult);

            //! 保存到全局落脚点数组中
            footholdResult.gait_cycle_id = gaitCycleIndex;
            globalFootholds_.gait_cycles_succeed = gaitCycleIndex + 1;
            globalFootholds_.success = true;             

            footholdResult.point = RF_footholdResult.point;
            footholdResult.foot_id = 0;
            globalFootholds_.footholds.push_back(footholdResult);

            footholdResult.point = RH_footholdResult.point;
            footholdResult.foot_id = 1;
            globalFootholds_.footholds.push_back(footholdResult);      

            footholdResult.point = LH_footholdResult.point;
            footholdResult.foot_id = 2;
            globalFootholds_.footholds.push_back(footholdResult);    

            footholdResult.point = LF_footholdResult.point;
            footholdResult.foot_id = 3;
            globalFootholds_.footholds.push_back(footholdResult);                                            

            //! 发布全局落脚点markders           
            publishGlobalFootholdMarkers(RF_footholdResult, RH_footholdResult, LH_footholdResult, LF_footholdResult);

            //! 本次步态周期的规划结果作为下一个步态周期的起始落脚点位置
            RF_currentPosition_ = RF_footholdResult;       
            RH_currentPosition_ = RH_footholdResult;       
            LH_currentPosition_ = LH_footholdResult;       
            LF_currentPosition_ = LF_footholdResult;    
        }
        else{
            if(debug_) ROS_ERROR("No traversable footholds are found!");   
            globalFootholds_.success = false;         
            // break; // 若本次步态周期规划失败，那么就停止后面其他步态的继续规划    
        }

    } // for, 所有步态周期规划结束

    //! 服务response.
    response.footholds = globalFootholds_;

    //! 发布全局落脚点规划结果
    globalFootholdsPub_.publish(globalFootholds_);


    ROS_INFO("Global footholds plan Done.");
    cout<<"===================================================================================="<<endl;
    cout<<endl<<endl;

    return true;
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
        footholdResult.point.z = getFootholdMeanHeight(gridmap, center, footRadius);
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

  ROS_INFO("Done.");
      
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

        // for(grid_map::CircleIterator iterator2(gridmap, footholdCenter, footRadius); !iterator2.isPastEnd(); ++iterator2)
        // {
        //   grid_map::Index CircleIterator_index((*iterator2).transpose());
        //   grid_map::Position CircleIterator_position;
        //   gridmap.getPosition(CircleIterator_index, CircleIterator_position);
        //   //ROS_ERROR_STREAM("foot_id: " << foot_id << "  CircleIterator_index: " << CircleIterator_index << "  CircleIterator_position: " << CircleIterator_position); // ??????
        //   //gridmap.getPosition((*iterator2).transpose(), CircleIterator_position)
        //   if( gridmap.at("traversability", *iterator2) < candidateFootholdThreshold || false == polygon.isInside(CircleIterator_position) )	      
        //   {
        // validation = false;
        // break; 
        //   }
        //   validation = true;
        // }

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

    ROS_INFO("Done.");

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

  ROS_INFO("Done.");
      
  return validation;
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
    polygonCenter.z = 0.0;
    
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
                                                float radius)
{
    if(debug_){
        ROS_INFO("Get foothold height ...");
        // ROS_INFO_STREAM("Foot center height = " << gridmap.atPosition("elevation", center)); // 若所查询的坐标超出TM地图大小，将导致TM节点挂掉！      
    }
  
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
            if(debug_) ROS_INFO_STREAM("Cell i " << i << " foot height = " << iHeight);
        }       
    }    
    
    if(i != 0){
        meanHeight = meanHeight / i;
    }else{
        meanHeight = iHeight;
    }
    
    if(debug_) ROS_INFO("Done.");
    return meanHeight;
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


void FootholdPlanner::publishGlobalFootholdMarkers( geometry_msgs::PointStamped rfFootholdResult, 
                                                    geometry_msgs::PointStamped rhFootholdResult, 
                                                    geometry_msgs::PointStamped lhFootholdResult, 
                                                    geometry_msgs::PointStamped lfFootholdResult)
{
    //visualization_msgs::Marker RF_sphere_list, RH_sphere_list, LH_sphere_list, LF_sphere_list;
    
    RF_sphere_list.type = RH_sphere_list.type = LH_sphere_list.type = LF_sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;   
    RF_sphere_list.id = 0;
    RH_sphere_list.id = 1;
    LH_sphere_list.id = 2;
    LF_sphere_list.id = 3;    
    
    RF_sphere_list.action = RH_sphere_list.action = LH_sphere_list.action = LF_sphere_list.action = visualization_msgs::Marker::ADD;
    RF_sphere_list.lifetime = RH_sphere_list.lifetime = LH_sphere_list.lifetime = LF_sphere_list.lifetime = ros::Duration(0);
    
    RF_sphere_list.header.frame_id = RH_sphere_list.header.frame_id = LH_sphere_list.header.frame_id = LF_sphere_list.header.frame_id = mapFrame_;
    RF_sphere_list.header.stamp = RH_sphere_list.header.stamp = LH_sphere_list.header.stamp = LF_sphere_list.header.stamp = ros::Time::now();    

    RF_sphere_list.scale.x = RF_sphere_list.scale.y = RF_sphere_list.scale.z = 0.05;;
    RH_sphere_list.scale.x = RH_sphere_list.scale.y = RH_sphere_list.scale.z = 0.05;
    LH_sphere_list.scale.x = LH_sphere_list.scale.y = LH_sphere_list.scale.z = 0.05;
    LF_sphere_list.scale.x = LF_sphere_list.scale.y = LF_sphere_list.scale.z = 0.05;	
    
    RF_sphere_list.color.r = 1.0;
    RF_sphere_list.color.a = 1.0;
    RH_sphere_list.color.g = 1.0;
    RH_sphere_list.color.a = 1.0;
    LH_sphere_list.color.b = 1.0;
    LH_sphere_list.color.a = 1.0;
    LF_sphere_list.color.r = 1.0;
    LF_sphere_list.color.g = 1.0;	
    LF_sphere_list.color.a = 1.0;	
    
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
    
    //ROS_ERROR_STREAM("RF_sphere_list.points = " << RF_sphere_list.points[0]);
    
    /*
    RF_sphere_list.points.push_back(rfFootholdResult.position);
    RH_sphere_list.points.push_back(rhFootholdResult.position);
    LH_sphere_list.points.push_back(lhFootholdResult.position);
    LF_sphere_list.points.push_back(lfFootholdResult.position);	  
    */

    globalFootholdsMarkerPub_.publish(RF_sphere_list);
    globalFootholdsMarkerPub_.publish(RH_sphere_list);
    globalFootholdsMarkerPub_.publish(LH_sphere_list);
    globalFootholdsMarkerPub_.publish(LF_sphere_list);	   	
}  


}