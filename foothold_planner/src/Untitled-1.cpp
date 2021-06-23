void setCurrentFootholdsPosition(bool RF_FIRST,
                            geometry_msgs::PointStamped &RF_currentPosition,
                            geometry_msgs::PointStamped &RH_currentPosition,
                            geometry_msgs::PointStamped &LH_currentPosition,
                            geometry_msgs::PointStamped &LF_currentPosition)
{
    if(RF_FIRST_){
        if(debug_) cout<<"RF and LH first move."<<endl;
        RF_currentPosition.header.frame_id = RH_currentPosition.header.frame_id 
                                           = LH_currentPosition.header.frame_id 
                                           = LF_currentPosition.header.frame_id 
                                           = mapFrame_;
        RF_currentPosition.point.x = laikagoKinematics_.lengthBase * 0.5 + isos_.skew;
        RF_currentPosition.point.y = -laikagoKinematics_.widthBase * 0.5;
        RF_currentPosition.point.z = 0;

        RH_currentPosition.point.x = -laikagoKinematics_.lengthBase * 0.5 - isos_.skew;
        RH_currentPosition.point.y = -laikagoKinematics_.widthBase * 0.5;
        RH_currentPosition.point.z = 0;

        LH_currentPosition.point.x = -laikagoKinematics_.lengthBase * 0.5 + isos_.skew;
        LH_currentPosition.point.y = laikagoKinematics_.widthBase * 0.5;
        LH_currentPosition.point.z = 0;

        LF_currentPosition.point.x = laikagoKinematics_.lengthBase * 0.5 - isos_.skew;
        LF_currentPosition.point.y = laikagoKinematics_.widthBase * 0.5;
        LF_currentPosition.point.z = 0;
    }else{
        if(debug_) cout<<"LF and RH first move."<<endl;
        RF_currentPosition.header.frame_id = RH_currentPosition.header.frame_id 
                                                    = LH_currentPosition.header.frame_id 
                                                    = LF_currentPosition.header.frame_id 
                                                    = mapFrame_;
        RF_currentPosition.point.x = laikagoKinematics_.lengthBase*0.5 - isos_.skew;
        RF_currentPosition.point.y = -laikagoKinematics_.widthBase*0.5;
        RF_currentPosition.point.z = 0;

        RH_currentPosition.point.x = -laikagoKinematics_.lengthBase*0.5 + isos_.skew;
        RH_currentPosition.point.y = -laikagoKinematics_.widthBase*0.5;
        RH_currentPosition.point.z = 0;    

        LH_currentPosition.point.x = -laikagoKinematics_.lengthBase*0.5 - isos_.skew;
        LH_currentPosition.point.y = laikagoKinematics_.widthBase*0.5;
        LH_currentPosition.point.z = 0;    

        LF_currentPosition.point.x = laikagoKinematics_.lengthBase*0.5 + isos_.skew ;
        LF_currentPosition.point.y = laikagoKinematics_.widthBase*0.5;
        LF_currentPosition.point.z = 0;           
    }

}