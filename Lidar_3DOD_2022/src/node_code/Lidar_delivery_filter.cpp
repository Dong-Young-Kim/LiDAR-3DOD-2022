#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

inline float retSize(float a, float b) { return abs(a - b);}

void yesBongYZsizeFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (retSize(it->zMax, it->zMin) > 1.7 ||
            retSize(it->zMax, it->zMin) < 1.4 ||
            retSize(it->yMax, it->yMin) > 0.7 ||
            retSize(it->yMax, it->yMin) < 0.4 ) it = retMsg.erase(it);
        else it++;
    }
}

void noBongYZsizeFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (retSize(it->zMax, it->zMin) > 0.7 ||
            retSize(it->zMax, it->zMin) < 0.4 ||
            retSize(it->yMax, it->yMin) > 0.7 ||
            retSize(it->yMax, it->yMin) < 0.4 ) it = retMsg.erase(it);
        else it++;
    }
}

void yesBongYZpositionFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (it->zMax < 1.5 ||
            it->zMax > 2.5 ) it = retMsg.erase(it);
        else it++;
    }
}

void noBongYZpositionFilter (vector<Lidar_3DOD_2022::object_msg>& retMsg){
    for (vector<Lidar_3DOD_2022::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (it->zMin < 0.5 ||
            it->zMax > 2.0 ) it = retMsg.erase(it);
        else it++;
    }
}




void delivery_filter_process(const Lidar_3DOD_2022::object_msg_arrConstPtr& objs){
    RT::start();

    vector<Lidar_3DOD_2022::object_msg> retMsgYesBong = objs->object_msg_arr;
    vector<Lidar_3DOD_2022::object_msg> retMsgNoBong = objs->object_msg_arr;
    cout << "initial size   " << retMsgYesBong.size() << endl;    

    yesBongYZsizeFilter     (retMsgYesBong);
    noBongYZsizeFilter      (retMsgNoBong);
    //yesBongYZpositionFilter (retMsgYesBong);
    //noBongYZpositionFilter  (retMsgNoBong);

    cout << "pole O size   " << retMsgYesBong.size() << endl; 
    cout << "pole X size   " << retMsgNoBong.size() << endl; 

    retMsgYesBong.insert(retMsgYesBong.end(), retMsgNoBong.begin(), retMsgNoBong.end());
    Lidar_3DOD_2022::object_msg_arr finMsg;
    finMsg.objc = retMsgYesBong.size();
    finMsg.object_msg_arr = retMsgYesBong;

    cout << "total size   " << retMsgYesBong.size() << endl; 

    pub_object.publish(finMsg);

    RT::end_cal("filter");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "delivery filter");         //node name 
	ros::NodeHandle nh;                         //nodehandle

	ros::Subscriber sub = nh.subscribe<Lidar_3DOD_2022::object_msg_arr> ("/Lidar_object", 1, delivery_filter_process);
    pub_object = nh.advertise<Lidar_3DOD_2022::object_msg_arr> ("/Lidar_delivery_filtered_object", 1);


    ros::spin();
}