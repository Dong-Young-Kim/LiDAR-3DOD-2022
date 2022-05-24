#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

string img;

void show_process(const Lidar_3DOD_2022::obj_msg objs){
    RT1.start();
    
    cout << objs.objc << endl;

    cv::Mat src = cv::imread(img, cv::IMREAD_COLOR);

    //cv::rectangle(src, cv::Rect(20,20,40,20), cv::Scalar(0,0,255), cv::FILLED, -1,0);

    cv::imshow("Lidar Detected objs", src);
    cv::waitKey(1);


    RT1.end_cal("visual");
    cout << "-------------------------------------------------" << endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Visual_2D"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/visual_2D_node/location", img);

	ros::Subscriber sub = nh.subscribe<Lidar_3DOD_2022::obj_msg> ("/Lidar_obj", 100, show_process);

    ros::spin();
}
