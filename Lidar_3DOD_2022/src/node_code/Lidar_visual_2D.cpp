#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

string img;

#define METRETOPIXEL 29.1 //convert to real object size(m) to pixel size

void flat_visual_process(const Lidar_3DOD_2022::object_msg_arrConstPtr& objs){
    RT::start();

    cv::Mat src = cv::imread(img, cv::IMREAD_COLOR);

    //car
    cv::rectangle(src, cv::Rect(236,525,20,50), cv::Scalar(150,100,0), 20, 8, 0);

    //object
    for (const Lidar_3DOD_2022::object_msg& msgobj : objs->object_msg_arr){
        float uCor = 236 - msgobj.y * METRETOPIXEL;
        float vCor = 525 - msgobj.x * METRETOPIXEL;
        float uSiz = (msgobj.yMax - msgobj.yMin) * METRETOPIXEL;
        float vSiz = (msgobj.xMax - msgobj.xMin) * METRETOPIXEL;
        
        cv::rectangle(src, cv::Rect(uCor - uSiz/2, vCor - vSiz/2, uSiz, vSiz), cv::Scalar(0,255,205), -1, 8, 0); //obj size
        cv::circle(src,cv::Point(uCor,vCor),3.5,cv::Scalar(0,0,255),-1); //center point
    }

    cv::imshow("Lidar Detected objs", src);
    cv::waitKey(2);


    RT::end_cal("visual");
    cout << "\033[1;34m--------------------------------------------------------\033[0m" << endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Visual_2D");         //node name 
	ros::NodeHandle nh;                         //nodehandle

    nh.getParam("/visual_2D_node/switch_visual_2D", switch_visual_2D);
    nh.getParam("/visual_2D_node/location", img);
    if (!switch_visual_2D) exit(0); //exit node if switch is 0

	ros::Subscriber sub = nh.subscribe<Lidar_3DOD_2022::object_msg_arr> ("/Lidar_object", 1, flat_visual_process);

    ros::spin();
}
