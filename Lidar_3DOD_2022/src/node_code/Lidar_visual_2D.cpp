#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

string img;

void show_process(const Lidar_3DOD_2022::obj_msg objs){
    RT1.start();
    
    //cout << objs.objc << endl;

    cv::Mat src = cv::imread(img, cv::IMREAD_COLOR);

    //car
    cv::rectangle(src, cv::Rect(236,525,20,50), cv::Scalar(150,100,0), 20, 8, 0);

    //object
    for (int i = 0; i < objs.objc; i++){
        float uCor = 236 - objs.y[i] * 29.1;
        float vCor = 525 - objs.x[i] * 29.1;
        cv::circle(src,cv::Point(uCor,vCor),3.5,cv::Scalar(0,255,205),-1);
    }

    cv::imshow("Lidar Detected objs", src);
    cv::waitKey(2);


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
