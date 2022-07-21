#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

Fps::Fps(): m_count(0), prev_clock(1650000000){
    cout << "\033[1;42mprocessing node pid = " << getpid() << "\033[0m" << endl;
};

void Fps::update(){
    double cur_clock = ros::Time::now().toSec();
    interval = cur_clock - prev_clock;
    prev_clock = cur_clock;
    m_fps = 1 / interval;
    m_count++;
        
    printf("Interval : \033[1;35m%.1f\033[0m ms", interval * 1000);
    printf("\tFPS : \033[1;35m%.1f\033[0m frame/sec", m_fps);
    printf("\tLoop %zu\n", m_count);
    if (interval > 0.1) printf("\033[1;93m[WARN] low speed warning\033[0m\n");
}

RT::RT(){}

void RT::start(){
    prev_clock = ros::Time::now().toSec();
}

void RT::end_cal(const char* nodeName){
    cur_clock = ros::Time::now().toSec();
    interval = cur_clock - prev_clock;
    //cout << setprecision(2) << "\033[36m" << nodeName << " runtime\033[0m : \033[1;35m" << interval * 1000 << "\033[0m ms" << endl;
    printf("\033[36m%s runtime\033[0m : \033[1;35m%.1f\033[0m ms\n", nodeName, interval * 1000);
}