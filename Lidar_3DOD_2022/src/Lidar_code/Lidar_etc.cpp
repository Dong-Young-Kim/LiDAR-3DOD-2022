#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

Fps::Fps(): m_count(0){ prev_clock = 1650000000; };

void Fps::update(){
    double cur_clock = ros::Time::now().toSec();
    interval = cur_clock - prev_clock;
    prev_clock = cur_clock;
    m_fps = 1 / interval;
    m_count++;
        
    printf("Interval : \033[1;35m%.3f\033[0m sec", interval);
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
    cout << nodeName << " runtime: " << interval * 1000 << " ms" << endl;
}