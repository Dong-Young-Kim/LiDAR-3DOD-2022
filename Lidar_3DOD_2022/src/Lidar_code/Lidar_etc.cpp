#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

Fps::Fps(): m_count(0){ prev_clock = 0; };


void Fps::update(){
    double tmp = ros::Time::now().toSec();        
    cur_clock = tmp;
    interval = cur_clock - prev_clock;
    prev_clock = cur_clock;
    m_fps = 1 / interval;
    m_count++;
        
    cout << "Interval: " << interval << " sec";
    cout << "\tFPS: " << m_fps << " frame/sec";
    cout << "\tLoop " << m_count << endl;
}