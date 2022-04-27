#include <Lidar_process_div/Lidar_declare.h>

using namespace std;

string send_msg_minmax(float xMin,float xMax, float yMin, float yMax){
    string tmp_xMax = (xMax < 0) ? to_string(((int)(xMax * -100) / 2) * 2 + 1) : to_string(((int)(xMax * 100) / 2) * 2);
    string tmp_xMin = (xMin < 0) ? to_string(((int)(xMin * -100) / 2) * 2 + 1) : to_string(((int)(xMin * 100) / 2) * 2);
    string tmp_yMax = (yMax < 0) ? to_string(((int)(yMax * -100) / 2) * 2 + 1) : to_string(((int)(yMax * 100) / 2) * 2);
    string tmp_yMin = (yMin < 0) ? to_string(((int)(yMin * -100) / 2) * 2 + 1) : to_string(((int)(yMin * 100) / 2) * 2);
    string tmp, st;

    for(int i = 4; i > tmp_xMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = tmp + tmp_xMin;
    tmp.clear();
    for(int i = 4; i > tmp_xMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_xMax;
    tmp.clear();
    for(int i = 4; i > tmp_yMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMin;
    tmp.clear();
    for(int i = 4; i > tmp_yMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMax;
    return st;
}

string send_msg_DXY(PXYZI obj) {
	string dist = to_string(((int)(cal_dist(obj.x, obj.y) * 100) / 2) * 2);
	string tmp_x = (obj.x < 0) ? to_string(((int)(obj.x * -100) / 2) * 2 + 1) : to_string(((int)(obj.x * 100) / 2) * 2);
	string tmp_y = (obj.y < 0) ? to_string(((int)(obj.y * -100) / 2) * 2 + 1) : to_string(((int)(obj.y * 100) / 2) * 2);
	string tmp, st;

	for (int i = 4; i > dist.size(); i--) {
		tmp = "0" + tmp;
	}
	st = tmp + dist;
	tmp.clear();
	for (int i = 4; i > tmp_x.size(); i--) {
		tmp = "0" + tmp;
	}
	st = st + tmp + tmp_x;
	tmp.clear();
	for (int i = 4; i > tmp_y.size(); i--) {
		tmp = "0" + tmp;
	}
	st = st + tmp + tmp_y;
	return st;
}

string send_msg_cnt(int sz){
    string st = to_string(sz);
    string tmp;
    for (int i = 4; i > st.size(); i--) {
		tmp = "0" + tmp;
	}
    tmp += st;
    return tmp;
}

void msg_process(vector<pair<PXYZI,string>>& sorted_OBJ){
    std_msgs::String Lidar_string;
    Lidar_string.data = send_msg_cnt(sorted_OBJ.size());
    for(int i = 0; i < sorted_OBJ.size(); i++){
        Lidar_string.data = Lidar_string.data + send_msg_DXY(sorted_OBJ[i].first) + sorted_OBJ[i].second;
        //Lidar_string.data +="/";
    }
    //cout << Lidar_string.data << endl;
    pub_msg.publish(Lidar_string);
}