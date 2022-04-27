#include <Lidar_process_div/Lidar_declare.h>

using namespace std;

void Filter::DY_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag){
    if(!flag) return;
    vector<pair<PXYZI,string>>::iterator it = sorted_OBJ.begin();
    for (int i = 0; i < sorted_OBJ.size(); i++) {
        if(sorted_OBJ[i].first.z < Ransac_Z_ROI){
            it = sorted_OBJ.erase(it);
        }
    }
}

void Filter::jiwon_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag) {
    if(!flag) return;
    vector<pair<PXYZI,string>>::iterator it = sorted_OBJ.begin();
        for (int i = 0; i < sorted_OBJ.size(); i++) {
        it = sorted_OBJ.begin();
        for (int k = 0; k < i + 1; k++) {
            it++;
        }
        for (int j = i + 1; j < sorted_OBJ.size(); j++) {
            if (check_in(sorted_OBJ[i].first, sorted_OBJ[j].first)) {
                it = sorted_OBJ.erase(it);
                it--;
                j--;
            }
            it++;
        }
    }
}