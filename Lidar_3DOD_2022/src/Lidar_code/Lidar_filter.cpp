#include <Lidar_3DOD_2022/Lidar_declare.h>

using namespace std;

void Filter::DY_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag){
    if(!flag) return;
    vector<pair<PXYZI,string>>::iterator it = sorted_OBJ.begin();
    for (int i = 0; i < sorted_OBJ.size(); i++, it++) {
        if(sorted_OBJ[i].first.z < Ransac_Z_ROI){
            it = sorted_OBJ.erase(it);
            it--;
            i--;
        }
    }
}

void Filter::DY_filter(vector<objInfo>& objs, bool flag){
    if(!flag) return;
    for (vector<objInfo>::iterator it = objs.begin(); it != objs.end();){
        if (it->zMax < Ransac_Z_ROI){
            //for (it->objPoints)
            it = objs.erase(it);            
        }
        else{
            it++;
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