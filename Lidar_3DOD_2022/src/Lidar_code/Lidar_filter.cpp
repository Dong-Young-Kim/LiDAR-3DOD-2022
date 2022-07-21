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
            cout << "DY_Filter erase obj info ";
            cout << "  z max = " << it->zMax;
            cout << "  z center = " << it->z << endl;
            it = objs.erase(it); //edit object vector
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

void Filter::jiwon_filter(vector<objInfo>& objs, bool flag) {
    if(!flag) return;
    for (vector<objInfo>::iterator iit = objs.begin(), jit; iit != objs.end(); iit++){
        jit = iit + 1;
        for(jit; jit != objs.end();){
            if(((abs(iit->x - jit->x) <= REMOVE_FACTOR) && (abs(iit->y - jit->y) <= REMOVE_FACTOR))){
                cout << "jiwon_Filter erase obj info " << endl;
                //iit->objPoints->indices += jit->objPoints->indices;
                cout << "indices size" << iit->objPoints->indices.size() << endl;
                jit = objs.erase(jit); //edit object vector
            }
            else{
                jit++;
            }
        }

    }
}