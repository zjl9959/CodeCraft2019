#include "Output.h"

#include <fstream>

using namespace std;

namespace codecraft2019 {

bool Output::save(Environment & env) {
    if (!ins_->valid) {
        Log(Log::Fatal) << "[Fatal] Instance is invalid!" << endl;
        return false;
    }
    ofstream ofs;
    ofs.open(env.answer_path, ios::out);
    if (!ofs.is_open()) {
        Log(Log::Fatal) << "[Fatal] Invalid answer path!" << endl;
    } else {
        ofs << "#(carId,StartTime,RoadId...)" << endl;
        for (int i = 0; i < routines.size(); ++i) { // 逐行打印每辆车的规划路径。
            ofs << "(" << ins_->car_map.toArbitraryId(routines[i].car_id)
                << "," << routines[i].start_time;
            for (int j = 0; j < routines[i].roads.size(); ++j) {
                ofs << "," << ins_->road_map.toArbitraryId(routines[i].roads[j]);
            }
            ofs << ")" << endl;
        }
    }
    ofs.close();
    return true;
}

}
