#include "Solver.h"

using namespace std;

namespace codecraft2019 {

// �˺������ڲ���IO��IDӳ���Ƿ���ȷ��������㷨�޹ء�
void Solver::testIO() {
    output_->routines.resize(ins_->cars.size());
    for (int i = 0; i < ins_->cars.size(); ++i) {
        output_->routines[i].car_id = ins_->cars[i].id;
        output_->routines[i].start_time = ins_->cars[i].plan_time;
        output_->routines[i].roads.push_back(ins_->cars[i].from);
        output_->routines[i].roads.push_back(ins_->cars[i].to);
    }
}
    
}
