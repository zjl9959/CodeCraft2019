#pragma once
#ifndef CODE_CRAFT_2019_OUTPUT_H
#define CODE_CRAFT_2019_OUTPUT_H
/*
 * 将Solution转换为官方规定的输出格式。
 * 将结果保存到指定路径的文件中。
 */
#include "Instance.h"
#include<alg.h>
namespace codecraft2019 {

struct Routine {
    ID car_id;
    Time start_time;
	Time cost_time;
    std::vector<ID> roads;
	std::vector<ID> crosses;//由roads转换而来的路口
    Routine() {};
    Routine(const Routine &rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads = rhs.roads;
        crosses = rhs.crosses;
    }
    Routine(Routine &&rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads.swap(rhs.roads);
        crosses.swap(rhs.crosses);
    }

    Routine& operator = (const Routine &rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads = rhs.roads;
        crosses = rhs.crosses;
        return *this;
    }
    Routine& operator = (Routine &&rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads.swap(rhs.roads);
        crosses.swap(rhs.crosses);
        return *this;
    }
	void printRoads() {
		Log(FY_TEST) << "car_id"<<car_id;
		for (int i = 0; i < roads.size(); ++i) {
			Log(FY_TEST) << " " << roads[i];
		}
		Log(FY_TEST) << "\n";
	}
	Time get_time(Instance *ins_) {
		double approxi_time = 0;
		for (int j = 0; j < roads.size(); ++j) {
			RawRoad *raw_road = &ins_->raw_roads[roads[j]];
			Speed speed = min(ins_->raw_cars[car_id].speed, raw_road->speed);
			approxi_time += (raw_road->speed * 1.0) / speed;
		}
		cost_time = ceil(approxi_time);
		return cost_time;
	}
};

struct Output {
    Output(Instance* instance): ins_(instance) {};
    ~Output() { ins_ = nullptr; }
    bool save(Environment &env);
    Instance* ins_;
    std::vector<Routine> routines;
};

}

#endif // !CODE_CRAFT_2019_OUTPUT_H
