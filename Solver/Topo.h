#pragma once
#ifndef CODE_CRAFT_2019_TOPO_H
#define CODE_CRAFT_2019_TOPO_H
#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

struct CarLocationOnRoad {
	ID car_id;
	STATE state;//车的状态
	int location;//车在道路中的位置
	int index;//当前道路在路径中的index
	ID channel_id;
	Turn turn;
	Time start_time;
};
typedef struct Cross Cross;
typedef struct Road Road;

struct Cross {
	RawCross *raw_cross;
	std::vector<Road *> road;
	Cross(RawCross *raw_cross) {
		this->raw_cross = raw_cross;
	}
};
struct Road {
	RawRoad *raw_road;
	Cross *from, *to;
	ID from_id, to_id;
	std::vector<std::vector<CarLocationOnRoad *>> channel_carL;
	std::vector<CarLocationOnRoad *> waitOutCarL;
	std::vector<CarLocationOnRoad *> willOnRoad;
	Road(RawRoad *raw_road,Cross *from,Cross *to):raw_road(raw_road),from(from),to(to) {
		from_id = from->raw_cross->id;
		to_id = to->raw_cross->id;
	}

};

struct Car{
	RawCar *raw_car;
	Cross *from, *to;
	Car(RawCar *raw_car) {
		this->raw_car = raw_car;
	}
};
struct InterRoutine {
	Car *car;
	Time run_time;
	InterRoutine(Car *car, Time run_time) :car(car), run_time(run_time) {}
};
struct Aux {
	std::vector<std::vector<InterRoutine *>> **car_same;//出发点和起点及计划出发时间都相同的车辆
	Aux(int cross_size) {
		car_same = new std::vector<std::vector<InterRoutine *>>*[cross_size];
		for (int i = 0; i < cross_size; i++) {
			car_same[i] = new std::vector<std::vector<InterRoutine *>>[cross_size];
		}
		for (int i = 0; i < cross_size; i++) {
			for (int j = 0; j < cross_size; ++j) {
				car_same[i][j].resize(LATEST_PLAN_TIME);
			}
		}
	}
};
class Topo {
public:
	Topo(Instance *ins);
	void Floyd();
	void printPath();
	void init_myTopo();
	Turn getRoadTurn(ID cross_id, ID road1, ID road2);//获取cross的两条道路的转向

	std::vector<Road *> roads;
	std::vector<Cross *> crosses;
	std::vector<Car *> cars;

	Road ***adjRoad;

	ID **adjRoadID;//两个路口之间的道路ID（原始道路）
	ID **pathID;//最短路经过的crossID
	Length **sPathLen;//最短路长度
	Turn **RoadTurn;
	int cross_size;
	int car_size;
	int rsize;
	std::vector<CarLocationOnRoad *> allCarLOnRoad;
protected:
	Instance *ins_;
};
}
#endif // !CODE_CRAFT_2019_TOPO_H
