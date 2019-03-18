#pragma once
#ifndef CODE_CRAFT_2019_TOPO_H
#define CODE_CRAFT_2019_TOPO_H
#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

struct CarLocationOnRoad {
	ID car_id;
	STATE state;
	int location;
};
typedef struct Cross Cross;
struct Road {
	RawRoad *raw_road;
	Cross *from, *to;
	std::vector<std::vector<std::vector<CarLocationOnRoad *>>> channel_carL;
	Road(RawRoad *raw_road,Cross *from,Cross *to):raw_road(raw_road),from(from),to(to) {
	}
};
struct Cross {
	RawCross *raw_cross;
	std::vector<Road *> road;
	Cross(RawCross *raw_cross) {
		this->raw_cross = raw_cross;
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
	std::vector<InterRoutine *> **car_same;//出发点和起点都相同的车辆
	Aux(int cross_size) {
		car_same = new std::vector<InterRoutine*>*[cross_size];
		for (int i = 0; i < cross_size; i++) {
			car_same[i] = new std::vector<InterRoutine *>[cross_size];
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

	ID **adjRoadID;
	ID **pathID;//最短路经过的crossID
	Length **sPathLen;//最短路长度
	Turn **RoadTurn;
	int cross_size;
	int car_size;
	int rsize;
	std::vector<std::vector<int>>  carsWillOnRoad;
	std::vector<std::vector<CarLocationOnRoad>> carsOnRoad;
	std::vector<std::vector<std::vector<std::vector<CarLocationOnRoad>>>> road_channel_car;
protected:
	Instance *ins_;
};
}
#endif // !CODE_CRAFT_2019_TOPO_H
