#pragma once
#ifndef CODE_CRAFT_2019_TOPO_H
#define CODE_CRAFT_2019_TOPO_H
#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

struct CarLocationOnRoad {
	ID car_id;
	STATE state;//����״̬
	int location;//���ڵ�·�е�λ��
	int index;//��ǰ��·��·���е�index
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
	std::vector<std::vector<InterRoutine *>> **car_same;//���������㼰�ƻ�����ʱ�䶼��ͬ�ĳ���
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
	Turn getRoadTurn(ID cross_id, ID road1, ID road2);//��ȡcross��������·��ת��

	std::vector<Road *> roads;
	std::vector<Cross *> crosses;
	std::vector<Car *> cars;

	Road ***adjRoad;

	ID **adjRoadID;//����·��֮��ĵ�·ID��ԭʼ��·��
	ID **pathID;//���·������crossID
	Length **sPathLen;//���·����
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
