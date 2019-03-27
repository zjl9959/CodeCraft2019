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
	int indexInRoutine; //�ó�������һ��routine
	ID channel_id;
	Turn turn;
	Time start_time;
	int real_speed; //������ʵ����ʻ�ٶ�
};
typedef struct Cross Cross;
typedef struct Road Road;

struct Cross {
	RawCross *raw_cross;
	std::vector<Road *> road;//��·�ڷ����·
	std::vector<Road *> inCrossRoad;//��·�ڷ����·
	Cross(RawCross *raw_cross) {
		this->raw_cross = raw_cross;
	}
};
struct Road {
	RawRoad *raw_road;
	Cross *from, *to;
	ID from_id, to_id;
	int vacancy_thresh; // ��λ����ֵ
	std::vector<std::vector<CarLocationOnRoad *>> channel_carL;
	std::vector<CarLocationOnRoad *> waitOutCarL;
	std::vector<CarLocationOnRoad *> willOnRoad;
	Road(RawRoad *raw_road,Cross *from,Cross *to):raw_road(raw_road),from(from),to(to) {
		from_id = from->raw_cross->id;
		to_id = to->raw_cross->id;
		vacancy_thresh = (raw_road->channel*raw_road->length*1.0) / 2.0;
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
	//std::vector<std::vector<std::vector<int>>> roadNullNumOfTime; //��·i->j ��ʱ��Ŀ�λ��
	//std::vector<std::vector<int>> type_numTime; //ĳһʱ�̴ﵽ�յ��ĳһ�೵����Ŀ
	//int type_num;
	int real_car_num; // ʵ�ʲ�����ȵĳ�����
	Aux(int cross_size,int road_size) {
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
