#include "Solver.h"

using namespace std;

namespace codecraft2019 {

void Solver::run() {
    init();
    // ����㷨������
}

// �˺������ڲ���IO��IDӳ���Ƿ���ȷ��������㷨�޹ء�
void Solver::testIO() {
	output_->routines.resize(ins_->raw_cars.size());
	for (int i = 0; i < ins_->raw_cars.size(); ++i) {
		output_->routines[i]->car_id = ins_->raw_cars[i].id;
		output_->routines[i]->start_time = ins_->raw_cars[i].plan_time;
		output_->routines[i]->roads.push_back(ins_->raw_cars[i].from);
		output_->routines[i]->roads.push_back(ins_->raw_cars[i].to);
	}
}

// �������㷨֮ǰ��Ҫ��һЩ��ʼ���Ķ�����
void Solver::init() {
    // ��������֮������·����shortest_paths
    shortest_paths.reserve(topo.cross_size*topo.cross_size*topo.cross_size/3);
    shortest_paths.resize(topo.cross_size);
    for (ID i = 0; i < topo.cross_size; ++i) {
        shortest_paths[i].resize(topo.cross_size);
        for (ID j = 0; j < topo.cross_size; ++j) {
            if (topo.pathID[i][j] != INVALID_ID) {
                ID temp = i;
                while (temp != j) {
                    ID next = topo.pathID[temp][j];
                    shortest_paths[i][j].push_back(topo.adjRoadID[temp][next]);
                    temp = next;
                }
            }
        }
    }
    // �������ݵĳ�ʼ��������
}

void Solver::init_solution()
{
	int car_size = ins_->raw_cars.size();
	int cross_size = ins_->raw_crosses.size();
	ID temp, next;
	Time temp_time,latest_time=-1;
	Speed speed=0;
	//vector<Car> notPlanCar;

	for (auto i = 0; i < car_size; ++i) {
		Routine *routine = new Routine;
		RawCar *raw_car = &ins_->raw_cars[i];
		routine->car_id = raw_car->id;
		if (raw_car->plan_time > latest_time) latest_time = raw_car->plan_time;
		temp = raw_car->from;
		while (temp != raw_car->to)
		{
			next = topo.pathID[temp][raw_car->to];
			routine->roads.push_back(topo.adjRoadID[temp][next]);
			temp = next;
		}

		temp_time = 0;
		for (auto j = 0; j < routine->roads.size(); ++j) {
			RawRoad road = ins_->raw_roads[routine->roads[j]];
			speed = min(road.speed, raw_car->speed);
			temp_time += road.length / speed;
		}//����û��ӵ��ʱ�ĺ�ʱ
		InterRoutine *interR = new InterRoutine(topo.cars[i],temp_time);
		output_->routines.push_back(routine);//ʹ�����·����ʱroutine�ĳ���ʱ�仹δȷ��
		//aux.car_same[raw_car->from][raw_car->to][raw_car->plan_time].push_back(interR);
	}

	
	/**����������������ʵ�ʳ���ʱ��*/
	vector<Routine *> *time_car;
	time_car = new vector<Routine *>[latest_time+1];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = ins_->raw_cars[i].plan_time;
		time_car[start_time].push_back(output_->routines[i]);
	}
	/*for (int i = 0; i <= latest_time; ++i) {
		if (time_car[i].size()>0){
			std::cout << "the plan time is" << i << " the carnum is"<<time_car[i].size()<<std::endl;
			int cnt = i;
			while (time_car[i].size() != 0)
			{
				int hasChoose = 0;
				for (int j = 0; j < time_car[i].size(); ++j) {
					if ((rand() % (time_car[i].size() - j)) < (150 - hasChoose)) {
						time_car[i][j]->start_time = cnt;
						hasChoose++;
					}
					else {
						time_car[i][j]->start_time = cnt + 1;
					}
				}

			}
		}
		
	}*/
	std::cout << "the latest time is" << latest_time << std::endl;
}

/* 
 * ͬʱ����һ����Ŀ��������������ȫ�����յ���ٳ�����һ������,ֱ�����г������滮·�ߡ�
 * ÿ������·��Ĭ���������·����Ĭ�ϳ��������ӵ�£���ӵ��ʱ��Ϊ����ʱ���һ�룩��
 * ÿ����һ���⡾���ܲ��Ϸ�����������һ��checker�������Ϸ����Ӵ�ƻ���������Ŀ�����������Ŀ��
 */
void Solver::binary_generate_solution() {
    int car_num_left = 1;               // �����������ָ��
    int car_num_right = topo.car_size;  // ���������ұ�ָ��
    while (car_num_left < car_num_right) {
        int car_num_mid = car_num_left + ((car_num_right - car_num_left) / 2);
        // TODO....
    }
}

void Solver::check_solution()
	{
		int car_size = ins_->raw_cars.size();
		int road_size = ins_->raw_roads.size();
		int cross_size = ins_->raw_crosses.size();
		vector<ID> *time_car;
		STATE *car_state;

		car_state = new STATE[car_size];
		time_car = new vector<ID>[total_time];
		for (int i = 0; i < car_size; ++i) {
			Time start_time = output_->routines[i]->start_time;
			time_car[start_time].push_back(i);
		}
		for (int i = 0; i <= MAX_TIME; i++) {
			for (int r = 0; r < road_size; ++r) {
				/* �����г������е���  */
				Road *road = topo.roads[r];
				int roads_num = road->raw_road->is_duplex ? 2 : 1;//˫�򳵵��͵��򳵵�
				for (int j = 0; j < roads_num; ++j) {// 0��ԭ������ʻ�ĳ��������¼��from��toһ�£���1Ϊ����������ʻ�ĳ���
					for (int c = 0; c < road->channel_carL[j].size(); ++c) {//��ȡ����
						for (int cL = 0; cL < road->channel_carL[j][c].size(); ++cL) {
							CarLocationOnRoad *carL = road->channel_carL[j][c][cL];//�����ڵĳ���id��λ��
							Speed speed = min(ins_->raw_cars[carL->car_id].speed, road->raw_road->speed);
							if (cL == 0) {//˵���ǳ����ڵĵ�һ����
								if (carL->location + speed > road->raw_road->length) {//���·��
									carL->state = STATE_waitRun;
								}
								else {
									carL->location = carL->location + speed;
									carL->state = STATE_terminated;
								}
							}
							else {
								CarLocationOnRoad *prev_carL = road->channel_carL[j][c][cL - 1];
								if (prev_carL->location - carL->location > speed) {//ǰ��������ڸó�1��ʱ�䵥λ�ڿ���ʻ�������룬�������赲
									carL->location += speed;
									carL->state = STATE_terminated;
								}
								else {
									if (prev_carL->state == STATE_terminated) {
										speed = min(prev_carL->location - carL->location - 1, speed);
										carL->location += speed;
										carL->state = STATE_terminated;
									}
									else {
										carL->state = STATE_waitRun;
									}
								}
							}
						}
					}
				}
			}

			for (int c = 0; c < cross_size; ++c) {
				Cross *cross = topo.crosses[c];
				for (int k = 0; k <cross->road.size(); ++k) {//����ID�����road
					Road *road = cross->road[k];
					int road_direction;

					/* �жϵ�·�ķ��� */
					if (road->to == cross) {
						road_direction = 0;
					}
					else if (road->raw_road->is_duplex && road->from == cross) {
						road_direction = 1;
					}
					else {//˵���õ�·�޷���·��
						continue;
					}
					//��ȡ��ǰ��·�ϵĳ�������ǰ��Ҫ������ȷ����·�ڵȴ���ʻ�ĳ�����״̬
					//���Գ�·�ڵĳ�������Щ
					for (int c = 0; c < road->channel_carL[road_direction].size(); ++c) {//����
						CarLocationOnRoad *carL = road->channel_carL[road_direction][c][0];//��һ���ȼ��ĳ���
						if (carL->state == STATE_terminated) { //�����ڵ�һ����Ϊ��ֹ״̬����ó�������������Ҳ���Խ�����ֹ״̬
							CarLocationOnRoad *prev_carL = carL;
							for (int cL = 1; cL < road->channel_carL[road_direction][c].size(); cL++) {
								CarLocationOnRoad *next_carL = road->channel_carL[road_direction][c][cL];
								Speed speed = min(prev_carL->location - next_carL->location - 1,ins_->raw_cars[next_carL->car_id].speed);
								speed = min(speed, road->raw_road->speed);
								next_carL->location += speed;
								next_carL->state = STATE_waitRun;
								prev_carL = next_carL;
							}
						}
						else {//�����ڵ�һ����Ϊ�ȴ�״̬����ó���Ҫ��·��

						}

					}
				}
			}
			for (int j = 0; j < time_car[i].size(); j++) {//�����и�ʱ��Ҫ�����ĳ���
				ID first_road = output_->routines[time_car[i][j]]->roads[0];
				ID car_id = output_->routines[time_car[i][j]]->car_id;
				topo.carsWillOnRoad[first_road].push_back(car_id);
			}
		}
	}

}