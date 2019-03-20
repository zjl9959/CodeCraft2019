#include "Solver.h"

using namespace std;

namespace codecraft2019 {
bool carL_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {//location�󣬳�����С�����ȼ���
	bool res;
	if (carL1->location > carL2->location) {//carL1->location >= carL2->location
		return true;
	}
	else if (carL1->location == carL2->location){
		return carL1->channel_id < carL2->channel_id;
	}
	return false;
}

bool car_id_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {
	return carL1->car_id < carL2->car_id;
}
void Solver::run() {
    init();
    binary_generate_solution();

	/*init_solution_once();
	check_solution();*/
    // ����㷨������
}

// �˺������ڲ���IO��IDӳ���Ƿ���ȷ��������㷨�޹ء�
void Solver::testIO() {
	output_->routines.resize(ins_->raw_cars.size());
	for (int i = 0; i < ins_->raw_cars.size(); ++i) {
		output_->routines[i].car_id = ins_->raw_cars[i].id;
		output_->routines[i].start_time = ins_->raw_cars[i].plan_time;
		output_->routines[i].roads.push_back(ins_->raw_cars[i].from);
		output_->routines[i].roads.push_back(ins_->raw_cars[i].to);
	}
}

// �������㷨֮ǰ��Ҫ��һЩ��ʼ���Ķ�����
void Solver::init() {
    // ��������֮������·����shortest_paths
    shortest_paths.reserve(topo.cross_size*topo.cross_size*topo.cross_size/3);
    shortest_paths.resize(topo.cross_size);
    Log(Log::ZJL) << "\t------shortest path--------" << endl;
    Log(Log::ZJL) << "cross size:" << topo.cross_size << endl;
    for (ID i = 0; i < topo.cross_size; ++i) {
        shortest_paths[i].resize(topo.cross_size);
        for (ID j = 0; j < topo.cross_size; ++j) {
            //Log(Log::ZJL) << "from " << i << " to " << j << ":";
            if (topo.pathID[i][j] != INVALID_ID) {
                ID temp = i;
                while (temp != j && temp != INVALID_ID) {
                    ID next = topo.pathID[temp][j];
                    shortest_paths[i][j].push_back(topo.adjRoadID[temp][next]);
                    temp = next;
                    //Log(Log::ZJL) << shortest_paths[i][j].back() << " ";
                }
            }
            //Log(Log::ZJL) << endl;
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
		Routine routine;
		RawCar *raw_car = &ins_->raw_cars[i];
		routine.car_id = raw_car->id;
		if (raw_car->plan_time > latest_time) latest_time = raw_car->plan_time;
		temp = raw_car->from;
		while (temp != raw_car->to)
		{
			next = topo.pathID[temp][raw_car->to];
			routine.roads.push_back(topo.adjRoadID[temp][next]);
			temp = next;
		}

		temp_time = 0;
		for (auto j = 0; j < routine.roads.size(); ++j) {
			RawRoad road = ins_->raw_roads[routine.roads[j]];
			speed = min(road.speed, raw_car->speed);
			temp_time += road.length / speed;
		}//����û��ӵ��ʱ�ĺ�ʱ
		InterRoutine *interR = new InterRoutine(topo.cars[i],temp_time);
		output_->routines.push_back(routine);//ʹ�����·����ʱroutine�ĳ���ʱ�仹δȷ��
		aux.car_same[raw_car->from][raw_car->to][raw_car->plan_time].push_back(interR);
	}

	
	/**����������������ʵ�ʳ���ʱ��*/
	vector<Routine *> *time_car;
	time_car = new vector<Routine *>[latest_time+1];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = ins_->raw_cars[i].plan_time;
		time_car[start_time].push_back(&output_->routines[i]);
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


// �����ʼ��汾��
void Solver::binary_generate_solution() {
    Log(Log::ZJL) << "start binary search" << endl;
    int car_num_left = 1;
    int car_num_right = topo.car_size;
    const int total_car_num = ins_->raw_cars.size();
    output_->routines.clear();
    for (int i = 0; i < total_car_num; ++i) {   // ÿ�����������·��
        Routine routine;
        routine.car_id = ins_->raw_cars[i].id;
        routine.roads = shortest_paths[ins_->raw_cars[i].from][ins_->raw_cars[i].to];
        output_->routines.push_back(move(routine));
    }
    vector<Time> best_start_times;              // ���ŵĳ�������ʱ��
    best_start_times.reserve(total_car_num);
    Time best_schdeule_time = MAX_TIME;
    vector<pair<Time, ID>> cars_run_time;       // ����ÿ������·�����ٻ��ѵ�ʱ�䣨�������·�ߣ���
    cars_run_time.reserve(total_car_num);
    for (auto car = ins_->raw_cars.begin(); car != ins_->raw_cars.end(); ++car) {
        cars_run_time.push_back(make_pair(
            min_time_cost(car->id, car->from, car->to), car->id));
    }
    auto cmp_less = [](pair<Time, ID> &lhs, pair<Time, ID> &rhs) {return lhs.first < rhs.first; };
    auto cmp_greater = [](pair<Time, ID> &lhs, pair<Time, ID> &rhs) { return lhs.first > rhs.first; };
    sort(cars_run_time.begin(), cars_run_time.end(), cmp_less);  // ��·�Ϻ�ʱ�ٵĳ����ȳ�����
    int binsearch_turn = 1;
    while (car_num_left < car_num_right) {      // ���ֵ��δ�
        Log(Log::ZJL) << "\t-----binary search turn " << binsearch_turn << "------" << endl;
        int car_num_mid = car_num_left + ((car_num_right - car_num_left) / 2);
        vector<Time> start_times;
        start_times.resize(total_car_num);
        priority_queue<pair<Time, ID>, vector<pair<Time,ID>>, decltype(cmp_greater)> pqueue(cmp_greater);
        int start_car_num = 0;
        Time current_time = 0;
        while (start_car_num < total_car_num) { // ���ղ�ͬ�Ĳ�������Ϊÿ�������ó���ʱ�䡣
            for (int i = 0; i < cars_run_time.size() && pqueue.size() < car_num_mid; ++i) {
                ID car_id = cars_run_time[i].second;
                if (cars_run_time[i].first == MAX_TIME)
                    break;
                if (ins_->raw_cars[car_id].plan_time <= current_time) {
                    pqueue.push(make_pair(cars_run_time.back().first + current_time, car_id));
                    start_times[car_id] = current_time;
                    ++start_car_num;
                    cars_run_time[i].first = MAX_TIME;
                    Log(Log::ZJL) << "car " << car_id << " start at time " << current_time << endl;
                }
            }
            sort(cars_run_time.begin(), cars_run_time.end(), cmp_less);
            while (!pqueue.empty()) {           // ��������Ŀ�ĵصĳ���
                if (pqueue.top().first <= current_time) {
                    pqueue.pop();
                } else {
                    break;
                }
            }
            current_time++;
        }
        for (int i = 0; i < start_times.size(); ++i) {
            output_->routines[i].start_time = start_times[i];
        }
        Time cur_schedule_time = check_solution();
        Log(Log::ZJL) << "current time:" << current_time << " schedule time:" << cur_schedule_time << endl;
        if (cur_schedule_time != -1) {                 // ���ǺϷ��ģ�˵���������car_num_mid
            car_num_left = car_num_mid + 1;
            if (cur_schedule_time < best_schdeule_time) {  // ��¼���Ž⣬����check_solution�ܷ񴫻�׼ȷ�ĵ���ʱ�䣿
                best_schdeule_time = cur_schedule_time;
                best_start_times = start_times;
            }
        } else {
            car_num_right = car_num_mid - 1;
        }
    }
    for (int i = 0; i < best_start_times.size(); ++i) {
        output_->routines[i].start_time = best_start_times[i];
    }
}

int Solver::check_solution()
{
	
	vector<ID> *time_car;
	STATE *car_state;

	car_state = new STATE[car_size];
	time_car = new vector<ID>[100000];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = output_->routines[i].start_time;
		time_car[start_time].push_back(i);
	}
	inDst_num = 0;
	for (int j = 0; j < car_size; ++j) {
		Log(FY_TEST) << "car_id:" << ins_->changeToOriginalID(j, CarMap)<<"  ";
		for (int i = 0; i < output_->routines[j].roads.size(); ++i) {
			Log(FY_TEST) << ins_->changeToOriginalID(output_->routines[j].roads[i], RoadMap)<< "  ";
		}
		Log(FY_TEST) << endl;
	}
	
	for (int i = 0; i <= MAX_TIME; i++) {
		if (i == 12) {
			int j;
			j = i* 1;
		}

		for (int r = 0; r <double_road_size; ++r) {
			/* �����г������е���  */
			Road *road = topo.roads[r];
			driveAllCarJustOnRoadToEndState(road);
		}

		/*���ȵȴ����еĳ���*/
		for (int c = 0; c < cross_size; ++c) {//�Ƚ����еȴ���·�ڵļ�¼����
			Cross *cross = topo.crosses[c];

			for (int k = 0; k < cross->road.size(); ++k) {//ȷ����Щ������Ҫ����
				Road *road = cross->road[k];
				//��ȡ��ǰ��·�ϵĳ�������ǰ��Ҫ������ȷ����·�ڵȴ���ʻ�ĳ�����״̬
				//���Գ�·�ڵĳ�������Щ
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {//����
					if (road->channel_carL[ch].size() > 0) {
						CarLocationOnRoad *carL = road->channel_carL[ch][0];//ĳһ�������ڵĵ�һ����
						/*Log(FY_TEST) << "car_id: " <<ins_->changeToOriginalID( carL->car_id ,CarMap)<< "  location: " << carL->location
							<< "  road_id :" <<ins_->changeToOriginalID( road->raw_road->id ,RoadMap)<< endl;*/
						if (carL->state == STATE_waitRun) {//�����ڵ�һ����Ϊ�ȴ�״̬����ó�������Ҫ��·��
							recordProbOutCross(carL, cross, road);

							for (int cL = 1; cL < road->channel_carL[ch].size(); cL++) {//���ó��������п���������·�ڵĳ�����
								CarLocationOnRoad *next_carL = road->channel_carL[ch][cL];
								Speed speed = min(road->raw_road->speed, ins_->raw_cars[next_carL->car_id].speed);
								if (speed + next_carL->location > road->raw_road->length) {
									recordProbOutCross(next_carL, cross, road);
								}
							}

						}
					}
					
				}
				//�Կ��ܿ��Գ�·�ڵĳ����������ȼ�����
				sort(road->waitOutCarL.begin(), road->waitOutCarL.end(), carL_sort);
			}
		}

		while (true) {
			bool canSchedule = false;
			for (int c = 0; c < cross_size; ++c) {//����ID������ȸ���·��
				Cross *cross = topo.crosses[c];
				/*���յ�·ID����Գ������е���*/
				while (true) {
					bool isCarRun = false;
					for (int k = 0; k < cross->road.size(); ++k) {
						Road *road = cross->road[k];//����жϳ�ͻ
						bool conflict = false;
						int old_size = road->waitOutCarL.size();
						for (int r = 0; r < road->waitOutCarL.size(); ++r) {//�������ȼ�˳�����α���
							CarLocationOnRoad *carL = road->waitOutCarL[r];
							Road *next_road = getNextRoad(carL, cross);

							if (carL->turn != Front ) {//ֱ�е����ȼ�Ҫ��������,�����г�ͻ
								for (int k1 = 0; k1 < cross->road.size(); ++k1) {
									if (k1 != k) {
										Road *other_road = cross->road[k1];
										if (other_road->waitOutCarL.size() > 0) {
											CarLocationOnRoad *first_carL = other_road->waitOutCarL[0];//��ȡ��һ���ȼ�����
											Road *other_next_road = getNextRoad(first_carL, cross);
											if (other_next_road == next_road && first_carL->turn < carL->turn) {
												conflict = true;
												break;
											}
										}
									}
								}
								if (conflict)//�г�ͻ�˴ε��Ƚ���
									break;
							}
							if (moveToNextRoad(road, next_road, carL) ||
								carL->state == STATE_terminated) {//˵���ڸ�ʱ���Ѿ������ܳ�·��
								road->waitOutCarL.erase(road->waitOutCarL.begin());
								driveCarOnChannelToEndState(road, carL->channel_id);
								isCarRun = true;
								canSchedule = true;
								r = -1;
							}
						}
					}
					if (!isCarRun)
						break;
				}
			}
			if (!canSchedule)
				break;
		}

		bool isValid = true;
		for (int c = 0; c < cross_size && isValid; ++c) {
			Cross *cross = topo.crosses[c];
			for (int k = 0; k < cross->road.size(); ++k) {
				Road *road = cross->road[k];
				if (road->waitOutCarL.size() > 0) {
					isValid = false;
					break;
				}
			}
		}
		if (!isValid) {
			return -1;
		}

		driveCarInGarage();
		if (inDst_num == car_size) {
			cout << " the cost time of scheduler is " << i << endl;
			return i;
		}
		for (int j = 0; j < time_car[i].size(); j++) {//�����и�ʱ��Ҫ�����ĳ�����¼����
			ID first_road = output_->routines[time_car[i][j]].roads[0];
			ID car_id = output_->routines[time_car[i][j]].car_id;
			CarLocationOnRoad *new_carL = new CarLocationOnRoad;
			new_carL->car_id = car_id;
			new_carL->index = 0;
			new_carL->state = STATE_start;
			if (ins_->raw_roads[first_road].from == ins_->raw_cars[car_id].from) {
				topo.roads[first_road]->willOnRoad.push_back(new_carL);
			}
			else {
				topo.roads[first_road +road_size]->willOnRoad.push_back(new_carL);
			}
		}

#ifdef FY_TEST
		cout << "------------------- Time  " << i << "-------------------" << endl;
		for (int i = 0; i < road_size * 2; ++i) {
			Road *road = topo.roads[i];
			for (int c = 0; c < road->channel_carL.size(); ++c) {
				for (int cL = 0; cL < road->channel_carL[c].size(); ++cL) {
					CarLocationOnRoad *carL = road->channel_carL[c][cL];
					Log(FY_TEST) << "car_id: " << ins_->changeToOriginalID(carL->car_id, CarMap) << "  location: " << carL->location
						<< "  road_id :" << ins_->changeToOriginalID(road->raw_road->id, RoadMap) <<" ch: "<<
						carL->channel_id<<" from: "
						<< ins_->changeToOriginalID(road->from_id, CrossMap) << " to: "
						<< ins_->changeToOriginalID(road->to_id, CrossMap) << endl;
				}
			}
		}
#endif // FY_TEST
	}
}
void Solver::driveCarOnChannelToEndState(Road *road, int ch) {
	for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
		CarLocationOnRoad *carL = road->channel_carL[ch][cL];//�����ڵĳ���id��λ��
		if (carL->state == STATE_terminated) { continue; }

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
			CarLocationOnRoad *prev_carL = road->channel_carL[ch][cL - 1];
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
void Solver::driveAllCarJustOnRoadToEndState(Road *road)
{
	if (road == NULL) return;
	for (int c = 0; c < road->channel_carL.size(); ++c) {//��ȡ����
		for (int cL = 0; cL < road->channel_carL[c].size(); ++cL) {
			CarLocationOnRoad *carL = road->channel_carL[c][cL];//�����ڵĳ���id��λ��
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
				CarLocationOnRoad *prev_carL = road->channel_carL[c][cL - 1];
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
void Solver::driveCarInGarage()
{
	int rsize = road_size * 2;
	for (int i = 0; i < rsize; ++i) {
		Road *road = topo.roads[i];
		if (road) {//������������·��
			sort(road->willOnRoad.begin(), road->willOnRoad.end(), car_id_sort);
			for (int j = 0; j < road->willOnRoad.size(); ++j) {//����id˳��ʻ���·
				CarLocationOnRoad * carLWillOnRoad = road->willOnRoad[j]; 
				Speed speed = min(road->raw_road->speed, ins_->raw_cars[carLWillOnRoad->car_id].speed);
				bool canDriveIn = false;
				for (int c = 0; c < road->channel_carL.size(); ++c) {
					if (road->channel_carL[c].size() == 0) {
						carLWillOnRoad->channel_id = c;
						carLWillOnRoad->location = speed;
						road->channel_carL[c].push_back(carLWillOnRoad);
						canDriveIn = true;
						break;
					}
					else {
						CarLocationOnRoad *carL = road->channel_carL[c].back();
						if (carL->location > 1) {
							speed = min(carL->location - 1, speed);
							carLWillOnRoad->channel_id = c;
							carLWillOnRoad->location = speed;
							road->channel_carL[c].push_back(carLWillOnRoad);
							canDriveIn = true;
							break;
						}
					}
				}
				if (canDriveIn) {
					road->willOnRoad.erase(road->willOnRoad.begin());
					j = -1;
				}
				else {//����ĳ�Ҳ�޷�ʻ�룬ֱ��break
					break;
				}

			}
		}
	}

}
/*
* carL: ����Ҫ����ǰ·�ڵ�carL
* cross: ��ǰ·��
* road: ��ǰ��·
*/
void Solver::recordProbOutCross(CarLocationOnRoad * carL, Cross * cross, Road * road)
{
	Turn turn =Front;
	if (carL->index + 1 < output_->routines[carL->car_id].roads.size()) {
		ID next_road_id = output_->routines[carL->car_id].roads[carL->index + 1];//����֮����ܻ����޸�
		Road *next_road = getNextRoad(carL, cross);
		if (next_road == NULL) {
			cout << "next_road is NULL \n";
			exit(1);
		}
		turn = topo.RoadTurn[road->raw_road->id][next_road_id];
	}
	
	carL->turn = turn;
	road->waitOutCarL.push_back(carL);
}

Road* Solver::getNextRoad(CarLocationOnRoad * carL, Cross * cross)
{
	Road *next_road = NULL;
	if (carL->index + 1 < output_->routines[carL->car_id].roads.size()) {
		ID next_road_id = output_->routines[carL->car_id].roads[carL->index + 1];//����֮����ܻ����޸�
		if (cross->raw_cross->id == ins_->raw_roads[next_road_id].from) {
			next_road = topo.roads[next_road_id];
		}
		else if (cross->raw_cross->id == ins_->raw_roads[next_road_id].to) {
			next_road = topo.roads[next_road_id + road_size];
		}
	}
	return next_road;
}

bool Solver::moveToNextRoad(Road * road, Road * next_road, CarLocationOnRoad * carL)
{
	if (carL->state == STATE_terminated)
		return false;
	Length S1 = road->raw_road->length - carL->location;//��ǰ��·�Ŀ���ʻ����
	if (next_road == NULL) {
		Speed speed = min(road->raw_road->speed, ins_->raw_cars[carL->car_id].speed);
		if (speed > S1) {
			carL->state = STATE_in_dst;
			road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());//��һ��Ԫ�������·�ڵĳ���
			inDst_num++;
		}
		return true;
	}
	else {
		Speed V2 = min(next_road->raw_road->speed, ins_->raw_cars[carL->car_id].speed);//��һ����·����ʻ������ٶ�
		if (S1 >= V2) {//�ó�����ͨ��·��
			
			carL->location = road->raw_road->length;
			carL->state = STATE_terminated;
			return false;
		}
		for (int c = 0; c < next_road->channel_carL.size(); ++c) {//������һ��·�����г���
			if (next_road->channel_carL[c].size() == 0) {
				road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());
				carL->location = V2 - S1;
				carL->channel_id = c;
				carL->state = STATE_terminated;
				carL->index += 1;
				next_road->channel_carL[carL->channel_id].push_back(carL);
				return true;
			}
			else {
				CarLocationOnRoad *last_carL = next_road->channel_carL[c].back();//ĳһ���������һ����
				if (last_carL->state == STATE_terminated && last_carL->location > 1) {//�����ڵĳ���������ֹ״̬�����п�λ
					road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());//��һ��Ԫ�������·�ڵĳ���
					carL->location = min(last_carL->location - 1, V2 - S1);
					carL->channel_id = c;
					carL->state = STATE_terminated;
					carL->index += 1;
					next_road->channel_carL[carL->channel_id].push_back(carL);
					return true;
				}
				else if (last_carL->state == STATE_terminated && last_carL->location == 1
					&& c == (next_road->channel_carL.size() - 1)) {//������Ҫ�ص�����
					carL->location = road->raw_road->length;
					carL->state = STATE_terminated;
					return false;
				}
			}
			
		}
	}
	
	return false;
}

// ����һ������һ��·�ڵ���һ��·�ڵ���̺�ʱ�����費��³���
Time Solver::min_time_cost(const ID car, const ID from, const ID to) const {
    Time time_cost = 0;
    const Speed car_speed = ins_->raw_cars[car].speed;
    Length run_more_length = 0;             // ����һ����·���ܹ��ߵľ���
    for (auto road = shortest_paths[from][to].begin();
        road != shortest_paths[from][to].end(); ++road) {
        Length this_road_length = ins_->raw_roads[*road].length - run_more_length;
        if (this_road_length <= 0) {        // �ܵ��������죬��������һ��·���ˡ�
            run_more_length = 0;            // �����·���ȴ��ڵ�·������٣���������Ͳ��ᷢ����
            continue;
        }
        Speed this_road_speed = ins_->raw_roads[*road].speed;
        Time this_road_cost_time = this_road_length / min(this_road_speed, car_speed);
        time_cost += this_road_cost_time;
        Length this_road_left_length =  // this_road_cost_timeʱ�䵥λ�󣬵��ﵱǰ·����ʣ��ľ��롣
            this_road_length - this_road_cost_time * min(this_road_speed, car_speed);
        if (!this_road_left_length)continue;
        if ((road + 1) != shortest_paths[from][to].end()) {
            Speed next_road_speed = ins_->raw_roads[*(road + 1)].speed;
            run_more_length = min(next_road_speed, car_speed) > this_road_left_length ?
                min(next_road_speed, car_speed) - this_road_left_length : 0;
        }
        ++time_cost;    // ��һ������ʣ���·�������ǲ������һ��·��
    }
    return time_cost;
}

void Solver::read_from_file()
{

}


void Solver::init_solution_once() {
	int car_size = ins_->raw_cars.size();
	ID temp, next;
	Time  temp_time;
	Speed speed;
	//vector<Car> notPlanCar;
	/*for (auto i = 0; i < car_size; ++i) {
		Routine routine;
		RawCar car = ins_->raw_cars[i];
		routine.car_id = car.id;
		routine.start_time = car.plan_time;
		temp = car.from;
		while (temp != car.to)
		{
			next = topo.pathID[temp][car.to];
			routine.roads.push_back(topo.adjRoadID[temp][next]);
			temp = next;
		}

		temp_time = 0;
		for (auto j = 0; j < routine.roads.size(); ++j) {
			RawRoad road = ins_->raw_roads[routine.roads[j]];
			speed = min(road.speed, car.speed);
			temp_time += road.length / speed;
		}
		total_time += temp_time;
		output_->routines.push_back(routine);
	}*/
	for (auto i = 0; i < 10; ++i) {
		Routine routine;
		RawCar car = ins_->raw_cars[i];
		routine.car_id = car.id;
		routine.start_time = car.plan_time;
		routine.roads.push_back(26);
		routine.roads.push_back(31);
		routine.roads.push_back(36);
		routine.roads.push_back(30);
		routine.roads.push_back(24);

		output_->routines.push_back(routine);
	}
	for (auto i = 10; i < car_size; ++i) {
		Routine routine;
		RawCar car = ins_->raw_cars[i];
		routine.car_id = car.id;
		routine.start_time = car.plan_time;
		routine.roads.push_back(35);
		routine.roads.push_back(30);
		routine.roads.push_back(25);
		routine.roads.push_back(31);
		routine.roads.push_back(37);

		output_->routines.push_back(routine);
	}
	total_time = 100;
	std::cout << "the cost time is " << total_time << std::endl;
}
}