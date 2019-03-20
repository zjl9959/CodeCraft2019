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
    //cout << "The car_size is: " << car_size << endl;                      //xxf:debug
	for (auto i = 0; i < car_size; ++i) {
		Routine *routine = new Routine;
		RawCar *raw_car = &ins_->raw_cars[i];
		routine->car_id = raw_car->id;
        routine->start_time = raw_car->plan_time; //xxf:�Ƚ�routine�ĳ���ʱ���¼Ϊ����ԭʼ����ʱ��
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
        routine->temp_runtime = temp_time;     //xxf:��¼ÿ����·������ʱ����ʱ��
		output_->routines.push_back(routine);//ʹ�����·����ʱroutine�ĳ���ʱ�仹δȷ��
		aux.car_same[raw_car->from][raw_car->to][raw_car->plan_time].push_back(interR);
	}

	
	/**����������������ʵ�ʳ���ʱ��*/
	vector<Routine *> *time_car;             //xxf:������ʱ����ͬ�ĳ���routine ����һ��vector
	time_car = new vector<Routine *>[latest_time+1];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = ins_->raw_cars[i].plan_time;
		time_car[start_time].push_back(output_->routines[i]);    
	}                                       
    //xxf����������ʵ�ʳ���ʱ�䣬��֤·���ϵĳ�����������������ֵMaxNumCarInRoad;
    list<Routine *> startOrder;         //xxf:����list���水����ʱ�������routine
    startOrder.clear();
    for (int i = 0; i <= latest_time; ++i) {
        if (time_car[i].size() > 0)
            for (int j = 0; j < time_car[i].size(); j++) startOrder.push_back(time_car[i][j]);
    }
    //cout << "The size of startOrder list is :" << startOrder.size() << endl;
    int numCarInRoad = 0, MaxnumCarInRoad = 70;
    Time current_time = -1;
    map<Time, int> same_timeRequired_Num;      //xxf��car����Ҫ�����յ��ʱ�� ��ͬ�� ��������  
    int flag = 0;
    while (!startOrder.empty()) {
        if (numCarInRoad < MaxnumCarInRoad) {
            Routine* first = startOrder.front();
            if (current_time == -1) {
                current_time = first->start_time;
            }
            Time needtime = first->temp_runtime + first->start_time - current_time;
            if (same_timeRequired_Num.find(needtime) == same_timeRequired_Num.end()) same_timeRequired_Num.insert(make_pair(needtime, 1));
            else same_timeRequired_Num[needtime]++;
            numCarInRoad++;
            first = NULL;
            delete first;
            startOrder.pop_front();
        }
        else {
            //debug
            //cout << "when list startorder comes else �� done add 200 cars in road ������" << endl << endl;
            //cout << "tne current map size is :" << same_timeRequired_Num.size() << endl;
            //map<Time, int>::iterator iter;
            //for (iter = same_timeRequired_Num.begin(); iter != same_timeRequired_Num.end(); iter++)
            //{
            //    cout << "                " << iter->first << "     " << iter->second << endl; 
            //}
            //cout << endl << endl;
            //cout << " the current time is:" << current_time << endl;
            //debugend
            map<Time, int>::iterator iter = same_timeRequired_Num.begin();
            int spend_time = iter->first;
            current_time += spend_time;                                  
            int numToGoCars = iter->second;
            same_timeRequired_Num.erase(iter);
            //debug
            //cout << " The first spend_time is :" << spend_time << endl;
            //cout << "The current_time��add spend_time��is:" << current_time << endl;
            //cout << "Tne current map size is (delete the first line):" << same_timeRequired_Num.size() << endl;
            //for (iter = same_timeRequired_Num.begin(); iter != same_timeRequired_Num.end(); iter++) {
            //    cout << "                " << iter->first << "     " << iter->second << endl;
            //}
            //cout << endl << endl;
            //debugend
            iter = same_timeRequired_Num.begin();  //xxf:����same_timeRequired_Num�г��������յ������ʱ�� ��ȥ iter->first
            for (; iter != same_timeRequired_Num.end(); ) {
                Time old_time = iter->first;
                same_timeRequired_Num.insert(make_pair(old_time - spend_time, iter->second));
                iter++;
                same_timeRequired_Num.erase(old_time);
                //map<Time, int>::iterator iter_;
                //int j = 0;
                //for (iter_ = same_timeRequired_Num.begin(); iter != same_timeRequired_Num.end(); iter++,j<7) {
                //    cout << "                " << iter_->first << "     " << iter_->second << endl;
                //    j++;
                //}
                //cout << "done done done " << endl << endl;
            }
            //cout << "Tne current map size is (update):" << same_timeRequired_Num.size() << endl;
            //for (iter = same_timeRequired_Num.begin(); iter != same_timeRequired_Num.end(); iter++) {
            //    cout << "                " << iter->first << "     " << iter->second << endl;
            //}
            //cout << endl << endl;
            for (int i = 0; i < numToGoCars; i++) {
                if (startOrder.empty()) {
                    flag = 1;
                    break;
                }
                Routine* first = startOrder.front();
                if (first->start_time < current_time) first->start_time = current_time;
                Time needtime = first->temp_runtime + first->start_time - current_time;
                if (same_timeRequired_Num.find(needtime) == same_timeRequired_Num.end()) same_timeRequired_Num.insert(make_pair(needtime, 1));
                else same_timeRequired_Num[needtime]++;
                first = NULL;
                delete first;
                startOrder.pop_front();
            }
            if (flag)break;
        }

    } 
    //cout << endl << endl;
    //map<Time, int>::iterator iter;
    //for (iter = same_timeRequired_Num.begin(); iter != same_timeRequired_Num.end(); iter++) {
    //    cout << "                " << iter->first << "     " << iter->second << endl;
    //}
    //cout << endl << endl;
    cout << "done" << endl;
    startOrder.clear();

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

// ���ö��������ķ���������ʼ��
void Solver::binary_generate_solution() {

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
				driveAllCarJustOnRoadToEndState(road);
			}

			for (int c = 0; c < cross_size; ++c) {//����ID������ȸ���·��
				Cross *cross = topo.crosses[c];
				for (int k = 0; k <cross->road.size(); ++k) {//����ID�����road���е���
					Road *road = cross->road[k];
					//��ȡ��ǰ��·�ϵĳ�������ǰ��Ҫ������ȷ����·�ڵȴ���ʻ�ĳ�����״̬
					//���Գ�·�ڵĳ�������Щ
					for (int c = 0; c < road->channel_carL.size(); ++c) {//����
						CarLocationOnRoad *carL = road->channel_carL[c][0];//ĳһ�������ڵĵ�һ����
						if (carL->state == STATE_terminated) { //�����ڵ�һ����Ϊ��ֹ״̬����ó�������������Ҳ���Խ�����ֹ״̬
							CarLocationOnRoad *prev_carL = carL;
							for (int cL = 1; cL < road->channel_carL[c].size(); cL++) {
								CarLocationOnRoad *next_carL = road->channel_carL[c][cL];
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

void Solver::driveAllCarJustOnRoadToEndState(Road *road)
{
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

}