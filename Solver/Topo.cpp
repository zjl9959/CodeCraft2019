#include "Topo.h"
using namespace std;
namespace codecraft2019 {
Turn cross_roadTurn[MAX_CROSS_ROAD_NUM][MAX_CROSS_ROAD_NUM] = {
{InvalidTurn,Left,Front,Right},
{Right,InvalidTurn,Left,Front},
{Front,Right,InvalidTurn,Left},
{Left,Front,Right,InvalidTurn}
	};

bool road_sort(const Road *road1,const Road *road2) {
	return road1->raw_road->id < road2->raw_road->id;
}
Topo::Topo(Instance *ins) :ins_(ins), cross_size(ins->raw_crosses.size()),rsize(ins->raw_roads.size())
,car_size(ins->raw_cars.size()){
	init_myTopo();
	sPathLen = new Length*[cross_size];
	adjRoadID = new ID*[cross_size];
	pathID = new ID*[cross_size];
	RoadTurn = new Turn*[rsize];

	for (int i = 0; i < cross_size; ++i) {
		sPathLen[i] = new Length[cross_size];
		adjRoadID[i] = new ID[cross_size];
		pathID[i] = new ID[cross_size];
	}
	for (int i = 0; i < cross_size; ++i) {
		for (int j = 0; j < cross_size; ++j) {
			adjRoadID[i][j] = INVALID_ID;
			sPathLen[i][j] = LENGTH_MAX;
			pathID[i][j] = INVALID_ID;
		}
	}
	for (int i = 0; i < rsize; ++i) {
		RoadTurn[i] = new Turn[rsize];

		ID from = ins->raw_roads[i].from;
		ID to = ins->raw_roads[i].to;
		adjRoadID[from][to] = i;
		pathID[from][to] = to;
		sPathLen[from][to] = ins->raw_roads[i].length;
		if (ins->raw_roads[i].is_duplex) {
			adjRoadID[to][from] = i;
			pathID[to][from] = from;
			sPathLen[to][from] = ins->raw_roads[i].length;
		}
	}

	for (int i = 0; i < rsize; ++i) {
		for (int j = 0; j < rsize; ++j) {
			if (i == j) {
				RoadTurn[i][j] = InvalidTurn;
				continue;
			}
			if (ins->raw_roads[i].to == ins->raw_roads[j].from) {
				RoadTurn[i][j] = getRoadTurn(ins->raw_roads[i].to, ins->raw_roads[i].id, ins->raw_roads[j].id);
			}
			else if (ins->raw_roads[i].is_duplex&&ins->raw_roads[i].from == ins->raw_roads[j].from) {
				RoadTurn[i][j] = getRoadTurn(ins->raw_roads[i].from, ins->raw_roads[i].id, ins->raw_roads[j].id);
			}
			else if (ins->raw_roads[i].to == ins->raw_roads[j].to && ins->raw_roads[j].is_duplex) {
				RoadTurn[i][j] = getRoadTurn(ins->raw_roads[i].to, ins->raw_roads[i].id, ins->raw_roads[j].id);
			}
			else if (ins->raw_roads[i].is_duplex&&ins->raw_roads[i].from == ins->raw_roads[j].to &&ins->raw_roads[j].is_duplex) {
				RoadTurn[i][j] = getRoadTurn(ins->raw_roads[i].from, ins->raw_roads[i].id, ins->raw_roads[j].id);
			}
			else {
				RoadTurn[i][j] = InvalidTurn;
			}
		}
	}
	/*for (int i = 0; i < rsize; ++i) {
		for (int j = 0; j < rsize; ++j) {
			if (i != j) {
				cout << ins->changeToOriginalID(i, RoadMap) << "," << ins->changeToOriginalID(j, RoadMap) << "  ";
				if (RoadTurn[i][j] == Left)cout << "Left" << endl;
				if (RoadTurn[i][j] == Right)cout << "Right" << endl;
				if (RoadTurn[i][j] == Front)cout << "Front" << endl;
				if (RoadTurn[i][j] == InvalidTurn)cout << "InvalidTurn" << endl;
			}
		}
	}*/
	Floyd();
	//printPath();
};

void Topo::Floyd()
{
	int csize = ins_->raw_crosses.size();
	for (int i = 0; i < csize; i++) {
		for (int j = 0; j < csize; j++) {
			for (int k = 0; k < csize; k++) {
				Length select = (sPathLen[j][i] == LENGTH_MAX || sPathLen[i][k] == LENGTH_MAX)
					? LENGTH_MAX : (sPathLen[j][i] + sPathLen[i][k]);

				if (sPathLen[j][k] > select) {
					pathID[j][k] = pathID[j][i];
					sPathLen[j][k] = select;
				}
			}
		}
	}
}

vector<ID> Topo::Dijkstra(ID src, ID dst,ID tabuRoad)
{
	vector<int> dist,dist2;
	vector<ID> prev;
	dist.resize(cross_size);
	dist2.resize(cross_size);
	prev.resize(cross_size);
	dist[src] = 0;
	clear(q);
	for (int i = 0; i < cross_size; ++i) {
		if (i != src) {
			dist[i] = INT_MAX;
			dist2[i] = INT_MAX;
			prev[i] = INVALID_ID;
		}
	}
	q.push(node(src, 0));
	while (!q.empty())
	{
		node n = q.top();
		q.pop();
		ID v = n.id;
		//if (dist2[v] < n.dist) continue;
		if (dst == v) break;
		for (int i = 0; i < crosses[v]->inCrossRoad.size(); ++i) {
			Road *road = crosses[v]->inCrossRoad[i];
			if (road->raw_road->id == tabuRoad) continue;
			if (road->from_id != v) {
				Log(FY_TEST) << "ID error\n";
				exit(5);
			}
			ID u = road->to_id;
			Length alt = n.dist + road->raw_road->length;
			if (dist[u] > alt) {
				swap(dist[u], alt);
				q.push(node(u, dist[u]));
				prev[u] = v;
			}
			/*if (dist2[u] >alt && dist[u] < alt) {
				dist2[u] = alt;
				q.push(node(u, dist2[u]));
				
			}*/
		}
	}
	prev[src] = INVALID_ID;
	//Log(FY_TEST) << dist[dst] << "  " << dist2[dst] << endl;
	ID v = dst;
	vector<ID> path;
	while (prev[v]!= INVALID_ID)
	{
		path.push_back(adjRoadID[prev[v]][v]);
		//Log(FY_TEST) << adjRoadID[prev[v]][v] << " ";
		v = prev[v];
	}
	//Log(FY_TEST) << endl;
	//Log(FY_TEST) << roads[prev[v]]->raw_road->id << endl;
	reverse(path.begin(), path.end());
	return path;

}

void Topo::printPath()
{
	cout << "��������Ե����·����" << endl;
	int row = 0;
	int col = 0;
	int temp = 0;
	ID next;
	for (row = 0; row < this->cross_size; row++) {
		for (col = row + 1; col < this->cross_size; col++) {
			cout << "v" << to_string(row + 1) << "---" << "v" << to_string(col + 1) << " weight: "
				<< sPathLen[row][col] << " path: ";
			temp = row;
			//ѭ�����;����ÿ��·����
			while (temp != col) {
				//cout << "-->" << "v" << to_string(temp + 1);
				next = pathID[temp][col];
				cout << ins_->changeToOriginalID(adjRoadID[temp][next], RoadMap) << ", ";
				temp = next;
			}
			cout << endl;
		}

		cout << endl;
	}

}

void Topo::init_myTopo()
{
	cars.resize(car_size);
	crosses.resize(cross_size);
	roads.resize(rsize * 2);

	for (int i = 0; i < car_size; ++i) {
		Car *car = new Car(&ins_->raw_cars[i]);
		cars[i] = car;
	}
	for (int i = 0; i < cross_size; ++i) {
		Cross *cross = new Cross(&ins_->raw_crosses[i]);
		crosses[i] = cross;
	}

	adjRoad = new Road**[cross_size]();
	for (int i = 0; i < cross_size; ++i) {
		adjRoad[i] = new Road*[cross_size]();
	}
	for (int i = 0; i < car_size; ++i) {
		cars[i]->from = crosses[ins_->raw_cars[i].from];
		cars[i]->to = crosses[ins_->raw_cars[i].to];

	}
	for (int i = 0; i < rsize; ++i) {
		ID from = ins_->raw_roads[i].from;
		ID to = ins_->raw_roads[i].to;
		Road *road = new Road(&ins_->raw_roads[i], crosses[from], crosses[to]);
		road->channel_carL.resize(road->raw_road->channel);
		roads[i] = road;
		adjRoad[from][to] = road;

		if (ins_->raw_roads[i].is_duplex) {
			Road *road = new Road(&ins_->raw_roads[i], crosses[to], crosses[from]);
			road->channel_carL.resize(road->raw_road->channel);
			roads[i+ rsize]= road;
			adjRoad[to][from] = road;
		}
		else {
			roads[i + rsize] = NULL;
		}
	}
	for (int i = 0; i < cross_size; ++i) {
		RawCross *raw_cross = crosses[i]->raw_cross;
		for (int j = 0; j < MAX_CROSS_ROAD_NUM; ++j) {//��ÿ��·��ָ���·�ڵĵ�·��������
			ID road_id = raw_cross->road[j];
			if (road_id != INVALID_ID) {
				if(roads[road_id]->to_id == raw_cross->id)
					crosses[i]->road.push_back(roads[road_id]);
				else if(roads[road_id+rsize] && roads[road_id + rsize]->to_id == raw_cross->id)
					crosses[i]->road.push_back(roads[road_id +rsize ]);

				if (roads[road_id]->from_id == raw_cross->id)
					crosses[i]->inCrossRoad.push_back(roads[road_id]);
				else if(roads[road_id+rsize] && roads[road_id + rsize]->from_id == raw_cross->id)
					crosses[i]->inCrossRoad.push_back(roads[road_id + rsize]);

			}
		}
		sort(crosses[i]->road.begin(), crosses[i]->road.end(), road_sort);//ÿ��·�ڵĳ����ճ���id��������
		/*for (int j = 0; j < crosses[i]->road.size(); ++j) {
			if (crosses[i]->road[j]->from == crosses[i]) {

			}
		}*/
	}

}

Turn Topo::getRoadTurn(ID cross_id, ID road1, ID road2)
{
	int r1, r2;
	RawCross cross = ins_->raw_crosses[cross_id];
	for (int i = 0; i < MAX_CROSS_ROAD_NUM; ++i) {
		if (cross.road[i] == road1)
			r1 = i;
		else if (cross.road[i] == road2)
			r2 = i;
	}
	return cross_roadTurn[r1][r2];
}

}