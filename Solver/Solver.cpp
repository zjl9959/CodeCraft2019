#include "Solver.h"

using namespace std;

namespace codecraft2019 {

// 此函数用于测试IO的ID映射是否正确，与求解算法无关。
void Solver::testIO() {
    output_->routines.resize(ins_->cars.size());
    for (int i = 0; i < ins_->cars.size(); ++i) {
        output_->routines[i].car_id = ins_->cars[i].id;
        output_->routines[i].start_time = ins_->cars[i].plan_time;
        output_->routines[i].roads.push_back(ins_->cars[i].from);
        output_->routines[i].roads.push_back(ins_->cars[i].to);
    }
}
    
void Topo::Floyd()
{
	int csize = ins_->crosses.size();
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

void Topo::printPath()
{
	cout << "各个顶点对的最短路径：" << endl;
	int row = 0;
	int col = 0;
	int temp = 0;
	for (row = 0; row < this->vexnum; row++) {
		for (col = row + 1; col < this->vexnum; col++) {
			cout << "v" << to_string(row + 1) << "---" << "v" << to_string(col + 1) << " weight: "
				<< sPathLen[row][col] << " path: " << " v" << to_string(row + 1);
			temp = pathID[row][col];
			//循环输出途径的每条路径。
			while (temp != col) {
				cout << "-->" << "v" << to_string(temp + 1);
				temp = pathID[temp][col];
			}
			cout << "-->" << "v" << to_string(col + 1) << endl;
		}

		cout << endl;
	}
	
}

}
