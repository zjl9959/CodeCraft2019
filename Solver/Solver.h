#pragma once
#ifndef CODE_CRAFT_2019_SOLVER_H
#define CODE_CRAFT_2019_SOLVER_H
/*
 * 求解的核心算法。
 */

#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

class Topo {
public:
	Topo(Instance *ins):ins_(ins),vexnum(ins->crosses.size()) {
		int rsize = ins->roads.size();
		sPathLen = new Length*[vexnum];
		adjRoadID = new ID*[vexnum];
		pathID = new ID*[vexnum];
		for (int i = 0; i < vexnum; ++i) {
			sPathLen[i] = new Length[vexnum];
			adjRoadID[i] = new ID[vexnum];
			pathID[i] = new ID[vexnum];
		}
		for (int i = 0; i < vexnum; ++i) {
			for (int j = 0; j < vexnum; ++j) {
				adjRoadID[i][j] = INVALID_ID;
				sPathLen[i][j] = LENGTH_MAX;
				pathID[i][j] = INVALID_ID;
			}
		}
		for (int i = 0; i < rsize; ++i) {
			ID from = ins->roads[i].from;
			ID to = ins->roads[i].to;
			adjRoadID[from][to] = i;
			pathID[from][to] = to;
			sPathLen[from][to] = ins->roads[i].length;
			if (ins->roads[i].is_duplex){
				adjRoadID[to][from] = i;
				pathID[to][from] = from;
				sPathLen[to][from] = ins->roads[i].length;
			}
		}
		Floyd();
		//printPath();
	};
	void Floyd();
	void printPath();
	ID **adjRoadID;
	ID **pathID;
	Length **sPathLen;
	Turn **RoadTurn;
	int vexnum;
protected:
	Instance *ins_;
};
class Solver {
public:
    Solver(Instance *ins, Output *output, Environment *env, Configure *cfg) :
        ins_(ins), output_(output), env_(env), cfg_(cfg) ,topo(ins){
	};
    ~Solver() {
        ins_ = nullptr;
        output_ = nullptr;
        env_ = nullptr;
        cfg_ = nullptr;
    };
    void testIO();
	void init_solution();		
protected:
    Instance* ins_;
    Output* output_;
    Environment* env_;
    Configure* cfg_;
public:
	Topo topo; 
    // [TODO]添加算法用到的数据结构
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
