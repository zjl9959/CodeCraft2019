#include <iostream>

#include "Instance.h"
#include "Output.h"

using namespace std;
using namespace codecraft2019;

int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;
	
	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);
	
	std::cout << "carPath is " << carPath.data() << std::endl;
	std::cout << "roadPath is " << roadPath.data() << std::endl;
	std::cout << "crossPath is " << crossPath.data() << std::endl;
	std::cout << "answerPath is " << answerPath.data() << std::endl;

    Environment env(argv[1], argv[2], argv[3], argv[4]);
    Instance ins(env);
	
	// TODO:process
	// TODO:write output file
	
	return 0;
}