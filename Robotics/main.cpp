#include<vector>
#include<iostream>
#include<fstream>
#include<cmath>
#include <rw/rw.hpp>
#include<rw/math.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;	
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 100
#define n 32

void generate_lua(QPath&,double);

std::pair<double,double> plan_and_calculate(Q q_start, Q q_end, double eps, QToQPlanner::Ptr planner, WorkCell::Ptr,Device::Ptr);

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		std::cerr << "Configuration in collision: " << q << std::endl;
		std::cerr << "Colliding frames: " << std::endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
		}
		return false;
	}
	return true;
}


int main(int argc, char* argv[]){
	if (argc != 5){
	 std::cerr << "Wrong number of arguments\n";
	 return 1;
	}
	
	Math::seed();
	const std::string wcFile = "/home/matthias/RobtekE15/ROVIProblem/Robotics/Workcells/Kr16WallWorkCell/Scene.wc.xml";
	const std::string deviceName = "KukaKr16";
	const std::string bottleName = "PG70";
	std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		std::cerr << "Device: " << deviceName << " not found!" << std::endl;
		return 0;
	}
	else{
	  std::cout << "Device Found! \n";
	}
	
	State state = wc->getDefaultState();
	Q q_pick(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	
	Frame* bottle_frame = wc->findFrame("Bottle");
	Frame* tcp_frame = wc->findFrame("Tool");
	
	if(bottle_frame == NULL){
	  std::cout << "Bottle does not exist\n";
	}
	 else if (tcp_frame == NULL){
		std::cout << "TCP does not exist\n";
	}
	
	device->setQ(q_pick,state);
	
	Kinematics::gripFrame(bottle_frame,tcp_frame,state);
	
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
	
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	Q q_place(6,1.571,0.006,0.030,0.153,0.762,4.490);
	
	if (!checkCollisions(device, state, detector, q_pick))
		return 0;
	if (!checkCollisions(device, state, detector, q_place))
		return 0;
	
	std::ofstream datapoints;
	datapoints.open(argv[4]);
	datapoints << "Epsilon \t Mean time \t Mean path length\tMean distance traveled\n";

	double extend = atof(argv[1]);
	double extend_max = atof(argv[2]);
	double extend_step = atof(argv[3]);
	std::pair<double,double> values;
	while (extend<extend_max){
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
	
	values = plan_and_calculate(q_pick,q_place,extend,planner,wc,device);
	
	datapoints << extend << "\t" << values.first << "\t" << values.second << "\t" << values.second*extend << "\n";
	extend +=extend_step;
	}
	datapoints.close();
	
return 0;
}

void generate_lua(QPath &q_vector,double eps){
  std::string app = std::to_string(eps);
  std::ofstream myfile;
  myfile.open("luaE" + app + ".txt");
  
  myfile << "wc = rws.getRobWorkStudio():getWorkCell() \n";
  myfile << "state = wc:getDefaultState()\n";
  myfile << "device = wc:findDevice(\"KukaKr16\")\n\n";
  
  myfile << "function setQ(q)\n";
  myfile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n";
  myfile << "device:setQ(qq,state)\n";
  myfile << "rws.getRobWorkStudio():setState(state)\n";
  myfile << "rw.sleep(0.1)\n";
  myfile << "end\n\n";

  Q temp;
  myfile << "setQ({";
  for(int i = 0;i < q_vector[0].size()-1;i++){
	myfile << q_vector[0][i] << ", ";
  }
  	  myfile << q_vector[0][q_vector[0].size()-1];
	  myfile << "})\n";
	  myfile << "bottle_frame = wc:findFrame(\"Bottle\")\n";
	  myfile << "tool_frame = wc:findFrame(\"Tool\")\n";
	  myfile << "rw.gripFrame(bottle_frame,tool_frame,state)\n\n";

  for(int i = 1; i < q_vector.size(); i++){
	myfile << "setQ({";
	  temp = q_vector[i];
	  for(int j = 0;j < temp.size()-1;j++){
		myfile << temp[j] << ",";
	  }
	  myfile << temp[temp.size()-1];
	  myfile << "})\n";
	}
	myfile.close();
}

std::pair<double,double> plan_and_calculate(Q q_start, Q q_end, double eps, QToQPlanner::Ptr planner, WorkCell::Ptr wc,Device::Ptr device){
	std::pair<double,double> mu_pathl;
	std::vector<double> total_time;
	Timer t;
	QPath temp;
	PathAnalyzer::CartesianAnalysis distance_traveled;
	State state = wc->getDefaultState();
	int min_size = 20;
	double path_length = 0;
	for(int i = 0;i<n;i++){
	Frame* tcp_frame = wc->findFrame("Tool");
	device->setQ(q_start,state);
	PathAnalyzer path_analyse(device,state);
	QPath path;
	t.resetAndResume();
	planner->query(q_start,q_end,path,MAXTIME);
	t.pause();
	total_time.push_back(t.getTimeMs());
	distance_traveled = path_analyse.analyzeCartesian(path,tcp_frame);
	path_length += distance_traveled.length;
	if(distance_traveled.length < min_size){
	 min_size = distance_traveled.length;
	 temp = path;
	 std::cout << "Min_size: " << distance_traveled.length << std::endl;
	}
	}
	generate_lua(temp,eps);
	double mean = 0;
	
	for(int i = 0;i<n;i++){
	 mean += total_time[i];
	}
	mean /= (double)n;
	path_length /= (double)n;
	
	mu_pathl = std::make_pair(mean,path_length);
	std::cout << "Epsilon = "  << eps << "\nMean time: " << mean << "\nMean distance traveled: " << path_length<<std::endl;

  return mu_pathl;
}