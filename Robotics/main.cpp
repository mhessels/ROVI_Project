#include<vector>
#include<iostream>
#include<fstream>
#include<cmath>
#include <rw/rw.hpp>
#include<rw/math.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;	
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


void generate_lua(QPath);

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

#define MAXTIME 10.

int main(){
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
	
	const State state = wc->getDefaultState();
	State new_state = state;
	Q q_pick(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	
	Frame* bottle_frame = wc->findFrame("Bottle");
	Frame* tcp_frame = wc->findFrame("Tool");
	
	if(bottle_frame == NULL){
	  std::cout << "Bottle does not exist\n";
	}
	 else if (tcp_frame == NULL){
		std::cout << "TCP does not exist\n";
	}
	
	device->setQ(q_pick,new_state);
	Kinematics::gripFrame(bottle_frame,tcp_frame,new_state);


	
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,new_state);
	
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = 0.1;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
	
	Q q_place(6,1.571,0.006,0.030,0.153,0.762,4.490);
	
	if (!checkCollisions(device, new_state, detector, q_pick))
		return 0;
	if (!checkCollisions(device, new_state, detector, q_place))
		return 0;

	std::cout << "Planning from " << q_pick << " to " << q_place << std::endl;
	QPath path;
	Timer t;
	
	t.resetAndResume();
	planner->query(q_pick,q_place,path,MAXTIME);
	t.pause();
	generate_lua(path);
	
return 0;
}

void generate_lua(QPath q_vector ){
  std::ofstream myfile;
  myfile.open("q_vectors.txt");
  
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
  
  for(int i = 0; i < q_vector.size() - 1; i++){
	myfile << "setQ({";
	  temp = q_vector[i];
	  for(int j = 0;j < temp.size()-1;j++){
		myfile << temp[j] << ",";
	  }
	  myfile << temp[temp.size()-1];
	  myfile << "})\n";
	}
}

