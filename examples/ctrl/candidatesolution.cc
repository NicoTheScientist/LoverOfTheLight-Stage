#include "stage.hh"
#include "structs.h"
#include <iostream>

using namespace Stg;

int SonarUpdate( Model* mod, robot_t* robot );
int PositionUpdate( Model* mod, robot_t* robot );

double SigmoidFunction(double x)
{
	return 1.0 / (1.0 + pow(euler_e, -x));
}

int ComputeFitness(int* w)
{
	int fitness=0;
	for (int i=0; i<slidingWindowSize; i++){
		fitness+=w[i];
	}
	return fitness;
}

void InitializeCandidateSolution(robot_t* robot)
{
	for (size_t i = 0; i < candidateSolutionProblemDimension; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			robot->candidateSolutionGenome.values[i][j] = drand48() * candidateSolutionGenomeRange + candidateSolutionGenomeLowerBound;
		}
	}

	for (int i=0; i<slidingWindowSize; i++){
		robot->window[i]=0;
	}
	
	robot->fitness = ComputeFitness(robot->window);
}

void AgentInit(robot_t* robot)
{
	robot->type = CandidateSolution;
	robot->fitness = 0;
	robot->iteration = 0;
	robot->out=false;
	robot->empty=false;
	robot->pos->SetColor(Color::blue);
	
	InitializeCandidateSolution(robot);

	robot->iterationsFromLastEvaluation = (int)(drand48() * (double)numberOfIterationsPerEvaluation);
}

// Stage calls this when the model starts up
extern "C" int Init( Model* mod, CtrlArgs* args )
{

  robot_t* robot = new robot_t;

  robot->pos = (ModelPosition*)mod;

  //if( verbose )
  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
  robot->pos->Subscribe(); // starts the position updates

  robot->sonar = (ModelRanger*)mod->GetChild( "ranger:0" );
  robot->sonar->AddCallback( Model::CB_UPDATE, (model_callback_t)SonarUpdate, robot );
  robot->sonar->Subscribe(); // starts the ranger updates

  robot->pos->setUserData(robot);

  AgentInit(robot);

  return 0; //ok
}

// Sets speed to the robot given in left and right wheel speeds
void SetSpeed(robot_t* robot, double lws, double rws) // lws - leftWheelSpeed, rws rightWhe....
{
	robot->rightWheelSpeed = rws;
	robot->leftWheelSpeed = lws;

	double newSpeed, newTurnRate;

	if (lws > rws)
	{
		newSpeed = rws + (lws - rws) / 2.0;
		newTurnRate = (rws - lws) / wheelSeparation;
	}
	else if (rws > lws)
	{
		newSpeed = lws + (rws - lws) / 2.0;
		newTurnRate = (rws - lws) / wheelSeparation;
	}
	else
	{
		newSpeed = lws;
		newTurnRate = 0.0;
	}

	robot->pos->SetSpeed(newSpeed, 0.0, newTurnRate);
}

Model* getLightInRange(robot_t* robot){
	const std::set<Model*> & models = robot->pos->GetWorld()->GetAllModels();
	Pose myPose = robot->pos->GetGlobalPose();
	
	for (std::set<Model*>::const_iterator it = models.begin(); it != models.end(); ++it)
	{
		if ((*it)->GetColor() == Color::yellow)
		{
			Pose lightPose = (*it)->GetGlobalPose();
			double dx = lightPose.x - myPose.x;
			double dy = lightPose.y - myPose.y;
			double range = hypot( dy, dx );

			if (range < 0.5)
			{
				std::cout<<"Old light pose x: "<<lightPose.x<<" y: "<<lightPose.y<<std::endl;
				return *it;
			}
		}
	}
	return NULL;
}

void CandidateSolutionBehaviour(robot_t* robot)
{
	double leftOutput, rightOutput; // output neurons of the neural net
	
	double input[candidateSolutionProblemDimension];
	double maxActivation = 0;
	for (int i = 0; i < candidateSolutionProblemDimension; i++) {
		input[i] = (sensorRange - robot->sonar->GetSensors()[i].ranges[0]) / sensorRange;
		if (input[i] > maxActivation)
			maxActivation = input[i];
		leftOutput += input[i] * robot->candidateSolutionGenome.values[i][0];
		rightOutput += input[i] * robot->candidateSolutionGenome.values[i][1];
	}
	leftOutput = SigmoidFunction(leftOutput);
	rightOutput = SigmoidFunction(rightOutput);

	robot->leftWheelSpeed = minSpeed + leftOutput * (maxSpeed - minSpeed);
	robot->rightWheelSpeed = minSpeed + rightOutput * (maxSpeed - minSpeed);
	
	SetSpeed(robot, robot->leftWheelSpeed, robot->rightWheelSpeed);
	
	if(maxActivation>activationThreshold*sensorRange) {
	    // find which light is catched
	    Model* light=getLightInRange(robot);
	    if(light!=NULL){
	    	// update light position
    		Pose lightPose(drand48()*(sizeArena*2)-sizeArena,drand48()*(sizeArena*2)-sizeArena,0,0);
	   	light->SetPose(lightPose);
	   	std::cout<<"New light pose x: "<<lightPose.x<<" y: "<<lightPose.y<<std::endl;
	   	// increment lights counter
	   	robot->window[0]++;
	   	robot->fitness = ComputeFitness(robot->window);
	    }   
    	}
}

bool isOutOfBound(robot_t* robot){
	Pose myPose = robot->pos->GetGlobalPose();
	return (myPose.x>sizeArena || myPose.x<-sizeArena || myPose.y>sizeArena || myPose.y<-sizeArena);
}

// inspect the ranger data and decide what to do
int SonarUpdate( Model* mod, robot_t* robot )
{
	// get the data
	const std::vector<ModelRanger::Sensor>& sensors = robot->sonar->GetSensors();
	uint32_t sample_count = sensors.size();
	if( sample_count < 1 )
		return 0;

	robot->iteration++;
	robot->iterationsFromLastEvaluation++;
	
	if (!robot->empty){
		CandidateSolutionBehaviour(robot);
		if (iter % numberOfIterationsPerEvaluation == 0)
			fprintf(fitnessCS,"%f ",robot->fitness);
	}
	else{
		SetSpeed(robot, 0, 0);
	}
	
	// shift the window if window time for unit is reached
	if (robot->iteration % windowTimeForUnit == 0) {
    		for (int i = slidingWindowSize - 2; i >= 0; i--) {
        		robot->window[i+1] = robot->window[i];
    		}
    		robot->window[0] = 0;
    	}
    	
    	if (iter < robot->iteration) {
		if (iter % numberOfIterationsPerEvaluation == 0)
			fprintf(fitnessCS,"%d \n",-1);
		iter++;
	}
	
	return 0; // run again
}

int PositionUpdate( Model* mod, robot_t* robot )
{
	if(isOutOfBound(robot)){
		if(!robot->out){
			Pose myPose = robot->pos->GetGlobalPose();
			myPose.a+=dtor(180);
			robot->pos->SetPose(myPose);
			robot->out=true;
		}
	}
	else
		robot->out=false;
	
	return 0; // run again
}
