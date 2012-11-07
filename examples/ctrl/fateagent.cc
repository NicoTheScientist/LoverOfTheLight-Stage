#include "stage.hh"
#include "structs.h"
#include <set>
#include <iostream>

using namespace Stg;

int PositionUpdate( Model* mod, robot_t* robot );

// Random number from a normal distribution, taken from: http://www.dreamincode.net/code/snippet1446.htm
double randn_notrig(double mu = 0.0, double sigma = 1.0)
{
	static bool deviateAvailable=false;        //        flag
	static float storedDeviate;                        //        deviate from previous calculation
	double polar, rsquared, var1, var2;

	//        If no deviate has been stored, the polar Box-Muller transformation is
	//        performed, producing two independent normally-distributed random
	//        deviates.  One is stored for the next round, and one is returned.
	if (!deviateAvailable) {

		//        choose pairs of uniformly distributed deviates, discarding those
		//        that don't fall within the unit circle
		do {
			var1=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
			var2=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
			rsquared=var1*var1+var2*var2;
		} while ( rsquared>=1.0 || rsquared == 0.0);

		//        calculate polar tranformation for each deviate
		polar=sqrt(-2.0*log(rsquared)/rsquared);

		//        store first deviate and set flag
		storedDeviate=var1*polar;
		deviateAvailable=true;

		//        return second deviate
		return var2*polar*sigma + mu;
	}

	//        If a deviate is available from a previous call to this function, it is
	//        returned, and the flag is set to false.
	else {
		deviateAvailable=false;
		return storedDeviate*sigma + mu;
	}
}

bool compareRobots(robot_t* lhs, robot_t* rhs)
{
    return lhs->fitness < rhs->fitness;
}


void InitializeFateAgent(robot_t* robot)
{
	assert(robot->type == Cupid || robot->type == Reaper);

	robot->fateGenome.tournamentSize = (int)(drand48() * (maxTournamentSize - 1)) + 1;
	double maxInitProb = robot->type == Cupid ? maxInitialSelectionProbabilitiesCupid : maxInitialSelectionProbabilitiesReaper;
	for (int i = 0; i < 4; ++i)
	{
		robot->fateGenome.selectionProbabilities[i] = drand48() * maxInitProb;
	}
}

void InitializeBreeder(robot_t* robot)
{
	assert(robot->type == Breeder);

	robot->breederGenome.candidateSolutionMutationSize = drand48() * MAX_INITIAL_CS_MUTATION_SIZE;
	robot->breederGenome.fateAgentMutationSize = drand48() * MAX_INITIAL_FATE_MUTATION_SIZE;
	robot->breederGenome.tournamentMutationSize = drand48() * MAX_INITIAL_TOURNAMENT_MUTATION_SIZE;
}

AgentType RandomAgent()
{
	double selection = drand48();

	if (selection < fateAgentSpawnProbability){
		return Cupid;
	}
	else{ 
		if (selection < 2 * fateAgentSpawnProbability)
			return Reaper;
		else
			return Breeder;
	}
}

void AssignColor(robot_t* robot)
{
	if(robot->empty){
		robot->pos->SetColor(Color("grey"));
	}
	else{
		switch (robot->type)
		{
		case Cupid:
			robot->pos->SetColor(Color::red);
			break;
		case Reaper:
			robot->pos->SetColor(Color("black"));
			break;
		case Breeder:
			robot->pos->SetColor(Color::green);
			break;
		case CandidateSolution:
			robot->pos->SetColor(Color::blue);
			break;
		}
	}
}

void AgentInit(robot_t* robot)
{
	robot->type = RandomAgent();
	robot->fitness = 0;
	robot->iteration = 0;
	robot->out=false;
	robot->empty=false;
	AssignColor(robot);
	switch (robot->type)
	{
		case Cupid:
		case Reaper:
			InitializeFateAgent(robot);
			break;
		case Breeder:
			InitializeBreeder(robot);
			break;
		default:
			break;
	}
	robot->iterationsFromLastEvaluation = (int)(drand48() * (double)numberOfIterationsPerEvaluation);
}

void GetCandidateSolutionsInRange(robot_t* robot, std::vector<robot_t*> & candidateSolutions)
{
	candidateSolutions.clear();
	const std::set<Model*> & models = robot->pos->GetWorld()->GetAllModels();
	Pose myPose = robot->pos->GetGlobalPose();

	for (std::set<Model*>::const_iterator it = models.begin(); it != models.end(); ++it)
	{
		if ((*it)->getUserData() != NULL)
		{
			Pose hisPose = (*it)->GetGlobalPose();
			double dx = hisPose.x - myPose.x;
			double dy = hisPose.y - myPose.y;
			double range = hypot( dy, dx );

			robot_t* rob=(robot_t*)(*it)->getUserData();
			if (range < fateAgentRange && (rob->type == CandidateSolution && !rob->empty) )
			{
				candidateSolutions.push_back(rob);
			}
		}
	}
}

void EvaluateBreeder(robot_t* robot)
{
	assert (robot->type == Breeder);
    
    // reset counter from last evaluation
	robot->iterationsFromLastEvaluation = 0;

    // find candidate solutions in the neighbourhood
	std::vector<robot_t*> css;
	GetCandidateSolutionsInRange(robot, css);
	if(verbose){
		fprintf(systemBehavior,"\t neighbourhood\n");
		fprintf(systemBehavior,"\t\t candidate solutions: %d \n", (int)css.size());
	}
	
    // evaluate fitness (average fitness of Candidate Solutions in the neighbourhood)
	if (css.size() != 0) {
		double fitness = 0;
		for (std::vector<robot_t*>::const_iterator it = css.begin(); it != css.end(); ++it)
		{
			fitness += (*it)->fitness;
		}
		robot->fitness = fitness / (double)css.size();
	}
	if(verbose)
		fprintf(systemBehavior,"\t fitness (average): %f \n", robot->fitness);
}

void GetAllRobotsInRange(robot_t* robot, std::vector<robot_t*> & cupids, std::vector<robot_t*> & reapers, std::vector<robot_t*> & breeders, std::vector<robot_t*> & css, std::vector<robot_t*> & emptyCS, std::vector<robot_t*> & emptyFA)
{
	cupids.clear();
	reapers.clear();
	breeders.clear();
	css.clear();
	emptyCS.clear();
	emptyFA.clear();
	
	const std::set<Model*> & models = robot->pos->GetWorld()->GetAllModels();
	Pose myPose = robot->pos->GetGlobalPose();

	for (std::set<Model*>::const_iterator it = models.begin(); it != models.end(); ++it){
		if ((*it)->getUserData() != NULL){
			Pose hisPose = (*it)->GetGlobalPose();
			double dx = hisPose.x - myPose.x;
			double dy = hisPose.y - myPose.y;
			double range = hypot( dy, dx );
			if (range < fateAgentRange){
				robot_t* him = (robot_t*)(*it)->getUserData();
				switch (him->type){
					case Cupid:
						if(him->empty)
							emptyFA.push_back(him);
						else
							cupids.push_back(him);
						break;
					case Reaper:
						if(him->empty)
							emptyFA.push_back(him);
						else
							reapers.push_back(him);
						break;
					case Breeder:
						if(him->empty)
							emptyFA.push_back(him);
						else
							breeders.push_back(him);
						break;
					case CandidateSolution:
						if(him->empty)
							emptyCS.push_back(him);
						else
							css.push_back(him);
						break;
				}
			}
		}
	}
}

void CupidSelection(std::vector<robot_t*> & selectFrom, std::vector<robot_t*> & selectTo, double selectionProbability, size_t tournamentSize)
{
	if (selectFrom.size() < 1)
	{
		return;
	}

	selectTo.clear();
	size_t realTournamentSize = tournamentSize < selectFrom.size() ? tournamentSize : selectFrom.size();
	std::vector<robot_t*> selectedForTournament;

	for (size_t i = 0; i < realTournamentSize; ++i)
	{
		selectedForTournament.push_back(NULL);
	}

    // play a number of tournament that depends on the number of cupids in neighbourhood and the selection probability for cupids
	for (int i = 0; i < (selectFrom.size() * 2); ++i)
	{
		if(drand48() < selectionProbability)
		{
			for (size_t j = 0; j < realTournamentSize; ++j)
			{
				selectedForTournament.at(j) = selectFrom.at((int)(drand48() * (double)(selectFrom.size() - 1)));
			}

			sort(selectedForTournament.begin(), selectedForTournament.end(), compareRobots);

			selectTo.push_back(selectedForTournament.at(realTournamentSize - 1));
		}
	}

    // always a even number of selected parents
	if (selectTo.size() % 2 == 1)
	{
		selectTo.push_back(selectTo.at((int)(drand48() * (double)(selectTo.size()))));
	}
}

// breeding for cupids and reapers (for breeders see BreedBreeder) 
void BreedFateAgent(robot_t* parentA, robot_t* parentB, robot_t* empty, robot_t* breeder)
{
    // breed selection probabilities for each type of agent
	for (int i = 0; i < 4; ++i)
	{
        // uniform crossover
		if (drand48() < 0.5) // TODO this crossover is not present in the original algorithm
		{
			empty->fateGenome.selectionProbabilities[i] = parentA->fateGenome.selectionProbabilities[i];
		}
		else
		{
			empty->fateGenome.selectionProbabilities[i] = parentB->fateGenome.selectionProbabilities[i];
		}
        // average crossover
        //double averageGene = (parentA->fateGenome.selectionProbabilities[i] + parentB->fateGenome.selectionProbabilities[i] ) / 2;
        //empty->fateGenome.selectionProbabilities[i] = averageGene;
        // mutation
		if (drand48() < FATE_MUTATION_RATE)
		{
			empty->fateGenome.selectionProbabilities[i] += breeder->breederGenome.fateAgentMutationSize * randn_notrig();

			if (empty->fateGenome.selectionProbabilities[i] > 1.0)
			{
				empty->fateGenome.selectionProbabilities[i] = 1.0;
			}
			else if (empty->fateGenome.selectionProbabilities[i] < 0.0)
			{
				empty->fateGenome.selectionProbabilities[i] = 0.0;
			}
		}
	}

    // breed tournamente size
    // uniform crossover
	if (drand48() < 0.5)
	{
		empty->fateGenome.tournamentSize = parentA->fateGenome.tournamentSize;
	}
	else
	{
		empty->fateGenome.tournamentSize = parentB->fateGenome.tournamentSize;
	}
    // average crossover
    //int averageGene = ( parentA->fateGenome.tournamentSize + parentB->fateGenome.tournamentSize ) / 2;
    //empty->fateGenome.tournamentSize = averageGene;
    // average crossover
    // mutation
	if (drand48() < FATE_MUTATION_RATE)
	{
		empty->fateGenome.tournamentSize += (int)((randn_notrig() * breeder->breederGenome.tournamentMutationSize) + 0.5);

		if (empty->fateGenome.tournamentSize > maxTournamentSize)
		{
			empty->fateGenome.tournamentSize = maxTournamentSize;
		}
		else if (empty->fateGenome.tournamentSize < 1)
		{
			empty->fateGenome.tournamentSize = 1;
		}
	}
}

void BreedCupid(robot_t* parentA, robot_t* parentB, robot_t* empty, robot_t* breeder)
{
	empty->type = Cupid;
	empty->out=false;
	empty->empty=false;
	AssignColor(empty);
	BreedFateAgent(parentA, parentB, empty, breeder);
	empty->iterationsFromLastEvaluation = 0;
}

void BreedReaper(robot_t* parentA, robot_t* parentB, robot_t* empty, robot_t* breeder)
{
	empty->type = Reaper;
	empty->out=false;
	empty->empty=false;
	AssignColor(empty);
	BreedFateAgent(parentA, parentB, empty, breeder);
	empty->iterationsFromLastEvaluation = 0;
}

void BreedBreeder(robot_t* parentA, robot_t* parentB, robot_t* empty, robot_t* breeder)
{
	empty->type = Breeder;
	empty->out=false;
	empty->empty=false;
	AssignColor(empty);
	empty->iterationsFromLastEvaluation = 0;

    // breed mutation step sizes (uniform)
	if (drand48() < 0.5)
	{
		empty->breederGenome.candidateSolutionMutationSize = parentA->breederGenome.candidateSolutionMutationSize * exp(BREEDER_LEARNING_RATE * randn_notrig());
		empty->breederGenome.fateAgentMutationSize = parentA->breederGenome.fateAgentMutationSize * exp(BREEDER_LEARNING_RATE * randn_notrig());
		empty->breederGenome.tournamentMutationSize = parentA->breederGenome.tournamentMutationSize * exp(BREEDER_LEARNING_RATE * randn_notrig());
	}
	else
	{
		empty->breederGenome.candidateSolutionMutationSize = parentB->breederGenome.candidateSolutionMutationSize * exp(BREEDER_LEARNING_RATE * randn_notrig());
		empty->breederGenome.fateAgentMutationSize = parentB->breederGenome.fateAgentMutationSize * exp(BREEDER_LEARNING_RATE * randn_notrig());
		empty->breederGenome.tournamentMutationSize = parentB->breederGenome.tournamentMutationSize * exp(BREEDER_LEARNING_RATE * randn_notrig());
	}
    // breed mutation step sizes (average)
    /*double averageGene = (parentA->breederGenome.candidateSolutionMutationSize + parentB->breederGenome.candidateSolutionMutationSize ) / 2;
    empty->breederGenome.candidateSolutionMutationSize = averageGene * exp(BREEDER_LEARNING_RATE * randn_notrig());
    averageGene = (parentA->breederGenome.fateAgentMutationSize + parentB->breederGenome.fateAgentMutationSize ) / 2;
    empty->breederGenome.fateAgentMutationSize = averageGene * exp(BREEDER_LEARNING_RATE * randn_notrig());
    averageGene = (parentA->breederGenome.tournamentMutationSize + parentB->breederGenome.tournamentMutationSize ) / 2;
    empty->breederGenome.tournamentMutationSize = averageGene * exp(BREEDER_LEARNING_RATE * randn_notrig());*/
}

void BreedCandidateSolution(robot_t* parentA, robot_t* parentB, robot_t* empty, robot_t* breeder)
{
	empty->type = CandidateSolution;
	empty->out=false;
	empty->empty=false;
	AssignColor(empty);
	empty->iterationsFromLastEvaluation = 0;

    // breed neural network weights
	for (size_t i = 0; i < candidateSolutionProblemDimension; ++i)
	{
		for (int j = 0; j < 2; ++j){
            // uniform crossover
			if (drand48() < 0.5)
			{
				empty->candidateSolutionGenome.values[i][j] = parentA->candidateSolutionGenome.values[i][j];
			}
			else
			{
				empty->candidateSolutionGenome.values[i][j] = parentB->candidateSolutionGenome.values[i][j];
			}
            // average crossover
            //double geneA = parentA->candidateSolutionGenome.values[i][j];
            //double gebeB = parentB->candidateSolutionGenome.values[i][j];
            //empty->candidateSolutionGenome.values[i][j] = ( geneA + gebeB ) / 2;
            // mutation
			if (drand48() < CS_MUTATION_RATE)
			{
				empty->candidateSolutionGenome.values[i][j] += breeder->breederGenome.candidateSolutionMutationSize * randn_notrig();

				if (empty->candidateSolutionGenome.values[i][j] > candidateSolutionGenomeUpperBound)
				{
					empty->candidateSolutionGenome.values[i][j] = candidateSolutionGenomeUpperBound;
				}
				else if (empty->candidateSolutionGenome.values[i][j] < candidateSolutionGenomeLowerBound)
				{
					empty->candidateSolutionGenome.values[i][j] = candidateSolutionGenomeLowerBound;
				}
			}
		}
	}

//	empty->fitness = ComputeFitness(&(empty->candidateSolutionGenome));
}

void EvaluateCupid(robot_t* robot)
{
	assert(robot->type == Cupid);

    // reset counter from last evaluation
	robot->iterationsFromLastEvaluation = 0;

    // compute neighbourhood
	std::vector<robot_t*> cupids;
	std::vector<robot_t*> reapers;
	std::vector<robot_t*> breeders;
	std::vector<robot_t*> css;
	std::vector<robot_t*> emptyCS;
	std::vector<robot_t*> emptyFA;
	GetAllRobotsInRange(robot, cupids, reapers, breeders, css, emptyCS, emptyFA);
	if(verbose){
		fprintf(systemBehavior,"\t neighbourhood\n");
		fprintf(systemBehavior,"\t\t candidate solutions: %d \n", (int)css.size());
		fprintf(systemBehavior,"\t\t cupids: %d \n", (int)cupids.size());
		fprintf(systemBehavior,"\t\t breeders: %d \n", (int)breeders.size());
		fprintf(systemBehavior,"\t\t reapers: %d \n", (int)reapers.size());
		fprintf(systemBehavior,"\t\t empty CS: %d \n", (int)emptyCS.size());
		fprintf(systemBehavior,"\t\t empty FA: %d \n", (int)emptyFA.size());
	}
    
    // evaluate fitness (average fitness of candidate solutions in the neighbourhood)
    	if (css.size() != 0) {
		double fitness = 0;
		for (std::vector<robot_t*>::const_iterator it = css.begin(); it != css.end(); ++it)
		{
			fitness += (*it)->fitness;
		}
		robot->fitness = fitness / (double)css.size();
	}
	
	if(verbose)
		fprintf(systemBehavior,"\t fitness (average): %f \n", robot->fitness);
    
    // if there are no breeders or no empty robots in the neighbourhood the cupid cannot perform
	if (breeders.empty() || (emptyCS.empty() && emptyFA.empty()))
	{
		return;
	}
    
    // otherwise it performs the selection for each type of agent
	std::vector<robot_t*> selectedCupids;
	std::vector<robot_t*> selectedReapers;
	std::vector<robot_t*> selectedBreeders;
	std::vector<robot_t*> selectedCss;
    CupidSelection(cupids, selectedCupids, robot->fateGenome.selectionProbabilities[Cupid], robot->fateGenome.tournamentSize);
	CupidSelection(reapers, selectedReapers, robot->fateGenome.selectionProbabilities[Reaper], robot->fateGenome.tournamentSize);
	CupidSelection(breeders, selectedBreeders, robot->fateGenome.selectionProbabilities[Breeder], robot->fateGenome.tournamentSize);
	CupidSelection(css, selectedCss, robot->fateGenome.selectionProbabilities[CandidateSolution], robot->fateGenome.tournamentSize);

    // compute types of robot in the neighbourhood
	std::vector<AgentType> typesToChooseFrom;
	if (!selectedCss.empty() && !emptyCS.empty()) {typesToChooseFrom.push_back(CandidateSolution);}
	if (!selectedCupids.empty() && !emptyFA.empty()) {typesToChooseFrom.push_back(Cupid);}
	if (!selectedBreeders.empty() && !emptyFA.empty()) {typesToChooseFrom.push_back(Breeder);}
	if (!selectedReapers.empty() && !emptyFA.empty()) {typesToChooseFrom.push_back(Reaper);}

    // make breeders perform as long as they can do
    
    if(verbose)
		fprintf(systemBehavior,"\t actions\n");
    
	while( !typesToChooseFrom.empty() )
	{
        // select a random type and perform breeding between two parents of this type and an empty robot
		AgentType chosenType = typesToChooseFrom.at((int)(drand48() * ((double)typesToChooseFrom.size() - 1)));
		switch(chosenType)
		{
		case Cupid:
			{
				BreedCupid(selectedCupids[selectedCupids.size() - 1], selectedCupids[selectedCupids.size() - 2], emptyFA[emptyFA.size() - 1], breeders[(int)(drand48() * (double)(breeders.size() - 1))]);
				selectedCupids.pop_back();
				selectedCupids.pop_back();
				emptyFA.pop_back();
				if(verbose)
					fprintf(systemBehavior,"\t\t generate new cupid\n");
				break;
			}
		case Reaper:
			{
				BreedReaper(selectedReapers[selectedReapers.size() - 1], selectedReapers[selectedReapers.size() - 2], emptyFA[emptyFA.size() - 1], breeders[(int)(drand48() * (double)(breeders.size() - 1))]);
				selectedReapers.pop_back();
				selectedReapers.pop_back();
				emptyFA.pop_back();
				if(verbose)
					fprintf(systemBehavior,"\t\t generate new reaper\n");
				break;
			}
		case Breeder:
			{
				BreedBreeder(selectedBreeders[selectedBreeders.size() - 1], selectedBreeders[selectedBreeders.size() - 2], emptyFA[emptyFA.size() - 1], breeders[(int)(drand48() * (double)(breeders.size() - 1))]);
				selectedBreeders.pop_back();
				selectedBreeders.pop_back();
				emptyFA.pop_back();
				if(verbose)
					fprintf(systemBehavior,"\t\t generate new breeder\n");
				break;
			}
		case CandidateSolution:
			{
				BreedCandidateSolution(selectedCss[selectedCss.size() - 1], selectedCss[selectedCss.size() - 2], emptyCS[emptyCS.size() - 1], breeders[(int)(drand48() * (double)(breeders.size() - 1))]);
				selectedCss.pop_back();
				selectedCss.pop_back();
				emptyCS.pop_back();
				if(verbose)
					fprintf(systemBehavior,"\t\t generate new candidate solution\n");
				break;
			}
		default:
			{
				break;
			}
		}
        // re-compute types of robot in the neighbourhood
		typesToChooseFrom.clear();
		if (!selectedCss.empty() && !emptyCS.empty()) {typesToChooseFrom.push_back(CandidateSolution);}
		if (!selectedCupids.empty() && !emptyFA.empty()) {typesToChooseFrom.push_back(Cupid);}
		if (!selectedBreeders.empty() && !emptyFA.empty()) {typesToChooseFrom.push_back(Breeder);}
		if (!selectedReapers.empty() && !emptyFA.empty()) {typesToChooseFrom.push_back(Reaper);}
	}
}

void EmptyRobot(robot_t* robot)
{
	robot->empty = true;
	AssignColor(robot);
}

void ReaperSelection(std::vector<robot_t*> & selectFrom, std::vector<robot_t*> & selectTo, double selectionProbability, size_t tournamentSize)
{
	size_t realTournamentSize = tournamentSize < selectFrom.size() ? tournamentSize : selectFrom.size();
	std::vector<robot_t*> selectedForTournament;

	for (size_t i = 0; i < realTournamentSize; ++i)
	{
		selectedForTournament.push_back(NULL);
	}

	int count = selectFrom.size();
	for (int i = 0; i < count; ++i)
	{
		if(drand48() < selectionProbability)
		{
			for (size_t j = 0; j < realTournamentSize; ++j)
			{
				selectedForTournament.at(j) = selectFrom.at((int)(drand48() * (double)(selectFrom.size() - 1)));
			}

			sort(selectedForTournament.begin(), selectedForTournament.end(), compareRobots);

			selectTo.push_back(selectedForTournament.at(0));

			for (std::vector<robot_t*>::iterator it = selectFrom.begin(); it != selectFrom.end(); ++it)
			{
				if ((*it) == selectedForTournament.at(0))
				{
					selectFrom.erase(it);
					break;
				}
			}
		}
	}
}

void EvaluateReaper(robot_t* robot)
{
	assert(robot->type == Reaper);
    
    // reset counter from last evaluation
	robot->iterationsFromLastEvaluation = 0;

    // compute neighbourhood
	std::vector<robot_t*> cupids;
	std::vector<robot_t*> reapers;
	std::vector<robot_t*> breeders;
	std::vector<robot_t*> css;
	std::vector<robot_t*> emptyCS;
	std::vector<robot_t*> emptyFA;
    GetAllRobotsInRange(robot, cupids, reapers, breeders, css, emptyCS, emptyFA);
    if(verbose){
		fprintf(systemBehavior,"\t neighbourhood\n");
		fprintf(systemBehavior,"\t\t candidate solutions: %d \n", (int)css.size());
		fprintf(systemBehavior,"\t\t cupids: %d \n", (int)cupids.size());
		fprintf(systemBehavior,"\t\t breeders: %d \n", (int)breeders.size());
		fprintf(systemBehavior,"\t\t reapers: %d \n", (int)reapers.size());
		fprintf(systemBehavior,"\t\t empty CS: %d \n", (int)emptyCS.size());
		fprintf(systemBehavior,"\t\t empty FA: %d \n", (int)emptyFA.size());
	}
	
    // evaluate fitness (average fitness of Casndidate Solutions in the neighbourhood)
    	if (css.size() != 0) {
		double fitness = 0;
		for (std::vector<robot_t*>::const_iterator it = css.begin(); it != css.end(); ++it)
		{
			fitness += (*it)->fitness;
		}
		robot->fitness = fitness / (double)css.size();
	}
	
	if(verbose)
		fprintf(systemBehavior,"\t fitness(average): %f \n", robot->fitness);

    // perform selection for each types of agent
	std::vector<robot_t*> selectedCupids;
	std::vector<robot_t*> selectedReapers;
	std::vector<robot_t*> selectedBreeders;
	std::vector<robot_t*> selectedCss;
	ReaperSelection(cupids, selectedCupids, robot->fateGenome.selectionProbabilities[Cupid], robot->fateGenome.tournamentSize);
	ReaperSelection(reapers, selectedReapers, robot->fateGenome.selectionProbabilities[Reaper], robot->fateGenome.tournamentSize);
	ReaperSelection(breeders, selectedBreeders, robot->fateGenome.selectionProbabilities[Breeder], robot->fateGenome.tournamentSize);
	ReaperSelection(css, selectedCss, robot->fateGenome.selectionProbabilities[CandidateSolution], robot->fateGenome.tournamentSize);

    // compute types of agents in the neighbourhood
	std::vector<AgentType> typesToChooseFrom;
	if (!selectedCss.empty()) {typesToChooseFrom.push_back(CandidateSolution);}
	if (!selectedCupids.empty()) {typesToChooseFrom.push_back(Cupid);}
	if (!selectedBreeders.empty()) {typesToChooseFrom.push_back(Breeder);}
	if (!selectedReapers.empty()) {typesToChooseFrom.push_back(Reaper);}

    // kill selected agents for each type
    if(verbose)
    	fprintf(systemBehavior,"\t actions\n");
    
    if (!typesToChooseFrom.empty())
    {
        // select random type and kill an agent of this type
        AgentType chosenType = typesToChooseFrom.at((int)(drand48() * ((double)typesToChooseFrom.size() - 1)));
        switch(chosenType)
        {
        case Cupid:
            {
                EmptyRobot(selectedCupids[selectedCupids.size() - 1]);
                //emptyFA.push_back(selectedCupids[selectedCupids.size() - 1]);
                selectedCupids.pop_back();
                if(verbose)
                	fprintf(systemBehavior,"\t\t kill cupid \n");
                break;
            }
        case Reaper:
            {
                EmptyRobot(selectedReapers[selectedReapers.size() - 1]);
                //emptyFA.push_back(selectedReapers[selectedReapers.size() - 1]);
                selectedReapers.pop_back();
                if(verbose)
                	fprintf(systemBehavior,"\t\t kill reaper \n");
                break;
            }
        case Breeder:
            {
                EmptyRobot(selectedBreeders[selectedBreeders.size() - 1]);
                //emptyFA.push_back(selectedBreeders[selectedBreeders.size() - 1]);
                selectedBreeders.pop_back();
                if(verbose)
                	fprintf(systemBehavior,"\t\t kill breeder \n");
                break;
            }
        case CandidateSolution:
            {
                EmptyRobot(selectedCss[selectedCss.size() - 1]);
                //emptyCS.push_back(selectedCss[selectedCss.size() - 1]);
                selectedCss.pop_back();
                if(verbose)
                	fprintf(systemBehavior,"\t\t kill candidate solution \n");
                break;
            }
        default:
            {
                break;
            }
        }
        // re-compute types of agent
        typesToChooseFrom.clear();
        if (!selectedCss.empty()) {typesToChooseFrom.push_back(CandidateSolution);}
        if (!selectedCupids.empty()) {typesToChooseFrom.push_back(Cupid);}
        if (!selectedBreeders.empty()) {typesToChooseFrom.push_back(Breeder);}
        if (!selectedReapers.empty()) {typesToChooseFrom.push_back(Reaper);}
    }
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

// Stage calls this when the model starts up
extern "C" int Init( Model* mod, CtrlArgs* args )
{

  robot_t* robot = new robot_t;

  robot->pos = (ModelPosition*)mod;

  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
  robot->pos->Subscribe(); // starts the position updates

  robot->pos->setUserData(robot);

  AgentInit(robot);

  return 0; //ok
}

double SigmoidFunction(double x)
{
	return 1.0 / (1.0 + pow(euler_e, -x));
}

bool isOutOfBound(robot_t* robot){
	Pose myPose = robot->pos->GetGlobalPose();
	return (myPose.x>sizeArena || myPose.x<-sizeArena || myPose.y>sizeArena || myPose.y<-sizeArena);
}

int PositionUpdate( Model* mod, robot_t* robot )
{

	robot->iteration++;
	robot->iterationsFromLastEvaluation++;
	
	char* robotType = "";

    bool perform = PERFORM_PROBABILITY > drand48();
	if (!robot->empty && iter % numberOfIterationsPerEvaluation == 0 && robot->iteration > initialCooldownPriodLength)
	{
        
        SetSpeed(robot,drand48()*maxSpeed,drand48()*maxSpeed);
        
        if (perform) {
            switch (robot->type)
            {
                case Cupid:
                    EvaluateCupid(robot);
                    robotType = "Cupid";
                    break;
                case Reaper:
                    EvaluateReaper(robot);
                    robotType = "Reaper";
                    break;
                case Breeder:
                    EvaluateBreeder(robot);
                    robotType = "Breeder";
                    break;
                default:
                    break;
            }
        }
		
		fprintf(fitnessFA,"%f ",robot->fitness);

		robot->iterationsFromLastEvaluation = 0;
		
		if(verbose)
			fprintf(systemBehavior,"%s ITERATION %d \n", robotType, robot->iteration);
	}

	if (robot->empty) {
		SetSpeed(robot,0,0);
	}

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
		
	if (iter < robot->iteration) {
		if (iter % numberOfIterationsPerEvaluation == 0)
			fprintf(fitnessFA,"%d \n",-1);
		iter++;
	}
	
	return 0; // run again
}


