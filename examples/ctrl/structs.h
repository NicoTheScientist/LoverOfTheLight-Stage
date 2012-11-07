#ifndef STRUCTS_H
#define STRUCTS_H

#include "stage.hh"
#include <fstream>
#include <iostream>

using namespace Stg;

static const bool verbose = false;

static const double wheelSeparation = 0.38;
static const double maxSpeed = 0.4;
static const double minSpeed = -0.1;
static const double sensorRange = 1.0;
static const double activationThreshold=0.98;

static const double fateAgentSpawnProbability = 0.33;
static const double fateAgentRange = 5;

static const int maxTournamentSize = 50;
static const double maxInitialSelectionProbabilitiesCupid = 0.1;
static const double maxInitialSelectionProbabilitiesReaper = 0.1;

static const double CS_MUTATION_RATE = 0.95;
static const double FATE_MUTATION_RATE = 0.95;
static const double BREEDER_LEARNING_RATE = 0.25;
static const double MAX_INITIAL_CS_MUTATION_SIZE = 0.05;
static const double MAX_INITIAL_FATE_MUTATION_SIZE = 0.4;
static const double MAX_INITIAL_TOURNAMENT_MUTATION_SIZE = 0.4;

static const double candidateSolutionGenomeLowerBound = -1.0;
static const double candidateSolutionGenomeUpperBound = 1.0;
static const double candidateSolutionGenomeRange = candidateSolutionGenomeUpperBound - candidateSolutionGenomeLowerBound;
static const size_t candidateSolutionProblemDimension = 8;

static const double euler_e = 2.71828182845904523536028747135266249775;

static const int slidingWindowSize = 10;
static const int windowTimeForUnit = 120;
static const int numberOfIterationsPerEvaluation = 100;
static const int initialCooldownPriodLength = 0; // 60 secs of movement

static const int sizeArena = 25;

static int iter = 1;


static FILE *systemBehavior = fopen( "../ps/statistics/system_behavior.txt", "w");
static FILE *fitnessCS = fopen( "../ps/statistics/fitnessCS.txt", "w");
static FILE *fitnessFA = fopen( "../ps/statistics/fitnessFA.txt", "w");


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// STRUCTURES ///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Types of robot
typedef enum
{
	Cupid = 0,
	Reaper,
	Breeder,
	CandidateSolution
} AgentType;

// Fate Agent Genome (only Cupids and Reapers)
typedef struct
{
	double selectionProbabilities[4];
	int tournamentSize;
} FateAgentGenome;

// Breeder Genome
typedef struct
{
	double fateAgentMutationSize;
	double candidateSolutionMutationSize;
	double tournamentMutationSize;
} BreederGenome;

// Candidate Solution Genome
typedef struct
{
	double values[candidateSolutionProblemDimension][2];
} CandidateSolutionGenome;

// Robot
typedef struct
{
	ModelPosition* pos;
	ModelRanger* sonar;

	AgentType type;
	double fitness;

	int window[slidingWindowSize];
	bool out;
	bool empty;

	FateAgentGenome fateGenome;
	BreederGenome breederGenome;
	CandidateSolutionGenome candidateSolutionGenome;

	int iterationsFromLastEvaluation;
	int iteration;

	double rightWheelSpeed, leftWheelSpeed;

} robot_t;

#endif
