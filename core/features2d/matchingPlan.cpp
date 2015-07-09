#include "matchingPlan.h"

#include <cassert>
#include <fstream>

std::ostream& operator<<(std::ostream& os, const MatchPlan& mp)
{
	mp.save(os);
	return os;
}

std::istream& operator>>(std::istream& is, MatchPlan& mp)
{
	mp.load(is);
	return is;
}

std::ostream& operator<<(std::ostream& os, const MatchPlanEntry& mp)
{
	mp.save(os);
	return os;
}

std::istream& operator>>(std::istream& is, MatchPlanEntry& mp)
{
	mp.load(is);
	return is;
}

bool MatchPlanEntry::isBetween(uint16_t imgA, uint16_t imgB)
{
	return (queryImg == imgA && trainImg == imgB) || (trainImg == imgA && queryImg == imgB);
}

void MatchPlanEntry::load(std::istream &is)
{
	size_t I, J, K, L;
	is >> I >> J >> K >> L;

	queryImg = I;
	trainImg = J;

	queryFeatures.resize(K);
	for (size_t i = 0; i < K; ++i)
	{
		assert(is);
		is >> queryFeatures[i];
	}

	trainFeatures.resize(L);
	for (size_t i = 0; i < L; ++i)
	{
		assert(is);
		is >> trainFeatures[i];
	}
}

void MatchPlanEntry::save(std::ostream &os) const
{
	size_t I = queryImg, J = trainImg, K = queryFeatures.size(), L = trainFeatures.size();
	os << I << " " << J << " " <<  K << " " <<  L << std::endl;

	for (size_t i = 0; i < K; ++i)
	{
		os << queryFeatures[i] << " ";
	}
	os << std::endl;

	for (size_t i = 0; i < L; ++i)
	{
		os << trainFeatures[i] << " ";
	}
	os << std::endl;
}

void MatchPlan::load(const std::string &filename)
{
	std::ifstream is;
	is.open(filename, std::ifstream::in);

	load(is);
}

void MatchPlan::save(const std::string &filename) const
{
	std::ofstream os;
	os.open(filename, std::ofstream::out);

	save(os);
}

void MatchPlan::load(std::istream &is)
{
	plan.clear();

	size_t M;
	is >> M;

	plan.resize(M);

	for (size_t j = 0; j < M; ++j)
	{
		assert(is);
		is >> plan[j];
	}
}

void MatchPlan::save(std::ostream &os) const
{
	size_t M = plan.size();
	os << M << std::endl;

	for (size_t j = 0; j < M; ++j)
	{
		assert(os);
		os << plan[j];
	}
}
