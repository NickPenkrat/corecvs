#ifndef MATCHINGPLAN_H
#define MATCHINGPLAN_H

#include <vector>
#include <iostream>

struct MatchPlanEntry {
	void save(std::ostream &os) const;
	void load(std::istream &is);

	friend std::ostream& operator<<(std::ostream& os, const MatchPlanEntry& mp);
	friend std::istream& operator>>(std::istream& is, MatchPlanEntry& mp);

	size_t queryImg;
	size_t trainImg;

	std::vector<size_t> queryFeatures;
	std::vector<size_t> trainFeatures;
};

struct MatchPlan {
	void save(std::ostream &os) const;
	void load(std::istream &is);
	void save(const std::string &filename) const;
	void load(const std::string &filename);

	friend std::ostream& operator<<(std::ostream& os, const MatchPlan& mp);
	friend std::istream& operator>>(std::istream& is, MatchPlan& mp);

	std::vector<MatchPlanEntry> plan;
};

#endif
