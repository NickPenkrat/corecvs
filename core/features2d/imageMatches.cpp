#include "imageMatches.h"

#include <fstream>
#include <iomanip>
#include <cassert>

std::istream& operator>>(std::istream& is, RawMatch &rm) {
	is >> rm.imgQ >> rm.imgT >> rm.featureQ >> rm.featureT >> rm.distance;
	return is;
}

std::ostream& operator<<(std::ostream& os, const RawMatch &rm) {
	os << std::setprecision(15) << std::scientific;
	os << std::setw(20) << rm.imgQ << "\t" <<
		  std::setw(20) << rm.imgT << "\t" << 
		  std::setw(20) << rm.featureQ << "\t" <<
		  std::setw(20) << rm.featureT << "\t" <<
		  std::setw(20) << rm.distance << "\t";
	return os;
}

void RawMatches::load(const std::string &filename) {
	std::ifstream is;
	is.open(filename, std::ifstream::in);

	load(is);
}

void RawMatches::save(const std::string &filename) const {
	std::ofstream os;
	os.open(filename, std::ofstream::out);

	save(os);
}

void RawMatches::load(std::istream &ifs) {
	size_t M, K;
	ifs >> M;
	std::cerr << M << " matching sets" << std::endl;

	matches.clear();
	matches.resize(M);

	for(size_t i = 0; i < M; ++i) {
		assert(ifs);

		size_t L;

		ifs >> L;

		matches[i].resize(L);

		for(size_t j = 0; j < L; ++j) {
//			matches[i][j].resize(K);
			ifs >> K;
			matches[i][j].resize(K);

			for(size_t k = 0; k < K; ++k) {
#if 0
				ifs >> matches[i][j][k].trainIdx
					>> matches[i][j][k].queryIdx
					>> matches[i][j][k].distance;
//		rawMatches[i][j][k].queryIdx = matchPlan.plan[i].queryFeatures[j];
#else
				ifs >> matches[i][j][k];
#endif
			}

		}
	}
}

void RawMatches::save(std::ostream &ofs) const {
	ofs.precision(9);

	size_t M = matches.size(), K ;
	ofs << M << std::endl;
	std::cerr << M << " matching sets" << std::endl;


	for(size_t i = 0; i < M; ++i) {
		assert(ofs);

		size_t L = matches[i].size();

		ofs << L << std::endl;

		for(size_t j = 0; j < L; ++j) {
			ofs << (K = matches[i][j].size()) << "\t";
			for(size_t k = 0; k < K; ++k) {
#if 0
				ofs << std::setw(15) << matches[i][j][k].trainIdx << "\t" 
					<< std::setw(15) << matches[i][j][k].queryIdx << "\t"
					<< std::setw(15) << matches[i][j][k].distance << "\t";
#else
				ofs << matches[i][j][k];
#endif
			}
			ofs << std::endl;		
		}
	}
}

void RefinedMatches::load(const std::string &filename) {
	std::ifstream is;
	is.open(filename, std::ifstream::in);

	load(is);
}

void RefinedMatches::save(const std::string &filename) const {
	std::ofstream os;
	os.open(filename, std::ofstream::out);

	save(os);
}

void RefinedMatches::load(std::istream &ifs) {
	size_t M;
	ifs >> M;
	std::cerr << M << " matched image pairss" << std::endl;

	matchSets.clear();
	matchSets.reserve(M);

	for(size_t i = 0; i < M; ++i) {
		size_t I, J, K;
		assert(ifs);
		ifs >> I >> J >> K;

		RefinedMatchSet set(I, J);
		set.matches.reserve(K);

		for(size_t j = 0; j < K; ++j) {
			size_t mi, mj;
			double dist;
			ifs >> mi >> mj >> dist;
			set.matches.push_back(Match(I, J, mi, mj, dist));
		}
		matchSets.push_back(set);
	}
}

void RefinedMatches::save(std::ostream &ofs) const {
	ofs.precision(9);

	size_t M = matchSets.size();
	ofs << M << std::endl;

	for(size_t i = 0; i < M; ++i) {
		size_t I = matchSets[i].imgA, J = matchSets[i].imgB, K = matchSets[i].matches.size();
		ofs << I << " " << J << " " << K << std::endl;

		for(size_t j = 0; j < K; ++j) {
			ofs << std::setw(15) << matchSets[i].matches[j].featureA
				<< std::setw(15) << matchSets[i].matches[j].featureB
				<< std::setw(15) << matchSets[i].matches[j].distance
				<< std::endl;
		}
	}

}
