#ifndef IMAGEMATCHES_H
#define IMAGEMATCHES_H

#include <vector>
#include <string>
#include <iostream>

struct RawMatch {
	size_t imgQ;
	size_t imgT;

	size_t featureQ;
	size_t featureT;

	double distance;

	RawMatch(const size_t &imgQ = ~(size_t)0, const size_t &imgT = ~(size_t)0, const size_t &featureQ = ~(size_t)0, const size_t &featureT = ~(size_t)0, const double &distance = 1000.0)
		: imgQ(imgQ), imgT(imgT), featureQ(featureQ), featureT(featureT), distance(distance) {
		}
#if 0
	RawMatch(const cv::DMatch &dmatch, const size_t &imgQ, const size_t &imgT)
		: imgQ(imgQ), imgT(imgT), featureQ(dmatch.queryIdx), featureT(dmatch.trainIdx), distance(dmatch.distance)
	{
	}

	operator cv::DMatch() const {
		return cv::DMatch(featureQ, featureT, distance);
	}
#endif

	friend std::istream& operator>>(std::istream& is, RawMatch &rm);
	friend std::ostream& operator<<(std::ostream& os, const RawMatch &rm);
};

struct Match {
	size_t imgA;
	size_t imgB;

	size_t featureA;
	size_t featureB;

	double distance;

	Match(const size_t &imgA, const size_t &imgB, const size_t &featureA, const size_t &featureB, const double &distance) : distance(distance) {
		if(imgA < imgB) {
			this->imgA = imgA;
			this->featureA = featureA;
			this->imgB = imgB;
			this->featureB = featureB;
		} else {
			this->imgA = imgB;
			this->featureA = featureB;
			this->imgB = imgA;
			this->featureB = featureA;
		}
	}
};

struct RefinedMatchSet {
	size_t imgA;
	size_t imgB;

	std::vector<Match> matches;

	RefinedMatchSet(const size_t &_imgA, const size_t &_imgB) {
		imgB = std::max(_imgA, _imgB);
		imgA = std::min(_imgA, _imgB);
	}
	RefinedMatchSet(const size_t &_imgA, const size_t &_imgB, const std::vector<Match> &_matches) : matches(_matches) {
		imgB = std::max(_imgA, _imgB);
		imgA = std::min(_imgA, _imgB);
	}
};

struct RefinedMatches {
	std::vector<RefinedMatchSet> matchSets;

	void load(std::istream &is);
	void save(std::ostream &os) const;

	void load(const std::string &filename);
	void save(const std::string &filename) const;
};

struct RawMatches {
	std::vector<std::vector<std::vector<RawMatch>>> matches;
	
	void load(std::istream &is);
	void save(std::ostream &os) const;

	void load(const std::string &filename);
	void save(const std::string &filename) const;
};


#endif
