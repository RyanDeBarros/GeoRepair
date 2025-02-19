#pragma once

#include <Eigen/core>

#include <unordered_map>
#include <unordered_set>

class EquivalenceClasses
{
	std::unordered_map<Eigen::Index, Eigen::Index> parent;
	std::unordered_map<Eigen::Index, size_t> rank;

	void make_set(Eigen::Index x);
	Eigen::Index find(Eigen::Index x);
	void unite(Eigen::Index x, Eigen::Index y);

public:
	void clear();
	void add(const std::vector<Eigen::Index>& group);
	std::unordered_map<Eigen::Index, std::unordered_set<Eigen::Index>> gen_classes();
};
