#include "EquivalenceClasses.h"

void EquivalenceClasses::make_set(Eigen::Index x)
{
	if (!parent.count(x))
	{
		parent[x] = x;
		rank[x] = 1;
	}
}

Eigen::Index EquivalenceClasses::find(Eigen::Index x)
{
	if (parent[x] != x)
		parent[x] = find(parent[x]);
	return parent[x];
}

void EquivalenceClasses::unite(Eigen::Index x, Eigen::Index y)
{
	x = find(x);
	y = find(y);

	if (x == y)
		return;

	if (rank[x] < rank[y])
		std::swap(x, y);

	parent[y] = x;
	if (rank[x] == rank[y])
		++rank[x];
}

void EquivalenceClasses::clear()
{
	parent.clear();
	rank.clear();
}

bool EquivalenceClasses::empty() const
{
	return parent.empty();
}

void EquivalenceClasses::add(const std::vector<Eigen::Index>& group)
{
	for (size_t i = 0; i < group.size(); ++i)
	{
		make_set(group[i]);
		if (i > 0)
			unite(group[i], group[i - 1]);
	}
}

void EquivalenceClasses::add(Eigen::Index x, Eigen::Index y)
{
	make_set(x);
	make_set(y);
	unite(x, y);
}

std::unordered_map<Eigen::Index, std::unordered_set<Eigen::Index>> EquivalenceClasses::gen_classes()
{
	decltype(gen_classes()) classes;
	for (auto iter = parent.begin(); iter != parent.end(); ++iter)
		classes[find(iter->first)].insert(iter->first);
	return classes;
}
