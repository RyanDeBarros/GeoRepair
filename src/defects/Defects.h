#pragma once

#include "DegenerateFaces.h"
#include "DegenerateVertexPatch.h"
#include "DuplicateFaces.h"
#include "GeneralDuplicateVertices.h"
#include "InvertedNormals.h"
#include "NoiseSmoothing.h"
#include "NonManifoldEdges.h"
#include "NullFaces.h"
#include "UnboundVertices.h"
#include "UnconnectedSubmeshes.h"
#include "UnpatchedHoles.h"

enum Defect
{
	DEGENERATE_FACES,
	DEGENERATE_VERTEX_PATCH,
	DUPLICATE_FACES,
	GENERAL_DUPLICATE_VERTICES,
	INVERTED_NORMALS,
	NOISE_SMOOTHING,
	NON_MANIFOLD_EDGES,
	NULL_FACES,
	UNBOUND_VERTICES,
	UNCONNECTED_SUBMESHES,
	UNPATCHED_HOLES
};

class DefectList
{
	std::vector<std::unique_ptr<defects::DefectBase>> list;

public:
	DefectList()
	{
		list.push_back(std::make_unique<defects::DegenerateFaces>());
		list.push_back(std::make_unique<defects::DegenerateVertexPatch>());
		list.push_back(std::make_unique<defects::DuplicateFaces>());
		list.push_back(std::make_unique<defects::GeneralDuplicateVertices>());
		list.push_back(std::make_unique<defects::InvertedNormals>());
		list.push_back(std::make_unique<defects::NoiseSmoothing>());
		list.push_back(std::make_unique<defects::NonManifoldEdges>());
		list.push_back(std::make_unique<defects::NullFaces>());
		list.push_back(std::make_unique<defects::UnboundVertices>());
		list.push_back(std::make_unique<defects::UnconnectedSubmeshes>());
		list.push_back(std::make_unique<defects::UnpatchedHoles>());
	}

	template<Defect defect>
	auto& get()
	{
#define condition(_enum, _type) if constexpr (defect == _enum) return *(defects::_type*)list[Defect::_enum].get();

		condition(DEGENERATE_FACES, DegenerateFaces)
		else
		condition(DEGENERATE_VERTEX_PATCH, DegenerateVertexPatch)
		else
		condition(DUPLICATE_FACES, DuplicateFaces)
		else
		condition(GENERAL_DUPLICATE_VERTICES, GeneralDuplicateVertices)
		else
		condition(INVERTED_NORMALS, InvertedNormals)
		else
		condition(NOISE_SMOOTHING, NoiseSmoothing)
		else
		condition(NON_MANIFOLD_EDGES, NonManifoldEdges)
		else
		condition(NULL_FACES, NullFaces)
		else
		condition(UNBOUND_VERTICES, UnboundVertices)
		else
		condition(UNCONNECTED_SUBMESHES, UnconnectedSubmeshes)
		else
		condition(UNPATCHED_HOLES, UnpatchedHoles)
		else
		static_assert(false, "defect is not valid");
#undef condition
	}

	defects::DefectBase& operator[](Defect defect)
	{
		return *list[defect];
	}

	void repair_all(Mesh& mesh)
	{
		bool any_detected = false;
		for (auto& defect : list)
		{
			if (defect->in_detected_state())
			{
				defect->_repair(mesh);
				defect->reset();
				any_detected = true;
			}
		}
		if (any_detected)
			mesh.push();
	}

	void reset_all()
	{
		for (auto& defect : list)
			defect->reset();
	}

	size_t size() const { return list.size(); }
};
