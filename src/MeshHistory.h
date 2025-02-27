#pragma once

#include <deque>

#include "MeshData.h"

// TODO store MeshPrimaryData *and* MeshAuxiliaryData?
class MeshHistory
{
	int tracking_length = 20;
	std::deque<std::shared_ptr<MeshPrimaryData>> undo_deque;
	std::deque<std::shared_ptr<MeshPrimaryData>> redo_deque;
	int _index = 0;

public:
	void push(const std::shared_ptr<MeshPrimaryData>& data);
	void clear();
	std::shared_ptr<MeshPrimaryData> undo();
	std::shared_ptr<MeshPrimaryData> redo();
	void set_tracking_length(int length);
	int index() const { return _index; }
};
