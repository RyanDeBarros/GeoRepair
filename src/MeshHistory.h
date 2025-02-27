#pragma once

#include <deque>

#include "MeshData.h"

class MeshHistory
{
	int tracking_length = 20;
	std::deque<std::shared_ptr<MeshData>> undo_deque;
	std::deque<std::shared_ptr<MeshData>> redo_deque;
	int _index = 0;

public:
	void push(const std::shared_ptr<MeshData>& data);
	void clear();
	std::shared_ptr<MeshData> undo();
	std::shared_ptr<MeshData> redo();
	void set_tracking_length(int length);
	int index() const { return _index; }
};
