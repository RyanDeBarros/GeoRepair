#include "MeshHistory.h"

void MeshHistory::push(const std::shared_ptr<MeshData>& data)
{
	redo_deque.clear();
	if (undo_deque.size() == tracking_length)
		undo_deque.pop_front();
	undo_deque.push_back(data);
	++_index;
}

void MeshHistory::clear()
{
	undo_deque.clear();
	redo_deque.clear();
}

std::shared_ptr<MeshData> MeshHistory::undo()
{
	if (undo_deque.size() > 1)
	{
		auto data = std::move(undo_deque.back());
		undo_deque.pop_back();
		redo_deque.push_back(std::move(data));
		--_index;
		return undo_deque.back();
	}
	return nullptr;
}

std::shared_ptr<MeshData> MeshHistory::redo()
{
	if (!redo_deque.empty())
	{
		undo_deque.push_back(std::move(redo_deque.back()));
		redo_deque.pop_back();
		++_index;
		return undo_deque.back();
	}
	else
		return nullptr;
}

void MeshHistory::set_tracking_length(int length)
{
	if (length < 1)
		return;
	tracking_length = length;
	while (undo_deque.size() > tracking_length)
		undo_deque.pop_front();
}
