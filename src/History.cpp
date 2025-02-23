#include "History.h"

void History::push(const std::shared_ptr<MeshPrimaryData>& data)
{
	redo_deque.clear();
	if (undo_deque.size() == tracking_length)
		undo_deque.pop_front();
	undo_deque.push_back(data);
}

void History::clear()
{
	undo_deque.clear();
	redo_deque.clear();
}

std::shared_ptr<MeshPrimaryData> History::undo()
{
	if (undo_deque.size() == 1)
		return undo_deque.back();
	else if (!undo_deque.empty())
	{
		redo_deque.push_back(std::move(undo_deque.back()));
		undo_deque.pop_back();
		return undo_deque.back();
	}
	else
		return nullptr;
}

std::shared_ptr<MeshPrimaryData> History::redo()
{
	if (!redo_deque.empty())
	{
		undo_deque.push_back(std::move(redo_deque.back()));
		redo_deque.pop_back();
		return undo_deque.back();
	}
	else if (!undo_deque.empty())
		return undo_deque.back();
	else
		return nullptr;
}

void History::set_tracking_length(int length)
{
	if (length < 1)
		return;
	tracking_length = length;
	while (undo_deque.size() > tracking_length)
		undo_deque.pop_front();
}
