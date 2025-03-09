#include "UnconnectedSubmeshes.h"

void defects::UnconnectedSubmeshes::_detect(const Mesh& mesh)
{
    submeshes = mesh.get_connected_submeshes();
}

void defects::UnconnectedSubmeshes::_repair(Mesh& mesh)
{
    // nothing to repair, as there is no obvious general way of connected distinct submeshes, which might even be intentional.
}

void defects::UnconnectedSubmeshes::reset()
{
    submeshes.clear();
}

bool defects::UnconnectedSubmeshes::in_detected_state() const
{
    return submeshes.size() > 1;
}
