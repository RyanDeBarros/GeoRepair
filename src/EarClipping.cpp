#include "EarClipping.h"

#include "defects/Common.h"

struct Node;
struct EarClippingData
{
	std::vector<std::shared_ptr<Node>> nodes;
	std::weak_ptr<Node> head_polygon;
	std::weak_ptr<Node> head_convex;
	std::weak_ptr<Node> tail_convex;
	std::weak_ptr<Node> head_reflex;
	std::weak_ptr<Node> tail_reflex;
	std::weak_ptr<Node> head_ear;
	size_t size = 0;
	Eigen::RowVector3d normal;
	const Eigen::MatrixXd* vertices;
};

struct Node
{
	Eigen::Index v;

	Node(Eigen::Index v) : v(v) {}

	// cyclical
	std::weak_ptr<Node> next_vertex;
	std::weak_ptr<Node> prev_vertex;

	bool is_reflex = false;
	bool is_ear = false;
	// linear
	std::weak_ptr<Node> next_convex;
	std::weak_ptr<Node> prev_convex;
	// linear
	std::weak_ptr<Node> next_reflex;
	std::weak_ptr<Node> prev_reflex;
	// cyclical
	std::weak_ptr<Node> next_ear;
	std::weak_ptr<Node> prev_ear;

	struct Triangle
	{
		Eigen::RowVector3d root, prev, next;
	};

	Triangle triangle(const EarClippingData& data) const
	{
		return Triangle{ data.vertices->row(v), data.vertices->row(prev_vertex.lock()->v), data.vertices->row(next_vertex.lock()->v)};
	}

	Eigen::RowVector3i face() const
	{
		return Eigen::RowVector3i(prev_vertex.lock()->v, v, next_vertex.lock()->v);
	}

	bool should_be_reflexive(const EarClippingData& data) const
	{
		auto tr = triangle(data);
		return data.normal.dot((tr.next - tr.root).cross(tr.prev - tr.root)) <= 0.0;
	}

	bool should_be_ear(const EarClippingData& data) const
	{
		if (is_reflex)
			return false;
		auto tr = triangle(data);
		auto pv = prev_vertex.lock();
		for (auto tester = next_vertex.lock()->next_vertex.lock(); tester != pv; tester = tester->next_vertex.lock())
		{
			if (tester->should_be_reflexive(data))
			{
				if (projects_onto_triangle(data.vertices->row(tester->v), tr.root, tr.prev, tr.next))
					return false;
			}
		}
		return true;
	}
};

static std::shared_ptr<Node> append_vertex(EarClippingData& data, Eigen::Index v)
{
	std::shared_ptr<Node> insert = std::make_shared<Node>(v);
	data.nodes.push_back(insert);
	if (auto hp = data.head_polygon.lock())
	{
		const std::shared_ptr<Node>& tail = hp->prev_vertex.lock();
		insert->prev_vertex = tail;
		tail->next_vertex = insert;
		insert->next_vertex = data.head_polygon;
		hp->prev_vertex = insert;
	}
	else
	{
		data.head_polygon = insert;
		hp = data.head_polygon.lock();
		hp->next_vertex = data.head_polygon;
		hp->prev_vertex = data.head_polygon;
	}
	return insert;
}

static void append_reflex(EarClippingData& data, const std::shared_ptr<Node>& insert)
{
	if (data.head_reflex.lock())
	{
		data.tail_reflex.lock()->next_reflex = insert;
		insert->prev_reflex = data.tail_reflex;
		data.tail_reflex = insert;
	}
	else
	{
		data.head_reflex = insert;
		data.tail_reflex = insert;
	}
}

static void append_convex(EarClippingData& data, const std::shared_ptr<Node>& insert)
{
	if (data.head_convex.lock())
	{
		data.tail_convex.lock()->next_convex = insert;
		insert->prev_convex = data.tail_convex;
		data.tail_convex = insert;
	}
	else
	{
		data.head_convex = insert;
		data.tail_convex = insert;
	}
}

static void append_ear(EarClippingData& data, const std::shared_ptr<Node>& insert)
{
	if (auto he = data.head_ear.lock())
	{
		const std::shared_ptr<Node>& tail = he->prev_ear.lock();
		insert->prev_ear = tail;
		tail->next_ear = insert;
		insert->next_ear = data.head_ear;
		he->prev_ear = insert;
	}
	else
	{
		data.head_ear = insert;
		he = data.head_ear.lock();
		he->next_ear = data.head_ear;
		he->prev_ear = data.head_ear;
	}
}
static void update_adjacent(EarClippingData& data, const std::shared_ptr<Node>& adj)
{
	// note that if an adjacent vertex is convex, it will remain convex after removing ear.
	if (adj->is_reflex)
	{
		if (adj->should_be_reflexive(data))
			return; // was reflexive and continues to be reflexive --> no change
		else
		{
			// change to convex
			adj->is_reflex = false;
			append_convex(data, adj);

			// remove from reflexive
			if (data.head_reflex.lock() == adj)
			{
				if (data.tail_reflex.lock() == adj)
					data.tail_reflex.reset();
				else
					adj->next_reflex.lock()->prev_reflex.reset();
				data.head_reflex = adj->next_reflex;
			}
			else
			{
				if (data.tail_reflex.lock() == adj)
					data.tail_reflex = adj->prev_reflex;
				else
					adj->next_reflex.lock()->prev_reflex = adj->prev_reflex;
				adj->prev_reflex.lock()->next_reflex = adj->next_reflex;
			}
			adj->next_reflex.reset();
			adj->prev_reflex.reset();
		}
	}

	if (adj->should_be_ear(data))
	{
		if (!adj->is_ear)
		{
			adj->is_ear = true;
			append_ear(data, adj);
		}
	}
	else
	{
		if (adj->is_ear)
		{
			adj->is_ear = false;

			// remove from ear
			if (adj == data.head_ear.lock())
			{
				if (adj->next_ear.lock() == adj)
					data.head_ear.reset();
				else
				{
					adj->next_ear.lock()->prev_ear = adj->prev_ear;
					adj->prev_ear.lock()->next_ear = adj->next_ear;
					data.head_ear = adj->next_ear;
				}
			}
			else
			{
				adj->next_ear.lock()->prev_ear = adj->prev_ear;
				adj->prev_ear.lock()->next_ear = adj->next_ear;
			}
			adj->next_ear.reset();
			adj->prev_ear.reset();
		}
	}
};

static void recompute_normal(EarClippingData& data)
{
	std::shared_ptr<Node> indexer = data.head_polygon.lock();
	data.normal.setZero();
	auto hp = data.head_polygon.lock();
	do
	{
		auto triangle = indexer->triangle(data);
		data.normal += (triangle.root - triangle.next).cross(triangle.root + triangle.next);

		indexer = indexer->next_vertex.lock();
	} while (indexer != hp);
	data.normal.normalize();
}

static void remove_ear(EarClippingData& data, std::shared_ptr<Node> remove, bool flatten)
{
	assert(remove->is_ear);
	// remove from polygon
	auto hp = data.head_polygon.lock();
	if (remove == hp)
	{
		if (hp->next_vertex.lock() == hp)
			data.head_polygon.reset();
		else
		{
			data.head_polygon = remove->next_vertex;
			remove->next_vertex.lock()->prev_vertex = remove->prev_vertex;
			remove->prev_vertex.lock()->next_vertex = remove->next_vertex;
		}
	}
	else
	{
		remove->next_vertex.lock()->prev_vertex = remove->prev_vertex;
		remove->prev_vertex.lock()->next_vertex = remove->next_vertex;
	}
	// remove from reflex
	if (remove->is_reflex)
	{
		auto hr = data.head_reflex.lock();
		if (remove == hr)
		{
			if (hr->next_reflex.lock() == hr)
				data.head_reflex.reset();
			else
			{
				data.head_reflex = remove->next_reflex;
				remove->next_reflex.lock()->prev_vertex.reset();
			}
		}
		else
		{
			std::shared_ptr<Node> following = remove->next_reflex.lock();
			if (following != nullptr)
				following->prev_reflex = remove->prev_reflex;
			remove->prev_reflex.lock()->next_reflex = following;
		}
		remove->next_reflex.reset();
		remove->prev_reflex.reset();
	}
	// remove from convex
	else
	{
		auto hc = data.head_convex.lock();
		if (remove == hc)
		{
			if (hc->next_convex.lock() == hc)
				data.head_convex.reset();
			else
			{
				data.head_convex = remove->next_convex;
				remove->next_convex.lock()->prev_convex.reset();
			}
		}
		else
		{
			std::shared_ptr<Node> following = remove->next_convex.lock();
			if (following != nullptr)
				following->prev_convex = remove->prev_convex;
			remove->prev_convex.lock()->next_convex = following;
		}
		remove->next_convex.reset();
		remove->prev_convex.reset();
	}
	// remove from ears
	auto he = data.head_ear.lock();
	if (remove == he)
	{
		if (he->next_ear.lock() == he)
			data.head_ear.reset();
		else
		{
			remove->next_ear.lock()->prev_ear = remove->prev_ear;
			remove->prev_ear.lock()->next_ear = remove->next_ear;
			data.head_ear = remove->next_ear;
		}
	}
	else
	{
		remove->next_ear.lock()->prev_ear = remove->prev_ear;
		remove->prev_ear.lock()->next_ear = remove->next_ear;
	}
	remove->next_ear.reset();
	remove->prev_ear.reset();

	// update categorization of adjacent vertices
	update_adjacent(data, remove->next_vertex.lock());
	update_adjacent(data, remove->prev_vertex.lock());
	if (flatten)
		recompute_normal(data);
	remove->next_vertex.reset();
	remove->prev_vertex.reset();
	--data.size;
}

void ear_clipping(const std::vector<Eigen::Index>& boundary, const Eigen::MatrixXd& vertices, std::vector<Eigen::RowVector3i>& add_faces, bool increasing, int ear_cycle, int reference_point_offset, bool flatten)
{
	if (boundary.size() == 3)
	{
		Eigen::RowVector3i face(boundary[pos_mod(0 + reference_point_offset, boundary.size())], boundary[pos_mod(1 + reference_point_offset, boundary.size())], boundary[pos_mod(2 + reference_point_offset, boundary.size())]);
		add_faces.push_back(increasing ? face : face.reverse());
		return;
	}

	EarClippingData data{};
	data.size = boundary.size();
	data.vertices = &vertices;

	// load polygon vertices
	for (int i = 0; i < boundary.size(); ++i)
		append_vertex(data, boundary[pos_mod(i + reference_point_offset, boundary.size())]);

	recompute_normal(data);

	std::shared_ptr<Node> indexer = data.head_polygon.lock();
	// categorize initial vertices
	do
	{
		if (indexer->should_be_reflexive(data))
		{
			indexer->is_reflex = true;
			append_reflex(data, indexer);
		}
		else
		{
			indexer->is_reflex = false;
			append_convex(data, indexer);
			if (indexer->should_be_ear(data))
			{
				indexer->is_ear = true;
				append_ear(data, indexer);
			}
		}

		indexer = indexer->next_vertex.lock();
	} while (indexer != data.head_polygon.lock());

	// remove ears and form faces
	if (ear_cycle == 0)
	{
		while (data.size > 3)
		{
			Eigen::RowVector3i face = data.head_ear.lock()->face();
			add_faces.push_back(increasing ? face : face.reverse());
			remove_ear(data, data.head_ear.lock(), flatten);
		}
	}
	else if (ear_cycle > 0)
	{
		std::shared_ptr<Node> indexer = data.head_ear.lock();
		while (data.size > 3)
		{
			std::shared_ptr<Node> next_indexer = indexer;
			for (int i = 0; i < ear_cycle; ++i)
				next_indexer = next_indexer->next_ear.lock();
			Eigen::RowVector3i face = indexer->face();
			add_faces.push_back(increasing ? face : face.reverse());
			remove_ear(data, indexer, flatten);
			indexer = next_indexer;
		}
	}
	else // if (ear_cycle < 0)
	{
		std::shared_ptr<Node> indexer = data.head_ear.lock();
		while (data.size > 3)
		{
			std::shared_ptr<Node> prev_indexer = indexer;
			for (int i = 0; i > ear_cycle; --i)
				prev_indexer = prev_indexer->prev_ear.lock();
			Eigen::RowVector3i face = indexer->face();
			add_faces.push_back(increasing ? face : face.reverse());
			remove_ear(data, indexer, flatten);
			indexer = prev_indexer;
		}
	}
	// final face
	Eigen::RowVector3i face = data.head_ear.lock()->face();
	add_faces.push_back(increasing ? face : face.reverse());
}
