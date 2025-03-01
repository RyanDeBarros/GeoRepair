#include "EarClipping.h"

#include "defects/Common.h"

struct Node;
struct EarClippingData
{
	std::shared_ptr<Node> head_polygon = nullptr;
	std::shared_ptr<Node> head_convex = nullptr;
	std::shared_ptr<Node> tail_convex = nullptr;
	std::shared_ptr<Node> head_reflex = nullptr;
	std::shared_ptr<Node> tail_reflex = nullptr;
	std::shared_ptr<Node> head_ear = nullptr;
	size_t size = 0;
	Eigen::RowVector3d normal;
	const Eigen::MatrixXd* vertices;
};

struct Node
{
	Eigen::Index v;

	Node(Eigen::Index v) : v(v) {}

	// cyclical
	std::shared_ptr<Node> next_vertex = nullptr;
	std::shared_ptr<Node> prev_vertex = nullptr;

	bool is_reflex = false;
	bool is_ear = false;
	// linear
	std::shared_ptr<Node> next_convex = nullptr;
	std::shared_ptr<Node> prev_convex = nullptr;
	// linear
	std::shared_ptr<Node> next_reflex = nullptr;
	std::shared_ptr<Node> prev_reflex = nullptr;
	// cyclical
	std::shared_ptr<Node> next_ear = nullptr;
	std::shared_ptr<Node> prev_ear = nullptr;

	struct Triangle
	{
		Eigen::RowVector3d root, prev, next;
	};

	Triangle triangle(const EarClippingData& data) const
	{
		return Triangle{ data.vertices->row(v), data.vertices->row(prev_vertex->v), data.vertices->row(next_vertex->v) };
	}

	bool should_be_reflexive(const EarClippingData& data) const
	{
		auto tr = triangle(data);
		return data.normal.dot((tr.next - tr.root).cross(tr.prev - tr.root)) < 0.0;
	}

	bool should_be_ear(const EarClippingData& data) const
	{
		if (is_reflex)
			return false;
		auto tr = triangle(data);
		for (auto tester = next_vertex->next_vertex; tester != prev_vertex; tester = tester->next_vertex)
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
	if (data.head_polygon == nullptr)
	{
		data.head_polygon = insert;
		data.head_polygon->next_vertex = data.head_polygon;
		data.head_polygon->prev_vertex = data.head_polygon;
	}
	else
	{
		const std::shared_ptr<Node>& tail = data.head_polygon->prev_vertex;
		insert->prev_vertex = tail;
		tail->next_vertex = insert;
		insert->next_vertex = data.head_polygon;
		data.head_polygon->prev_vertex = insert;
	}
	return insert;
}

static void append_reflex(EarClippingData& data, const std::shared_ptr<Node>& insert)
{
	if (data.head_reflex == nullptr)
	{
		data.head_reflex = insert;
		data.tail_reflex = insert;
	}
	else
	{
		data.tail_reflex->next_reflex = insert;
		insert->prev_reflex = data.tail_reflex;
		data.tail_reflex = insert;
	}
}

static void append_convex(EarClippingData& data, const std::shared_ptr<Node>& insert)
{
	if (data.head_convex == nullptr)
	{
		data.head_convex = insert;
		data.tail_convex = insert;
	}
	else
	{
		data.tail_convex->next_convex = insert;
		insert->prev_convex = data.tail_convex;
		data.tail_convex = insert;
	}
}

static void append_ear(EarClippingData& data, const std::shared_ptr<Node>& insert)
{
	if (data.head_ear == nullptr)
	{
		data.head_ear = insert;
		data.head_ear->next_ear = data.head_ear;
		data.head_ear->prev_ear = data.head_ear;
	}
	else
	{
		const std::shared_ptr<Node>& tail = data.head_ear->prev_ear;
		insert->prev_ear = tail;
		tail->next_ear = insert;
		insert->next_ear = data.head_ear;
		data.head_ear->prev_ear= insert;
	}
}
static void update_adjacent(std::shared_ptr<Node>& adj, EarClippingData& data)
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
			if (data.head_reflex == adj)
			{
				if (data.tail_reflex == adj)
					data.tail_reflex = nullptr;
				else
					adj->next_reflex->prev_reflex = nullptr;
				data.head_reflex = adj->next_reflex;
			}
			else
			{
				if (data.tail_reflex == adj)
					data.tail_reflex = adj->prev_reflex;
				else
					adj->next_reflex->prev_reflex = adj->prev_reflex;
				adj->prev_reflex->next_reflex = adj->next_reflex;
			}
			adj->next_reflex = nullptr;
			adj->prev_reflex = nullptr;
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
			if (adj == data.head_ear)
			{
				if (adj->next_ear == data.head_ear)
					data.head_ear = nullptr;
				else
				{
					adj->next_ear->prev_ear = adj->prev_ear;
					adj->prev_ear->next_ear = adj->next_ear;
					data.head_ear = adj->next_ear;
				}
			}
			else
			{
				adj->next_ear->prev_ear = adj->prev_ear;
				adj->prev_ear->next_ear = adj->next_ear;
			}
			adj->next_ear = nullptr;
			adj->prev_ear = nullptr;
		}
	}
};

static void remove_ear(EarClippingData& data, std::shared_ptr<Node> remove)
{
	assert(remove->is_ear);
	// remove from polygon
	if (remove == data.head_polygon)
	{
		if (data.head_polygon->next_vertex == data.head_polygon)
			data.head_polygon = nullptr;
		else
		{
			data.head_polygon = remove->next_vertex;
			remove->next_vertex->prev_vertex = remove->prev_vertex;
			remove->prev_vertex->next_vertex = remove->next_vertex;
		}
	}
	else
	{
		remove->next_vertex->prev_vertex = remove->prev_vertex;
		remove->prev_vertex->next_vertex = remove->next_vertex;
	}
	// remove from reflex
	if (remove->is_reflex)
	{
		if (remove == data.head_reflex)
		{
			if (data.head_reflex->next_reflex == data.head_reflex)
				data.head_reflex = nullptr;
			else
			{
				data.head_reflex = remove->next_reflex;
				remove->next_reflex->prev_vertex = nullptr;
			}
		}
		else
		{
			std::shared_ptr<Node> following = remove->next_reflex;
			if (following != nullptr)
				following->prev_reflex = remove->prev_reflex;
			remove->prev_reflex->next_reflex = following;
		}
		remove->next_reflex = nullptr;
		remove->prev_reflex = nullptr;
	}
	// remove from convex
	else
	{
		if (remove == data.head_convex)
		{
			if (data.head_convex->next_convex == data.head_convex)
				data.head_convex = nullptr;
			else
			{
				data.head_convex = remove->next_convex;
				remove->next_convex->prev_convex = nullptr;
			}
		}
		else
		{
			std::shared_ptr<Node> following = remove->next_convex;
			if (following != nullptr)
				following->prev_convex = remove->prev_convex;
			remove->prev_convex->next_convex = following;
		}
		remove->next_convex = nullptr;
		remove->prev_convex = nullptr;
	}
	// remove from ears
	if (remove == data.head_ear)
	{
		if (data.head_ear->next_ear == data.head_ear)
			data.head_ear = nullptr;
		else
		{
			remove->next_ear->prev_ear = remove->prev_ear;
			remove->prev_ear->next_ear = remove->next_ear;
			data.head_ear = remove->next_ear;
		}
	}
	else
	{
		remove->next_ear->prev_ear = remove->prev_ear;
		remove->prev_ear->next_ear = remove->next_ear;
	}
	remove->next_ear = nullptr;
	remove->prev_ear = nullptr;

	// update categorization of adjacent vertices
	update_adjacent(remove->next_vertex, data);
	update_adjacent(remove->prev_vertex, data);	
	remove->next_vertex = nullptr;
	remove->prev_vertex = nullptr;
	--data.size;
}

void ear_clipping(const std::vector<Eigen::Index>& boundary, const Eigen::MatrixXd& vertices, std::vector<Eigen::RowVector3i>& add_faces, bool increasing)
{
	if (boundary.size() == 3)
	{
		Eigen::RowVector3i face(boundary[0], boundary[1], boundary[2]);
		add_faces.push_back(increasing ? face : face.reverse());
		return;
	}

	EarClippingData data{};
	data.size = boundary.size();
	data.vertices = &vertices;

	// load polygon vertices
	for (Eigen::Index v : boundary)
		append_vertex(data, v);

	// compute polygonal normal
	std::shared_ptr<Node> indexer = data.head_polygon;
	data.normal.setZero();
	do
	{
		auto triangle = indexer->triangle(data);
		data.normal += (triangle.root - triangle.next).cross(triangle.root + triangle.next);

		indexer = indexer->next_vertex;
	} while (indexer != data.head_polygon);
	data.normal.normalize();

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

		indexer = indexer->next_vertex;
	} while (indexer != data.head_polygon);

	// remove ears and form faces
	while (data.size > 3)
	{
		Eigen::RowVector3i face(data.head_ear->prev_vertex->v, data.head_ear->v, data.head_ear->next_vertex->v);
		add_faces.push_back(increasing ? face : face.reverse());
		remove_ear(data, data.head_ear);
	}
	// final face
	Eigen::RowVector3i face(data.head_ear->prev_vertex->v, data.head_ear->v, data.head_ear->next_vertex->v);
	add_faces.push_back(increasing ? face : face.reverse());
}
