/**
 * @file collision.cpp
 * @brief Collision detection system implementation
 *
 * Implements:
 * - GJK algorithm for collision testing
 * - EPA algorithm for contact generation
 * - Dynamic AABB tree for broad-phase
 * - Collision world management
 * - Event system integration
 */

#include "jaguar/physics/collision.h"
#include "jaguar/events/event.h"
#include "jaguar/events/event_dispatcher.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stack>

namespace jaguar::physics {

// ============================================================================
// GJK Implementation
// ============================================================================

Vec3 GJK::minkowski_support(
    const CollisionShape& shape_a, const Vec3& pos_a, const Quat& rot_a,
    const CollisionShape& shape_b, const Vec3& pos_b, const Quat& rot_b,
    const Vec3& direction)
{
    // Support(A-B, d) = Support(A, d) - Support(B, -d)
    Vec3 support_a = shape_a.support(direction, pos_a, rot_a);
    Vec3 support_b = shape_b.support(-direction, pos_b, rot_b);
    return support_a - support_b;
}

bool GJK::test_collision(
    const CollisionShape& shape_a, const Vec3& pos_a, const Quat& rot_a,
    const CollisionShape& shape_b, const Vec3& pos_b, const Quat& rot_b)
{
    // Initial direction: from A to B
    Vec3 direction = pos_b - pos_a;
    if (direction.length_squared() < EPSILON * EPSILON) {
        direction = Vec3(1, 0, 0);
    }

    Simplex simplex;
    simplex.push(minkowski_support(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, direction));

    // New direction towards origin
    direction = -simplex[0];

    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        Vec3 new_point = minkowski_support(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, direction);

        // Check if new point passed the origin
        if (new_point.dot(direction) < 0) {
            return false;  // No collision
        }

        simplex.push(new_point);

        if (do_simplex(simplex, direction)) {
            return true;  // Collision found
        }
    }

    return false;  // Max iterations reached
}

bool GJK::get_contact(
    const CollisionShape& shape_a, const Vec3& pos_a, const Quat& rot_a,
    const CollisionShape& shape_b, const Vec3& pos_b, const Quat& rot_b,
    ContactPoint& contact)
{
    // First, run GJK to check collision
    Vec3 direction = pos_b - pos_a;
    if (direction.length_squared() < EPSILON * EPSILON) {
        direction = Vec3(1, 0, 0);
    }

    Simplex simplex;
    simplex.push(minkowski_support(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, direction));
    direction = -simplex[0];

    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        Vec3 new_point = minkowski_support(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, direction);

        if (new_point.dot(direction) < 0) {
            return false;
        }

        simplex.push(new_point);

        if (do_simplex(simplex, direction)) {
            // Collision confirmed, now run EPA for contact info
            // Simplified EPA: use the closest feature of the simplex

            // For now, use a simplified contact generation
            // Full EPA would expand the polytope to find the exact penetration

            // Use deepest point in simplex direction
            contact.normal = direction.normalized();
            contact.penetration_depth = 0;

            // Find closest point to origin on the Minkowski difference boundary
            // This is a simplified version - full EPA would be more accurate
            Vec3 closest = simplex[0];
            Real min_dist = simplex[0].length_squared();

            for (int j = 1; j < simplex.count; ++j) {
                Real dist = simplex[j].length_squared();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest = simplex[j];
                }
            }

            contact.penetration_depth = std::sqrt(min_dist);

            // Compute contact position (midpoint between supports)
            Vec3 support_a = shape_a.support(contact.normal, pos_a, rot_a);
            Vec3 support_b = shape_b.support(-contact.normal, pos_b, rot_b);
            contact.position = (support_a + support_b) * 0.5;

            // Local coordinates
            contact.local_point_a = rot_a.conjugate().rotate(contact.position - pos_a);
            contact.local_point_b = rot_b.conjugate().rotate(contact.position - pos_b);

            return true;
        }
    }

    return false;
}

bool GJK::do_simplex(Simplex& simplex, Vec3& direction) {
    switch (simplex.count) {
        case 2: return line_case(simplex, direction);
        case 3: return triangle_case(simplex, direction);
        case 4: return tetrahedron_case(simplex, direction);
        default: return false;
    }
}

bool GJK::line_case(Simplex& simplex, Vec3& direction) {
    Vec3 a = simplex[0];
    Vec3 b = simplex[1];

    Vec3 ab = b - a;
    Vec3 ao = -a;

    if (ab.dot(ao) > 0) {
        // Origin is in region of line
        direction = ab.cross(ao).cross(ab);
    } else {
        // Origin is behind a
        simplex.count = 1;
        direction = ao;
    }

    return false;
}

bool GJK::triangle_case(Simplex& simplex, Vec3& direction) {
    Vec3 a = simplex[0];
    Vec3 b = simplex[1];
    Vec3 c = simplex[2];

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ao = -a;

    Vec3 abc = ab.cross(ac);

    // Check which side of triangle the origin is on
    Vec3 abc_ac = abc.cross(ac);
    if (abc_ac.dot(ao) > 0) {
        if (ac.dot(ao) > 0) {
            // Origin in region of AC edge
            simplex.count = 2;
            simplex[1] = simplex[2];
            direction = ac.cross(ao).cross(ac);
        } else {
            // Origin in region of AB edge
            simplex.count = 2;
            direction = ab.cross(ao).cross(ab);
        }
        return false;
    }

    Vec3 ab_abc = ab.cross(abc);
    if (ab_abc.dot(ao) > 0) {
        // Origin in region of AB edge
        simplex.count = 2;
        direction = ab.cross(ao).cross(ab);
        return false;
    }

    // Origin is inside the triangle prism
    if (abc.dot(ao) > 0) {
        // Origin above triangle
        direction = abc;
    } else {
        // Origin below triangle
        std::swap(simplex[1], simplex[2]);
        direction = -abc;
    }

    return false;
}

bool GJK::tetrahedron_case(Simplex& simplex, Vec3& direction) {
    Vec3 a = simplex[0];
    Vec3 b = simplex[1];
    Vec3 c = simplex[2];
    Vec3 d = simplex[3];

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ad = d - a;
    Vec3 ao = -a;

    Vec3 abc = ab.cross(ac);
    Vec3 acd = ac.cross(ad);
    Vec3 adb = ad.cross(ab);

    // Check each face
    if (abc.dot(ao) > 0) {
        // Origin outside ABC face
        simplex.count = 3;
        return triangle_case(simplex, direction);
    }

    if (acd.dot(ao) > 0) {
        // Origin outside ACD face
        simplex[1] = simplex[2];
        simplex[2] = simplex[3];
        simplex.count = 3;
        return triangle_case(simplex, direction);
    }

    if (adb.dot(ao) > 0) {
        // Origin outside ADB face
        simplex[2] = simplex[1];
        simplex[1] = simplex[3];
        simplex.count = 3;
        return triangle_case(simplex, direction);
    }

    // Origin is inside the tetrahedron
    return true;
}

// ============================================================================
// AABB Tree Implementation
// ============================================================================

AABBTree::AABBTree() : root_(NULL_NODE), free_list_(NULL_NODE) {
    nodes_.reserve(256);
}

int AABBTree::allocate_node() {
    if (free_list_ != NULL_NODE) {
        int node = free_list_;
        free_list_ = nodes_[node].parent;
        nodes_[node].parent = NULL_NODE;
        nodes_[node].left = NULL_NODE;
        nodes_[node].right = NULL_NODE;
        nodes_[node].height = 0;
        return node;
    }

    int node = static_cast<int>(nodes_.size());
    nodes_.push_back(Node());
    nodes_[node].parent = NULL_NODE;
    nodes_[node].left = NULL_NODE;
    nodes_[node].right = NULL_NODE;
    nodes_[node].height = 0;
    return node;
}

void AABBTree::free_node(int node_index) {
    nodes_[node_index].parent = free_list_;
    free_list_ = node_index;
}

int AABBTree::insert(EntityId entity_id, const AABB& aabb) {
    int leaf = allocate_node();
    nodes_[leaf].aabb = aabb.expand(AABB_MARGIN);  // Fat AABB
    nodes_[leaf].entity_id = entity_id;
    nodes_[leaf].height = 0;

    entity_to_node_[entity_id] = leaf;

    insert_leaf(leaf);
    return leaf;
}

void AABBTree::insert_leaf(int leaf) {
    if (root_ == NULL_NODE) {
        root_ = leaf;
        nodes_[leaf].parent = NULL_NODE;
        return;
    }

    // Find best sibling using Surface Area Heuristic
    AABB leaf_aabb = nodes_[leaf].aabb;
    int sibling = root_;

    while (!nodes_[sibling].is_leaf()) {
        int left = nodes_[sibling].left;
        int right = nodes_[sibling].right;

        Real area = nodes_[sibling].aabb.surface_area();

        AABB combined = nodes_[sibling].aabb.merge(leaf_aabb);
        Real combined_area = combined.surface_area();

        // Cost of creating new parent for this node and the new leaf
        Real cost = 2.0 * combined_area;
        Real inheritance_cost = 2.0 * (combined_area - area);

        // Cost of descending into left child
        Real cost_left;
        if (nodes_[left].is_leaf()) {
            cost_left = nodes_[left].aabb.merge(leaf_aabb).surface_area() + inheritance_cost;
        } else {
            Real old_area = nodes_[left].aabb.surface_area();
            Real new_area = nodes_[left].aabb.merge(leaf_aabb).surface_area();
            cost_left = (new_area - old_area) + inheritance_cost;
        }

        // Cost of descending into right child
        Real cost_right;
        if (nodes_[right].is_leaf()) {
            cost_right = nodes_[right].aabb.merge(leaf_aabb).surface_area() + inheritance_cost;
        } else {
            Real old_area = nodes_[right].aabb.surface_area();
            Real new_area = nodes_[right].aabb.merge(leaf_aabb).surface_area();
            cost_right = (new_area - old_area) + inheritance_cost;
        }

        // Descend or create sibling
        if (cost < cost_left && cost < cost_right) {
            break;
        }

        sibling = (cost_left < cost_right) ? left : right;
    }

    // Create new parent
    int old_parent = nodes_[sibling].parent;
    int new_parent = allocate_node();
    nodes_[new_parent].parent = old_parent;
    nodes_[new_parent].aabb = leaf_aabb.merge(nodes_[sibling].aabb);
    nodes_[new_parent].height = nodes_[sibling].height + 1;

    if (old_parent != NULL_NODE) {
        if (nodes_[old_parent].left == sibling) {
            nodes_[old_parent].left = new_parent;
        } else {
            nodes_[old_parent].right = new_parent;
        }
    } else {
        root_ = new_parent;
    }

    nodes_[new_parent].left = sibling;
    nodes_[new_parent].right = leaf;
    nodes_[sibling].parent = new_parent;
    nodes_[leaf].parent = new_parent;

    // Walk back and fix heights
    int index = nodes_[leaf].parent;
    while (index != NULL_NODE) {
        index = balance(index);

        int left = nodes_[index].left;
        int right = nodes_[index].right;

        nodes_[index].height = 1 + std::max(nodes_[left].height, nodes_[right].height);
        nodes_[index].aabb = nodes_[left].aabb.merge(nodes_[right].aabb);

        index = nodes_[index].parent;
    }
}

void AABBTree::remove(int node_index) {
    auto it = std::find_if(entity_to_node_.begin(), entity_to_node_.end(),
        [node_index](const auto& pair) { return pair.second == node_index; });

    if (it != entity_to_node_.end()) {
        entity_to_node_.erase(it);
    }

    remove_leaf(node_index);
    free_node(node_index);
}

void AABBTree::remove_leaf(int leaf) {
    if (leaf == root_) {
        root_ = NULL_NODE;
        return;
    }

    int parent = nodes_[leaf].parent;
    int grandparent = nodes_[parent].parent;
    int sibling = (nodes_[parent].left == leaf) ? nodes_[parent].right : nodes_[parent].left;

    if (grandparent != NULL_NODE) {
        if (nodes_[grandparent].left == parent) {
            nodes_[grandparent].left = sibling;
        } else {
            nodes_[grandparent].right = sibling;
        }
        nodes_[sibling].parent = grandparent;

        // Fix heights
        int index = grandparent;
        while (index != NULL_NODE) {
            index = balance(index);

            int left = nodes_[index].left;
            int right = nodes_[index].right;

            nodes_[index].height = 1 + std::max(nodes_[left].height, nodes_[right].height);
            nodes_[index].aabb = nodes_[left].aabb.merge(nodes_[right].aabb);

            index = nodes_[index].parent;
        }
    } else {
        root_ = sibling;
        nodes_[sibling].parent = NULL_NODE;
    }

    free_node(parent);
}

bool AABBTree::update(int node_index, const AABB& new_aabb) {
    // Check if the new AABB fits within the fat AABB
    if (nodes_[node_index].aabb.contains(new_aabb.min) &&
        nodes_[node_index].aabb.contains(new_aabb.max)) {
        return false;  // No reinsert needed
    }

    // Need to reinsert
    EntityId entity_id = nodes_[node_index].entity_id;
    remove_leaf(node_index);
    nodes_[node_index].aabb = new_aabb.expand(AABB_MARGIN);
    insert_leaf(node_index);
    return true;
}

int AABBTree::balance(int node_index) {
    if (nodes_[node_index].is_leaf() || nodes_[node_index].height < 2) {
        return node_index;
    }

    int left = nodes_[node_index].left;
    int right = nodes_[node_index].right;

    int balance_factor = nodes_[right].height - nodes_[left].height;

    // Rotate right
    if (balance_factor > 1) {
        int right_left = nodes_[right].left;
        int right_right = nodes_[right].right;

        // Swap node and right
        nodes_[right].left = node_index;
        nodes_[right].parent = nodes_[node_index].parent;
        nodes_[node_index].parent = right;

        // Update parent
        if (nodes_[right].parent != NULL_NODE) {
            if (nodes_[nodes_[right].parent].left == node_index) {
                nodes_[nodes_[right].parent].left = right;
            } else {
                nodes_[nodes_[right].parent].right = right;
            }
        } else {
            root_ = right;
        }

        // Rotate
        if (nodes_[right_left].height > nodes_[right_right].height) {
            nodes_[right].right = right_left;
            nodes_[node_index].right = right_right;
            nodes_[right_right].parent = node_index;

            nodes_[node_index].aabb = nodes_[left].aabb.merge(nodes_[right_right].aabb);
            nodes_[right].aabb = nodes_[node_index].aabb.merge(nodes_[right_left].aabb);

            nodes_[node_index].height = 1 + std::max(nodes_[left].height, nodes_[right_right].height);
            nodes_[right].height = 1 + std::max(nodes_[node_index].height, nodes_[right_left].height);
        } else {
            nodes_[right].right = right_right;
            nodes_[node_index].right = right_left;
            nodes_[right_left].parent = node_index;

            nodes_[node_index].aabb = nodes_[left].aabb.merge(nodes_[right_left].aabb);
            nodes_[right].aabb = nodes_[node_index].aabb.merge(nodes_[right_right].aabb);

            nodes_[node_index].height = 1 + std::max(nodes_[left].height, nodes_[right_left].height);
            nodes_[right].height = 1 + std::max(nodes_[node_index].height, nodes_[right_right].height);
        }

        return right;
    }

    // Rotate left
    if (balance_factor < -1) {
        int left_left = nodes_[left].left;
        int left_right = nodes_[left].right;

        nodes_[left].left = node_index;
        nodes_[left].parent = nodes_[node_index].parent;
        nodes_[node_index].parent = left;

        if (nodes_[left].parent != NULL_NODE) {
            if (nodes_[nodes_[left].parent].left == node_index) {
                nodes_[nodes_[left].parent].left = left;
            } else {
                nodes_[nodes_[left].parent].right = left;
            }
        } else {
            root_ = left;
        }

        if (nodes_[left_left].height > nodes_[left_right].height) {
            nodes_[left].right = left_left;
            nodes_[node_index].left = left_right;
            nodes_[left_right].parent = node_index;

            nodes_[node_index].aabb = nodes_[right].aabb.merge(nodes_[left_right].aabb);
            nodes_[left].aabb = nodes_[node_index].aabb.merge(nodes_[left_left].aabb);

            nodes_[node_index].height = 1 + std::max(nodes_[right].height, nodes_[left_right].height);
            nodes_[left].height = 1 + std::max(nodes_[node_index].height, nodes_[left_left].height);
        } else {
            nodes_[left].right = left_right;
            nodes_[node_index].left = left_left;
            nodes_[left_left].parent = node_index;

            nodes_[node_index].aabb = nodes_[right].aabb.merge(nodes_[left_left].aabb);
            nodes_[left].aabb = nodes_[node_index].aabb.merge(nodes_[left_right].aabb);

            nodes_[node_index].height = 1 + std::max(nodes_[right].height, nodes_[left_left].height);
            nodes_[left].height = 1 + std::max(nodes_[node_index].height, nodes_[left_right].height);
        }

        return left;
    }

    return node_index;
}

void AABBTree::query_pairs(std::vector<std::pair<EntityId, EntityId>>& pairs) const {
    pairs.clear();

    if (root_ == NULL_NODE) return;

    // For each leaf, query other leaves that might overlap
    for (const auto& [entity_id, node_index] : entity_to_node_) {
        const AABB& aabb = nodes_[node_index].aabb;

        std::vector<EntityId> results;
        query_recursive(root_, aabb, results);

        for (EntityId other_id : results) {
            if (other_id > entity_id) {  // Avoid duplicates
                pairs.emplace_back(entity_id, other_id);
            }
        }
    }
}

void AABBTree::query(const AABB& aabb, std::vector<EntityId>& results) const {
    results.clear();
    if (root_ == NULL_NODE) return;
    query_recursive(root_, aabb, results);
}

void AABBTree::query_recursive(int node_index, const AABB& aabb,
                                std::vector<EntityId>& results) const {
    if (node_index == NULL_NODE) return;

    const Node& node = nodes_[node_index];

    if (!node.aabb.intersects(aabb)) return;

    if (node.is_leaf()) {
        results.push_back(node.entity_id);
    } else {
        query_recursive(node.left, aabb, results);
        query_recursive(node.right, aabb, results);
    }
}

void AABBTree::raycast(const Vec3& origin, const Vec3& direction,
                        Real max_distance, std::vector<EntityId>& results) const {
    results.clear();
    if (root_ == NULL_NODE) return;

    std::stack<int> stack;
    stack.push(root_);

    Vec3 inv_dir(
        1.0 / direction.x,
        1.0 / direction.y,
        1.0 / direction.z
    );

    while (!stack.empty()) {
        int node_index = stack.top();
        stack.pop();

        const Node& node = nodes_[node_index];

        // Ray-AABB intersection test
        Real tmin = 0, tmax = max_distance;

        for (int i = 0; i < 3; ++i) {
            Real min_val = (i == 0) ? node.aabb.min.x : (i == 1) ? node.aabb.min.y : node.aabb.min.z;
            Real max_val = (i == 0) ? node.aabb.max.x : (i == 1) ? node.aabb.max.y : node.aabb.max.z;
            Real orig = (i == 0) ? origin.x : (i == 1) ? origin.y : origin.z;
            Real inv = (i == 0) ? inv_dir.x : (i == 1) ? inv_dir.y : inv_dir.z;

            Real t1 = (min_val - orig) * inv;
            Real t2 = (max_val - orig) * inv;

            if (t1 > t2) std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax) goto next_node;
        }

        if (node.is_leaf()) {
            results.push_back(node.entity_id);
        } else {
            stack.push(node.left);
            stack.push(node.right);
        }

        next_node:;
    }
}

void AABBTree::clear() {
    nodes_.clear();
    entity_to_node_.clear();
    root_ = NULL_NODE;
    free_list_ = NULL_NODE;
}

// ============================================================================
// Collision World Implementation
// ============================================================================

CollisionWorld::CollisionWorld() : broad_phase_(std::make_unique<AABBTree>()) {
}

CollisionWorld::~CollisionWorld() = default;

void CollisionWorld::add_body(EntityId entity_id, std::unique_ptr<CollisionShape> shape,
                               CollisionLayer layer) {
    CollisionBody body;
    body.entity_id = entity_id;
    body.shape = std::move(shape);
    body.layer = layer;
    body.collision_mask = collision_masks::default_mask_for_layer(layer);
    body.update_aabb();

    int node = broad_phase_->insert(entity_id, body.cached_aabb);
    body_to_tree_node_[entity_id] = node;

    bodies_[entity_id] = std::move(body);
}

void CollisionWorld::remove_body(EntityId entity_id) {
    auto it = body_to_tree_node_.find(entity_id);
    if (it != body_to_tree_node_.end()) {
        broad_phase_->remove(it->second);
        body_to_tree_node_.erase(it);
    }

    bodies_.erase(entity_id);
}

void CollisionWorld::update_body(EntityId entity_id, const Vec3& position,
                                  const Quat& orientation) {
    auto it = bodies_.find(entity_id);
    if (it == bodies_.end()) return;

    CollisionBody& body = it->second;
    body.position = position;
    body.orientation = orientation;
    body.aabb_dirty = true;
    body.update_aabb();

    auto tree_it = body_to_tree_node_.find(entity_id);
    if (tree_it != body_to_tree_node_.end()) {
        broad_phase_->update(tree_it->second, body.cached_aabb);
    }
}

void CollisionWorld::set_static(EntityId entity_id, bool is_static) {
    auto it = bodies_.find(entity_id);
    if (it != bodies_.end()) {
        it->second.is_static = is_static;
    }
}

void CollisionWorld::set_trigger(EntityId entity_id, bool is_trigger) {
    auto it = bodies_.find(entity_id);
    if (it != bodies_.end()) {
        it->second.is_trigger = is_trigger;
    }
}

void CollisionWorld::set_collision_mask(EntityId entity_id, const CollisionMask& mask) {
    auto it = bodies_.find(entity_id);
    if (it != bodies_.end()) {
        it->second.collision_mask = mask;
    }
}

const CollisionBody* CollisionWorld::get_body(EntityId entity_id) const {
    auto it = bodies_.find(entity_id);
    return (it != bodies_.end()) ? &it->second : nullptr;
}

void CollisionWorld::detect_collisions() {
    stats_.reset();
    stats_.body_count = bodies_.size();

    // Broad phase
    std::vector<std::pair<EntityId, EntityId>> pairs;
    auto start = std::chrono::steady_clock::now();
    run_broad_phase(pairs);
    auto end = std::chrono::steady_clock::now();
    stats_.broad_phase_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    stats_.broad_phase_pairs = pairs.size();

    // Narrow phase
    start = std::chrono::steady_clock::now();
    run_narrow_phase(pairs);
    end = std::chrono::steady_clock::now();
    stats_.narrow_phase_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    stats_.contacts_generated = contacts_.size();

    // Process triggers
    process_triggers(contacts_);
}

void CollisionWorld::run_broad_phase(std::vector<std::pair<EntityId, EntityId>>& pairs) {
    broad_phase_->query_pairs(pairs);

    // Filter by collision masks
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(),
        [this](const std::pair<EntityId, EntityId>& pair) {
            auto it_a = bodies_.find(pair.first);
            auto it_b = bodies_.find(pair.second);
            if (it_a == bodies_.end() || it_b == bodies_.end()) return true;
            return !it_a->second.can_collide_with(it_b->second);
        }), pairs.end());
}

void CollisionWorld::run_narrow_phase(const std::vector<std::pair<EntityId, EntityId>>& pairs) {
    contacts_.clear();
    std::unordered_set<uint64_t> current_collisions;

    for (const auto& [entity_a, entity_b] : pairs) {
        auto it_a = bodies_.find(entity_a);
        auto it_b = bodies_.find(entity_b);
        if (it_a == bodies_.end() || it_b == bodies_.end()) continue;

        const CollisionBody& body_a = it_a->second;
        const CollisionBody& body_b = it_b->second;

        if (!body_a.shape || !body_b.shape) continue;

        stats_.narrow_phase_tests++;

        ContactPoint contact;
        if (GJK::get_contact(*body_a.shape, body_a.position, body_a.orientation,
                             *body_b.shape, body_b.position, body_b.orientation,
                             contact)) {
            ContactManifold manifold;
            manifold.entity_a = entity_a;
            manifold.entity_b = entity_b;
            manifold.is_trigger = body_a.is_trigger || body_b.is_trigger;
            manifold.contacts.push_back(contact);

            contacts_.push_back(manifold);

            if (!manifold.is_trigger) {
                uint64_t pair_id = pack_entity_pair(entity_a, entity_b);
                current_collisions.insert(pair_id);

                // Check for new collision (collision enter)
                if (active_collisions_.find(pair_id) == active_collisions_.end()) {
                    // Emit CollisionEnter event
                    if (event_dispatcher_) {
                        auto event = events::Event::create_collision_enter(
                            entity_a, entity_b,
                            Vec3(contact.position.x, contact.position.y, contact.position.z),
                            Vec3(contact.normal.x, contact.normal.y, contact.normal.z),
                            contact.penetration_depth,
                            current_time_);
                        event_dispatcher_->dispatch(event);
                    }
                }

                if (collision_callback_) {
                    collision_callback_(manifold);
                }
            }
        }
    }

    // Check for collision exits
    for (uint64_t pair_id : active_collisions_) {
        if (current_collisions.find(pair_id) == current_collisions.end()) {
            EntityId a = static_cast<EntityId>(pair_id >> 32);
            EntityId b = static_cast<EntityId>(pair_id & 0xFFFFFFFF);

            // Emit CollisionExit event
            if (event_dispatcher_) {
                auto event = events::Event::create_collision_exit(a, b, current_time_);
                event_dispatcher_->dispatch(event);
            }
        }
    }

    active_collisions_ = std::move(current_collisions);
}

void CollisionWorld::process_triggers(const std::vector<ContactManifold>& new_contacts) {
    std::unordered_set<uint64_t> current_triggers;

    for (const auto& manifold : new_contacts) {
        if (!manifold.is_trigger) continue;

        uint64_t pair_id = pack_entity_pair(manifold.entity_a, manifold.entity_b);
        current_triggers.insert(pair_id);

        // Check for new trigger entry
        if (active_triggers_.find(pair_id) == active_triggers_.end()) {
            if (trigger_enter_callback_) {
                trigger_enter_callback_(manifold.entity_a, manifold.entity_b, true);
            }
        }
    }

    // Check for trigger exits
    for (uint64_t pair_id : active_triggers_) {
        if (current_triggers.find(pair_id) == current_triggers.end()) {
            EntityId a = static_cast<EntityId>(pair_id >> 32);
            EntityId b = static_cast<EntityId>(pair_id & 0xFFFFFFFF);
            if (trigger_exit_callback_) {
                trigger_exit_callback_(a, b, false);
            }
        }
    }

    active_triggers_ = std::move(current_triggers);
}

std::vector<EntityId> CollisionWorld::query_point(const Vec3& point) const {
    AABB point_aabb(point, point);
    point_aabb = point_aabb.expand(0.001);  // Small epsilon

    std::vector<EntityId> candidates;
    broad_phase_->query(point_aabb, candidates);

    // Filter by actual shape containment
    std::vector<EntityId> results;
    for (EntityId id : candidates) {
        auto it = bodies_.find(id);
        if (it != bodies_.end() && it->second.cached_aabb.contains(point)) {
            // For now, use AABB containment
            // More accurate would be to test against actual shape
            results.push_back(id);
        }
    }

    return results;
}

std::vector<EntityId> CollisionWorld::query_aabb(const AABB& aabb) const {
    std::vector<EntityId> results;
    broad_phase_->query(aabb, results);
    return results;
}

std::vector<CollisionWorld::RayHit> CollisionWorld::raycast(
    const Vec3& origin, const Vec3& direction, Real max_distance) const {
    std::vector<EntityId> candidates;
    broad_phase_->raycast(origin, direction, max_distance, candidates);

    std::vector<RayHit> hits;
    // For accurate raycast, would need to implement ray-shape intersection
    // For now, return all candidates with estimated distances
    for (EntityId id : candidates) {
        auto it = bodies_.find(id);
        if (it != bodies_.end()) {
            RayHit hit;
            hit.entity_id = id;
            hit.distance = (it->second.position - origin).length();
            hit.point = origin + direction.normalized() * hit.distance;
            hit.normal = -direction.normalized();
            hits.push_back(hit);
        }
    }

    // Sort by distance
    std::sort(hits.begin(), hits.end(),
        [](const RayHit& a, const RayHit& b) { return a.distance < b.distance; });

    return hits;
}

bool CollisionWorld::raycast_closest(const Vec3& origin, const Vec3& direction,
                                      RayHit& hit, Real max_distance) const {
    auto hits = raycast(origin, direction, max_distance);
    if (hits.empty()) return false;
    hit = hits[0];
    return true;
}

void CollisionWorld::set_event_dispatcher(events::EventDispatcher* dispatcher) {
    event_dispatcher_ = dispatcher;
}

} // namespace jaguar::physics
