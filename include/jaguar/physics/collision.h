/**
 * @file collision.h
 * @brief Collision detection and response system for JaguarEngine
 *
 * Implements a two-phase collision detection system:
 * - Broad phase: AABB tree for spatial partitioning (O(n log n))
 * - Narrow phase: GJK + EPA for precise collision detection
 *
 * Features:
 * - Dynamic AABB tree with incremental updates
 * - Contact point generation
 * - Collision groups/layers/masks
 * - Continuous collision detection (CCD) for fast-moving objects
 * - Event system integration for collision events
 */

#pragma once

#include "jaguar/core/types.h"

#include <vector>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <limits>
#include <algorithm>

// Forward declaration for event system integration
namespace jaguar::events {
    class EventDispatcher;
}

namespace jaguar::physics {

// Forward declarations
class CollisionWorld;
class BroadPhase;
class NarrowPhase;

// ============================================================================
// Collision Types and Constants
// ============================================================================

/**
 * @brief Maximum number of collision layers
 */
constexpr size_t MAX_COLLISION_LAYERS = 32;

/**
 * @brief Collision layer bitmask
 */
using CollisionMask = std::bitset<MAX_COLLISION_LAYERS>;

/**
 * @brief Collision layer enumeration
 */
enum class CollisionLayer : uint32_t {
    Default = 0,
    Aircraft = 1,
    GroundVehicle = 2,
    Ship = 3,
    Submarine = 4,
    Spacecraft = 5,
    Missile = 6,
    Projectile = 7,
    Terrain = 8,
    Ocean = 9,
    Structure = 10,
    Sensor = 11,
    // User-defined layers: 12-31
};

/**
 * @brief Collision shape type
 */
enum class ShapeType : uint8_t {
    Sphere,
    Box,
    Capsule,
    Cylinder,
    ConvexHull,
    Mesh,  // For terrain/static objects
    Compound
};

// ============================================================================
// Axis-Aligned Bounding Box
// ============================================================================

/**
 * @brief Axis-Aligned Bounding Box for broad-phase collision
 */
struct AABB {
    Vec3 min;
    Vec3 max;

    AABB() : min(Vec3::Zero()), max(Vec3::Zero()) {}
    AABB(const Vec3& min_, const Vec3& max_) : min(min_), max(max_) {}

    /**
     * @brief Get center of AABB
     */
    Vec3 center() const {
        return (min + max) * 0.5;
    }

    /**
     * @brief Get half-extents (half-size)
     */
    Vec3 half_extents() const {
        return (max - min) * 0.5;
    }

    /**
     * @brief Get surface area (for tree optimization)
     */
    Real surface_area() const {
        Vec3 d = max - min;
        return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    /**
     * @brief Get volume
     */
    Real volume() const {
        Vec3 d = max - min;
        return d.x * d.y * d.z;
    }

    /**
     * @brief Test intersection with another AABB
     */
    bool intersects(const AABB& other) const {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        if (max.z < other.min.z || min.z > other.max.z) return false;
        return true;
    }

    /**
     * @brief Test if point is inside AABB
     */
    bool contains(const Vec3& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    /**
     * @brief Merge with another AABB
     */
    AABB merge(const AABB& other) const {
        return AABB(
            Vec3(
                std::min(min.x, other.min.x),
                std::min(min.y, other.min.y),
                std::min(min.z, other.min.z)
            ),
            Vec3(
                std::max(max.x, other.max.x),
                std::max(max.y, other.max.y),
                std::max(max.z, other.max.z)
            )
        );
    }

    /**
     * @brief Expand AABB by margin
     */
    AABB expand(Real margin) const {
        Vec3 m(margin, margin, margin);
        return AABB(min - m, max + m);
    }

    /**
     * @brief Create AABB from center and half-extents
     */
    static AABB from_center_extents(const Vec3& center, const Vec3& half_extents) {
        return AABB(center - half_extents, center + half_extents);
    }
};

// ============================================================================
// Collision Shapes
// ============================================================================

/**
 * @brief Base collision shape
 */
class CollisionShape {
public:
    virtual ~CollisionShape() = default;

    /**
     * @brief Get shape type
     */
    virtual ShapeType type() const = 0;

    /**
     * @brief Compute AABB for this shape at given transform
     */
    virtual AABB compute_aabb(const Vec3& position,
                               const Quat& orientation) const = 0;

    /**
     * @brief Get support point in given direction (for GJK)
     */
    virtual Vec3 support(const Vec3& direction,
                                const Vec3& position,
                                const Quat& orientation) const = 0;

    /**
     * @brief Clone this shape
     */
    virtual std::unique_ptr<CollisionShape> clone() const = 0;
};

/**
 * @brief Sphere collision shape
 */
class SphereShape : public CollisionShape {
public:
    explicit SphereShape(Real radius) : radius_(radius) {}

    ShapeType type() const override { return ShapeType::Sphere; }

    AABB compute_aabb(const Vec3& position,
                       const Quat& /*orientation*/) const override {
        Vec3 extents(radius_, radius_, radius_);
        return AABB(position - extents, position + extents);
    }

    Vec3 support(const Vec3& direction,
                        const Vec3& position,
                        const Quat& /*orientation*/) const override {
        return position + direction.normalized() * radius_;
    }

    std::unique_ptr<CollisionShape> clone() const override {
        return std::make_unique<SphereShape>(radius_);
    }

    Real radius() const { return radius_; }

private:
    Real radius_;
};

/**
 * @brief Box collision shape
 */
class BoxShape : public CollisionShape {
public:
    explicit BoxShape(const Vec3& half_extents) : half_extents_(half_extents) {}

    ShapeType type() const override { return ShapeType::Box; }

    AABB compute_aabb(const Vec3& position,
                       const Quat& orientation) const override {
        // Transform each corner and find min/max
        Vec3 corners[8] = {
            Vec3(-half_extents_.x, -half_extents_.y, -half_extents_.z),
            Vec3(+half_extents_.x, -half_extents_.y, -half_extents_.z),
            Vec3(-half_extents_.x, +half_extents_.y, -half_extents_.z),
            Vec3(+half_extents_.x, +half_extents_.y, -half_extents_.z),
            Vec3(-half_extents_.x, -half_extents_.y, +half_extents_.z),
            Vec3(+half_extents_.x, -half_extents_.y, +half_extents_.z),
            Vec3(-half_extents_.x, +half_extents_.y, +half_extents_.z),
            Vec3(+half_extents_.x, +half_extents_.y, +half_extents_.z)
        };

        Vec3 min_pt(std::numeric_limits<Real>::max(),
                          std::numeric_limits<Real>::max(),
                          std::numeric_limits<Real>::max());
        Vec3 max_pt(std::numeric_limits<Real>::lowest(),
                          std::numeric_limits<Real>::lowest(),
                          std::numeric_limits<Real>::lowest());

        for (const auto& corner : corners) {
            Vec3 world_pt = position + orientation.rotate(corner);
            min_pt.x = std::min(min_pt.x, world_pt.x);
            min_pt.y = std::min(min_pt.y, world_pt.y);
            min_pt.z = std::min(min_pt.z, world_pt.z);
            max_pt.x = std::max(max_pt.x, world_pt.x);
            max_pt.y = std::max(max_pt.y, world_pt.y);
            max_pt.z = std::max(max_pt.z, world_pt.z);
        }

        return AABB(min_pt, max_pt);
    }

    Vec3 support(const Vec3& direction,
                        const Vec3& position,
                        const Quat& orientation) const override {
        // Transform direction to local space
        Vec3 local_dir = orientation.conjugate().rotate(direction);

        // Find support point in local space
        Vec3 local_support(
            (local_dir.x > 0) ? half_extents_.x : -half_extents_.x,
            (local_dir.y > 0) ? half_extents_.y : -half_extents_.y,
            (local_dir.z > 0) ? half_extents_.z : -half_extents_.z
        );

        // Transform back to world space
        return position + orientation.rotate(local_support);
    }

    std::unique_ptr<CollisionShape> clone() const override {
        return std::make_unique<BoxShape>(half_extents_);
    }

    const Vec3& half_extents() const { return half_extents_; }

private:
    Vec3 half_extents_;
};

/**
 * @brief Capsule collision shape (cylinder with hemispherical caps)
 */
class CapsuleShape : public CollisionShape {
public:
    CapsuleShape(Real radius, Real height)
        : radius_(radius), half_height_(height * 0.5) {}

    ShapeType type() const override { return ShapeType::Capsule; }

    AABB compute_aabb(const Vec3& position,
                       const Quat& orientation) const override {
        // Capsule endpoints in local space
        Vec3 top(0, half_height_, 0);
        Vec3 bottom(0, -half_height_, 0);

        // Transform to world space
        Vec3 world_top = position + orientation.rotate(top);
        Vec3 world_bottom = position + orientation.rotate(bottom);

        // Compute AABB including sphere radius at each end
        Vec3 min_pt(
            std::min(world_top.x, world_bottom.x) - radius_,
            std::min(world_top.y, world_bottom.y) - radius_,
            std::min(world_top.z, world_bottom.z) - radius_
        );
        Vec3 max_pt(
            std::max(world_top.x, world_bottom.x) + radius_,
            std::max(world_top.y, world_bottom.y) + radius_,
            std::max(world_top.z, world_bottom.z) + radius_
        );

        return AABB(min_pt, max_pt);
    }

    Vec3 support(const Vec3& direction,
                        const Vec3& position,
                        const Quat& orientation) const override {
        // Transform direction to local space
        Vec3 local_dir = orientation.conjugate().rotate(direction);

        // Find which hemisphere cap
        Vec3 local_support;
        if (local_dir.y > 0) {
            local_support = Vec3(0, half_height_, 0);
        } else {
            local_support = Vec3(0, -half_height_, 0);
        }

        // Add sphere support
        local_support = local_support + local_dir.normalized() * radius_;

        // Transform back to world space
        return position + orientation.rotate(local_support);
    }

    std::unique_ptr<CollisionShape> clone() const override {
        return std::make_unique<CapsuleShape>(radius_, half_height_ * 2);
    }

    Real radius() const { return radius_; }
    Real half_height() const { return half_height_; }

private:
    Real radius_;
    Real half_height_;
};

/**
 * @brief Convex hull collision shape
 */
class ConvexHullShape : public CollisionShape {
public:
    explicit ConvexHullShape(const std::vector<Vec3>& vertices)
        : vertices_(vertices) {
        compute_local_aabb();
    }

    ShapeType type() const override { return ShapeType::ConvexHull; }

    AABB compute_aabb(const Vec3& position,
                       const Quat& orientation) const override {
        Vec3 min_pt(std::numeric_limits<Real>::max(),
                          std::numeric_limits<Real>::max(),
                          std::numeric_limits<Real>::max());
        Vec3 max_pt(std::numeric_limits<Real>::lowest(),
                          std::numeric_limits<Real>::lowest(),
                          std::numeric_limits<Real>::lowest());

        for (const auto& vertex : vertices_) {
            Vec3 world_pt = position + orientation.rotate(vertex);
            min_pt.x = std::min(min_pt.x, world_pt.x);
            min_pt.y = std::min(min_pt.y, world_pt.y);
            min_pt.z = std::min(min_pt.z, world_pt.z);
            max_pt.x = std::max(max_pt.x, world_pt.x);
            max_pt.y = std::max(max_pt.y, world_pt.y);
            max_pt.z = std::max(max_pt.z, world_pt.z);
        }

        return AABB(min_pt, max_pt);
    }

    Vec3 support(const Vec3& direction,
                        const Vec3& position,
                        const Quat& orientation) const override {
        // Transform direction to local space
        Vec3 local_dir = orientation.conjugate().rotate(direction);

        // Find vertex with maximum dot product
        Real max_dot = std::numeric_limits<Real>::lowest();
        Vec3 support_vertex = vertices_[0];

        for (const auto& vertex : vertices_) {
            Real d = vertex.dot(local_dir);
            if (d > max_dot) {
                max_dot = d;
                support_vertex = vertex;
            }
        }

        // Transform back to world space
        return position + orientation.rotate(support_vertex);
    }

    std::unique_ptr<CollisionShape> clone() const override {
        return std::make_unique<ConvexHullShape>(vertices_);
    }

    const std::vector<Vec3>& vertices() const { return vertices_; }

private:
    std::vector<Vec3> vertices_;
    AABB local_aabb_;

    void compute_local_aabb() {
        if (vertices_.empty()) return;

        local_aabb_.min = vertices_[0];
        local_aabb_.max = vertices_[0];

        for (const auto& v : vertices_) {
            local_aabb_.min.x = std::min(local_aabb_.min.x, v.x);
            local_aabb_.min.y = std::min(local_aabb_.min.y, v.y);
            local_aabb_.min.z = std::min(local_aabb_.min.z, v.z);
            local_aabb_.max.x = std::max(local_aabb_.max.x, v.x);
            local_aabb_.max.y = std::max(local_aabb_.max.y, v.y);
            local_aabb_.max.z = std::max(local_aabb_.max.z, v.z);
        }
    }
};

// ============================================================================
// Contact Information
// ============================================================================

/**
 * @brief Contact point between two colliders
 */
struct ContactPoint {
    Vec3 position;        // World-space contact position
    Vec3 normal;          // Contact normal (from A to B)
    Real penetration_depth;     // Overlap distance
    Vec3 local_point_a;   // Contact in A's local space
    Vec3 local_point_b;   // Contact in B's local space

    ContactPoint()
        : position(Vec3::Zero())
        , normal(Vec3(0, 1, 0))
        , penetration_depth(0)
        , local_point_a(Vec3::Zero())
        , local_point_b(Vec3::Zero()) {}
};

/**
 * @brief Contact manifold (multiple contact points)
 */
struct ContactManifold {
    EntityId entity_a;
    EntityId entity_b;
    std::vector<ContactPoint> contacts;
    bool is_trigger;  // True if one collider is a trigger

    ContactManifold() : entity_a(INVALID_ENTITY_ID), entity_b(INVALID_ENTITY_ID), is_trigger(false) {}

    bool has_contacts() const { return !contacts.empty(); }

    /**
     * @brief Get deepest contact point
     */
    const ContactPoint* deepest_contact() const {
        if (contacts.empty()) return nullptr;

        const ContactPoint* deepest = &contacts[0];
        for (const auto& c : contacts) {
            if (c.penetration_depth > deepest->penetration_depth) {
                deepest = &c;
            }
        }
        return deepest;
    }
};

// ============================================================================
// Collision Body
// ============================================================================

/**
 * @brief Collision body attached to entity
 */
struct CollisionBody {
    EntityId entity_id;
    std::unique_ptr<CollisionShape> shape;

    // Transform (from EntityState)
    Vec3 position;
    Quat orientation;

    // Collision filtering
    CollisionLayer layer;
    CollisionMask collision_mask;

    // Flags
    bool is_static;    // Never moves
    bool is_trigger;   // No collision response, only detection
    bool is_active;    // Participates in collision

    // Cached AABB
    AABB cached_aabb;
    bool aabb_dirty;

    // User data
    void* user_data;

    CollisionBody()
        : entity_id(INVALID_ENTITY_ID)
        , shape(nullptr)
        , position(Vec3::Zero())
        , orientation(Quat::Identity())
        , layer(CollisionLayer::Default)
        , is_static(false)
        , is_trigger(false)
        , is_active(true)
        , aabb_dirty(true)
        , user_data(nullptr) {
        collision_mask.set();  // Collide with all by default
    }

    /**
     * @brief Update cached AABB
     */
    void update_aabb() {
        if (shape && aabb_dirty) {
            cached_aabb = shape->compute_aabb(position, orientation);
            aabb_dirty = false;
        }
    }

    /**
     * @brief Check if can collide with another body
     */
    bool can_collide_with(const CollisionBody& other) const {
        if (!is_active || !other.is_active) return false;

        // Check collision masks
        uint32_t my_layer = static_cast<uint32_t>(layer);
        uint32_t other_layer = static_cast<uint32_t>(other.layer);

        return collision_mask.test(other_layer) && other.collision_mask.test(my_layer);
    }
};

// ============================================================================
// GJK + EPA Algorithm
// ============================================================================

/**
 * @brief GJK/EPA collision detection
 */
class GJK {
public:
    static constexpr int MAX_ITERATIONS = 64;
    static constexpr Real EPSILON = 1e-6;

    /**
     * @brief Test collision between two shapes
     * @return true if shapes are intersecting
     */
    static bool test_collision(
        const CollisionShape& shape_a, const Vec3& pos_a, const Quat& rot_a,
        const CollisionShape& shape_b, const Vec3& pos_b, const Quat& rot_b);

    /**
     * @brief Get penetration depth and normal using EPA
     * @return true if collision found with contact info
     */
    static bool get_contact(
        const CollisionShape& shape_a, const Vec3& pos_a, const Quat& rot_a,
        const CollisionShape& shape_b, const Vec3& pos_b, const Quat& rot_b,
        ContactPoint& contact);

private:
    struct Simplex {
        Vec3 points[4];
        int count;

        Simplex() : count(0) {}

        void push(const Vec3& point) {
            for (int i = count; i > 0; --i) {
                points[i] = points[i - 1];
            }
            points[0] = point;
            count = std::min(count + 1, 4);
        }

        Vec3& operator[](int i) { return points[i]; }
        const Vec3& operator[](int i) const { return points[i]; }
    };

    static Vec3 minkowski_support(
        const CollisionShape& shape_a, const Vec3& pos_a, const Quat& rot_a,
        const CollisionShape& shape_b, const Vec3& pos_b, const Quat& rot_b,
        const Vec3& direction);

    static bool do_simplex(Simplex& simplex, Vec3& direction);
    static bool line_case(Simplex& simplex, Vec3& direction);
    static bool triangle_case(Simplex& simplex, Vec3& direction);
    static bool tetrahedron_case(Simplex& simplex, Vec3& direction);
};

// ============================================================================
// Dynamic AABB Tree (Broad Phase)
// ============================================================================

/**
 * @brief Dynamic AABB tree for broad-phase collision detection
 *
 * Uses a balanced binary tree structure with surface area heuristic (SAH)
 * for efficient incremental updates and queries.
 */
class AABBTree {
public:
    static constexpr int NULL_NODE = -1;
    static constexpr Real AABB_MARGIN = 0.1;  // Margin for fat AABBs

    AABBTree();
    ~AABBTree() = default;

    /**
     * @brief Insert a body into the tree
     * @return Node index
     */
    int insert(EntityId entity_id, const AABB& aabb);

    /**
     * @brief Remove a body from the tree
     */
    void remove(int node_index);

    /**
     * @brief Update a body's AABB
     * @return true if reinsert was needed
     */
    bool update(int node_index, const AABB& new_aabb);

    /**
     * @brief Query all potentially overlapping pairs
     */
    void query_pairs(std::vector<std::pair<EntityId, EntityId>>& pairs) const;

    /**
     * @brief Query all bodies overlapping with AABB
     */
    void query(const AABB& aabb, std::vector<EntityId>& results) const;

    /**
     * @brief Ray cast against tree
     */
    void raycast(const Vec3& origin, const Vec3& direction,
                 Real max_distance, std::vector<EntityId>& results) const;

    /**
     * @brief Get number of bodies in tree
     */
    size_t size() const { return entity_to_node_.size(); }

    /**
     * @brief Clear all bodies
     */
    void clear();

private:
    struct Node {
        AABB aabb;         // Fat AABB
        EntityId entity_id;
        int parent;
        int left;
        int right;
        int height;        // For balancing

        bool is_leaf() const { return left == NULL_NODE; }
    };

    std::vector<Node> nodes_;
    int root_;
    int free_list_;
    std::unordered_map<EntityId, int> entity_to_node_;

    int allocate_node();
    void free_node(int node_index);
    void insert_leaf(int leaf);
    void remove_leaf(int leaf);
    int balance(int node_index);
    void query_recursive(int node_index, const AABB& aabb,
                         std::vector<EntityId>& results) const;
};

// ============================================================================
// Collision World
// ============================================================================

/**
 * @brief Callback types
 */
using CollisionCallback = std::function<void(const ContactManifold&)>;
using TriggerCallback = std::function<void(EntityId, EntityId, bool)>;  // bool = enter/exit

/**
 * @brief Main collision detection world
 */
class CollisionWorld {
public:
    CollisionWorld();
    ~CollisionWorld();

    // ========================================================================
    // Body Management
    // ========================================================================

    /**
     * @brief Add collision body for entity
     */
    void add_body(EntityId entity_id, std::unique_ptr<CollisionShape> shape,
                  CollisionLayer layer = CollisionLayer::Default);

    /**
     * @brief Remove collision body
     */
    void remove_body(EntityId entity_id);

    /**
     * @brief Update body transform
     */
    void update_body(EntityId entity_id, const Vec3& position,
                     const Quat& orientation);

    /**
     * @brief Set body as static (never moves)
     */
    void set_static(EntityId entity_id, bool is_static);

    /**
     * @brief Set body as trigger (no collision response)
     */
    void set_trigger(EntityId entity_id, bool is_trigger);

    /**
     * @brief Set collision mask
     */
    void set_collision_mask(EntityId entity_id, const CollisionMask& mask);

    /**
     * @brief Get collision body
     */
    const CollisionBody* get_body(EntityId entity_id) const;

    // ========================================================================
    // Collision Detection
    // ========================================================================

    /**
     * @brief Perform collision detection
     *
     * Runs broad phase, then narrow phase, and invokes callbacks.
     */
    void detect_collisions();

    /**
     * @brief Get all contact manifolds from last detection
     */
    const std::vector<ContactManifold>& get_contacts() const { return contacts_; }

    // ========================================================================
    // Queries
    // ========================================================================

    /**
     * @brief Test if point is inside any body
     */
    std::vector<EntityId> query_point(const Vec3& point) const;

    /**
     * @brief Query all bodies overlapping with AABB
     */
    std::vector<EntityId> query_aabb(const AABB& aabb) const;

    /**
     * @brief Ray cast and get all hits
     */
    struct RayHit {
        EntityId entity_id;
        Vec3 point;
        Vec3 normal;
        Real distance;
    };

    std::vector<RayHit> raycast(const Vec3& origin, const Vec3& direction,
                                 Real max_distance = std::numeric_limits<Real>::max()) const;

    /**
     * @brief Ray cast and get closest hit
     */
    bool raycast_closest(const Vec3& origin, const Vec3& direction,
                         RayHit& hit, Real max_distance = std::numeric_limits<Real>::max()) const;

    // ========================================================================
    // Callbacks
    // ========================================================================

    void set_collision_callback(CollisionCallback callback) {
        collision_callback_ = std::move(callback);
    }

    void set_trigger_enter_callback(TriggerCallback callback) {
        trigger_enter_callback_ = std::move(callback);
    }

    void set_trigger_exit_callback(TriggerCallback callback) {
        trigger_exit_callback_ = std::move(callback);
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    struct Statistics {
        size_t body_count;
        size_t broad_phase_pairs;
        size_t narrow_phase_tests;
        size_t contacts_generated;
        double broad_phase_time_ms;
        double narrow_phase_time_ms;

        Statistics() { reset(); }
        void reset() {
            body_count = 0;
            broad_phase_pairs = 0;
            narrow_phase_tests = 0;
            contacts_generated = 0;
            broad_phase_time_ms = 0;
            narrow_phase_time_ms = 0;
        }
    };

    const Statistics& get_statistics() const { return stats_; }

    // ========================================================================
    // Event System Integration
    // ========================================================================

    /**
     * @brief Set event dispatcher for collision events
     *
     * When set, the CollisionWorld will emit events on:
     * - CollisionEnter: When two bodies start colliding
     * - CollisionExit: When two bodies stop colliding
     * - TriggerEnter/TriggerExit: For trigger volumes
     *
     * @param dispatcher Pointer to event dispatcher (can be null to disable)
     */
    void set_event_dispatcher(events::EventDispatcher* dispatcher);

    /**
     * @brief Get current event dispatcher
     */
    events::EventDispatcher* get_event_dispatcher() const { return event_dispatcher_; }

    /**
     * @brief Set current simulation time for event timestamps
     */
    void set_current_time(Real time) { current_time_ = time; }

private:
    std::unordered_map<EntityId, CollisionBody> bodies_;
    std::unordered_map<EntityId, int> body_to_tree_node_;
    std::unique_ptr<AABBTree> broad_phase_;

    // Event system integration
    events::EventDispatcher* event_dispatcher_{nullptr};
    Real current_time_{0.0};

    std::vector<ContactManifold> contacts_;
    std::unordered_set<uint64_t> active_triggers_;  // Packed entity pair IDs
    std::unordered_set<uint64_t> active_collisions_;  // Packed entity pair IDs for collision enter/exit

    CollisionCallback collision_callback_;
    TriggerCallback trigger_enter_callback_;
    TriggerCallback trigger_exit_callback_;

    Statistics stats_;

    void run_broad_phase(std::vector<std::pair<EntityId, EntityId>>& pairs);
    void run_narrow_phase(const std::vector<std::pair<EntityId, EntityId>>& pairs);
    void process_triggers(const std::vector<ContactManifold>& new_contacts);

    static uint64_t pack_entity_pair(EntityId a, EntityId b) {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    }
};

// ============================================================================
// Default Collision Masks
// ============================================================================

namespace collision_masks {

/**
 * @brief Get default collision mask for a layer
 */
inline CollisionMask default_mask_for_layer(CollisionLayer layer) {
    CollisionMask mask;
    mask.set();  // Collide with all by default

    switch (layer) {
        case CollisionLayer::Aircraft:
            // Aircraft collide with terrain, missiles, projectiles
            mask.reset();
            mask.set(static_cast<size_t>(CollisionLayer::Terrain));
            mask.set(static_cast<size_t>(CollisionLayer::Missile));
            mask.set(static_cast<size_t>(CollisionLayer::Projectile));
            mask.set(static_cast<size_t>(CollisionLayer::Aircraft));
            break;

        case CollisionLayer::GroundVehicle:
            mask.reset();
            mask.set(static_cast<size_t>(CollisionLayer::Terrain));
            mask.set(static_cast<size_t>(CollisionLayer::GroundVehicle));
            mask.set(static_cast<size_t>(CollisionLayer::Projectile));
            mask.set(static_cast<size_t>(CollisionLayer::Structure));
            break;

        case CollisionLayer::Ship:
            mask.reset();
            mask.set(static_cast<size_t>(CollisionLayer::Ocean));
            mask.set(static_cast<size_t>(CollisionLayer::Ship));
            mask.set(static_cast<size_t>(CollisionLayer::Projectile));
            break;

        case CollisionLayer::Missile:
            mask.set();  // Missiles collide with everything
            break;

        case CollisionLayer::Sensor:
            mask.reset();  // Sensors don't collide, only trigger
            break;

        default:
            break;
    }

    return mask;
}

} // namespace collision_masks

} // namespace jaguar::physics
