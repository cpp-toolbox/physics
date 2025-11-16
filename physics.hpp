#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "jolt_implementation.hpp"
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/StateRecorderImpl.h"
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include "sbpt_generated_includes.hpp"
#include <chrono>
#include <iterator>

struct PhysicsFrame {
    JPH::StateRecorderImpl &physics_state;
};

class Physics {
  public:
    Physics();
    ~Physics();

    JPH::PhysicsSystem physics_system;

    void update_characters_only(float delta_time);
    void update_specific_character_by_id(float delta_time, uint64_t id);
    void update_specific_character(float delta_time, JPH::Ref<JPH::CharacterVirtual> character,
                                   const JPH::BodyFilter &body_filter = JPH::BodyFilter());
    void update(float delta_time);

    JPH::BodyID sphere_id; // should be removed in a real program
    std::unordered_map<uint64_t, JPH::Ref<JPH::CharacterVirtual>> client_id_to_physics_character;
    void refresh_contacts(JPH::Ref<JPH::CharacterVirtual>);

    void add_shape_via_convex_hull(const std::vector<glm::vec3> &vertices);

    JPH::TriangleList ivp_to_triangle_list(const draw_info::IndexedVertexPositions &ivp);
    void load_model_into_physics_world(const std::vector<draw_info::IndexedVertexPositions> &ivps,
                                       const JPH::ObjectLayer &layer = Layers::NON_MOVING);

    JPH::Ref<JPH::CharacterVirtual> create_character(uint64_t client_id,
                                                     JPH::Vec3 initial_position = JPH::Vec3(0, 0, 0));
    void delete_character(uint64_t client_id);

    // If nothing was hit, all optionals remain std::nullopt.
    struct HitscanResult {
        // fraction along the ray (0–1) where a hit occurred.
        // if empty → no hit.
        std::optional<float> hit_fraction;

        // If a character was hit, this contains the client ID.
        std::optional<unsigned int> hit_character_id;

        // if a world object was hit, this contains the body id.
        std::optional<JPH::BodyID> hit_world_body_id;

        // underlying raycastresult (present only if something was hit)
        std::optional<JPH::RayCastResult> rcr;

        bool hit_something() const { return hit_fraction.has_value(); }
        bool hit_character() const { return hit_character_id.has_value(); }
        bool hit_world_object() const { return hit_world_body_id.has_value(); }
    };

    struct CharacterHitscanContext {
        std::reference_wrapper<JPH::Ref<JPH::CharacterVirtual>> physics_character;
    };

    /**
     * @brief Fires a hitscan weapon along a given ray and determines the closest hit.
     *
     * This function performs a hitscan operation by casting a ray (`aim_ray`) against
     * all potential targets: other characters (excluding the source) and world objects.
     * It returns the closest hit, if any, including information about whether a character
     * or a world object was hit.
     *
     * @param aim_ray The ray representing the weapon's trajectory.
     * @param client_id_to_character_hitscan_context Map from client IDs to their respective
     *        `CharacterHitscanContext` which contains character physics data.
     * @param client_id_of_source The client ID of the entity firing the weapon; this
     *        entity will be ignored in the character hitscan check.
     * @param hittable_world_objects Vector of world object `BodyID`s that the ray can hit.
     *
     * @return HitscanResult Structure containing information about the closest hit:
     *         - `hit_fraction`: Fraction along the ray where the hit occurred.
     *         - `hit_character_id`: Optional ID of the hit character, if any.
     *         - `hit_world_body_id`: Optional ID of the hit world object, if any.
     *         - `rcr`: Raw `RayCastResult` with detailed hit information.
     *
     * @note If multiple hits occur along the ray, only the closest hit (smallest fraction)
     *       is considered. Hits on the firing character are ignored.
     *
     * @warning This function assumes that `physics_system` is valid and accessible
     *          for world object raycasts.
     */
    HitscanResult fire_hitscan_weapon(
        const JPH::RayCast &aim_ray,
        std::unordered_map<unsigned int, CharacterHitscanContext> &client_id_to_character_hitscan_context,
        unsigned int client_id_of_source, const std::vector<JPH::BodyID> &hittable_world_objects);

    using IdToPhysicsState = std::unordered_map<unsigned int, JPH::StateRecorderImpl>;

    IdToPhysicsState get_current_physics_state_for_characters(
        std::unordered_map<unsigned int, JPH::Ref<JPH::CharacterVirtual>> id_to_character);

    /**
     * @brief restores the physics state for a mapping of id to character
     * @pre assumes that the id set for id_to_physics_state is the same for the one for id_to_character
     */
    void restore_physics_state_for_characters(
        IdToPhysicsState &id_to_physics_state,
        std::unordered_map<unsigned int, JPH::Ref<JPH::CharacterVirtual>> &id_to_character);

    void set_gravity(float acceleration);

    std::vector<JPH::BodyID> created_body_ids;

    // startfold character creation data

    static constexpr float inner_shape_fraction = 0.9f;
    // all of these values where computed from human averages or based on images
    // I found of humans whose dimensions looked average

    // note that half of this is 0.875
    const float character_height_standing = 1.75f;
    const float character_height_crouching = 1.f;

    const float character_diameter = 0.65;
    const float character_radius = character_diameter / 2;

    const float distance_from_eyes_to_top_of_head = 0.1;
    const float eyes_height_from_center = character_height_standing / 2.0f - distance_from_eyes_to_top_of_head;
    const JPH::Vec3 eyes_offset_from_center = JPH::Vec3(0, eyes_height_from_center, 0);

    // endfold

  private:
    void initialize_engine();
    void initialize_world_objects();
    void clean_up_world();

    const unsigned int cMaxBodies = 1024;
    const unsigned int cNumBodyMutexes = 0;
    const unsigned int cMaxBodyPairs = 1024;
    const unsigned int cMaxContactConstraints = 1024;
    const int cCollisionSteps = 1;

    JPH::TempAllocatorImpl *temp_allocator;
    JPH::JobSystemThreadPool *job_system;
    MyBodyActivationListener *body_activation_listener;
    MyContactListener *contact_listener;

    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_vs_object_layer_filter;
};

#endif // PHYSICS_HPP
