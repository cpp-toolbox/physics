#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "jolt_implementation.hpp"
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/StateRecorderImpl.h"
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

    void load_model_into_physics_world(const std::vector<draw_info::IndexedVertexPositions> &ivps,
                                       const JPH::ObjectLayer &layer = Layers::NON_MOVING);

    JPH::Ref<JPH::CharacterVirtual> create_character(uint64_t client_id,
                                                     JPH::Vec3 initial_position = JPH::Vec3(0, 0, 0));
    void delete_character(uint64_t client_id);

    /**
     * @brief checks if the given ray hits the character
     *
     * @param source_offset is a vector used to change the start position of the fired ray, this is useful if the place
     * where the ray should start is given by a constant offset from the characters origin.
     *
     * @warn \p ray is not a direction vector, its length matters, if you're aiming at something it's possible to not
     * hit it because the ray wasn't long enough.
     */
    bool check_if_ray_hits_target(JPH::Vec3 ray, JPH::Ref<JPH::CharacterVirtual> source,
                                  JPH::Ref<JPH::CharacterVirtual> target, JPH::Vec3 source_offset = JPH::Vec3(0, 0, 0));

    std::optional<unsigned int>
    check_if_ray_hits_any_target(JPH::Vec3 ray, JPH::Ref<JPH::CharacterVirtual> source,
                                 std::unordered_map<unsigned int, JPH::Ref<JPH::CharacterVirtual>> id_to_target,
                                 JPH::Vec3 source_offset = JPH::Vec3(0, 0, 0));

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
    //
    // note that half of this is 0.875
    const float character_height_standing = 1.75f;
    const float character_height_crouching = 1.f;

    // NOTE: this could be slightly smaller like 0.35 and it would be fine
    const float character_diameter = 0.425f;

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
