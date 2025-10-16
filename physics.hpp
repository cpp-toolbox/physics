#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "jolt_implementation.hpp"
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/StateRecorderImpl.h"
#include "sbpt_generated_includes.hpp"
#include <chrono>

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
     * @note that ray is not a direction vector, its length matters, if you're aiming at something it's possible to not
     * hit it because the ray wasn't long enough.
     */
    bool check_if_ray_hits_character(JPH::Vec3 ray, JPH::Ref<JPH::CharacterVirtual> character);

    std::optional<unsigned int>
    check_if_ray_hits_any_character(JPH::Vec3 ray,
                                    std::unordered_map<unsigned int, JPH::Ref<JPH::CharacterVirtual>> id_to_character);

    void set_gravity(float acceleration);

    std::vector<JPH::BodyID> created_body_ids;

    // character creation data [[
    static constexpr float inner_shape_fraction = 0.9f;
    // based on actual average human height in meters
    // note that half of this is 0.875
    const float character_height_standing = 1.75f;

    const float character_height_crouching = 1.f;

    // Male: ~0.45 m,  Female: ~0.40 m , average diameter ≈ 0.425 m
    const float character_radius = 0.425f;

    // eye position derivation:
    // we approximate the human body with a vertical cylinder:
    // - total height of the cylinder: 1.75 meters
    // - origin is at the center of the cylinder
    //   → half-height = 1.75 / 2 = 0.875 meters
    //
    // average eye level in adults is approximately 1.10 meters from the bottom of the body.
    // to compute the eye position relative to the center-origin:
    //
    //     eye_y = -half_height + eye_height_from_base
    //           = -0.875 + 1.10
    //           = +0.225 meters

    const float eyes_height_from_center = 0.5;

    // character creation data ]]

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
