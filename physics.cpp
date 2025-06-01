#include "physics.hpp"

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including
// any other Jolt header. You can use Jolt.h in your precompiled header to speed
// up compilation.
/*#include "../../math/conversions.hpp"*/
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/Collision/Shape/ConvexHullShape.h"
#include "Jolt/Physics/Collision/Shape/MeshShape.h"
#include "Jolt/Physics/StateRecorder.h"
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <stdexcept>

//// Disable common warnings triggered by Jolt, you can use
/// JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to
/// store and restore the warning state
// JPH_SUPPRESS_WARNINGS

Physics::Physics() {
    this->initialize_engine();
    this->initialize_world_objects();
}

Physics::~Physics() { this->clean_up_world(); }

void Physics::initialize_engine() {
    JPH::RegisterDefaultAllocator();

    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    // dynamic allocation, we don't worry about the rule of three since we never
    // copy the physics system, there is only ever one instance
    temp_allocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
    job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
                                              JPH::thread::hardware_concurrency() - 1);

    physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface,
                        object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

    body_activation_listener = new MyBodyActivationListener();
    contact_listener = new MyContactListener();

    physics_system.SetBodyActivationListener(body_activation_listener);
    physics_system.SetContactListener(contact_listener);
}

void Physics::initialize_world_objects() {

    physics_system.SetGravity(JPH::Vec3(0, -40, 0));

    /*JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();*/

    /*JPH::Array<JPH::Vec3> vertices = fibonacci_sphere(60);*/
    /*JPH::ConvexHullShapeSettings low_poly_ball_settings(vertices, JPH::cDefaultConvexRadius);*/
    /*JPH::ShapeSettings::ShapeResult ball_shape_result = low_poly_ball_settings.Create();*/
    /**/
    /*if (!ball_shape_result.IsValid()) {*/
    /*    throw std::runtime_error("ball shape is invalid");*/
    /*}*/
    /**/
    /*JPH::ShapeRefC ball_shape = ball_shape_result.Get(); // We don't expect an error here, but you can check*/
    /*                                                     // floor_shape_result for HasError() / GetError()*/
    /**/
    /*JPH::BodyCreationSettings ball_creation_settings(ball_shape, JPH::RVec3(5.0, 20.0, 5.0), JPH::Quat::sIdentity(),*/
    /*                                                 JPH::EMotionType::Dynamic, Layers::MOVING);*/
    /*JPH::Body *ball = body_interface.CreateBody(ball_creation_settings); // Note that if we run out of bodies this
     * can*/
    /*                                                                     // return nullptr*/
    /*body_interface.AddBody(ball->GetID(), JPH::EActivation::Activate);*/
    /*created_body_ids.push_back(ball->GetID());*/
    /*body_interface.SetLinearVelocity(ball->GetID(), JPH::Vec3(0.0f, -5.0f, 0.0f));*/
}

/**
 * \brief For every mesh in this model, we create a physics object that
 * represents the mesh
 */
void Physics::load_model_into_physics_world(const std::vector<draw_info::IndexedVertexPositions> &ivps) {

    JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();

    for (int i = 0; i < ivps.size(); i++) {

        draw_info::IndexedVertexPositions ivp = ivps[i];

        JPH::TriangleList triangles;

        /*assert(mesh.indices.size() % 3 == 0); // only contains triangles*/
        for (int j = 0; j < ivp.indices.size(); j += 3) {
            unsigned int j1 = ivp.indices[j];
            unsigned int j2 = ivp.indices[j + 1];
            unsigned int j3 = ivp.indices[j + 2];

            glm::vec3 temp_v1 = ivp.xyz_positions[j1];
            JPH::Float3 v1 = JPH::Float3(temp_v1.x, temp_v1.y, temp_v1.z);

            glm::vec3 temp_v2 = ivp.xyz_positions[j2];
            JPH::Float3 v2 = JPH::Float3(temp_v2.x, temp_v2.y, temp_v2.z);

            glm::vec3 temp_v3 = ivp.xyz_positions[j3];
            JPH::Float3 v3 = JPH::Float3(temp_v3.x, temp_v3.y, temp_v3.z);

            JPH::Triangle tri = JPH::Triangle(v1, v2, v3);

            triangles.push_back(tri);
        }

        JPH::MeshShapeSettings settings = JPH::MeshShapeSettings(triangles);

        JPH::Ref<JPH::Shape> mesh_shape;

        // Create shape
        JPH::Shape::ShapeResult result = settings.Create();
        if (result.IsValid()) {
            mesh_shape = result.Get();
        } else {
            throw std::runtime_error("couldn't get resulting shape");
        }

        JPH::BodyCreationSettings mesh_settings(mesh_shape, JPH::RVec3(0.0, 0.0, 0.0), JPH::Quat::sIdentity(),
                                                JPH::EMotionType::Static, Layers::NON_MOVING);
        JPH::Body *mesh_body = body_interface.CreateBody(mesh_settings); // Note that if we run out of bodies this can
                                                                         // return nullptr
        body_interface.AddBody(mesh_body->GetID(), JPH::EActivation::DontActivate);
        created_body_ids.push_back(mesh_body->GetID());
    }
}

/**
 * \brief create character controller for a user
 */
JPH::Ref<JPH::CharacterVirtual> Physics::create_character(uint64_t client_id, JPH::Vec3 initial_position) {

    JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();

    const float character_half_height = 0.5f * this->character_height_standing;

    settings->mShape = new JPH::CylinderShape(character_half_height, this->character_radius);
    settings->mInnerBodyShape = new JPH::CylinderShape(character_half_height, this->character_radius - 0.01);
    // settings->mInnerBodyShape = new JPH::CapsuleShape(character_half_height, this->character_radius);

    // n = (0, 1, 0) so if we use that as the normal, then (x, y, z) * n = y, thus the equation is of the form
    // X . n + c = 0, so if we make c = - radius, that would only accept contacts which are on the bottom semisphere
    // because X.n - radius = 0 <=> y - radius = 0 so if y = radius then its on the sphere, also this only works if you
    // translated your shape up so that the origin is on the bottom of the character, and we're not going to do that.
    //
    // since we're doing a cylinder we do it this way, we only allow contacts if they are on the very bottom of the
    // cylinder.
    settings->mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(),
                                             character_half_height - 0.01); // Accept contacts that touch the
                                                                            // lower part of the cylinder

    JPH::Ref<JPH::CharacterVirtual> character =
        new JPH::CharacterVirtual(settings, initial_position, JPH::Quat::sIdentity(), &physics_system);

    client_id_to_physics_character[client_id] = character;

    return character;
}

void Physics::delete_character(uint64_t client_id) { client_id_to_physics_character.erase(client_id); }

/**
 * \brief updates the objects part of this physics simulation
 */
void Physics::update(float delta_time) {
    // this->update_characters_only(delta_time); // accounts for predicted inputs
    physics_system.Update(delta_time, cCollisionSteps, temp_allocator, job_system);
    // JPH::StateRecorderImpl physics_state_after_update;
    // this->physics_state_history.push
    // this->physics_system.SaveState(physics_state_after_update);
    // PhysicsFrame physics_frame_after_update = {physics_state_after_update};
    // this->physics_frames.put(physics_frame_after_update);
}

void Physics::refresh_contacts(JPH::Ref<JPH::CharacterVirtual> character) {
    // character->RefreshContacts(this->broad_phase_layer_interface, this->object_vs_object_layer_filter, const
    // BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter, TempAllocator &inAllocator)
    //
    character->RefreshContacts(physics_system.GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                               physics_system.GetDefaultLayerFilter(Layers::MOVING), {}, {}, *temp_allocator);
}

void Physics::update_characters_only(float delta_time) {
    JPH::CharacterVirtual::ExtendedUpdateSettings update_settings;
    for (const auto &pair : client_id_to_physics_character) {
        update_specific_character_by_id(delta_time, pair.first);
    }
}

void Physics::update_specific_character_by_id(float delta_time, uint64_t id) {
    JPH::Ref<JPH::CharacterVirtual> character = client_id_to_physics_character[id];
    update_specific_character(delta_time, character);
}

void Physics::update_specific_character(float delta_time, JPH::Ref<JPH::CharacterVirtual> character,
                                        const JPH::BodyFilter &body_filter) {

    JPH::CharacterVirtual::ExtendedUpdateSettings update_settings;
    // update_settings.mStickToFloorStepDown = character->GetUp() *
    // update_settings.mStickToFloorStepDown.Length();
    // update_settings.mWalkStairsStepUp = character->GetUp() *
    // update_settings.mWalkStairsStepUp.Length();
    //
    // TODO: think if gravity needs to be applied because we're already applyign it?
    character->ExtendedUpdate(delta_time, -character->GetUp() * physics_system.GetGravity().Length(), update_settings,
                              physics_system.GetDefaultBroadPhaseLayerFilter(Layers::MOVING),
                              physics_system.GetDefaultLayerFilter(Layers::MOVING), body_filter, {}, *temp_allocator);
}

void Physics::clean_up_world() {
    JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();

    for (auto body_id : created_body_ids) {
        body_interface.RemoveBody(body_id);
        body_interface.DestroyBody(body_id);
    }

    JPH::UnregisterTypes();

    // de-allocate dynamic memory
    delete temp_allocator;
    delete job_system;
    delete body_activation_listener;
    delete contact_listener;

    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    std::cout << "successfully cleaned up world" << std::endl;
}
