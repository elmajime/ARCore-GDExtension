#include "plane_renderer.h"

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <functional>
#include <vector>

#include "utils.h"

using namespace arcore_plugin;
using namespace godot;

PlaneRenderer::PlaneRenderer()
{

}

PlaneRenderer::~PlaneRenderer()
{

}

void PlaneRenderer::initialize()
{

}

size_t hash_vector(const std::vector<float>& vec) {
    size_t hash = 0;
    for (const auto& element : vec) {
        // Combine the hash value of each element using std::hash
        hash ^= std::hash<float>{}(element) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
}

// Function to get the boundary polygon of an ArPlane
std::vector<float> get_plane_boundary(ArSession* ar_session, ArPlane* ar_plane) {
    int32_t polygon_size;
    ArPlane_getPolygonSize(ar_session, ar_plane, &polygon_size);
    std::vector<float> polygon(polygon_size);
    ArPlane_getPolygon(ar_session, ar_plane, polygon.data());
    return polygon;
}

ArPose* get_plane_center_pose(ArSession* ar_session, ArPlane* ar_plane) {
    ArPose* center_pose;
    ArPose_create(ar_session, nullptr, &center_pose);
    ArPlane_getCenterPose(ar_session, ar_plane, center_pose);
    return center_pose;
}

// Function to convert the boundary points to Godot vertices
Array convert_boundary_to_vertices(const std::vector<float>& boundary) {
    Array vertices;
    for (size_t i = 0; i < boundary.size(); i += 2) {
        Vector2 vertex(boundary[i], boundary[i + 1]);
        vertices.append(vertex);
    }
    return vertices;
}

// // Function to create a polygon mesh from vertices
// Ref<ArrayMesh> create_polygon_mesh(const Array& vertices) {
//     Ref<ArrayMesh> mesh = memnew(ArrayMesh);
//     Array arrays;
//     arrays.resize(Mesh::ARRAY_MAX);
//     arrays[Mesh::ARRAY_VERTEX] = vertices;
//     mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
//     return mesh;
// }

// // Function to create a MeshInstance3D and set the mesh
// MeshInstance3D* create_mesh_instance(const Ref<Mesh>& mesh) {
//     MeshInstance3D* mesh_instance = memnew(MeshInstance3D);
//     mesh_instance->set_mesh(mesh);
//     return mesh_instance;
// }

Transform3D ar_pose_to_godot_transform(ArSession* ar_session, ArPose* ar_pose) {
    float pose_raw[7];  // The ARCore pose is represented by a 7-element array (quaternion and translation)
    ArPose_getPoseRaw(ar_session, ar_pose, pose_raw);

    // ARCore provides a quaternion (x, y, z, w) and translation (x, y, z)
    Quaternion rotation(pose_raw[0], pose_raw[1], pose_raw[2], pose_raw[3]);
    Quaternion tweaking_rotation(Vector3(1.f, 0.f, 0.f), 3.14f/2);
    Vector3 translation(pose_raw[4], pose_raw[5], pose_raw[6]);

    Transform3D transform;
    transform.origin = translation;
    // transform.basis = Basis(tweaking_rotation * rotation);
    transform.basis = Basis(tweaking_rotation);

    return transform;
}

CSGPolygon3D* create_csg_polygon(const Array& vertices)
{
    CSGPolygon3D* csg_polygon = memnew(CSGPolygon3D);
    csg_polygon->set_polygon(vertices);
    csg_polygon->set_depth(0.01f);  // Set the depth of the polygon extrusion

    return csg_polygon;
}

// Function to remove the CSGPolygon3D node of a lost plane
void PlaneRenderer::remove_plane_node(const size_t& plane_guid) {
    if (m_plane_nodes.find(plane_guid) != m_plane_nodes.end()) {
        // Plane node found, remove it from the scene
        CSGPolygon3D* polygon_node = m_plane_nodes[plane_guid];
        polygon_node->queue_free();
        m_plane_nodes.erase(plane_guid);
    }
}

// Function to update the CSGPolygon3D node of a tracked plane
void PlaneRenderer::update_or_create_plane_node(const size_t& plane_guid, ArSession* ar_session, ArPlane* ar_plane, const std::vector<float> &boundary) {
    Array vertices = convert_boundary_to_vertices(boundary);
    if (m_plane_nodes.find(plane_guid) != m_plane_nodes.end()) {
        // Plane node found, update its position and shape
        CSGPolygon3D* polygon_node = m_plane_nodes[plane_guid];

        polygon_node->set_polygon(vertices);

        ArPose* center_pose = get_plane_center_pose(ar_session, ar_plane);
        Transform3D transform = ar_pose_to_godot_transform(ar_session, center_pose);
        polygon_node->set_transform(transform);
        ALOGE("MCT Godot ARCore: Updating plane: %ld", plane_guid);
    } 
    else
    {
        CSGPolygon3D* polygon_node = create_csg_polygon(vertices);

        // Get the center pose of the plane
        ArPose* center_pose = get_plane_center_pose(ar_session, ar_plane);
        Transform3D transform = ar_pose_to_godot_transform(ar_session, center_pose);

        // Apply the transform to the CSGPolygon3D node
        polygon_node->set_transform(transform);

        m_planes_node->add_child(polygon_node);
        m_plane_nodes[plane_guid] = polygon_node;
        ALOGE("MCT Godot ARCore: Creating plane: %ld", plane_guid);
        ALOGE("MCT Godot ARCore: parent_node children : %d", m_planes_node->get_child_count());
    }
}

// #define CREATE_ARRAY_MESH

// // Main function to generate the mesh from ArPlane and add it to the scene
// void generate_and_add_mesh(ArSession* ar_session, ArPlane* ar_plane, Node* parent_node) {
//     std::vector<float> boundary = get_plane_boundary(ar_session, ar_plane);
//     Array vertices = convert_boundary_to_vertices(boundary);

//     Node3D* child;
//     // #ifdef CREATE_ARRAY_MESH
//     //     Ref<ArrayMesh> mesh = create_polygon_mesh(vertices);
//     //     MeshInstance3D* mesh_instance = create_mesh_instance(mesh);
//     //     mesh_instance->set_name("ARPlane");
//     //     child = mesh_instance;
//     // #else
//         child = create_csg_polygon(vertices);
//     // #endif

//     // Get the center pose of the plane
//     ArPose* center_pose = get_plane_center_pose(ar_session, ar_plane);
//     Transform3D transform = ar_pose_to_godot_transform(ar_session, center_pose);

//     // Apply the transform to the CSGPolygon3D node
//     child->set_transform(transform);

//     parent_node->add_child(child);
//     ALOGE("MCT Godot ARCore: parent_node children : %d", parent_node->get_child_count());
//     ALOGE("MCT Godot ARCore: last plane nb vertices : %ld", vertices.size());

//     Vector3 min = vertices.min();
//     Vector3 max = vertices.max();
//     ALOGE("MCT Godot ARCore: last plane min and max : (%f,%f,%f) (%f,%f,%f)", min.x, min.y, min.z, max.x, max.y, max.z);
// }

Node* get_root_node() {
    // Get the SceneTree singleton
    SceneTree *scene_tree = (SceneTree *)Engine::get_singleton()->get_main_loop();

    // Get the current scene
    Node *current_scene = scene_tree->get_current_scene();

    return current_scene;
}

// void PlaneRenderer::clear() 
// {
//     if (m_planes_node == nullptr) {
//         return;
//     }

//     while(m_planes_node->get_child_count() != 0)
//     {
//         Node* node = m_planes_node->get_child(0);
//         if (node)
//         {
//             m_planes_node->remove_child(node);
//             node->queue_free();
//         }
//     }
// }

// // Function to handle the state of a detected plane
// void handle_plane_state(const std::string& plane_guid, ArSession* ar_session, ArPlane* ar_plane) {
//     ArTrackingState plane_tracking_state;
//     ArPlane_getTrackingState(ar_session, ar_plane, &plane_tracking_state);
//     if (plane_tracking_state == AR_TRACKING_STATE_TRACKING) {
//         // Plane is still tracked, update its representation
//         update_or_create_plane_node(plane_guid, ar_session, ar_plane);
//     } else {
//         // Plane is lost, remove its representation
//         remove_plane_node(plane_guid);
//     }
// }

size_t get_uid(ArSession* ar_session, const std::vector<float> &boundary)
{
    size_t boundary_hash = hash_vector(boundary);
    return boundary_hash;
}

// void on_plane_detected(ArSession* ar_session, ArPlane* ar_plane) {
//     // Convert the GUID to a std::string for convenience
//     std::string plane_guid = get_uid(ar_session, ar_plane);

//     // Check if the plane has been detected before
//     handle_plane_detection(plane_guid);
// }

void PlaneRenderer::draw(ArSession& p_ar_session)
{
    if (m_planes_node == nullptr) {
        m_planes_node = new Node();
        m_planes_node->set_name("ARPlanes_node");
        get_root_node()->add_child(m_planes_node);
    }

    // Update and render planes.
    ArTrackableList* plane_list = nullptr;
    ArTrackableList_create(&p_ar_session, &plane_list);
    ERR_FAIL_NULL(plane_list);

    ArTrackableType plane_tracked_type = AR_TRACKABLE_PLANE;
    ArSession_getAllTrackables(&p_ar_session, plane_tracked_type, plane_list);

    int32_t plane_list_size = 0;
    ArTrackableList_getSize(&p_ar_session, plane_list, &plane_list_size);
    // m_plane_count = plane_list_size;

    ALOGE("MCT Godot ARCore: Planes found %d", plane_list_size);

    std::vector<size_t> detected_guid;
    for (int i = 0; i < plane_list_size; ++i) {
        ArTrackable* ar_trackable = nullptr;
        ArTrackableList_acquireItem(&p_ar_session, plane_list, i, &ar_trackable);
        ArPlane* ar_plane = ArAsPlane(ar_trackable);
        ArTrackingState out_m_tracking_state;
        ArTrackable_getTrackingState(&p_ar_session, ar_trackable,
                                    &out_m_tracking_state);

        ArPlane* subsume_plane;
        ArPlane_acquireSubsumedBy(&p_ar_session, ar_plane, &subsume_plane);
        if (subsume_plane != nullptr) {
            ArTrackable_release(ArAsTrackable(subsume_plane));
            ArTrackable_release(ar_trackable);
            continue;
        }

        if (ArTrackingState::AR_TRACKING_STATE_TRACKING != out_m_tracking_state) {
            ArTrackable_release(ar_trackable);
            continue;
        }

        std::vector<float> boundary = get_plane_boundary(&p_ar_session, ar_plane);
        size_t plane_guid = get_uid(&p_ar_session, boundary);
        detected_guid.push_back(plane_guid);

        update_or_create_plane_node(plane_guid, &p_ar_session, ar_plane, boundary);

        ArTrackable_release(ar_trackable);
    }

    // Removing all lost planes
    std::vector<size_t> to_remove;
    using vec = std::unordered_map<size_t, godot::CSGPolygon3D*>;
    for (vec::const_iterator it = m_plane_nodes.cbegin(); it != m_plane_nodes.cend(); ++it) 
    {
        size_t uid = it->first;
        if (std::find(detected_guid.begin(), detected_guid.end(), uid) == detected_guid.end())
        {
            to_remove.push_back(uid);
        }
    }
    for(int i = 0; i < to_remove.size(); ++i) 
    {
        Node* node = m_plane_nodes[to_remove[i]];
        if (node)
        {
            node->queue_free();
        }
        m_plane_nodes.erase(to_remove[i]);
    }

    ArTrackableList_destroy(plane_list); //! Maxime: Might need to change that
}

void PlaneRenderer::clear() {
    if (get_root_node() && m_planes_node != nullptr) {
        get_root_node()->remove_child(m_planes_node);
        m_planes_node = nullptr;
    }
}