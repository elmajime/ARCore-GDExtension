#include "arcore_interface.h"
#include "arcore_plugin_wrapper.h"
#include "utils.h"

#include <godot_cpp/classes/main_loop.hpp>
#include <godot_cpp/classes/camera_server.hpp>
#include <godot_cpp/classes/display_server.hpp>

#include "glm.hpp"
#include "gtc/type_ptr.hpp"

using namespace godot;

void ARCoreInterface::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("_resume"), &ARCoreInterface::_resume);
    ClassDB::bind_method(D_METHOD("_pause"), &ARCoreInterface::_pause);
    ClassDB::bind_method(D_METHOD("enable_depth_estimation"), &ARCoreInterface::enable_depth_estimation);
    ClassDB::bind_method(D_METHOD("show_depth_map"), &ARCoreInterface::show_depth_map);
    ClassDB::bind_method(D_METHOD("set_depth_color_mapping"), &ARCoreInterface::set_depth_color_mapping);
    ClassDB::bind_method(D_METHOD("enable_plane_detection"), &ARCoreInterface::enable_plane_detection);
}

ARCoreInterface::ARCoreInterface()
{
    m_ar_session = nullptr;
    m_ar_frame = nullptr;
    m_init_status = NOT_INITIALISED;
    m_width = 1;
    m_height = 1;
    m_display_rotation = Orientation::UNKNOWN;
    m_last_anchor_id = 0;
    m_z_near = 0.01;
    m_z_far = 1000.0;
    m_plane_count = 0;
    m_is_instant_placement_enabled = false;
    m_depthColorVisualizationEnabled = false;
    m_enable_depth_estimation = false;
    m_enable_plane_detection = false;
    m_projection.set_perspective(60.0, 1.0, m_z_near, m_z_far, false); // this is just a default, will be changed by ARCore
}
ARCoreInterface::~ARCoreInterface()
{
    // remove_all_anchors();

    // and make sure we cleanup if we haven't already
    if (is_initialized()) {
        uninitialize();
    }
}

XRInterface::TrackingStatus ARCoreInterface::_get_tracking_status() const
{
    return m_tracking_state;
}

void ARCoreInterface::_resume()
{
    if (m_init_status == INITIALISED && m_ar_session != nullptr) {
        ArStatus status = ArSession_resume(m_ar_session);
        if (status != AR_SUCCESS) {
            ALOGE("Godot ARCore: Failed to resume.");

            // TODO quit? how?
        }
    }
}

void ARCoreInterface::_pause()
{
    if (m_ar_session != nullptr) {
        ArSession_pause(m_ar_session);
    }
}

StringName ARCoreInterface::_get_name() const
{
    StringName name("ARCore");
    return name;
}

uint32_t ARCoreInterface::_get_capabilities() const
{
    return XR_MONO + XR_AR;
}

int32_t ARCoreInterface::_get_camera_feed_id() const
{
    ALOGE("MCT Godot ARCore: _get_camera_feed_id");
    if (m_background_renderer.getFeed().is_valid()) {
        return m_background_renderer.getFeed()->get_id();
    } else {
        return 0;
    }
}


void ARCoreInterface::enable_depth_estimation(bool p_enable)
{
    m_enable_depth_estimation = p_enable;
    configureSession();
}

void ARCoreInterface::show_depth_map(bool p_enable)
{
    godot::Ref<godot::CameraFeed> feed = m_background_renderer.getFeed();
    if (feed.is_valid()) {
        feed->set_display_depthmap(p_enable);
    }
}

void ARCoreInterface::enable_plane_detection(bool p_enable) {
    m_enable_plane_detection = p_enable;
}

void ARCoreInterface::set_depth_color_mapping(float p_mid_depth_meters, float p_max_depth_meters)
{
    godot::Ref<godot::CameraFeed> feed = m_background_renderer.getFeed();
    if (feed.is_valid())
    {
        feed->set_depthmap_display_mapping(p_mid_depth_meters, p_max_depth_meters);
    }
}

bool ARCoreInterface::_is_initialized() const
{
    // if we're in the process of initialising we treat this as initialised...
    return (m_init_status != NOT_INITIALISED) && (m_init_status != INITIALISE_FAILED);
}

bool ARCoreInterface::_initialize()
{
    ALOGE("MCT Godot ARCore: _initialize");

    XRServer *xr_server = XRServer::get_singleton();
    ERR_FAIL_NULL_V(xr_server, false);


    if (m_init_status == INITIALISE_FAILED) {
        // if we fully failed last time, don't try again..
        return false;
    } else if (m_init_status == NOT_INITIALISED) {
        ALOGV("MCT Godot ARCore: Initialising...");
        m_init_status = START_INITIALISE;

        if (m_ar_session == nullptr) {
            ALOGV("MCT Godot ARCore: Getting environment");

            // get some android things
            JNIEnv *env = ARCorePluginWrapper::get_env();

            jobject context = ARCorePluginWrapper::get_global_context();

            // jobject context = ARCorePluginWrapper::get_activity();
            if (context == nullptr) {
                ALOGE("Godot ARCore: Couldn't get context");
                m_init_status = INITIALISE_FAILED; // don't try again.
                return false;
            }

            ALOGV("MCT Godot ARCore: Create ArSession");

            if (ArSession_create(env, context, &m_ar_session) != AR_SUCCESS || m_ar_session == nullptr) {
                ALOGE("Godot ARCore: ARCore couldn't be created.");
                m_init_status = INITIALISE_FAILED; // don't try again.
                return false;
            }

            ALOGV("MCT Godot ARCore: Create ArFrame.");

            ArFrame_create(m_ar_session, &m_ar_frame);
            if (m_ar_frame == nullptr) {
                ALOGE("Godot ARCore: Frame couldn't be created.");

                ArSession_destroy(m_ar_session);
                m_ar_session = nullptr;

                m_init_status = INITIALISE_FAILED; // don't try again.
                return false;
            }

            // Get our size, make sure we have these in portrait
            Size2 size = DisplayServer::get_singleton()->screen_get_size();
            if (size.x > size.y) {
                m_width = size.y;
                m_height = size.x;
            } else {
                m_width = size.x;
                m_height = size.y;
            }

            // Trigger display rotation
            m_display_rotation = Orientation::UNKNOWN;

            ALOGV("MCT Godot ARCore: Initialised.");
            m_init_status = INITIALISED;
        }

        // and call resume for the first time to complete this
        _resume();

        if (m_init_status != INITIALISE_FAILED) {
            // we must create a tracker for our head
            m_head.instantiate();
            m_head->set_tracker_type(XRServer::TRACKER_HEAD);
            m_head->set_tracker_name("head");
            m_head->set_tracker_desc("AR Device");
            xr_server->add_tracker(m_head);

            // make this our primary interface
            xr_server->set_primary_interface(this);
        }

        m_background_renderer.initialize(m_width, m_height);

        configureSession();
    }

    return is_initialized();
}

void ARCoreInterface::_uninitialize()
{
    if (_is_initialized()) {
        ALOGE("MCT Godot ARCore: _uninitialize");
        // TODO we may want to call ArSession_pauze here and introduce a new status PAUZED
        // then move cleanup to our destruct.

        XRServer *xr_server = XRServer::get_singleton();
        ERR_FAIL_NULL(xr_server);

        // make_anchors_stale();
        // remove_stale_anchors();

        if (m_ar_session != nullptr) {
            ArSession_destroy(m_ar_session);
            ArFrame_destroy(m_ar_frame);

            m_ar_session = nullptr;
            m_ar_frame = nullptr;
        }

        m_background_renderer.uninitialize();

        if (m_head.is_valid()) {
            xr_server->remove_tracker(m_head);

            m_head.unref();
        }

        m_init_status = NOT_INITIALISED;
    }
}

Vector2 ARCoreInterface::_get_render_target_size()
{
    // ALOGE("MCT Godot ARCore: _get_render_target_size");
    Vector2 target_size = DisplayServer::get_singleton()->screen_get_size();
    return target_size;
}

uint32_t ARCoreInterface::_get_view_count()
{
    // ALOGE("MCT Godot ARCore: _get_view_count");
    return 1;
}

Transform3D ARCoreInterface::_get_camera_transform()
{
    //ALOGE("MCT Godot ARCore: _get_camera_transform");
    Transform3D transform_for_eye;

    XRServer *xr_server = XRServer::get_singleton();
    ERR_FAIL_NULL_V(xr_server, Transform3D());

    if (m_init_status == INITIALISED) {
        float world_scale = xr_server->get_world_scale();

        // just scale our origin point of our transform, note that we really shouldn't be using world_scale in ARKit but....
        transform_for_eye = m_view;
        transform_for_eye.origin *= world_scale;

        transform_for_eye = xr_server->get_reference_frame() * transform_for_eye;
    }

    return transform_for_eye;
}

Transform3D ARCoreInterface::_get_transform_for_view(uint32_t p_view, const Transform3D &p_cam_transform)
{
    //ALOGE("MCT Godot ARCore: _get_transform_for_view");
    Transform3D transform_for_eye;

    XRServer *xr_server = XRServer::get_singleton();
    ERR_FAIL_NULL_V(xr_server, Transform3D());

    if (m_init_status == INITIALISED) {
        float world_scale = xr_server->get_world_scale();

        // just scale our origin point of our transform, note that we really shouldn't be using world_scale in ARKit but....
        transform_for_eye = m_view;
        transform_for_eye.origin *= world_scale;

        transform_for_eye = p_cam_transform * (xr_server->get_reference_frame()) * transform_for_eye;
    } else {
        // huh? well just return what we got....
        transform_for_eye = p_cam_transform;
    }

    return transform_for_eye;
}

PackedFloat64Array ARCoreInterface::_get_projection_for_view(uint32_t p_view, double p_aspect, double p_m_z_near, double p_z_far)
{
    //ALOGE("MCT Godot ARCore: _get_projection_for_view");
    PackedFloat64Array arr;
    arr.resize(16); // 4x4 matrix

    // Remember our near and far, we'll use it next frame
    m_z_near = p_m_z_near;
    m_z_far = p_z_far;

    real_t *p = (real_t *)&m_projection.columns;
    for(int i = 0; i < 16; i++) {
        arr[i] = p[i];
    }

    return arr;
}

void ARCoreInterface::_post_draw_viewport(const RID &p_render_target, const Rect2 &p_screen_rect)
{
    //ALOGE("MCT Godot ARCore: _post_draw_viewport");
    // We must have a valid render target
    ERR_FAIL_COND(!p_render_target.is_valid());

    // Because we are rendering to our device we must use our main viewport!
    ERR_FAIL_COND(p_screen_rect == Rect2());

    Rect2 src_rect(0.0f, 0.0f, 1.0f, 1.0f);
    Rect2 dst_rect = p_screen_rect;

    add_blit(p_render_target, src_rect, dst_rect, false, 0, false, Vector2(), 0, 0, 0.0, 1.0);
}

void ARCoreInterface::handleScreenOrientationChanges() 
{
    DisplayServer::ScreenOrientation orientation = DisplayServer::get_singleton()->screen_get_orientation(DisplayServer::get_singleton()->get_primary_screen());
    Orientation display_rotation = Orientation::UNKNOWN;

    switch (orientation)
    {
    case DisplayServer::SCREEN_LANDSCAPE:
        //ALOGE("MCT Godot ARCore: in SCREEN_LANDSCAPE mode");
        display_rotation = Orientation::ROTATION_90;
        break;
    case DisplayServer::SCREEN_PORTRAIT:
        //ALOGE("MCT Godot ARCore: in SCREEN_PORTRAIT mode");
        display_rotation = Orientation::ROTATION_0;
        break;

    case DisplayServer::SCREEN_REVERSE_LANDSCAPE:
        //ALOGE("MCT Godot ARCore: in SCREEN_REVERSE_LANDSCAPE mode");
        display_rotation = Orientation::ROTATION_270;
        break;

    case DisplayServer::SCREEN_REVERSE_PORTRAIT:
        //ALOGE("MCT Godot ARCore: in SCREEN_REVERSE_PORTRAIT mode");
        display_rotation = Orientation::ROTATION_180;
        break;

    case DisplayServer::SCREEN_SENSOR_LANDSCAPE:
        //ALOGE("MCT Godot ARCore: in SCREEN_SENSOR_LANDSCAPE mode");
        display_rotation = Orientation::ROTATION_90;
        break;

    case DisplayServer::SCREEN_SENSOR_PORTRAIT:
        //ALOGE("MCT Godot ARCore: in SCREEN_SENSOR_PORTRAIT mode");
        display_rotation = Orientation::ROTATION_0;
        break;

    case DisplayServer::SCREEN_SENSOR:
        ALOGE("Godot ARCore: SCREEN_SENSOR mode not handle yet");
        display_rotation = Orientation::UNKNOWN; //TODO
        break;
    default:
        break;
    }
    //! Maxime: Todo: Correctly handle orientation changes
    // display_rotation = Orientation::VERTICAL;

    if (m_display_rotation != display_rotation)
    {
        m_display_rotation = display_rotation;
        if (m_display_rotation == Orientation::ROTATION_90 || m_display_rotation == Orientation::ROTATION_270)
        {
            int tmp = m_height;
            m_height = m_width;
            m_width = tmp;
        }

        const int32_t c_ar_core_orientation = (int32_t)(Orientation::ROTATION_90); // For now, only godot's Landscape mode works with ARCore
        ArSession_setDisplayGeometry(m_ar_session, c_ar_core_orientation, m_width, m_height);

        ALOGV("MCT Godot ARCore: Window orientation changes to %d (%d, %d)", m_display_rotation, m_width, m_height);
    }
}

void ARCoreInterface::estimateLight() {
    // Get light estimation value.
    ArLightEstimate* ar_light_estimate;
    ArLightEstimateState ar_light_estimate_state;
    ArLightEstimate_create(m_ar_session, &ar_light_estimate);

    ArFrame_getLightEstimate(m_ar_session, m_ar_frame, ar_light_estimate);
    ArLightEstimate_getState(m_ar_session, ar_light_estimate,
                            &ar_light_estimate_state);

    // Set light intensity to default. Intensity value ranges from 0.0f to 1.0f.
    // The first three components are color scaling factors.
    // The last one is the average pixel intensity in gamma space.
    float color_correction[4] = {1.f, 1.f, 1.f, 1.f};
    if (ar_light_estimate_state == AR_LIGHT_ESTIMATE_STATE_VALID) {
        ArLightEstimate_getColorCorrection(m_ar_session, ar_light_estimate,
                                        color_correction);
    }

    ArLightEstimate_destroy(ar_light_estimate);
    ar_light_estimate = nullptr;
}

// void ARCoreInterface::updateAndRenderPlanes(const glm::mat4& p_projection_mat, const glm::mat4& p_view_mat)
// {
//     // Update and render planes.
//     ArTrackableList* plane_list = nullptr;
//     ArTrackableList_create(m_ar_session, &plane_list);
//     ERR_FAIL_NULL(plane_list);

//     ArTrackableType plane_tracked_type = AR_TRACKABLE_PLANE;
//     ArSession_getAllTrackables(m_ar_session, plane_tracked_type, plane_list);

//     int32_t plane_list_size = 0;
//     ArTrackableList_getSize(m_ar_session, plane_list, &plane_list_size);
//     m_plane_count = plane_list_size;

//     ALOGE("MCT Godot ARCore: Planes found %d", m_plane_count);

//     m_plane_renderer.clear(); //! Maxime: find a way to not needing to clear all planes each time
//     for (int i = 0; i < plane_list_size; ++i) {
//         ArTrackable* ar_trackable = nullptr;
//         ArTrackableList_acquireItem(m_ar_session, plane_list, i, &ar_trackable);
//         ArPlane* ar_plane = ArAsPlane(ar_trackable);
//         ArTrackingState out_m_tracking_state;
//         ArTrackable_getTrackingState(m_ar_session, ar_trackable,
//                                     &out_m_tracking_state);

//         ArPlane* subsume_plane;
//         ArPlane_acquireSubsumedBy(m_ar_session, ar_plane, &subsume_plane);
//         if (subsume_plane != nullptr) {
//             ArTrackable_release(ArAsTrackable(subsume_plane));
//             ArTrackable_release(ar_trackable);
//             continue;
//         }

//         if (ArTrackingState::AR_TRACKING_STATE_TRACKING != out_m_tracking_state) {
//             ArTrackable_release(ar_trackable);
//             continue;
//         }

//         m_plane_renderer.draw(*m_ar_session, *ar_plane);
//         ArTrackable_release(ar_trackable);
//     }

//     ArTrackableList_destroy(plane_list);
//     plane_list = nullptr;
// }

godot::Transform3D glm_to_godot_transform(const glm::mat4& glm_matrix) {
    // Extract the basis (3x3 part)
    godot::Basis basis(
        glm_matrix[0][0], glm_matrix[1][0], glm_matrix[2][0],
        glm_matrix[0][1], glm_matrix[1][1], glm_matrix[2][1],
        glm_matrix[0][2], glm_matrix[1][2], glm_matrix[2][2]
    );

    // Extract the origin (translation part)
    godot::Vector3 origin(
        glm_matrix[3][0],
        glm_matrix[3][1],
        glm_matrix[3][2]
    );

    // Create the Transform
    return godot::Transform3D(basis, origin);
}

godot::Projection glm_to_godot_projection(const glm::mat4& glm_matrix) 
{
    // Create a godot::Projection from the glm::mat4
    godot::Projection godot_projection;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            godot_projection[i][j] = glm_matrix[i][j];
        }
    }

    return godot_projection;
}

godot::XRInterface::TrackingStatus ar_tracking_state_to_tracking_status(const ArSession& p_ar_session, const ArCamera& p_ar_camera, const ArTrackingState& p_camera_tracking_state) 
{
    godot::XRInterface::TrackingStatus trackingStatus;
    switch (p_camera_tracking_state) {
        case AR_TRACKING_STATE_TRACKING:
            trackingStatus = XRInterface::XR_NORMAL_TRACKING;
            break;
        case AR_TRACKING_STATE_PAUSED:
            // lets find out why..
            ArTrackingFailureReason camera_tracking_failure_reason;
            ArCamera_getTrackingFailureReason(&p_ar_session, &p_ar_camera, &camera_tracking_failure_reason);
            switch (camera_tracking_failure_reason) {
                case AR_TRACKING_FAILURE_REASON_NONE:
                ALOGE("MCT Godot ARCore: AR_TRACKING_FAILURE_REASON_NONE");
                    trackingStatus = XRInterface::XR_UNKNOWN_TRACKING;
                    break;
                case AR_TRACKING_FAILURE_REASON_BAD_STATE:
                ALOGE("MCT Godot ARCore: AR_TRACKING_FAILURE_REASON_BAD_STATE");
                    trackingStatus = XRInterface::XR_INSUFFICIENT_FEATURES; // @TODO add bad state to XRInterface
                    break;
                case AR_TRACKING_FAILURE_REASON_INSUFFICIENT_LIGHT:
                ALOGE("MCT Godot ARCore: AR_TRACKING_FAILURE_REASON_INSUFFICIENT_LIGHT");
                    trackingStatus = XRInterface::XR_INSUFFICIENT_FEATURES; // @TODO add insufficient light to XRInterface
                    break;
                case AR_TRACKING_FAILURE_REASON_EXCESSIVE_MOTION:
                ALOGE("MCT Godot ARCore: AR_TRACKING_FAILURE_REASON_EXCESSIVE_MOTION");
                    trackingStatus = XRInterface::XR_EXCESSIVE_MOTION;
                    break;
                case AR_TRACKING_FAILURE_REASON_INSUFFICIENT_FEATURES:
                ALOGE("MCT Godot ARCore: AR_TRACKING_FAILURE_REASON_INSUFFICIENT_FEATURES");
                    trackingStatus = XRInterface::XR_INSUFFICIENT_FEATURES;
                    break;
                case AR_TRACKING_FAILURE_REASON_CAMERA_UNAVAILABLE:
                ALOGE("MCT Godot ARCore: AR_TRACKING_FAILURE_REASON_CAMERA_UNAVAILABLE");
                    trackingStatus = XRInterface::XR_INSUFFICIENT_FEATURES; // @TODO add no camera to XRInterface
                    break;
                default:
                ALOGE("MCT Godot ARCore: XR_UNKNOWN_TRACKING");
                    trackingStatus = XRInterface::XR_UNKNOWN_TRACKING;
                    break;
            };

            break;
        case AR_TRACKING_STATE_STOPPED:
            ALOGE("MCT Godot ARCore: AR_TRACKING_STATE_STOPPED");
            trackingStatus = XRInterface::XR_NOT_TRACKING;
            break;
        default:
            ALOGE("MCT Godot ARCore: XR_UNKNOWN_TRACKING");
            trackingStatus = XRInterface::XR_UNKNOWN_TRACKING;
            break;
    };

    return trackingStatus;
}


void ARCoreInterface::_process()
{
    if (m_init_status != INITIALISED) {
        // not yet initialised so....
        return;
    } else if ((m_ar_session == nullptr) or (m_background_renderer.getFeed().is_null())) {
        ALOGE("MCT Godot ARCore: _process don't have a session yet so...");
        // don't have a session yet so...
        return;
    }

    handleScreenOrientationChanges();

    ArSession_setCameraTextureName(m_ar_session,
                                 m_background_renderer.getTextureId());

    // Update session to get current frame and render camera background.
    if (ArSession_update(m_ar_session, m_ar_frame) != AR_SUCCESS) {
        ALOGW("Godot ARCore: OnDrawFrame ArSession_update error");
    }

    ArCamera *ar_camera;
    ArFrame_acquireCamera(m_ar_session, m_ar_frame, &ar_camera);

    //! Maxime: check this
    // int32_t geometry_changed = 0;
    // ArFrame_getDisplayGeometryChanged(m_ar_session, m_ar_frame, &geometry_changed);
    // if (geometry_changed != 0 || !calculate_uv_transform_) {
    //     // The UV Transform represents the transformation between screenspace in
    //     // normalized units and screenspace in units of pixels.  Having the size of
    //     // each pixel is necessary in the virtual object shader, to perform
    //     // kernel-based blur effects.
    //     calculate_uv_transform_ = false;
    //     glm::mat3 transform = GetTextureTransformMatrix(m_ar_session, m_ar_frame);
    //     andy_renderer_.SetUvTransformMatrix(transform);
    // }

    glm::mat4 view_mat;
    ArCamera_getViewMatrix(m_ar_session, ar_camera, glm::value_ptr(view_mat));

    // TODO: We may need to adjust this based on orientation
    m_view = glm_to_godot_transform(view_mat);

    // invert our view matrix
    m_view.invert();

    if (m_head.is_valid()) {
        // Set our head position, note in real space, reference frame and world scale is applied later
        m_head->set_pose("default", m_view, Vector3(), Vector3(), XRPose::XR_TRACKING_CONFIDENCE_HIGH);
    }

    glm::mat4 projection_mat;
    ArCamera_getProjectionMatrix(m_ar_session, ar_camera,
                                /*near=*/0.1f, /*far=*/100.f,
                                glm::value_ptr(projection_mat));

    m_projection = glm_to_godot_projection(projection_mat);

    m_background_renderer.draw(*m_ar_session, *m_ar_frame, m_depthColorVisualizationEnabled);
    
    ArTrackingState camera_m_tracking_state;
    ArCamera_getTrackingState(m_ar_session, ar_camera, &camera_m_tracking_state);

    m_tracking_state =  ar_tracking_state_to_tracking_status(*m_ar_session, *ar_camera, camera_m_tracking_state);

    ArCamera_release(ar_camera);

    // If the camera isn't tracking don't bother rendering other objects.
    //! Maxime: this might not be usefull in our case
    if (camera_m_tracking_state != AR_TRACKING_STATE_TRACKING) {
        ALOGE("MCT Godot ARCore: not AR_TRACKING_STATE_TRACKING");
        return;
    }

    int32_t is_depth_supported = 0;
    ArSession_isDepthModeSupported(m_ar_session, AR_DEPTH_MODE_AUTOMATIC,
                                 &is_depth_supported);

    if (is_depth_supported && m_enable_depth_estimation) {
        godot::Ref<godot::CameraFeed> feed = m_background_renderer.getFeed();

        ArImage* depthImage = nullptr;
        if (ArFrame_acquireDepthImage16Bits(m_ar_session, m_ar_frame, &depthImage) != AR_SUCCESS) {
            ALOGE("MCT Godot ARCore: Error: Failed to acquire depth image");
            return;
        }

        ALOGE("MCT Godot ARCore: Acquired depth image");
        int32_t width, height;
        ArImage_getWidth(m_ar_session, depthImage, &width);
        ArImage_getHeight(m_ar_session, depthImage, &height);

        const uint8_t* depthData = nullptr;
        int32_t depthDataSize = 0;
        ArImage_getPlaneData(m_ar_session, depthImage, 0, reinterpret_cast<const uint8_t**>(&depthData), &depthDataSize);

        PackedByteArray array;
        array.resize(width*height*2); // 2 bytes per pixel (16-bit)

        for (int i = 0; i < width * height * 2; ++i) {
            // uint16_t depth_value = *(depthData + i*sizeof(uint8_t));
            // // array[i * 2] = depth_value & 0xFF; // Low byte
            // // array[i * 2 + 1] = (depth_value >> 8) & 0xFF; // High byte
            // array.encode_u16(i*2, depth_value);
            array[i] = *(depthData + i*sizeof(uint8_t));
        }

        ArImage_release(depthImage);

        feed->set_external_depthmap(array, width, height);
        ALOGE("MCT Godot ARCore: sent depth image to godot");
    }

    estimateLight();

    if (m_enable_plane_detection) {
        m_plane_renderer.draw(*m_ar_session);
    } else {
        m_plane_renderer.clear();
    }
    
    //! Maxime: see what this is 
    // andy_renderer_.setUseDepthForOcclusion(asset_manager_, useDepthForOcclusion);

    // // Render Andy objects.
    // glm::mat4 model_mat(1.0f);
    // for (auto& colored_anchor : anchors_) {
    //     ArTrackingState m_tracking_state = AR_m_TRACKING_STATE_STOPPED;
    //     ArAnchor_getTrackingState(m_ar_session, colored_anchor.anchor,
    //                             &m_tracking_state);
    //     if (m_tracking_state == AR_TRACKING_STATE_TRACKING) {
    //     UpdateAnchorColor(&colored_anchor);
    //     // Render object only if the tracking state is AR_TRACKING_STATE_TRACKING.
    //     util::GetTransformMatrixFromAnchor(*colored_anchor.anchor, m_ar_session,
    //                                         &model_mat);
    //     andy_renderer_.Draw(projection_mat, view_mat, model_mat, color_correction,
    //                         colored_anchor.color);
    //     }
    // }

    //! Maxime : Todo
    // // Update and render point cloud.
    // ArPointCloud* ar_point_cloud = nullptr;
    // ArStatus point_cloud_status =
    //     ArFrame_acquirePointCloud(m_ar_session, m_ar_frame, &ar_point_cloud);
    // if (point_cloud_status == AR_SUCCESS) {
    //     point_cloud_renderer_.draw(projection_mat * view_mat, m_ar_session,
    //                             ar_point_cloud);
    //     ArPointCloud_release(ar_point_cloud);
    // }
}

void ARCoreInterface::notification(int p_what)
{
    ALOGE("MCT Godot ARCore: notification");
    // Needs testing, this should now be called

    // XRServer *xr_server = XRServer::get_singleton();
    // ERR_FAIL_NULL(xr_server);

    switch (p_what) {
        case MainLoop::NOTIFICATION_APPLICATION_RESUMED: {
            if (is_initialized()) {
                _resume();

                if (m_init_status == INITIALISE_FAILED) {
                    // if (xr_server->get_primary_interface() == this) {
                    //     xr_server->set_primary_interface(Ref<XRInterface>());
                    // }
                }
            }
        }; break;
        case MainLoop::NOTIFICATION_APPLICATION_PAUSED:
            if (is_initialized()) {
                _pause();
            }
            break;
        default:
            break;
    }
}

bool ARCoreInterface::isDepthSupported()
{
  int32_t is_supported = 0;
  ArSession_isDepthModeSupported(m_ar_session, AR_DEPTH_MODE_AUTOMATIC,
                                 &is_supported);
  return is_supported;
}

void ARCoreInterface::configureSession()
{     
  const bool is_depth_supported = isDepthSupported();

  ArConfig* ar_config = nullptr;
  ArConfig_create(m_ar_session, &ar_config);
  if (is_depth_supported && m_enable_depth_estimation) {
    ArConfig_setDepthMode(m_ar_session, ar_config, AR_DEPTH_MODE_AUTOMATIC);
  } else {
    ArConfig_setDepthMode(m_ar_session, ar_config, AR_DEPTH_MODE_DISABLED);
  }

//! Maxime : todo
//   if (m_is_instant_placement_enabled) {
//     ArConfig_setInstantPlacementMode(m_ar_session, ar_config,
//                                      AR_INSTANT_PLACEMENT_MODE_LOCAL_Y_UP);
//   } else {
//     ArConfig_setInstantPlacementMode(m_ar_session, ar_config,
//                                      AR_INSTANT_PLACEMENT_MODE_DISABLED);
//   }

  ERR_FAIL_NULL(ar_config);
  ERR_FAIL_COND(ArSession_configure(m_ar_session, ar_config) == AR_SUCCESS);
  ArConfig_destroy(ar_config);

}

// void ARCoreInterface::make_anchors_stale() {
//     for (const auto &anchor: anchors) {
//         anchor.second->stale = true;
//     }
// }

// void ARCoreInterface::remove_stale_anchors() {
//     for (auto it = anchors.cbegin(); it != anchors.cend();) {
//         ArPlane *ar_plane = it->first;
//         anchor_map *am = it->second;
//         if (am->stale) {
//             it = anchors.erase(it);

//             XRServer::get_singleton()->remove_tracker(am->tracker);
//             am->tracker.unref();
//             delete am;
//             ArTrackable_release(ArAsTrackable(ar_plane));
//         } else {
//             ++it;
//         }
//     }
// }