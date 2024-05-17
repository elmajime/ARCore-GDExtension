#include "background_renderer.h"
#include <godot_cpp/classes/camera_server.hpp>
#include "utils.h"

using namespace arcore_plugin;
using namespace godot;

BackgroundRenderer::BackgroundRenderer()
{

}

BackgroundRenderer::~BackgroundRenderer()
{

}

void BackgroundRenderer::initialize(int p_width, int p_height)
{
    // create our camera feed
    if (m_feed.is_null()) {
        ALOGV("MCT Godot ARCore: Creating camera feed...");

        m_feed.instantiate();
        m_feed->set_name("ARCore");
        m_feed->set_active(true);

        CameraServer *cs = CameraServer::get_singleton();
        if (cs != nullptr) {
            cs->add_feed(m_feed);

            ALOGV("MCT Godot ARCore: Feed %d added", m_feed->get_id());
        }
    }

    // make sure our feed is marked as active if we already have one...
    if (m_feed != nullptr) {
        m_feed->set_active(true);

        // The size here isn't actually used, ARCore will manage it, but set it just in case
        // Also this is a YCbCr texture, not RGB, should probably add a format for that some day :)
        m_feed->set_external(p_width, p_height);
        m_camera_texture_id = m_feed->get_texture_tex_id(CameraServer::FEED_YCBCR_IMAGE);

        ALOGV("MCT Godot ARCore: Created: %ld", m_camera_texture_id);
    }
}

void BackgroundRenderer::uninitialize()
{
    if (m_feed.is_valid()) {
        m_feed->set_active(false);

        CameraServer *cs = CameraServer::get_singleton();
        if (cs != nullptr) {
            cs->remove_feed(m_feed);
        }
        m_feed.unref();
    }
}

void BackgroundRenderer::draw(ArSession& p_ar_session, const ArFrame& p_ar_frame, bool p_depthColorVisualizationEnabled)
{
    // Have ARCore grab a camera frame, load it into our texture object and do its funky SLAM logic
    ArSession_setCameraTextureName(&p_ar_session, m_camera_texture_id);

    // Positions of the quad vertices in clip space (X, Y).
    static const float kVertices[] = {
            //	-1.0f, -1.0f, +1.0f, -1.0f, -1.0f, +1.0f, +1.0f, +1.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            1.0f,
            1.0f,
            1.0f,
    };
    // //! Maxime : do we need that ?
    // // If display rotation changed (also includes view size change), we need to
    // // re-query the uv coordinates for the on-screen portion of the camera image.
    // int32_t geometry_changed = 0;

    // ArFrame_getDisplayGeometryChanged(&p_ar_session, &p_ar_frame, &geometry_changed);
    // if (geometry_changed != 0 /*|| !have_display_transform*/) 
    // {
    //     ALOGV("MCT Godot ARCore: changing_display_transform");
    //     // update our transformed uvs
    //     float transformed_uvs[4 * 2];
    //     ArFrame_transformCoordinates2d(&p_ar_session, &p_ar_frame, AR_COORDINATES_2D_OPENGL_NORMALIZED_DEVICE_COORDINATES, 4, kVertices, AR_COORDINATES_2D_TEXTURE_NORMALIZED, transformed_uvs);
    //     // have_display_transform = true;

    //     // got to convert these uvs. They seem weird in portrait mode..
    //     bool shift_x = false;
    //     bool shift_y = false;

    //     // -1.0 - 1.0 => 0.0 - 1.0
    //     for (int i = 0; i < 8; i += 2) {
    //         transformed_uvs[i] = transformed_uvs[i] * 2.0 - 1.0;
    //         shift_x = shift_x || (transformed_uvs[i] < -0.001);
    //         transformed_uvs[i + 1] = transformed_uvs[i + 1] * 2.0 - 1.0;
    //         shift_y = shift_y || (transformed_uvs[i + 1] < -0.001);
    //     }

    //     // do we need to shift anything?
    //     if (shift_x || shift_y) {
    //         for (int i = 0; i < 8; i += 2) {
    //             if (shift_x) transformed_uvs[i] += 1.0;
    //             if (shift_y) transformed_uvs[i + 1] += 1.0;
    //         }
    //     }

    //     // Convert transformed_uvs to our display transform
    //     godot::Transform2D display_transform;
    //     display_transform.columns[0] = Vector2(transformed_uvs[2] - transformed_uvs[0], transformed_uvs[3] - transformed_uvs[1]);
    //     display_transform.columns[1] = Vector2(transformed_uvs[4] - transformed_uvs[0], transformed_uvs[5] - transformed_uvs[1]);
    //     display_transform.columns[2] = Vector2(transformed_uvs[0], transformed_uvs[1]);
    //     m_feed->set_transform(display_transform);
    // }

}