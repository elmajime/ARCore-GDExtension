//
// Created by Patrick on 07.09.2023.
//

#ifndef ARCOREGDEXTENSION_ARCORE_INTERFACE_H
#define ARCOREGDEXTENSION_ARCORE_INTERFACE_H

#include <map>

#include "godot_cpp/classes/xr_interface_extension.hpp"
#include "godot_cpp/classes/xr_positional_tracker.hpp"
#include "godot_cpp/variant/projection.hpp"

#include "include/arcore_c_api.h"

#include "background_renderer.h"
#include "plane_renderer.h"

namespace godot {
    class ARCoreInterface : public XRInterfaceExtension {
        GDCLASS(ARCoreInterface, XRInterfaceExtension);

    public:
        enum InitStatus {
            NOT_INITIALISED, // We're not initialised
            START_INITIALISE, // We just started our initialise process
            INITIALISED, // Yeah! we are up and running
            INITIALISE_FAILED // We failed to initialise
        };

        static void _bind_methods();

        ARCoreInterface();
        virtual ~ARCoreInterface();

        virtual XRInterface::TrackingStatus _get_tracking_status() const override;

        void _resume();

        void _pause();

        virtual StringName _get_name() const override;

        virtual uint32_t _get_capabilities() const override;

        virtual int32_t _get_camera_feed_id() const override;

        virtual bool _is_initialized() const override;

        virtual bool _initialize() override;

        virtual void _uninitialize() override;

        virtual Vector2 _get_render_target_size() override;

        virtual uint32_t _get_view_count() override;

        virtual Transform3D _get_camera_transform() override;

        virtual Transform3D _get_transform_for_view(uint32_t p_view, const Transform3D &p_cam_transform) override;

        virtual PackedFloat64Array _get_projection_for_view(uint32_t p_view, double p_aspect, double p_z_near, double p_z_far) override;

        virtual void _post_draw_viewport(const RID &p_render_target, const Rect2 &p_screen_rect) override;

        virtual void _process() override;

        virtual void notification(int p_what);

        bool isDepthSupported();

    private:
        enum class Orientation: int32_t {
            UNKNOWN = -1, //Not initialized
            ROTATION_0 = 0, // Natural orientation (portrait mode for phones)
            ROTATION_90 = 1, // 90° counter-clockwise rotation from the natural orientation
            ROTATION_180 = 2, // upside down
            ROTATION_270 = 3 // -270° counter-clockwise rotation from the natural orientation
        };

        arcore_plugin::BackgroundRenderer m_background_renderer;
        arcore_plugin::PlaneRenderer m_plane_renderer;

        InitStatus m_init_status;

        ArSession *m_ar_session;
        ArFrame *m_ar_frame;
        int m_width;
        int m_height;
        Orientation m_display_rotation;
        uint m_last_anchor_id;

        Ref<XRPositionalTracker> m_head;
        Transform3D m_view;
        Projection m_projection;
        float m_z_near, m_z_far;

        struct anchor_map {
            Ref<XRPositionalTracker> m_tracker;
            bool m_stale;
        };

        TrackingStatus m_tracking_state;

        std::map<ArPlane *, anchor_map *> m_anchors;

        int32_t m_plane_count;

        bool m_depthColorVisualizationEnabled;
        bool m_is_instant_placement_enabled;

        bool m_is_configured;

        // void make_anchors_stale();

        // void remove_stale_anchors();

        void configureSession();

        void handleScreenOrientationChanges();

        void estimateLight();

        // void updateAndRenderPlanes(const glm::mat4& p_projection_mat, const glm::mat4& p_view_mat);

    };
};

#endif //ARCOREGDEXTENSION_ARCORE_INTERFACE_H
