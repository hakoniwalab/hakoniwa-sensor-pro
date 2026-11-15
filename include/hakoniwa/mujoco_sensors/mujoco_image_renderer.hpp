#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

struct GLFWwindow;

namespace hakoniwa::mujoco_sensors {

/**
 * @brief The host's image renderer on a MuJoCo world (MuJoCo's built-in OpenGL renderer).
 *
 * Draws each request at its world pose and pinhole intrinsics. The model is not changed: the
 * camera is set on the scene after mjv_updateScene. Colour uses the model's multisampling;
 * depth is drawn without multisampling (a multisampled depth buffer is resolved from one
 * sample point, which shifts it by a fraction of a pixel). It needs an OpenGL context; it
 * makes its own with a hidden GLFW window. Call it from the thread that created it.
 */
// [RIS-MJI-080] Call from the thread that created it
class MujocoImageRenderer : public sensor_adapter::IImageRenderer {
public:
    MujocoImageRenderer() = default;
    MujocoImageRenderer(const MujocoImageRenderer&) = delete;
    MujocoImageRenderer& operator=(const MujocoImageRenderer&) = delete;
    ~MujocoImageRenderer() override;

    bool initialize(const mjModel* model, mjData* data, std::string& error);

    bool render(const sensor_adapter::RenderRequest* requests, std::size_t count,
                sensor_adapter::RenderStatus* statuses) override;
    sensor_adapter::ImageRendererCapabilities capabilities() const override;

private:
    const mjModel* model_{nullptr};
    mjData* data_{nullptr};
    bool open_context(std::string& error);
    GLFWwindow* window_{nullptr};
    bool scene_made_{false};
    bool validate(const sensor_adapter::RenderRequest& request, std::string& error) const;
    bool draw(const sensor_adapter::RenderRequest& request, mjrContext& context,
              unsigned char* rgb, float* depth);
    mjvScene scene_{};
    mjvOption option_{};
    mjvCamera camera_{};
    mjrContext color_context_{};   // the model's offscreen multisampling
    mjrContext depth_context_{};   // no multisampling
    bool depth_context_made_{false};
    std::vector<unsigned char> rgb_raw_;
    std::vector<float> depth_raw_;
};

}  // namespace hakoniwa::mujoco_sensors
