#pragma once

#include <string>

#include <mujoco/mujoco.h>

#include "hakoniwa/mujoco_sensors/mujoco_sensor_frame.hpp"
#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

namespace hakoniwa::mujoco_sensors {

/**
 * @brief The host's ray caster on a MuJoCo world.
 *
 * Rays start at a MuJoCo site whose axes are the sensor axes (x forward, y left,
 * z up), at the site pose of the last mj_forward / mj_step. Geoms of the bodies
 * under exclude_root_body (the vehicle that carries the sensor) are never hit.
 * object_id is the MuJoCo geom id.
 */
// [RIS-MJR-060] The caster reads the model and data and never changes them
class MujocoRayCaster : public sensor_adapter::IRayCaster {
public:
    /// Returns false (with error) when the site or the body does not exist.
    bool initialize(const mjModel* model, const mjData* data, const std::string& site_name,
                    const std::string& exclude_root_body, std::string& error);

    /// Mounted on a body: position [m] and orientation (w, x, y, z) in the body frame. The body
    /// and the bodies under it (the vehicle) are never hit. No site is needed in the model.
    bool initialize(const mjModel* model, const mjData* data, int body_id, const double position[3],
                    const double orientation[4], std::string& error);

    bool cast(const sensor_adapter::Ray* rays, std::size_t count, double max_distance,
              sensor_adapter::RayHit* hits, std::string& error) override;

private:
    bool is_excluded(int geom_id) const;

    const mjModel* model_{nullptr};
    const mjData* data_{nullptr};
    MujocoSensorFrame frame_;
    int exclude_body_{-1};
};

}  // namespace hakoniwa::mujoco_sensors
