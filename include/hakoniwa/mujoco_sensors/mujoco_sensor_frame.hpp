#pragma once

#include <mujoco/mujoco.h>

namespace hakoniwa::mujoco_sensors {

/**
 * @brief Where a sensor is on a MuJoCo model: a site, or a body and a mounting pose.
 *
 * The sensor axes are x forward, y left, z up. With a site, they are the site's axes.
 * With a body, the mounting position and orientation are given in that body's frame
 * (for example the vehicle's root body), so the model needs no site for the sensor.
 */
class MujocoSensorFrame {
public:
    void set_site(int site_id)
    {
        site_id_ = site_id;
        body_id_ = -1;
    }

    /// position [m] and orientation (unit quaternion w, x, y, z) in the body frame.
    void set_body(int body_id, const double position[3], const double orientation[4])
    {
        site_id_ = -1;
        body_id_ = body_id;
        mjtNum q[4] = {orientation[0], orientation[1], orientation[2], orientation[3]};
        mju_normalize4(q);
        mju_quat2Mat(mount_rotation_, q);
        for (int i = 0; i < 3; ++i) mount_position_[i] = position[i];
    }

    /// World position and rotation (row-major, columns = sensor axes in the world) at the
    /// state of the last mj_forward / mj_step.
    void pose(const mjData* data, mjtNum position[3], mjtNum rotation[9]) const
    {
        if (site_id_ >= 0) {
            mju_copy3(position, data->site_xpos + 3 * site_id_);
            mju_copy(rotation, data->site_xmat + 9 * site_id_, 9);
            return;
        }
        const mjtNum* p = data->xpos + 3 * body_id_;
        const mjtNum* R = data->xmat + 9 * body_id_;
        mjtNum offset[3];
        mju_mulMatVec(offset, R, mount_position_, 3, 3);
        mju_add3(position, p, offset);
        mju_mulMatMat(rotation, R, mount_rotation_, 3, 3, 3);
    }

    int body_id() const { return body_id_; }
    bool valid() const { return site_id_ >= 0 || body_id_ >= 0; }

private:
    int site_id_{-1};
    int body_id_{-1};
    mjtNum mount_position_[3]{0, 0, 0};
    mjtNum mount_rotation_[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
};

}  // namespace hakoniwa::mujoco_sensors
