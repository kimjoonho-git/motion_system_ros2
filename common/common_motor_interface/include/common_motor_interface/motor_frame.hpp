#ifndef COMMON_MOTOR_INTERFACE_MOTOR_FRAME_HPP_
#define COMMON_MOTOR_INTERFACE_MOTOR_FRAME_HPP_

#include <cstdint>
#include <string>

namespace motor_interface {

inline constexpr uint8_t MAX_INTERFACE_SIZE = 16;

struct motor_frame_t {
    uint8_t number_of_target_interfaces{0};
    uint8_t target_interface_id[MAX_INTERFACE_SIZE]{0};

    uint8_t controller_index{};
    std::string driver_name{};
    uint16_t controlword{};
    uint16_t statusword{};
    uint16_t errorcode{};
    uint16_t station_alias_register{};
    double position{};
    double velocity{};
    double torque{};
    double current{};
    int32_t position_raw{};
    int32_t velocity_raw{};
    int16_t torque_raw{};
    int16_t current_raw{};
};

}  // namespace motor_interface

#endif  // COMMON_MOTOR_INTERFACE_MOTOR_FRAME_HPP_
