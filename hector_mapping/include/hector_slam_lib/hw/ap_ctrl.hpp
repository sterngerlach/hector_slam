
/* ap_ctrl.hpp */

#ifndef HECTOR_SLAM_HW_AP_CTRL_HPP
#define HECTOR_SLAM_HW_AP_CTRL_HPP

#include <cstdint>

namespace hectorslam {
namespace hw {

/*
 * AxiLiteApCtrl enum represents the block-level control signals for
 * the AXI4-Lite slave interface
 */
enum class AxiLiteSApCtrl : std::uint32_t
{
    Start       = 0x01,
    Done        = 0x02,
    Idle        = 0x04,
    Ready       = 0x08,
    AutoRestart = 0x80,
};

} /* namespace hw */
} /* namespace hectorslam */

#endif /* HECTOR_SLAM_HW_AP_CTRL_HPP */
