// start of file: ros2_ws/src/wbp_ui/include/wbp_ui/ui_state.hpp
#pragma once
#include <string>

namespace wbp_ui
{

  enum class UiAction
  {
    START,
    PAUSE,
    RESUME,
    ABORT
  };

  struct UiActionState
  {
    bool enabled = false;
    std::string reason;
  };

  struct UiState
  {
    UiActionState start{};
    UiActionState pause{};
    UiActionState resume{};
    UiActionState abort{};

    bool motion_running;
    bool motion_fault;
  };

} // namespace wbp_ui

// end of file: ros2_ws/src/wbp_ui/include/wbp_ui/ui_state.hpp