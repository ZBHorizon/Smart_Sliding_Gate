#pragma once

namespace SlidingGate {

enum class GateState { CLOSED, CLOSING, HALF_OPEN_FROM_CLOSED, HALF_OPEN_FROM_OPEN, OPENING, OPEN };

class Control {
  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Public Interface ////////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

public:

  static void control_loop();

  static void remote_a_isr();
  static void remote_b_isr();
  static void remote_c_isr();
  static void remote_d_isr();

  /*------------------------------------------------------------------------------------------------------------------*/
  /*//////// Private Interface ///////////////////////////////////////////////////////////////////////////////////////*/
  /*------------------------------------------------------------------------------------------------------------------*/

private:

  struct _Param {
    static constexpr float _HALF_OPEN = 0.5f;
  };

  static void remote_button_press();
  static void user_input();

  // Current state and a helper to remember the last non-HALF_OPEN state.
  inline static GateState gateState = GateState::CLOSED;
};

} // namespace SlidingGate
