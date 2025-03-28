//////////////// GateSimulator Implementation ////////////////

// bool GateSimulator::running = false;

// void GateSimulator::simulation_loop() {
//     using namespace std::chrono;
//     // Update interval (adjust as needed)
//     auto interval = milliseconds(100);
//     running = true;
//     while (running) {
//         // Capture the current motor speed.
//         float motor_speed = 0.0f;// Test_IO::get_current_motor_speed();
//         // Determine delta based on direction.
//         float delta = 0.0f;
//         if (motor_speed > 0.0f) {
//             // Opening: full open takes TIME_TO_OPEN.
//             delta = motor_speed * (interval.count() / static_cast<float>(TIME_TO_OPEN.count()));
//         } else if (motor_speed < 0.0f) {
//             // Closing: full close takes TIME_TO_CLOSE.
//             delta = motor_speed * (interval.count() / static_cast<float>(TIME_TO_CLOSE.count()));
//         }
//         {
//             // std::lock_guard<std::mutex> lock(sim_mtx);
//             current_gate_position += delta;
//             // Clamp the position between 0.0f and 1.0f.
//             if (current_gate_position > 1.0f) {
//                 current_gate_position = 1.0f;
//             } else if (current_gate_position < 0.0f) {
//                 current_gate_position = 0.0f;
//             }
//         }
//         // std::cout << "[Sim] Gate Position updated: " << get_gate_position() * 100.0f << "%" << std::endl;
//         std::this_thread::sleep_for(interval);
//     }
// }