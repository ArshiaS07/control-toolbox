// examples/MassSpringDamper/MassSpringDamper.cpp

#define _USE_MATH_DEFINES // For M_PI
#include <cmath>
#include <iostream>
#include <memory>

#include "control_toolbox/SystemTraits.hpp"
#include "control_toolbox/StateSpaceSystem.hpp"
#include "control_toolbox/solvers/RK4Solver.hpp"
#include "control_toolbox/Simulation.hpp"

namespace ControlToolbox {

/**
 * @struct MassSpringDamperTraits
 * @brief System traits for mass-spring-damper system
 * @ingroup Examples
 * 
 * @tparam StateDim 2 (position and velocity)
 * @tparam InputDim 1 (control force)
 * @tparam OutputDim 1 (position measurement)
 */
struct MassSpringDamperTraits : public SystemTraits<2, 1, 1> {
    struct Parameters {
        double m = 1.0;  ///< Mass [kg]
        double k = 1.0;  ///< Spring constant [N/m]
        double c = 0.5; ///< Damping coefficient [Ns/m]
    };
};

/**
 * @class MassSpringDamperSystem
 * @brief Mass-spring-damper system implementation with PID control
 * @ingroup Examples
 *
 * Implements:
 * - System dynamics
 * - PID controller with derivative filtering
 * - Sinusoidal reference tracking
 */
class MassSpringDamperSystem : public StateSpaceSystem<MassSpringDamperSystem, MassSpringDamperTraits> {
public:
    MassSpringDamperSystem() = default;

    // CRTP implementation
    OutputVector computeReference_impl(double t) const;
    InputVector computeController_impl(double t, const OutputVector& ref, 
                                     const OutputVector& out, const StateVector& x) const;
    OutputVector computeOutput_impl(double t, const StateVector& x, const InputVector& u) const;
    StateVector computeDynamics_impl(double t, const StateVector& x, const InputVector& u) const;

private:
    /// @name System Parameters
    /// @{
    struct ReferenceSignal {
        double amplitude = 5.0;  ///< Reference amplitude [m]
        double frequency = 2.0;  ///< Reference frequency [rad/s]
    } refSignal_;

    struct ControllerParams {
        double kp = 1.0;       ///< Proportional gain
        double ki = 1.0;       ///< Integral gain
        double kd = 1.0;       ///< Derivative gain
        double dt = 1e-3/4.0;  ///< Controller timestep [s]
        double N = 100.0;      ///< Derivative filter coefficient
        mutable double prevError = 0;
        mutable double integralError = 0;
        mutable double filteredDerivative = 0;
    } ctrlParams_;
    /// @}
};


MassSpringDamperSystem::OutputVector MassSpringDamperSystem::computeReference_impl(double t) const
{
    /**
     * @brief Generates sinusoidal reference signal
     * 
     * @param t Current simulation time (seconds)
     * @return OutputVector Reference position [m]
     * 
     * @details Implements:
     * @f[
     * r(t) = A \sin(\omega t)
     * @f]
     * Where:
     * - @f$ A @f$ = refSignal_.amplitude
     * - @f$ \omega @f$ = refSignal_.frequency [rad/s]
     */
    return OutputVector{refSignal_.amplitude * std::sin(refSignal_.frequency * t)};
}

MassSpringDamperSystem::InputVector MassSpringDamperSystem::computeController_impl(
    double t, const OutputVector& ref, const OutputVector& out, const StateVector& x) const
{
    /**
     * @brief Implements discrete-time PID controller with derivative filtering
     * 
     * @param t Current simulation time
     * @param ref Reference signal vector
     * @param out System output vector
     * @param x Current state vector
     * @return InputVector Control force [N]
     * 
     * @details Control law implementation:
     * 1. Error calculation: @f$ e = r - y @f$
     * 2. Filtered derivative using Tustin approximation:
     * @f[
     * d_k = \frac{2Nk_d(e_k - e_{k-1}) - (N\Delta t - 2)d_{k-1}}{N\Delta t + 2}
     * @f]
     * 3. Integral term using forward Euler:
     * @f[
     * i_k = i_{k-1} + k_i e_k \Delta t
     * @f]
     * 4. PID output: @f$ u = k_p e + d_k + i_k @f$
     */
    
    // 1. Error calculation
    const double error = ref[0] - out[0];

    // 2. Filtered derivative term (Tustin discretization)
    const double delta_error = error - ctrlParams_.prevError;
    const double numerator = 2.0 * ctrlParams_.N * ctrlParams_.kd * delta_error;
    const double denominator = ctrlParams_.N * ctrlParams_.dt + 2.0;
    const double derivative_term = (numerator - (ctrlParams_.N * ctrlParams_.dt - 2.0) * ctrlParams_.filteredDerivative)
                                 / denominator;

    // 3. Integral term (Forward Euler)
    const double error_integral = error * ctrlParams_.dt + ctrlParams_.integralError;

    // 4. PID output
    const double u = ctrlParams_.kp * error + derivative_term + ctrlParams_.ki * error_integral;

    // Update controller states
    ctrlParams_.filteredDerivative = derivative_term;
    ctrlParams_.prevError = error;
    ctrlParams_.integralError = error_integral;

    return InputVector{u};
}

MassSpringDamperSystem::OutputVector MassSpringDamperSystem::computeOutput_impl(
    double t, const StateVector& x, const InputVector& u) const
{
    /**
     * @brief Outputs system position measurement
     * 
     * @param x Current state vector [position, velocity]
     * @return OutputVector Measured position [m]
     */
    return OutputVector{x[0]};
}

MassSpringDamperSystem::StateVector MassSpringDamperSystem::computeDynamics_impl(
    double t, const StateVector& x, const InputVector& u) const
{
    /**
     * @brief Implements mass-spring-damper dynamics
     * 
     * @param x State vector [position (m), velocity (m/s)]
     * @param u Control force [N]
     * @return StateVector State derivatives [velocity, acceleration]
     * 
     * @details Dynamics equations:
     * @f[
     * \begin{cases}
     * \dot{x}_1 = x_2 \\
     * \dot{x}_2 = \frac{1}{m}(u - c x_2 - k x_1)
     * \end{cases}
     * @f]
     */
    StateVector dxdt;
    dxdt[0] = x[1];  // Velocity
    dxdt[1] = (u[0] - parameters_.c * x[1] - parameters_.k * x[0]) / parameters_.m;
    return dxdt;
}

} // namespace ControlToolbox

/**
 * @brief Main function for mass-spring-damper validation case
 * 
 * Generates simulation data comparable with MATLAB results
 * Outputs CSV to validation/data/cpp/MassSpringDamper.csv
 */
int main() {
    using namespace ControlToolbox;
    
    auto solver = std::make_shared<RK4Solver<MassSpringDamperSystem>>();
    solver->set_max_time(10.0);

    auto system = std::make_shared<MassSpringDamperSystem>();
    MassSpringDamperTraits::StateVector x0 = {0.0, 0.0};

    Simulation<MassSpringDamperSystem, RK4Solver<MassSpringDamperSystem>> 
        simulator(system, solver, x0, 0.0);

    simulator.run(0.001);
    
    // Recommended output path
    const std::string output_path = "../../../validation/data/cpp/MassSpringDamper.csv";
    simulator.get_logger().export_csv(output_path);

    std::cout << "Simulation completed. Data saved to: " << output_path << "\n";
}