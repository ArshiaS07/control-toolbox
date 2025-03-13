// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#pragma once

#include "control_toolbox/solvers/SolverBase.hpp"
#include "Logger.hpp"
#include <vector>
#include <memory>

namespace ControlToolbox {

    /**
     * @class Simulation
     * @ingroup ControlToolbox
     * @brief Manages complete simulation execution and data collection
     * 
     * @tparam System Dynamical system type (StateSpaceSystem-based)
     * @tparam Solver Numerical integrator type (SolverBase-based)
     *
     * Coordinates the interaction between:
     * - Dynamical system model
     * - Numerical integration method
     * - Data logging infrastructure
     *
     * @note Uses fixed-time stepping scheme
     * @note Maintains ownership of system and solver instances
     */
    template <typename System, typename Solver>
    class Simulation {
    public:
        /// @name Type Aliases
        /// @{
        using StateVector = typename System::StateVector; ///< State vector type from system
        using TimeType = typename Solver::TimeType;       ///< Time representation type from solver
        /// @}

        /**
         * @brief Construct simulation manager
         * @param[in] system System instance to simulate (must be non-null)
         * @param[in] solver Numerical solver instance (must be non-null)
         * @param[in] initial_state Initial condition vector
         * @param[in] initial_time Starting simulation time (default: 0.0)
         *
         * @warning System and solver instances must outlive the simulation
         */
        Simulation(std::shared_ptr<System> system,
                  std::shared_ptr<Solver> solver,
                  StateVector initial_state,
                  TimeType initial_time = 0.0)
            : system_(std::move(system)),
              solver_(std::move(solver)),
              state_(std::move(initial_state)),
              time_(initial_time) {
        }

        /**
         * @brief Execute complete simulation
         * @param[in] dt Fixed time step size (seconds)
         *
         * @details Simulation loop steps:
         * 1. Check stop conditions
         * 2. Log current state through system derivative call
         * 3. Perform integration step
         * 4. Advance simulation time
         *
         * @note Logging is triggered via system's computeSystemDerivative
         */
        void run(TimeType dt) {
            while (!solver_->should_stop(time_, state_)) {
                // Store previous state for potential logging
                const auto prev_state = state_;

                // Integration step (triggers logging internally)
                state_ = solver_->step(*system_, time_, state_, dt);

                time_ += dt;
            }
        }

        /// @name Simulation State Access
        /// @{
        
        /**
         * @brief Access simulation data logger
         * @return const-reference to logger instance
         */
        const auto& get_logger() const { return system_->getLogger(); }

        /**
         * @brief Get current simulation time
         * @return TimeType Current simulation time (seconds)
         */
        TimeType current_time() const { return time_; }

        /**
         * @brief Get current system state
         * @return const StateVector& Current state vector
         */
        const StateVector& current_state() const { return state_; }
        /// @}

    private:
        /// @name Simulation Components
        /// @{
        std::shared_ptr<System> system_; ///< Managed system instance
        std::shared_ptr<Solver> solver_; ///< Managed solver instance
        /// @}

        /// @name Simulation State
        /// @{
        StateVector state_; ///< Current system state
        TimeType time_;     ///< Current simulation time
        /// @}
    };

} // namespace ControlToolbox