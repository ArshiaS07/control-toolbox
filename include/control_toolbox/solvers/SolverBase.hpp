// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#pragma once

#include "control_toolbox/SystemTraits.hpp"
#include <functional>
#include <memory>

namespace ControlToolbox {

    /**
     * @class SolverBase
     * @ingroup ControlToolbox
     * @brief CRTP base class for numerical integration algorithms
     * 
     * @tparam Derived Concrete solver implementation (CRTP derived class)
     * @tparam System Type of dynamical system to solve (StateSpaceSystem-based)
     *
     * Provides common interface for:
     * - Time-stepping algorithms
     * - Simulation control mechanisms
     * - Custom stop conditions
     * - Configuration management
     *
     * @note Derived classes must implement step_impl()
     */
    template <typename Derived, typename System>
    class SolverBase {
    public:
        /// @name Type Aliases
        /// @{
        using StateVector = typename System::StateVector; ///< State vector type from system
        using TimeType = double;                          ///< Time representation type
        /// @}

        /**
         * @brief Execute single integration step
         * @param[in,out] system System to integrate
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @param[in] dt Time step size (seconds)
         * @return StateVector New state after integration
         *
         * @note CRTP requirement: Derived classes must implement step_impl()
         */
        StateVector step(System& system, TimeType t, const StateVector& x, TimeType dt) {
            return static_cast<Derived*>(this)->step_impl(system, t, x, dt);
        }

        /// @name Solver Configuration
        /// @{
        
        /**
         * @brief Set custom simulation stop condition
         * @param[in] condition Function evaluating stop condition:
         *                      `bool(TimeType t, const StateVector& x)`
         *
         * @warning Overrides default time-based stopping
         */
        void set_stop_condition(std::function<bool(TimeType, const StateVector&)> condition) {
            stop_condition_ = std::move(condition);
        }

        /**
         * @brief Set maximum simulation duration
         * @param[in] max_time Maximum simulation time (seconds)
         */
        void set_max_time(TimeType max_time) { max_time_ = max_time; }
        /// @}

        /// @name Execution Control
        /// @{
        
        /**
         * @brief Evaluate stop conditions
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @return true if simulation should stop, false otherwise
         *
         * Evaluation order:
         * 1. Custom stop condition (if set)
         * 2. Default time-based condition
         */
        bool should_stop(TimeType t, const StateVector& x) const {
            return stop_condition_ ? stop_condition_(t, x) : default_stop(t);
        }
        /// @}

        virtual ~SolverBase() = default;

    protected:
        /**
         * @brief Default time-based stop condition
         * @param[in] t Current simulation time
         * @return true if t > max_time_
         */
        bool default_stop(TimeType t) const { return t > max_time_; }

        /// @name Solver State
        /// @{
        std::function<bool(TimeType, const StateVector&)> stop_condition_; ///< Custom stop condition
        TimeType max_time_ = 10.0; ///< Default maximum simulation time (seconds)
        /// @}
    };

} // namespace ControlToolbox