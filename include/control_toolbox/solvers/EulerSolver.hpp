// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#pragma once

#include "SolverBase.hpp"

namespace ControlToolbox {

    /**
     * @class EulerSolver
     * @ingroup ControlToolbox
     * @brief Explicit Euler method numerical integrator
     * 
     * @tparam System Type of dynamical system to solve (StateSpaceSystem-based)
     *
     * Implements the forward Euler method:
     * @f[
     * y_{n+1} = y_n + h \cdot f(t_n, y_n)
     * @f]
     * where:
     * - @f$ h @f$ is the time step (dt)
     * - @f$ f @f$ is the system derivative function
     *
     * @note Inherits from SolverBase using CRTP pattern
     * @note While simple to implement, consider using higher-order methods
     *       (like RK4) for better accuracy in most applications
     */
    template <typename System>
    class EulerSolver : public SolverBase<EulerSolver<System>, System> {
    public:
        /// @name Type Inheritance
        /// @{
        using Base = SolverBase<EulerSolver<System>, System>; ///< Base class type
        using typename Base::StateVector;                     ///< Import state vector type
        using typename Base::TimeType;                        ///< Import time type
        /// @}

        /**
         * @brief Implement Euler integration step
         * @param[in,out] system System to integrate
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @param[in] dt Time step size (seconds)
         * @return StateVector New state after integration
         *
         * @details Single-stage computation:
         * 1. Compute derivative at current state
         * 2. Apply Euler update rule
         */
        StateVector step_impl(System& system, TimeType t,
            const StateVector& x, TimeType dt) {
            const auto dx = system.computeSystemDerivative(t, x, true);
            return x + dt * dx;
        }
    };

} // namespace ControlToolbox