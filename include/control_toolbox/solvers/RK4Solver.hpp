// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#pragma once

#include "SolverBase.hpp"

namespace ControlToolbox {

    /**
     * @class RK4Solver
     * @ingroup ControlToolbox
     * @brief Classical 4th-order Runge-Kutta numerical integrator
     * 
     * @tparam System Type of dynamical system to solve (StateSpaceSystem-based)
     *
     * Implements the standard RK4 method:
     * @f[
     * y_{n+1} = y_n + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)
     * @f]
     * where:
     * - @f$ k_1 = f(t_n, y_n) @f$
     * - @f$ k_2 = f(t_n + h/2, y_n + hk_1/2) @f$
     * - @f$ k_3 = f(t_n + h/2, y_n + hk_2/2) @f$
     * - @f$ k_4 = f(t_n + h, y_n + hk_3) @f$
     *
     * @note Inherits from SolverBase using CRTP pattern
     * @note Only logs data from first stage (k1) to avoid duplicate entries
     */
    template <typename System>
    class RK4Solver : public SolverBase<RK4Solver<System>, System> {
    public:
        /// @name Type Inheritance
        /// @{
        using Base = SolverBase<RK4Solver<System>, System>; ///< Base class type
        using typename Base::StateVector;                   ///< Import state vector type
        using typename Base::TimeType;                      ///< Import time type
        /// @}

        /**
         * @brief Implement RK4 integration step
         * @param[in,out] system System to integrate
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @param[in] dt Time step size (seconds)
         * @return StateVector New state after integration
         *
         * @details Computation strategy:
         * 1. Compute k1 with logging enabled
         * 2. Compute k2-k4 with logging disabled
         * 3. Combine results using RK4 weights
         */
        StateVector step_impl(System& system, TimeType t,
            const StateVector& x, TimeType dt) {
            // First stage (with logging)
            const auto k1 = system.computeSystemDerivative(t, x, true);
            
            // Subsequent stages (no logging)
            const auto k2 = system.computeSystemDerivative(t + dt/2, x + dt*k1/2, false);
            const auto k3 = system.computeSystemDerivative(t + dt/2, x + dt*k2/2, false);
            const auto k4 = system.computeSystemDerivative(t + dt, x + dt*k3, false);

            return x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
        }
    };

} // namespace ControlToolbox