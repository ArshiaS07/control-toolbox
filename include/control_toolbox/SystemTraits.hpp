// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#pragma once

#include <Eigen/Core>
#include <type_traits>

/**
 * @defgroup ControlToolbox Core Framework
 * @brief Main components of the control systems toolbox
 */

namespace ControlToolbox {

    /**
     * @struct SystemTraits
     * @ingroup ControlToolbox
     * @brief Defines dimensional properties and core types for dynamical systems
     * 
     * @tparam StateDim Dimension of system state (Eigen::Dynamic for dynamic-size systems)
     * @tparam InputDim Dimension of control input (0 for autonomous systems)
     * @tparam OutputDim Dimension of system output (0 for state-only systems)
     *
     * Provides compile-time dimensional checking and type-safe algebraic definitions
     * for control system implementations. Integrates with Eigen for linear algebra
     * operations.
     *
     * @note All dimensions must satisfy:
     * - StateDim: Eigen::Dynamic or > 0
     * - InputDim: >= 0
     * - OutputDim: >= 0
     *
     * Example usage:
     * @code
     * using DoubleIntegratorTraits = SystemTraits<2, 1, 2>;
     * @endcode
     */
    template <int StateDim = Eigen::Dynamic, int InputDim = 0, int OutputDim = 0>
    struct SystemTraits {
        /// @name Dimension Constants
        /// @{
        static constexpr int StateDimension = StateDim;   ///< State space dimension
        static constexpr int InputDimension = InputDim;   ///< Input space dimension
        static constexpr int OutputDimension = OutputDim; ///< Output space dimension
        /// @}

        /// @name Compile-Time Validation
        /// @{
        static_assert(StateDim == Eigen::Dynamic || StateDim > 0, 
            "State dimension must be positive or Eigen::Dynamic");
        static_assert(InputDim >= 0, 
            "Input dimension must be non-negative");
        static_assert(OutputDim >= 0, 
            "Output dimension must be non-negative");
        /// @}

        /// @name Core Types
        /// @{
        /**
         * @typedef StateVector
         * @brief State vector type (Nx1 Eigen column vector)
         * @tparam StateDim Template parameter defining state dimension
         */
        using StateVector = Eigen::Matrix<double, StateDim, 1>;

        /**
         * @typedef InputVector
         * @brief Control input vector type (Mx1 Eigen column vector)
         * @tparam InputDim Template parameter defining input dimension
         * @note Only defined when InputDim > 0
         */
        template <int D = InputDim>
        using InputVector = typename std::enable_if_t<(D > 0), Eigen::Matrix<double, D, 1>>;

        /**
         * @typedef OutputVector
         * @brief System output vector type (Px1 Eigen column vector)
         * @tparam OutputDim Template parameter defining output dimension
         * @note Only defined when OutputDim > 0
         */
        template <int D = OutputDim>
        using OutputVector = typename std::enable_if_t<(D > 0), Eigen::Matrix<double, D, 1>>;
        /// @}

        /// @name System Properties
        /// @{
        static constexpr bool HasInput = (InputDim > 0);      ///< System has control inputs
        static constexpr bool HasOutput = (OutputDim > 0);    ///< System has defined outputs
        static constexpr bool IsDynamic =                     ///< Runtime-sized system flag
            (StateDim == Eigen::Dynamic) ||
            (InputDim == Eigen::Dynamic) ||
            (OutputDim == Eigen::Dynamic);
        /// @}
    };

} // namespace ControlToolbox