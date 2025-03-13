// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#ifndef STATE_SPACE_SYSTEM_HPP
#define STATE_SPACE_SYSTEM_HPP

#include "SystemTraits.hpp"
#include "Logger.hpp"
#include <Eigen/Dense>
#include <type_traits>

namespace ControlToolbox {

    // Forward declaration for CRTP base
    template <typename Derived, typename Traits>
    class StateSpaceSystem;

    namespace detail {
        /**
         * @internal
         * @brief CRTP implementation base for state-space systems
         * @tparam Derived Derived system implementation
         * @tparam Traits System dimensional traits
         * @tparam HasInput System has control inputs
         * @tparam HasOutput System has defined outputs
         * 
         * Provides specialized implementations based on system capabilities
         */
        template <typename Derived, typename Traits, bool HasInput, bool HasOutput>
        struct StateSpaceSystemImpl;

        /**
         * @internal
         * @brief Full-featured system specialization (input + output)
         * @ingroup ControlToolbox
         */
        template <typename Derived, typename Traits>
        struct StateSpaceSystemImpl<Derived, Traits, true, true> {
            /// @name System Types
            /// @{
            using StateVector = typename Traits::StateVector;     ///< State vector type
            using InputVector = typename Traits::template InputVector<  ///< Input vector type
                Traits::InputDimension>;
            using OutputVector = typename Traits::template OutputVector< ///< Output vector type
                Traits::OutputDimension>;
            /// @}

            /**
             * @brief Compute reference signal for tracking control
             * @param[in] t Current simulation time (seconds)
             * @return OutputVector Reference signal
             * 
             * @note Must be implemented in derived class
             */
            OutputVector computeReference(double t) const {
                return static_cast<const Derived*>(this)->computeReference_impl(t);
            }

            /**
             * @brief Compute control signal using output feedback
             * @param[in] t Current simulation time
             * @param[in] ref Current reference signal
             * @param[in] out System output measurement
             * @param[in] x Current state vector
             * @return InputVector Control signal
             * 
             * @note Derived classes must implement computeController_impl
             */
            InputVector computeController(double t, const OutputVector& ref, 
                                        const OutputVector& out, const StateVector& x) const {
                return static_cast<const Derived*>(this)->computeController_impl(t, ref, out, x);
            }

            // Similar documentation patterns for other methods...
        };

    } // namespace detail

    /**
     * @class StateSpaceSystem
     * @ingroup ControlToolbox
     * @brief CRTP base class for state-space control system implementations
     * 
     * @tparam Derived Concrete system implementation (CRTP derived class)
     * @tparam Traits System traits defining dimensions and types (SystemTraits-based)
     *
     * Provides complete signal processing chain:
     * - Reference generation
     * - Control computation
     * - Disturbance injection
     * - Dynamics calculation
     * - Output processing
     * - Data logging
     *
     * @note Derived classes must implement:
     * - computeDynamics_impl()
     * - computeController_impl()
     * - computeReference_impl() (if HasOutput)
     * - Other optional _impl methods as needed
     */
    template <typename Derived, typename Traits>
    class StateSpaceSystem : public detail::StateSpaceSystemImpl<Derived, Traits,
        Traits::HasInput, Traits::HasOutput> {
    public:
        /// @name System Types
        /// @{
        using StateVector = typename Traits::StateVector;     ///< State vector type
        using InputVector = typename Traits::template InputVector< ///< Input vector type
            Traits::InputDimension>;
        using OutputVector = typename Traits::template OutputVector< ///< Output vector type
            Traits::OutputDimension>;
        using LoggerType = SystemLogger<Traits>;              ///< Data logger type
        using Parameters = typename Traits::Parameters;       ///< System parameters type
        /// @}

        /**
         * @brief Compute system dynamics (dx/dt = f(t, x, u))
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @param[in] u Current input vector
         * @return StateVector State derivative vector
         * 
         * @note Pure virtual in derived classes via computeDynamics_impl()
         */
        StateVector computeDynamics(double t, const StateVector& x, const InputVector& u) const {
            return static_cast<const Derived*>(this)->computeDynamics_impl(t, x, u);
        }

        /**
         * @brief Compute system output (y = g(t, x, u))
         * @tparam B SFINAE enabler for output capability
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @param[in] u Current input vector
         * @return OutputVector System output
         * 
         * @pre System must have outputs (Traits::HasOutput == true)
         */
        template <bool B = Traits::HasOutput, typename = std::enable_if_t<B>>
        OutputVector computeOutput(double t, const StateVector& x, const InputVector& u) const {
            return static_cast<const Derived*>(this)->computeOutput_impl(t, x, u);
        }

        /**
         * @brief Execute complete simulation step with signal processing
         * @param[in] t Current simulation time
         * @param[in] x Current state vector
         * @param[in] log Enable data logging (default: true)
         * @return StateVector State derivative vector
         * 
         * @details Implements full signal processing chain:
         * 1. Reference signal generation (if HasOutput)
         * 2. Previous output handling (one-step delay)
         * 3. Measurement noise injection (if HasOutput)
         * 4. Control signal computation (if HasInput)
         * 5. Input disturbance injection (if HasInput)
         * 6. System dynamics calculation
         * 7. Output update for next step (if HasOutput)
         * 
         * @note Uses CRTP pattern to delegate implementation to derived class
         */
        StateVector computeSystemDerivative(double t, const StateVector& x, bool log = true) {
            SignalFlowData flow;
            flow.timestamp = t;
            flow.state = x;

            // 1. Reference Signal Generation (Tracking Systems)
            if constexpr (Traits::HasOutput) {
                flow.reference = static_cast<Derived*>(this)->computeReference(t);
            }

            // 2. Previous Output Handling (One-Step Delay)
            OutputVector y_clean = OutputVector::Zero();
            if constexpr (Traits::HasOutput) {
                if (previous_output_.size() > 0) {
                    y_clean = previous_output_;
                }
                flow.output_clean = y_clean;
            }

            // 3. Measurement Noise Injection
            OutputVector y_noisy = y_clean;
            if constexpr (Traits::HasOutput) {
                y_noisy = static_cast<Derived*>(this)->addOutputNoise(y_clean, t);
                flow.output_noisy = y_noisy;
            }

            // 4. Control Signal Computation
            InputVector u = InputVector::Zero();
            if constexpr (Traits::HasInput) {
                if constexpr (Traits::HasOutput) {
                    // Tracking Control: u = f(ref, y, x)
                    u = static_cast<Derived*>(this)->computeController(t, flow.reference, y_noisy, x);
                }
                else {
                    // Regulation Control: u = f(x)
                    u = static_cast<Derived*>(this)->computeController(t, x);
                }
                flow.control_input = u;
            }

            // 5. Input Disturbance Injection
            InputVector u_disturbed = u;
            if constexpr (Traits::HasInput) {
                u_disturbed = static_cast<Derived*>(this)->addInputDisturbance(u, t);
                flow.disturbed_input = u_disturbed;
            }

            // 6. System Dynamics Computation
            flow.derivative = static_cast<Derived*>(this)->computeDynamics(t, x, u_disturbed);

            // 7. Current Output Update (For Next Step)
            if constexpr (Traits::HasOutput) {
                previous_output_ = static_cast<Derived*>(this)->computeOutput(t, x, u);
            }

            // Data Logging
            if (log) {
                struct LoggerType::LogEntry entry;
                entry.timestamp = flow.timestamp;
                entry.state = flow.state;
                entry.derivative = flow.derivative;
                entry.control_input = flow.control_input;
                entry.disturbed_input = flow.disturbed_input;
                entry.reference = flow.reference;
                entry.output_clean = flow.output_clean;
                entry.output_noisy = flow.output_noisy;
                logger_.log(entry);
            }

            return flow.derivative;
        }

        /// @name Data Access
        /// @{
        /**
         * @brief Access system logger (const version)
         * @return const LoggerType& Immutable logger reference
         */
        const LoggerType& getLogger() const { return logger_; }
        
        /**
         * @brief Access system logger (mutable version)
         * @return LoggerType& Mutable logger reference
         */
        LoggerType& getLogger() { return logger_; }
        /// @}

    protected:
        /// @name Signal Processing Hooks
        /// @{
        /**
         * @brief Add output measurement noise
         * @param[in] y Clean output vector
         * @param[in] t Current simulation time
         * @return OutputVector Noisy output
         * 
         * @note Default implementation returns clean output. Override in derived class
         *       to add noise models
         */
        OutputVector addOutputNoise(const OutputVector& y, double) const { return y; }

        /**
         * @brief Add input disturbances
         * @param[in] u Nominal control input
         * @param[in] t Current simulation time
         * @return InputVector Disturbed input
         * 
         * @note Default implementation returns nominal input. Override in derived class
         *       to add disturbance models
         */
        InputVector addInputDisturbance(const InputVector& u, double) const { return u; }
        /// @}

        Parameters parameters_; ///< System parameter storage

    private:
        /// @internal
        /// @brief Signal flow tracking structure
        struct SignalFlowData {
            double timestamp;          ///< Simulation time (s)
            StateVector state;          ///< Current state
            StateVector derivative;     ///< State derivative
            InputVector control_input;  ///< Computed control
            InputVector disturbed_input; ///< Disturbed input
            OutputVector reference;     ///< Current reference
            OutputVector output_clean;  ///< Ideal output
            OutputVector output_noisy;  ///< Measured output
        };

        LoggerType logger_;             ///< Data logging subsystem
        OutputVector previous_output_ = OutputVector::Zero(); ///< Delayed output buffer
    };

} // namespace ControlToolbox

#endif // STATE_SPACE_SYSTEM_HPP