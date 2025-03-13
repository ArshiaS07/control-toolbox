// SPDX-FileCopyrightText: 2025 Arshia Saffari
// SPDX-License-Identifier: GNU GPLv3
#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "SystemTraits.hpp"
#include <Eigen/Core>
#include <vector>
#include <fstream>
#include <algorithm>
#include <stdexcept>
#include <filesystem>

namespace ControlToolbox {

    /**
     * @class SystemLogger
     * @ingroup ControlToolbox
     * @brief Comprehensive data logging and analysis utility for dynamical systems
     * 
     * @tparam Traits System traits type defining dimensional properties (SystemTraits-based)
     *
     * Provides:
     * - Time-series data recording at multiple signal points
     * - Temporal query operations
     * - CSV export/import capabilities
     * - Basic signal analysis functions
     *
     * @note Requires C++17 or later for filesystem support
     */
    template <typename Traits>
    class SystemLogger {
    public:
        /// @name Type Aliases
        /// @{
        using StateVector = typename Traits::StateVector;                   ///< State vector type
        using InputVector = typename Traits::template InputVector<         ///< Input vector type
            Traits::InputDimension>;
        using OutputVector = typename Traits::template OutputVector<        ///< Output vector type
            Traits::OutputDimension>;
        /// @}

        /**
         * @struct LogEntry
         * @brief Complete system snapshot at a specific timestep
         */
        struct LogEntry {
            double timestamp;          ///< Simulation time (seconds)
            StateVector state;         ///< Current system state (x)
            StateVector derivative;    ///< State derivatives (dx/dt)
            InputVector control_input; ///< Computed control signal (u_cmd)
            InputVector disturbed_input; ///< Actual input with disturbances (u_actual)
            OutputVector reference;    ///< Reference signal (r)
            OutputVector output_clean; ///< Ideal sensor output (y_ideal)
            OutputVector output_noisy; ///< Noisy sensor output (y_measured)
        };

        /// @name Data Recording Interface
        /// @{
        
        /**
         * @brief Record a new system state
         * @param entry Complete system snapshot to log
         * @throws std::invalid_argument If timestamps are non-monotonic
         */
        void log(const LogEntry& entry) {
            if (!entries_.empty() && entry.timestamp <= entries_.back().timestamp) {
                throw std::invalid_argument("Log entries must be monotonically increasing in time");
            }
            entries_.push_back(entry);
        }

        /**
         * @brief Access complete log history
         * @return Const reference to internal log storage
         */
        const std::vector<LogEntry>& entries() const { return entries_; }
        
        /// @}

        /// @name Data Retrieval Interface
        /// @{
        
        /**
         * @brief Access log entry by index
         * @param index Zero-based entry index
         * @return Const reference to requested entry
         * @throws std::out_of_range For invalid indices
         */
        const LogEntry& get_by_index(size_t index) const {
            return entries_.at(index);
        }

        /**
         * @brief Find entry by exact timestamp match
         * @param t Target simulation time
         * @return Const reference to matching entry
         * @throws std::out_of_range If no exact match found
         */
        const LogEntry& get_by_time(double t) const {
            auto it = std::lower_bound(entries_.begin(), entries_.end(), t,
                [](const LogEntry& e, double t) { return e.timestamp < t; });

            if (it != entries_.end() && it->timestamp == t) {
                return *it;
            }
            throw std::out_of_range("No log entry at requested time");
        }
        
        /// @}

        /// @name Data Export Interface
        /// @{
        
        /**
         * @brief Export logged data to CSV format
         * @param filename Output file path
         * @throws std::runtime_error On file open failure
         *
         * Generates a CSV with columns:
         * - Time, State components (x0, x1...), Derivatives (dx0, dx1...),
         * - Control inputs (u0, u1...), Disturbed inputs (u_dist0...),
         * - References (ref0...), Clean outputs (y_clean0...), Noisy outputs (y_noisy0...)
         */
        void export_csv(const std::filesystem::path& filename) const {
            std::ofstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("Failed to open file for writing");
            }

            write_header(file);
            for (const auto& entry : entries_) {
                write_entry(file, entry);
            }
        }

        /// @}

        /// @name Temporal Analysis
        /// @{
        
        /**
         * @brief Get simulation time boundaries
         * @return Pair of [start_time, end_time]
         */
        std::pair<double, double> time_range() const {
            if (entries_.empty()) {
                return { 0.0, 0.0 };
            }
            return { entries_.front().timestamp, entries_.back().timestamp };
        }

        /**
         * @brief Extract state component history in time window
         * @param idx State vector component index
         * @param t_start Start time (inclusive)
         * @param t_end End time (inclusive)
         * @return Vector of component values in specified range
         */
        std::vector<double> get_state_component(size_t idx, double t_start, double t_end) const {
            std::vector<double> values;
            for (const auto& entry : entries_) {
                if (entry.timestamp >= t_start && entry.timestamp <= t_end) {
                    values.push_back(entry.state[idx]);
                }
            }
            return values;
        }

        /**
         * @brief Calculate peak-to-peak amplitude of state component
         * @param state_idx State vector component index
         * @param t_start Start time (inclusive)
         * @param t_end End time (inclusive)
         * @return Amplitude (max - min) in specified range
         */
        double calculate_amplitude(size_t state_idx, double t_start, double t_end) const {
            auto values = get_state_component(state_idx, t_start, t_end);
            if (values.empty()) return 0.0;

            auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
            return *max_it - *min_it;
        }
        
        /// @}

    private:
        std::vector<LogEntry> entries_;

        /// @name CSV Implementation Details
        /// @{
        
        void write_header(std::ostream& os) const {
            os << "time";

            // State components
            for (int i = 0; i < Traits::StateDimension; ++i)
                os << ",x" << i;

            // State derivatives
            for (int i = 0; i < Traits::StateDimension; ++i)
                os << ",dx" << i;

            // Input components
            if constexpr (Traits::InputDimension > 0) {
                for (int i = 0; i < Traits::InputDimension; ++i)
                    os << ",u_cmd" << i;

                for (int i = 0; i < Traits::InputDimension; ++i)
                    os << ",u_actual" << i;
            }

            // Output components
            if constexpr (Traits::OutputDimension > 0) {
                for (int i = 0; i < Traits::OutputDimension; ++i)
                    os << ",ref" << i;

                for (int i = 0; i < Traits::OutputDimension; ++i)
                    os << ",y_ideal" << i;

                for (int i = 0; i < Traits::OutputDimension; ++i)
                    os << ",y_measured" << i;
            }

            os << "\n";
        }

        void write_entry(std::ostream& os, const LogEntry& entry) const {
            os << std::setprecision(14) << entry.timestamp;

            // State
            for (int i = 0; i < Traits::StateDimension; ++i)
                os << "," << entry.state[i];

            // Derivatives
            for (int i = 0; i < Traits::StateDimension; ++i)
                os << "," << entry.derivative[i];

            // Inputs
            if constexpr (Traits::InputDimension > 0) {
                for (int i = 0; i < Traits::InputDimension; ++i)
                    os << "," << entry.control_input[i];

                for (int i = 0; i < Traits::InputDimension; ++i)
                    os << "," << entry.disturbed_input[i];
            }

            // Outputs
            if constexpr (Traits::OutputDimension > 0) {
                for (int i = 0; i < Traits::OutputDimension; ++i)
                    os << "," << entry.reference[i];

                for (int i = 0; i < Traits::OutputDimension; ++i)
                    os << "," << entry.output_clean[i];

                for (int i = 0; i < Traits::OutputDimension; ++i)
                    os << "," << entry.output_noisy[i];
            }

            os << "\n";
        }
        
        /// @}
    };

} // namespace ControlToolbox

#endif // LOGGER_HPP