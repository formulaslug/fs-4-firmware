#pragma once

#include <cstdint>
#include <cmath>
#include <numbers>
#include <algorithm>

struct LapCounterData {
    double lap_home_latitude{0.0};
    double lap_home_longitude{0.0};

    double lap_prev_latitude{0.0};
    double lap_prev_longitude{0.0};
    double lap_prev_x{0.0};                 // relative to local coordinate plane (m)
    double lap_prev_y{0.0};                 // relative to local coordinate plane (m)

    double lap_curr_latitude{0.0};
    double lap_curr_longitude{0.0};
    double lap_curr_x{0.0};                 // relative to local coordinate plane (m)
    double lap_curr_y{0.0};                 // relative to local coordinate plane (m)

    uint16_t lap_counter{0};
    double lap_time{0.0};
    double last_lap_time{0.0};
    double best_lap_time{0.0};
};

class LapCounter {
public:
    static constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;
    static constexpr double EARTH_RADIUS = 6378137.0; // Earth radius in meters
    static constexpr double GATE_WIDTH = 16.0;        // Lateral tolerance (+/- 8 meters)
    static constexpr double MIN_LAP_TIME = 10.0;      // Minimum lap time in seconds

    LapCounterData data;

    LapCounter(double lat = 0.0, double lon = 0.0);

    void resetLapCounter(double lat = 0.0, double lon = 0.0);
    bool updateLapCounter(double lat, double lon, uint64_t timestamp_ms);
    double getTime() const { return data.lap_time; }

private:
    void lat_lon_to_xy(double lat, double lon, double& x, double& y) const;
    void update_time(uint64_t timestamp_ms);
    void update_position(double lat, double lon);
    void learn_direction();
    bool check_crossing(uint64_t timestamp_ms);
    void record_lap(double d_prev, double d_curr, uint64_t timestamp_ms);
    void store_prev_position(uint64_t timestamp_ms);

    double gate_nx{0.0};
    double gate_ny{1.0};
    bool direction_learned{false};
    bool has_prev_pos{false};
    bool armed{false};
    uint64_t lap_start_ms{0};
    uint64_t prev_timestamp_ms{0};
};
