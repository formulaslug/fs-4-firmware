#include "lap_counter.hpp"

LapCounter::LapCounter(double lat, double lon) {
    resetLapCounter(lat, lon);
}

void LapCounter::resetLapCounter(double lat, double lon) {
    data = LapCounterData{};

    if (lat != 0.0 || lon != 0.0) {
        data.lap_home_latitude = lat;
        data.lap_home_longitude = lon;
        armed = true;
    } else {
        armed = false;
    }

    gate_nx = 0.0;
    gate_ny = 1.0;
    direction_learned = false;
    has_prev_pos = false;
    lap_start_ms = 0;
    prev_timestamp_ms = 0;
}

void LapCounter::lat_lon_to_xy(double lat, double lon, double& x, double& y) const {
    double lat_rad = data.lap_home_latitude * DEG_TO_RAD;
    x = EARTH_RADIUS * ((lon - data.lap_home_longitude) * DEG_TO_RAD) * std::cos(lat_rad);
    y = EARTH_RADIUS * ((lat - data.lap_home_latitude) * DEG_TO_RAD);
}

void LapCounter::update_time(uint64_t timestamp_ms) {
    if (lap_start_ms == 0) {
        lap_start_ms = timestamp_ms;
    }
    if (timestamp_ms >= lap_start_ms) {
        data.lap_time = static_cast<double>(timestamp_ms - lap_start_ms) / 1000.0;
    }
}

void LapCounter::update_position(double lat, double lon) {
    data.lap_curr_latitude = lat;
    data.lap_curr_longitude = lon;
    lat_lon_to_xy(lat, lon, data.lap_curr_x, data.lap_curr_y);
}

void LapCounter::learn_direction() {
    double dist_sq = data.lap_curr_x * data.lap_curr_x + data.lap_curr_y * data.lap_curr_y;
    if (dist_sq > 4.0) { // Driven > 2 meters from start
        double len = std::sqrt(dist_sq);
        gate_nx = data.lap_curr_x / len;
        gate_ny = data.lap_curr_y / len;
        direction_learned = true;
    }
}

bool LapCounter::check_crossing(uint64_t timestamp_ms) {
    if (!direction_learned || data.lap_time < MIN_LAP_TIME) {
        return false;
    }

    // Normal distance along track forward vector (dot product)
    double d_prev = data.lap_prev_x * gate_nx + data.lap_prev_y * gate_ny;
    double d_curr = data.lap_curr_x * gate_nx + data.lap_curr_y * gate_ny;

    // Lateral distance perpendicular to forward direction
    double lateral_curr = std::abs(-data.lap_curr_x * gate_ny + data.lap_curr_y * gate_nx);

    // Gate crossing check: crossed plane within lateral gate width
    if (d_prev < 0.0 && d_curr >= 0.0 && lateral_curr <= (GATE_WIDTH / 2.0)) {
        record_lap(d_prev, d_curr, timestamp_ms);
        return true;
    }

    return false;
}

void LapCounter::record_lap(double d_prev, double d_curr, uint64_t timestamp_ms) {
    // Sub-sample linear interpolation for millisecond-level lap time precision
    double denom = d_curr - d_prev;
    double fraction = (denom > 1e-6) ? (-d_prev / denom) : 0.0;
    fraction = std::clamp(fraction, 0.0, 1.0);

    uint64_t dt = timestamp_ms - prev_timestamp_ms;
    uint64_t t_cross_ms = prev_timestamp_ms + static_cast<uint64_t>(fraction * static_cast<double>(dt));

    double precise_lap_time = static_cast<double>(t_cross_ms - lap_start_ms) / 1000.0;
    if (precise_lap_time < MIN_LAP_TIME) {
        precise_lap_time = data.lap_time;
    }

    data.lap_counter++;
    data.last_lap_time = precise_lap_time;
    if (data.best_lap_time == 0.0 || precise_lap_time < data.best_lap_time) {
        data.best_lap_time = precise_lap_time;
    }

    lap_start_ms = t_cross_ms;
    data.lap_time = 0.0;
}

void LapCounter::store_prev_position(uint64_t timestamp_ms) {
    data.lap_prev_x = data.lap_curr_x;
    data.lap_prev_y = data.lap_curr_y;
    data.lap_prev_latitude = data.lap_curr_latitude;
    data.lap_prev_longitude = data.lap_curr_longitude;
    prev_timestamp_ms = timestamp_ms;
    has_prev_pos = true;
}

bool LapCounter::updateLapCounter(double lat, double lon, uint64_t timestamp_ms) {
    if (!armed) {
        return false;
    }

    update_time(timestamp_ms);
    update_position(lat, lon);

    if (!has_prev_pos) {
        store_prev_position(timestamp_ms);
        return false;
    }

    if (!direction_learned) {
        learn_direction();
    }

    bool lap_completed = check_crossing(timestamp_ms);
    store_prev_position(timestamp_ms);
    return lap_completed;
}
