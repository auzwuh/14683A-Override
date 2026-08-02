#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "gen/pose.hpp"

gen::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

gen::Pose gen::Pose::operator+(const gen::Pose& other) const {
    return gen::Pose(this->x + other.x, this->y + other.y, this->theta);
}

gen::Pose gen::Pose::operator-(const gen::Pose& other) const {
    return gen::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float gen::Pose::operator*(const gen::Pose& other) const { return this->x * other.x + this->y * other.y; }

gen::Pose gen::Pose::operator*(const float& other) const {
    return gen::Pose(this->x * other, this->y * other, this->theta);
}

gen::Pose gen::Pose::operator/(const float& other) const {
    return gen::Pose(this->x / other, this->y / other, this->theta);
}

gen::Pose gen::Pose::lerp(gen::Pose other, float t) const {
    return gen::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float gen::Pose::distance(gen::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float gen::Pose::angle(gen::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

gen::Pose gen::Pose::rotate(float angle) const {
    return gen::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string gen::format_as(const gen::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("gen::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
