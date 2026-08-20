#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "gen/pose.hpp"

arc::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

arc::Pose arc::Pose::operator+(const arc::Pose& other) const {
    return arc::Pose(this->x + other.x, this->y + other.y, this->theta);
}

arc::Pose arc::Pose::operator-(const arc::Pose& other) const {
    return arc::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float arc::Pose::operator*(const arc::Pose& other) const { return this->x * other.x + this->y * other.y; }

arc::Pose arc::Pose::operator*(const float& other) const {
    return arc::Pose(this->x * other, this->y * other, this->theta);
}

arc::Pose arc::Pose::operator/(const float& other) const {
    return arc::Pose(this->x / other, this->y / other, this->theta);
}

arc::Pose arc::Pose::lerp(arc::Pose other, float t) const {
    return arc::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float arc::Pose::distance(arc::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float arc::Pose::angle(arc::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

arc::Pose arc::Pose::rotate(float angle) const {
    return arc::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string arc::format_as(const arc::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("arc::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
