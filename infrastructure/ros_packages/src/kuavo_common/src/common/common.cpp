#include "kuavo_common/common/common.h"
#include <iostream>
#include <algorithm>
#include <regex>
#include <stdexcept>

RobotVersion::RobotVersion(uint16_t major, uint16_t minor, uint16_t patch /* = 0 */) : Major(major), Minor(minor), Patch(patch) {
    if (Major > 9999 || Minor > 9 || Patch > 9999) {
        throw std::invalid_argument("RobotVersion:: Invalid version numbers: " + std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch));
    }
}

static void _extract_major_minor_patch(int big_number, uint16_t &major, uint16_t &minor, uint16_t &patch) {
    // PPPPMMMMN
    minor = big_number % 10;
    big_number /= 10;
    major = big_number % 10000;
    patch = big_number / 10000;
}

RobotVersion RobotVersion::create(int big_number) {
    if (!is_valid(big_number)) {
        throw std::invalid_argument("RobotVersion::create Invalid version string: " + std::to_string(big_number));
    }
    uint16_t major = 0, minor = 0, patch = 0;
    _extract_major_minor_patch(big_number, major, minor, patch);
    return RobotVersion(major, minor, patch);
}

bool RobotVersion::is_valid(int big_number) {
    uint16_t major = 0, minor = 0, patch = 0;
    _extract_major_minor_patch(big_number, major, minor, patch);
    if (major > 9999 || minor > 9 || patch > 9999) {
        return false;
    }
    return true;
}

bool RobotVersion::start_with(uint16_t major) const {
    return Major == major;
}

bool RobotVersion::start_with(uint16_t major, uint16_t minor) const {
    return Major == major && Minor == minor;
}

uint32_t RobotVersion::version_number() const {
    return Minor  + Major * 10 + Patch*100000;
}

std::string RobotVersion::version_name() const {
    return std::to_string(Major) + "." + std::to_string(Minor) + "." + std::to_string(Patch);
}

uint16_t RobotVersion::major() const {
    return Major;
}

uint16_t RobotVersion::minor() const {
    return Minor;
}

uint16_t RobotVersion::patch() const {
    return Patch;
}

std::string RobotVersion::to_string() const {
   return std::to_string(version_number());
}

bool RobotVersion::operator==(const RobotVersion &other) const { 
    return Major == other.Major && Minor == other.Minor && Patch == other.Patch; 
}

bool RobotVersion::operator!=(const RobotVersion &other) const { 
    return !(*this == other); 
}

bool RobotVersion::operator<(const RobotVersion &other) const { 
    return Major < other.Major || 
           (Major == other.Major && Minor < other.Minor) || 
           (Major == other.Major && Minor == other.Minor && Patch < other.Patch); 
}

std::ostream& operator<<(std::ostream &os, const RobotVersion &version) { 
    return os << "RobotVersion(" << version.to_string() << ")"; 
}
