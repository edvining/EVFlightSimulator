#pragma once
#include <iostream>
#include <initializer_list>
#include <cmath>
#include <string>

struct triple {
    // Public members
    double x, y, z;

    // Constructors
    triple(double x, double y, double z) : x(x), y(y), z(z) {};
    triple(double x, double y) : x(x), y(y), z(0) {};
    triple() : x(0), y(0), z(0) {};

    // Constructor that allows values to be assigned using initializer list
    triple(std::initializer_list<double> list) {
        if (list.size() == 3) {
            auto it = list.begin();
            x = *it++;
            y = *it++;
            z = *it;
        }
        else if (list.size() == 2) {
            auto it = list.begin();
            x = *it++;
            y = *it++;
            z = 0;
        }
        else {
            // Handle the case where the initializer list does not have exactly 2 or 3 elements
            throw std::invalid_argument("Initializer list must have exactly 2 or 3 elements.");
        }
    }

    // Operator to allow triple to be displayed directly
    friend std::ostream& operator<<(std::ostream& os, const triple& vec) {
        os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
        return os;
    }

    static const triple zero() {
        return triple(0, 0, 0);
    }

    // Vector addition
    triple operator+(const triple& vec2) const {
        return triple(x + vec2.x, y + vec2.y, z + vec2.z);
    }

    // Vector incrementation
    triple& operator+=(const triple& vec2) {
        x += vec2.x;
        y += vec2.y;
        z += vec2.z;
        return *this;
    }

    // Vector subtraction
    triple operator-(const triple& vec2) const {
        return triple(x - vec2.x, y - vec2.y, z - vec2.z);
    }

    // Vector decrementation
    triple& operator-=(const triple& vec2) {
        x -= vec2.x;
        y -= vec2.y;
        z -= vec2.z;
        return *this;
    }

    // Scalar multiplication
    triple operator*(const double& val) const {
        return triple(x * val, y * val, z * val);
    }

    // Scalar multiplication (reverse order)
    friend triple operator*(const double& val, const triple& vec) {
        return triple(vec.x * val, vec.y * val, vec.z * val);
    }

    // Dot product between two vectors
    friend double operator*(const triple& vec1, const triple& vec2) {
        return Dot(vec1, vec2);
    }

    triple& operator*=(const int& val) {
        x *= val;
        y *= val;
        z *= val;
        return *this;
    }

    // Scalar division
    triple operator/(const double& val) {
        double _inv = 1 / val;
        return triple(x * _inv, y * _inv, z * _inv);
    }

    // Scalar division
    friend triple operator/(const triple& vec, const double& val) {
        double _inv = 1 / val;
        return triple(vec.x * _inv, vec.y * _inv, vec.z * _inv);
    }

    // Static function to calculate dot product between two vectors
    static double Dot(const triple& a, const triple& b) {
        return (a.x * b.x + a.y * b.y + a.z * b.z);
    }

    // Static function to calculate dot product between two vectors
    double Dot(const triple& a) {
        return triple::Dot(a, *this);
    }

    // Static function to calculate cross product between two vectors
    static triple Cross(const triple& a, const triple& b) {
        double _x = a.y * b.z - a.z * b.y;
        double _y = a.z * b.x - a.x * b.z;
        double _z = a.x * b.y - a.y * b.x;
        return triple(_x, _y, _z);
    }

    // Cross product between two vectors
    triple operator^(const triple& vec2) const {
        return Cross(*this, vec2);
    }

    // Squared magnitude of the vector
    double SqrMag(const triple& v) const {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    // Squared magnitude of the vector
    double sqrMagnitude() const {
        return x * x + y * y + z * z;
    }

    // Magnitude (length) of the vector
    double magnitude() const {
        return sqrt(sqrMagnitude());
    }

    float magnitudef() const {
        return (float)sqrt(sqrMagnitude());
    }

    // Normalized version of the vector
    triple normalized() const {
        return (*this) / magnitude();
    }

    // Calculate the dot product between this vector and another
    double along(const triple& other) const {
        return Dot(*this, other) / other.magnitude();
    }

    // Calculate the angle (in radians) between this vector and another
    double between(const triple& vec2) const {
        return AngleBetween(*this, vec2);
    }

    // Calculate the projection of this vector onto another
    triple onto(const triple& other) const {
        return Project(*this, other);
    }

    // Static function to calculate the angle between two vectors
    static double AngleBetween(const triple& a, const triple& b) {
        return std::acos((a * b) / (a.magnitude() * b.magnitude()));
    }

    // Static function to calculate the projection of one vector onto another
    static triple Project(const triple& a, const triple& b) {
        return (b * ((a * b) / (b.magnitude() * b.magnitude())));
    }

    // Calculate the reflection of this vector off a surface with a given normal
    triple reflectionOff(const triple& normal) const {
        return *this - (normal * 2 * (*this * normal) / normal.sqrMagnitude());
    }

    // Print the vector components
    void Print() {
        std::cout << "(" << x << ", " << y << ", " << z << ")\n";
    }

    std::string string() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
    }

};
using vec3 = triple;
using color = triple;
using position = triple;