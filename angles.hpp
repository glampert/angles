
// ================================================================================================
// -*- C++ -*-
// File: angles.hpp
// Created on: 10/12/16
//
// Brief:
//  Strongly-typed angle classes for radians and degrees.
//  github.com/glampert/angles
//
// License:
//  MIT - Copyright (c) 2016 Guilherme Lampert
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included
//  in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
//  OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// ================================================================================================

#ifndef ANGLES_HPP
#define ANGLES_HPP

//
// Macro switches:
//
// #define ANGLES_ASSERT_360_RANGE
//   Asserts that values are always within:
//   [-360..360] range for degrees
//   [-2pi..2pi] range for radians
//
// #define ANGLES_DEFAULT_ALIASES
//   Defines some shorthand aliases for common
//   types such as float, double and long double.
//

#include <cmath>
#ifdef ANGLES_ASSERT_360_RANGE
#include <cassert>
#endif // ANGLES_ASSERT_360_RANGE

// ========================================================
// Forward declarations / common aliases:
// ========================================================

template<typename ScalarType> class Radians;
template<typename ScalarType> class Degrees;

#ifdef ANGLES_DEFAULT_ALIASES
using RadiansF  = Radians<float>;
using RadiansD  = Radians<double>;
using RadiansLD = Radians<long double>;
using DegreesF  = Degrees<float>;
using DegreesD  = Degrees<double>;
using DegreesLD = Degrees<long double>;
#endif // ANGLES_DEFAULT_ALIASES

// ========================================================
// class AngleBase:
// ========================================================

// Operators and methods shared by both Degrees and Radians. Used as CRTP.
template<typename DerivedType, typename ScalarType>
class AngleBase
{
public:

    //
    // Helper constants:
    //
    #ifdef M_PI
    static constexpr auto Pi         = ScalarType(M_PI);
    #else // !M_PI
    static constexpr auto Pi         = ScalarType(3.1415926535897931);
    #endif // M_PI
    static constexpr auto TwoPi      = ScalarType(2.0) * Pi;
    static constexpr auto DegToRad   = Pi / ScalarType(180.0);
    static constexpr auto RadToDeg   = ScalarType(180.0) / Pi;
    static constexpr auto Zero       = ScalarType(0.0);
    static constexpr auto One        = ScalarType(1.0);
    static constexpr auto OneEighty  = ScalarType(180.0);
    static constexpr auto ThreeSixty = ScalarType(360.0);

    //
    // Add, subtract and negate angles:
    //
    DerivedType operator - () const
    {
        return DerivedType{ -m_angle };
    }
    DerivedType operator + (const DerivedType & other) const
    {
        return DerivedType{ m_angle + other.m_angle };
    }
    DerivedType operator - (const DerivedType & other) const
    {
        return DerivedType{ m_angle - other.m_angle };
    }
    DerivedType & operator += (const DerivedType & other)
    {
        setScalarValue(m_angle + other.m_angle);
        return static_cast<DerivedType &>(*this);
    }
    DerivedType & operator -= (const DerivedType & other)
    {
        setScalarValue(m_angle - other.m_angle);
        return static_cast<DerivedType &>(*this);
    }

    //
    // Multiply and divide angle by a scalar value:
    //
    DerivedType operator * (const ScalarType scalar) const
    {
        return DerivedType{ m_angle * scalar };
    }
    DerivedType operator / (const ScalarType scalar) const
    {
        return DerivedType{ m_angle / scalar };
    }
    DerivedType & operator *= (const ScalarType scalar)
    {
        setScalarValue(m_angle * scalar);
        return static_cast<DerivedType &>(*this);
    }
    DerivedType & operator /= (const ScalarType scalar)
    {
        setScalarValue(m_angle / scalar);
        return static_cast<DerivedType &>(*this);
    }

    //
    // Expose the built-in comparison operators:
    //
    bool operator == (const DerivedType & other) const { return m_angle == other.m_angle; }
    bool operator != (const DerivedType & other) const { return m_angle != other.m_angle; }
    bool operator <= (const DerivedType & other) const { return m_angle <= other.m_angle; }
    bool operator >= (const DerivedType & other) const { return m_angle >= other.m_angle; }
    bool operator  < (const DerivedType & other) const { return m_angle  < other.m_angle; }
    bool operator  > (const DerivedType & other) const { return m_angle  > other.m_angle; }

    //
    // Access the underlaying scalar value:
    //
    ScalarType getScalarValue() const
    {
        return m_angle;
    }
    void setScalarValue(const ScalarType newAngle)
    {
        #ifdef ANGLES_ASSERT_360_RANGE
        assert(DerivedType::isValidAngle(newAngle));
        #endif // ANGLES_ASSERT_360_RANGE
        m_angle = newAngle;
    }

protected:

    AngleBase() : m_angle(Zero) { }

    // No virtual inheritance - used for composition only.
    ~AngleBase() = default;

private:

    // The actual angle. Interpreted as radians or degrees,
    // depending on the class that implements this.
    ScalarType m_angle;
};

// ========================================================
// class Radians:
// ========================================================

template<typename ScalarType>
class Radians final
    : public AngleBase<Radians<ScalarType>, ScalarType>
{
public:

    using Type = ScalarType;
    using Base = AngleBase<Radians<ScalarType>, ScalarType>;

    explicit Radians(ScalarType radians);
    explicit Radians(Degrees<ScalarType> degrees);
    template<typename OtherType> Radians(Radians<OtherType> other);

    Degrees<ScalarType> toDegrees() const;

    // Sine/cosine/tangent of this angle:
    ScalarType sin() const;
    ScalarType cos() const;
    ScalarType tan() const;

    // To radian angle:
    static Radians asin(ScalarType sine);
    static Radians acos(ScalarType cosine);
    static Radians atan(ScalarType tangent);
    static Radians atan(ScalarType x, ScalarType y);

    // Map an angle in radians to the [0,2pi] range.
    static Radians normalizeAngleTwoPi(ScalarType radians);

    // Map an angle in radians to the [-pi,+pi] range.
    static Radians normalizeAnglePi(ScalarType radians);

    // Test if a scalar value is in the [-2.pi,2.pi] range.
    static bool isValidAngle(ScalarType radians);
};

// ========================================================
// class Degrees:
// ========================================================

template<typename ScalarType>
class Degrees final
    : public AngleBase<Degrees<ScalarType>, ScalarType>
{
public:

    using Type = ScalarType;
    using Base = AngleBase<Degrees<ScalarType>, ScalarType>;

    explicit Degrees(ScalarType degrees);
    explicit Degrees(Radians<ScalarType> radians);
    template<typename OtherType> Degrees(Degrees<OtherType> other);

    Radians<ScalarType> toRadians() const;

    // Sine/cosine/tangent of this angle:
    ScalarType sin() const;
    ScalarType cos() const;
    ScalarType tan() const;

    // To degrees angle:
    static Degrees asin(ScalarType sine);
    static Degrees acos(ScalarType cosine);
    static Degrees atan(ScalarType tangent);
    static Degrees atan(ScalarType x, ScalarType y);

    // Map an angle in degrees to the [0,360] range.
    static Degrees normalizeAngle360(ScalarType degrees);

    // Map an angle in degrees to the [-180,+180] range.
    static Degrees normalizeAngle180(ScalarType degrees);

    // Test if a scalar value is in the [-360,360] range.
    static bool isValidAngle(ScalarType degrees);
};

// ========================================================
// Radians inline methods:
// ========================================================

template<typename ScalarType>
inline Radians<ScalarType>::Radians(const ScalarType radians)
{
    Base::setScalarValue(radians);
}

template<typename ScalarType>
inline Radians<ScalarType>::Radians(const Degrees<ScalarType> degrees)
{
    Base::setScalarValue(degrees.toRadians().getScalarValue());
}

template<typename ScalarType> template<typename OtherType>
inline Radians<ScalarType>::Radians(const Radians<OtherType> other)
{
    Base::setScalarValue(static_cast<ScalarType>(other.getScalarValue()));
}

template<typename ScalarType>
inline Degrees<ScalarType> Radians<ScalarType>::toDegrees() const
{
    return Degrees<ScalarType>{ Base::getScalarValue() * Base::RadToDeg };
}

template<typename ScalarType>
inline ScalarType Radians<ScalarType>::sin() const
{
    return std::sin(Base::getScalarValue());
}

template<typename ScalarType>
inline ScalarType Radians<ScalarType>::cos() const
{
    return std::cos(Base::getScalarValue());
}

template<typename ScalarType>
inline ScalarType Radians<ScalarType>::tan() const
{
    return std::tan(Base::getScalarValue());
}

template<typename ScalarType>
inline Radians<ScalarType> Radians<ScalarType>::asin(const ScalarType sine)
{
    return Radians{ std::asin(sine) };
}

template<typename ScalarType>
inline Radians<ScalarType> Radians<ScalarType>::acos(const ScalarType cosine)
{
    return Radians{ std::acos(cosine) };
}

template<typename ScalarType>
inline Radians<ScalarType> Radians<ScalarType>::atan(const ScalarType tangent)
{
    return Radians{ std::atan(tangent) };
}

template<typename ScalarType>
inline Radians<ScalarType> Radians<ScalarType>::atan(const ScalarType x, const ScalarType y)
{
    return Radians{ std::atan2(x, y) };
}

template<typename ScalarType>
inline Radians<ScalarType> Radians<ScalarType>::normalizeAngleTwoPi(ScalarType radians)
{
    if (radians >= Base::TwoPi || radians < Base::Zero)
    {
        radians -= std::floor(radians * (Base::One / Base::TwoPi)) * Base::TwoPi;
    }
    return Radians{ radians };
}

template<typename ScalarType>
inline Radians<ScalarType> Radians<ScalarType>::normalizeAnglePi(ScalarType radians)
{
    radians = normalizeAngleTwoPi(radians).getScalarValue();
    if (radians > Base::Pi)
    {
        radians -= Base::TwoPi;
    }
    return Radians{ radians };
}

template<typename ScalarType>
inline bool Radians<ScalarType>::isValidAngle(const ScalarType radians)
{
    return std::fabs(radians) <= Base::TwoPi;
}

// User defined suffix '_rad' for literals of type Radians.
// Note: The C++ Standard requires the input parameter to be 'long double'.
inline Radians<long double> operator "" _rad (long double radians)
{
    return Radians<long double>{ radians };
}

// ========================================================
// Degrees inline methods:
// ========================================================

template<typename ScalarType>
inline Degrees<ScalarType>::Degrees(const ScalarType degrees)
{
    Base::setScalarValue(degrees);
}

template<typename ScalarType>
inline Degrees<ScalarType>::Degrees(const Radians<ScalarType> radians)
{
    Base::setScalarValue(radians.toDegrees().getScalarValue());
}

template<typename ScalarType> template<typename OtherType>
inline Degrees<ScalarType>::Degrees(const Degrees<OtherType> other)
{
    Base::setScalarValue(static_cast<ScalarType>(other.getScalarValue()));
}

template<typename ScalarType>
inline Radians<ScalarType> Degrees<ScalarType>::toRadians() const
{
    return Radians<ScalarType>{ Base::getScalarValue() * Base::DegToRad };
}

template<typename ScalarType>
inline ScalarType Degrees<ScalarType>::sin() const
{
    return std::sin(Base::getScalarValue() * Base::DegToRad);
}

template<typename ScalarType>
inline ScalarType Degrees<ScalarType>::cos() const
{
    return std::cos(Base::getScalarValue() * Base::DegToRad);
}

template<typename ScalarType>
inline ScalarType Degrees<ScalarType>::tan() const
{
    return std::tan(Base::getScalarValue() * Base::DegToRad);
}

template<typename ScalarType>
inline Degrees<ScalarType> Degrees<ScalarType>::asin(const ScalarType sine)
{
    return Degrees{ std::asin(sine) * Base::RadToDeg };
}

template<typename ScalarType>
inline Degrees<ScalarType> Degrees<ScalarType>::acos(const ScalarType cosine)
{
    return Degrees{ std::acos(cosine) * Base::RadToDeg };
}

template<typename ScalarType>
inline Degrees<ScalarType> Degrees<ScalarType>::atan(const ScalarType tangent)
{
    return Degrees{ std::atan(tangent) * Base::RadToDeg };
}

template<typename ScalarType>
inline Degrees<ScalarType> Degrees<ScalarType>::atan(const ScalarType x, const ScalarType y)
{
    return Degrees{ std::atan2(x, y) * Base::RadToDeg };
}

template<typename ScalarType>
inline Degrees<ScalarType> Degrees<ScalarType>::normalizeAngle360(ScalarType degrees)
{
    if (degrees >= Base::ThreeSixty || degrees < Base::Zero)
    {
        degrees -= std::floor(degrees * (Base::One / Base::ThreeSixty)) * Base::ThreeSixty;
    }
    return Degrees{ degrees };
}

template<typename ScalarType>
inline Degrees<ScalarType> Degrees<ScalarType>::normalizeAngle180(ScalarType degrees)
{
    degrees = normalizeAngle360(degrees).getScalarValue();
    if (degrees > Base::OneEighty)
    {
        degrees -= Base::ThreeSixty;
    }
    return Degrees{ degrees };
}

template<typename ScalarType>
inline bool Degrees<ScalarType>::isValidAngle(const ScalarType degrees)
{
    return std::fabs(degrees) <= Base::ThreeSixty;
}

// User defined suffix '_deg' for literals of type Degrees.
// Note: The C++ Standard requires the input parameter to be 'long double'.
inline Degrees<long double> operator "" _deg (long double degrees)
{
    return Degrees<long double>{ degrees };
}

#endif // ANGLES_HPP
