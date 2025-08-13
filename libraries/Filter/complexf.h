#pragma once

struct ComplexF {
    float _real;
    float _imag;
    ComplexF(float r = 0.0f, float i = 0.0f) : _real(r), _imag(i) {}
    float real() const { return _real; }
    float imag() const { return _imag; }
    ComplexF operator+(const ComplexF& other) const {
        return ComplexF(_real + other._real, _imag + other._imag);
    }
    ComplexF& operator+=(const ComplexF& other) {
        _real += other._real;
        _imag += other._imag;
        return *this;
    }
    ComplexF operator+(float scalar) const {
        return ComplexF(_real + scalar, _imag);
    }
    ComplexF operator-(const ComplexF& other) const {
        return ComplexF(_real - other._real, _imag - other._imag);
    }
    ComplexF& operator-=(const ComplexF& other) {
        _real -= other._real;
        _imag -= other._imag;
        return *this;
    }
    ComplexF operator-(float scalar) const {
        return ComplexF(_real - scalar, _imag);
    }
    ComplexF operator*(const ComplexF& other) const {
        return ComplexF(
            _real * other._real - _imag * other._imag,
            _real * other._imag + _imag * other._real
        );
    }
    ComplexF operator*(float scalar) const {
        return ComplexF(_real * scalar, _imag * scalar );
    }
    ComplexF& operator*=(float scalar) {
        _real *= scalar;
        _imag *= scalar;
        return *this;
    }
    ComplexF operator/(float scalar) const {
        return ComplexF(_real / scalar, _imag / scalar);
    }
    ComplexF operator/(const ComplexF& other) const {
        float denom = other._real * other._real + other._imag * other._imag;
        return ComplexF(
            (_real * other._real + _imag * other._imag) / denom,
            (_imag * other._real - _real * other._imag) / denom
        );
    }
    ComplexF conj() const {
        return ComplexF(_real, -_imag);
    }
};
inline ComplexF operator+(float scalar, const ComplexF& c) {
    return ComplexF(c.real() + scalar, c.imag());
}
inline ComplexF operator-(float scalar, const ComplexF& c) {
    return ComplexF(c.real() - scalar, c.imag());
}
inline ComplexF operator*(float scalar, const ComplexF& c) {
    return ComplexF(c.real() * scalar, c.imag() * scalar);
}