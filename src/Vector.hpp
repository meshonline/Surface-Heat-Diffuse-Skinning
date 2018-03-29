#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <cmath>

const float Pi = 4 * atanf(1.0f);
const float TwoPi = 2 * Pi;

template <typename T>
struct Vector2 {
    inline Vector2():x(0.0f),y(0.0f) {}
    inline Vector2(T x, T y) : x(x), y(y) {}
    inline void Zero() { x = 0.0f; y = 0.0f; }
    inline T Dot(const Vector2& v) const
    {
        return x * v.x + y * v.y;
    }
    inline Vector2 operator+(const Vector2& v) const
    {
        return Vector2(x + v.x, y + v.y);
    }
    inline Vector2 operator-(const Vector2& v) const
    {
        return Vector2(x - v.x, y - v.y);
    }
    inline void operator+=(const Vector2& v)
    {
        *this = Vector2(x + v.x, y + v.y);
    }
    inline void operator-=(const Vector2& v)
    {
        *this = Vector2(x - v.x, y - v.y);
    }
    inline Vector2 operator/(float s) const
    {
        return Vector2(x / s, y / s);
    }
    inline Vector2 operator*(float s) const
    {
        return Vector2(x * s, y * s);
    }
    inline void operator/=(float s)
    {
        *this = Vector2(x / s, y / s);
    }
    inline void operator*=(float s)
    {
        *this = Vector2(x * s, y * s);
    }
    inline void Normalize()
    {
        T len = Length();
        if(len) {
            float s = 1.0f / len;
            x *= s;
            y *= s;
        }
    }
    inline Vector2 Normalized() const
    {
        Vector2 v = *this;
        v.Normalize();
        return v;
    }
    inline T LengthSquared() const
    {
        return x * x + y * y;
    }
    inline T Length() const
    {
        return sqrt(LengthSquared());
    }
    inline const T* Pointer() const
    {
        return &x;
    }
    inline operator Vector2<float>() const
    {
        return Vector2<float>(x, y);
    }
    inline bool operator==(const Vector2& v) const
    {
        return x == v.x && y == v.y;
    }
    inline Vector2 Lerp(float t, const Vector2& v) const
    {
        return Vector2(x * (1 - t) + v.x * t,
                       y * (1 - t) + v.y * t);
    }
    template <typename P>
    inline P* Write(P* pData)
    {
        Vector2* pVector = (Vector2*) pData;
        *pVector++ = *this;
        return (P*) pVector;
    }
    T x;
    T y;
};

template <typename T>
struct Vector3 {
    inline Vector3():x(0.0f),y(0.0f),z(0.0f) {}
    inline Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
    inline void Zero() { x = 0.0f; y = 0.0f; z = 0.0f; }
    inline T Length()
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    inline T SquareLength()
    {
        return x * x + y * y + z * z;
    }
    inline void Normalize()
    {
        T len = Length();
        if(len) {
            float s = 1.0f / len;
            x *= s;
            y *= s;
            z *= s;
        }
    }
    inline Vector3 Normalized() const
    {
        Vector3 v = *this;
        v.Normalize();
        return v;
    }
    inline Vector3 Cross(const Vector3& v) const
    {
        return Vector3(y * v.z - z * v.y,
                       z * v.x - x * v.z,
                       x * v.y - y * v.x);
    }
    inline T Dot(const Vector3& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    inline Vector3 operator+(const Vector3& v) const
    {
        return Vector3(x + v.x, y + v.y,  z + v.z);
    }
    inline void operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }
    inline void operator=(const Vector3& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }
    inline void operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }
    inline void operator/=(T s)
    {
        x /= s;
        y /= s;
        z /= s;
    }
    inline T operator[](const int idx) const
    {
        return (&x)[idx];
    }
    inline Vector3 operator-(const Vector3& v) const
    {
        return Vector3(x - v.x, y - v.y,  z - v.z);
    }
    inline Vector3 operator-() const
    {
        return Vector3(-x, -y, -z);
    }
    inline Vector3 operator*(T s) const
    {
        return Vector3(x * s, y * s, z * s);
    }
    inline Vector3 operator/(T s) const
    {
        return Vector3(x / s, y / s, z / s);
    }
    inline bool operator==(const Vector3& v) const
    {
        return x == v.x && y == v.y && z == v.z;
    }
    inline bool operator<(const Vector3& v) const
    {
        if(x == v.x && y == v.y) return z < v.z;
        if(x == v.x) return y < v.y;
        return x < v.x;
    }
    inline Vector3 Lerp(float t, const Vector3& v) const
    {
        return Vector3(x * (1 - t) + v.x * t,
                       y * (1 - t) + v.y * t,
                       z * (1 - t) + v.z * t);
    }
    inline const T* Pointer() const
    {
        return &x;
    }
    template <typename P>
    inline P* Write(P* pData)
    {
        Vector3<T>* pVector = (Vector3<T>*) pData;
        *pVector++ = *this;
        return (P*) pVector;
    }
    T x;
    T y;
    T z;
};

template <typename T>
struct Vector4 {
    inline Vector4() {}
    inline Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    inline Vector4(const Vector3<T>& v, T w) : x(v.x), y(v.y), z(v.z), w(w) {}
    inline Vector4 operator*(T s) const
    {
        return Vector4(x * s, y * s, z * s, w * s);
    }
    inline T Dot(const Vector4& v) const
    {
        return x * v.x + y * v.y + z * v.z + w * v.w;
    }
    inline bool operator==(const Vector4& v) const
    {
        return x == v.x && y == v.y && z == v.z && w == v.w;
    }
    inline Vector4 Lerp(float t, const Vector4& v) const
    {
        return Vector4(x * (1 - t) + v.x * t,
                       y * (1 - t) + v.y * t,
                       z * (1 - t) + v.z * t,
                       w * (1 - t) + v.w * t);
    }
    inline const T* Pointer() const
    {
        return &x;
    }
    T x;
    T y;
    T z;
    T w;
};

typedef Vector2<bool> structbvec2;

typedef Vector2<int> structivec2;
typedef Vector3<int> structivec3;
typedef Vector4<int> structivec4;

typedef Vector2<float> structvec2;
typedef Vector3<float> structvec3;
typedef Vector4<float> structvec4;

#endif // VECTOR_HPP
