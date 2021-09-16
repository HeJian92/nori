/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tent(float x)
{
    return x < 0.5f ? std::sqrt(2.0f * x) - 1.0f : 1.0f - std::sqrt(2.0f * (1.0f - x));
}

Point2f Warp::squareToTent(const Point2f &sample) {
    return Point2f(tent(sample.x()), tent(sample.y()));
}

float tentPdf(float t)
{
    return t >= -1 && t <= 1 ? 1 - std::abs(t) : 0;
}

float Warp::squareToTentPdf(const Point2f &p) {
    return tentPdf(p.x()) * tentPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    return Point2f(r * std::cos(theta), r * std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return  std::sqrt(p.x() * p.x() + p.y() * p.y()) <= 1.0f ? INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float z = 1 - 2 * sample.x();
    float r = std::sqrt(std::max(float(0.0), float(1.0) - z * z));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = sample.x();
    float r = std::sqrt(std::max(float(0.0), float(1.0) - z * z));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return v.z() < 0 ? 0 : INV_TWOPI;
}

Point2f Warp::squareToConcentricDisk(const Point2f& sample)
{
    Point2f uOffset = 2.0f * sample - Vector2f(1, 1);
    if (uOffset.x() == 0 && uOffset.y() == 0)
        return Point2f(0, 0);

    float theta, r;
    if (std::abs(uOffset.x()) > std::abs(uOffset.y()))
    {
        r = uOffset.x();
        theta = M_PI / 4.0 * (uOffset.y() / uOffset.x());
    }
    else
    {
        r = uOffset.y();
        theta = M_PI / 2.0 - M_PI / 4.0 * (uOffset.x() / uOffset.y());
    }
    return r * Point2f(std::cos(theta), std::sin(theta));
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Point2f d = squareToConcentricDisk(sample);
    float z = std::sqrt(std::max(float(0.0), float(1.0) - d.x() * d.x() - d.y() * d.y()));
    return Vector3f(d.x(), d.y(), z);

}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return v.z() < 0 ? 0 : v.z() * INV_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float phi = M_PI * 2 * sample.x();
    float theta = std::atan(std::sqrt(-alpha * alpha * std::log(1 - sample.y())));
    float cosPhi = std::cos(phi);
    float sinPhi = std::sin(phi);
    float cosTheta = std::cos(theta);
    float sinTheta = std::sin(theta);
    float x = sinTheta * cosPhi;
    float y = sinTheta * sinPhi;
    float z = cosTheta;
    return Vector3f(x, y, z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0)
    {
        return 0;
    }
    float alpha2 = alpha * alpha;
    float cosTheta = m.z();
    float tanTheta2 = (m.x() * m.x() + m.y() * m.y()) / (cosTheta * cosTheta);
    float cosTheta3 = cosTheta * cosTheta * cosTheta;
    float azimuthal = INV_PI;
    float longitudinal = std::exp(-tanTheta2 / alpha2) / (alpha2 * cosTheta3);
    return azimuthal * longitudinal;
}

NORI_NAMESPACE_END
