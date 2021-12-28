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

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct EmitterQueryRecord
{
    Point3f p; // 着色点p
    Point3f lightPos;  // 光源
    Vector3f wi; // 着色点到光源方向
    Normal3f n; // 法线
    float pdf;

    EmitterQueryRecord(const Point3f& p, const Point3f& lightPos, const Normal3f& n)
        :p(p), lightPos(lightPos), n(n)
    {
        wi = (lightPos - p).normalized();
    }
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    virtual Color3f sample(Mesh* mesh, EmitterQueryRecord& eRec, Sampler* sampler) const = 0;
    virtual Color3f eval(const EmitterQueryRecord& eRec) const = 0;
    virtual float pdf(Mesh* mesh, const EmitterQueryRecord& eRec) const = 0;


    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
};

NORI_NAMESPACE_END
