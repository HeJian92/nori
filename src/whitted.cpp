#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList& props)
    {
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }

        Color3f Le(0.0f);
        if (its.mesh->isEmitter())
        {
            EmitterQueryRecord eRec(ray.o, its.p, its.shFrame.n);
            Le = its.mesh->getEmitter()->eval(eRec);

        }

        BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
        Color3f fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

        Color3f Li(0.0f);
        Ray3f rayR = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));
        Intersection itsR;
        if (scene->rayIntersect(rayR, itsR))
        {
            if (itsR.mesh->isEmitter())
            {
                EmitterQueryRecord lRec(its.p, itsR.p, itsR.shFrame.n);
                Li = itsR.mesh->getEmitter()->eval(lRec);
            }
        }
        return Le + Li * fr;

    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format(
            "WhittedIntegrator[]"
        );
    }
protected:
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END