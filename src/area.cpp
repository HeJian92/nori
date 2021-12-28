

#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter
{
public:
    AreaLight(const PropertyList& propList)
    {
        m_radiance = propList.getColor("radiance", Color3f(1.0f));
    }

    Color3f eval(const EmitterQueryRecord& eRec) const
    {
        float cosTheta = eRec.n.dot(-eRec.wi);
        if (cosTheta > 0.0f)
            return m_radiance;
        return Color3f(0.0f);
    }

    float pdf(Mesh* mesh, const EmitterQueryRecord& eRec) const
    {
        float cosTheta = eRec.n.dot(-eRec.wi);
        if (cosTheta > 0.0f)
            return (eRec.lightPos - eRec.p).squaredNorm() / cosTheta /  mesh->getTotalArea();
        return 0.0f;
    }

    /// Draw a sample from the Emitter model
    Color3f sample(Mesh* mesh, EmitterQueryRecord& eRec, Sampler* sampler) const
    {
        SampleResult sampleResult = mesh->sample(sampler);
        eRec.lightPos = sampleResult.p;
        eRec.n = sampleResult.n;
        eRec.wi = (eRec.lightPos - eRec.p).normalized();
        float curPdf = pdf(mesh, eRec);
        if (curPdf > 0.0f && !std::isnan(curPdf) && !std::isinf(curPdf))
            return eval(eRec) / curPdf;
        return Color3f(0.0f);
    }

    /// Return a human-readable summary
    std::string toString() const
    {
        return tfm::format(
            "AreaLight[\n"
            "  radiance = %s\n"
            "]", m_radiance.toString());
    }

    EClassType getClassType() const { return EEmitter; }
private:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END
