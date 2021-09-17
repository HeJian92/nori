#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator
{
public:
    SimpleIntegrator(const PropertyList& props)
    {
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }
        Point3f x = its.p;
        Point3f p = m_position;
        Vector3f wo = p - x;
        Vector3f woNormalize = wo.normalized();
        float visible = 1;
        if (scene->rayIntersect(Ray3f(x + wo * 0.000001f, wo)))
        {
            visible = 0;
        }
        float cosTheta = its.shFrame.cosTheta(its.shFrame.toLocal(woNormalize));
        return m_energy * std::max(float(0.0), cosTheta) / (4 * M_PI * M_PI) / std::pow((x - p).norm(), 2) * visible;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format(
            "SimpleIntegrator[\n"
            "  m_position = \"%s\"\n"
            "  m_energy = \"%s\"\n"
            "]",
            m_position, m_energy
        );
    }
protected:
    Point3f m_position;
    Color3f m_energy;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END