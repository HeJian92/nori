#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
//#include <nori/sampler.h>


NORI_NAMESPACE_BEGIN

class AoIntegrator : public Integrator
{
public:
    AoIntegrator(const PropertyList& props)
    {
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }
        Point3f x = its.p;
        // 蒙特卡洛积分求L(x)
        Color3f Lx(0.0f);
        int cnt = 0;
        for (int i = 0; i < 16; i++)
        {
            Vector3f dir = Warp::squareToCosineHemisphere(sampler->next2D());
            float pdf = Warp::squareToCosineHemispherePdf(dir);
            Vector3f dirWorld = its.shFrame.toWorld(dir).normalized();
            float visible = 1;
            if (scene->rayIntersect(Ray3f(x + dirWorld * 0.000001f, dirWorld)))
            {
                visible = 0;
            }
            float cosTheta = its.shFrame.cosTheta(dir);
            if (pdf != 0.0)
            {
                Lx += Color3f(visible) * cosTheta / M_PI / pdf;
                cnt++;
            }
        }
        Lx /= float(cnt);
        return Lx;
    }

    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return "AoIntegrator[]";
    }
protected:
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END