#include "Material.h"
Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
    // TODO implement Diffuse and Specular phong terms
    float clamp_d;
    clamp_d = Vector3f::dot(hit.getNormal(),dirToLight);
    clamp_d = clamp_d>0? clamp_d:0;
    Vector3f I_diffuse(0,0,0);
    I_diffuse = clamp_d*lightIntensity*_diffuseColor;
    
    float clamp_s;
    Vector3f R = 2*(Vector3f::dot(-ray.getDirection().normalized(),hit.getNormal().normalized()))*hit.getNormal()+ray.getDirection();
    clamp_s = Vector3f::dot(dirToLight,R.normalized());
    clamp_s = clamp_s>0?clamp_s:0;
    Vector3f I_specular(0,0,0);
    I_specular = std::pow(clamp_s,_shininess)*_specularColor*lightIntensity;
    return I_diffuse+I_specular;

}
