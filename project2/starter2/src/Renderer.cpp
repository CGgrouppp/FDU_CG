#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
{
}

void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;

    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.
    Camera* cam = _scene.getCamera();
    for (int y = 0; y < h; ++y) {
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            Ray r = cam->generateRay(Vector2f(ndcx, ndcy));

            Hit h;
            Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, h, 0);

            image.setPixel(x, y, color);
            nimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                dimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }
    // END SOLN

    // save the files 
    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}



Vector3f
Renderer::traceRay(const Ray &r,
    float tmin,
    int bounces,
    Hit &h,
    int depth) const
{
    // The starter code only implements basic drawing of sphere primitives.
    // You will implement phong shading, recursive ray tracing, and shadow rays.

    // TODO: IMPLEMENT 
    
    Vector3f phong(0,0,0);
    // getGroup()物体对象
    if (_scene.getGroup()->intersect(r, tmin, h)) {//如果当前光线与物体相交
        Vector3f p = r.pointAtParameter(h.getT()); //获取击中的坐标；
        for(int i=0;i<_scene.getNumLights();i++) // 光源
        {
            Vector3f tolight,intensity; //_direction _color
            float distToLight;
            _scene.getLight(i)->getIllumination(p, tolight, intensity, distToLight);
            //阴影
            Ray r_temp(p, tolight);
            Hit h_o;
            // 判断是否与物体有交点
            if(!_args.shadows||!(_scene.getGroup()->intersect(r_temp, 1e-4, h_o))){ 
                phong += h.getMaterial()->shade(r,h,tolight,intensity);
            }
        }
        phong += _scene.getAmbientLight()*h.getMaterial()->getDiffuseColor();
        //递归
        Vector3f I_indirect(0,0,0);
        if(depth<bounces){
            // 计算ray的反射光线 R = 2*N(L*N)-L(单位矢量)
            Vector3f N = h.getNormal().normalized();
            Vector3f L = -r.getDirection().normalized();
            Vector3f Ray_dir = (2 * N * (Vector3f::dot(L, N)) - L).normalized();
            Ray newray(p, Ray_dir);
            Hit newh;
            //Camera* cam = _scene.getCamera();
            //I_indirect = traceRay(newray, cam->getTMin(), bounces, newh, depth+1);
            I_indirect = traceRay(newray, 1e-4, bounces, newh, depth+1);
        }
        phong += h.getMaterial()->getSpecularColor() * I_indirect;
        return phong;
    } else {
        return  _scene.getBackgroundColor(r.getDirection());
    };
}

