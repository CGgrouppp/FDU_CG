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

    Matrix3f Gaussian_Filter(1.0/16,2.0/16,1.0/16,2.0/16,4.0/16,2.0/16,1.0/16,2.0/16,1.0/16);// 高斯滤波
    if(_args.filter){ // 上采样
        w = 3 * w;
        h = 3 * h;
    }
    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.
    Camera* cam = _scene.getCamera();

    float x_step = 2.0f / (w - 1.0f);
    float y_step = 2.0f / (h - 1.0f);

    for (int y = 0; y < h; ++y) {
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            Vector3f color(0,0,0);
            Hit h;
            if(_args.jitter){
                for(int p = 0; p<4; p++){
                    for(int q = 0; q<4; q++){
                        float random_x = static_cast<double>(rand()) / RAND_MAX; // 生成0到1的随机数
                        float random_y = static_cast<double>(rand()) / RAND_MAX;
                        float x_rand = ndcx + (p+random_x)/4.0 * x_step;
                        float y_rand = ndcy + (q+random_y)/4.0 * y_step;
                        Ray r = cam->generateRay(Vector2f(x_rand, y_rand));
                        Hit newh;
                        Vector3f color_temp = traceRay(r, cam->getTMin(), _args.bounces, newh, 0);
                        color+= color_temp;
                    }
                }
                color = color/16.0;
            }else{
                Ray r = cam->generateRay(Vector2f(ndcx, ndcy)); // 根据像素位置生成光线
                color = traceRay(r, cam->getTMin(), _args.bounces, h, 0);
            }
            
            image.setPixel(x, y, color);
            nimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                dimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }

    if(_args.filter){
        int w_f = w/3;
        int h_f = h/3;
        Image f_image(w_f, h_f);
        Image f_nimage(w_f, h_f);
        Image f_dimage(w_f, h_f);
        for (int y = 0; y < h_f; ++y) {
            //float ndcy = 2 * (y / (h_f - 1.0f)) - 1.0f;
            for (int x = 0; x < w_f; ++x) {
                //float ndcx = 2 * (x / (w_f - 1.0f)) - 1.0f;
                //尝试都进行高斯滤波
                Vector3f color(0,0,0);
                Vector3f n(0,0,0);
                Vector3f d(0,0,0);
                for(int i = 0; i < 3; i++){
                    for(int j = 0; j < 3; j++){
                        int temp_x = 3*x + i;
                        int temp_y = 3*y + j;
                        color += Gaussian_Filter(i, j)*image.getPixel(temp_x, temp_y);
                        n += Gaussian_Filter(i, j)*nimage.getPixel(temp_x, temp_y);
                        d += Gaussian_Filter(i, j)*dimage.getPixel(temp_x, temp_y);
                    }   
                }
                f_image.setPixel(x, y, color);
                f_nimage.setPixel(x, y, n);
                f_dimage.setPixel(x, y, d);
            }
        }
        // save the files 
        if (_args.output_file.size()) {
            f_image.savePNG(_args.output_file);
        }
        if (_args.depth_file.size()) {
            f_dimage.savePNG(_args.depth_file);
        }
        if (_args.normals_file.size()) {
            f_nimage.savePNG(_args.normals_file);
        }
    }else{
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

