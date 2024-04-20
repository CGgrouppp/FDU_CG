# <center> 图形学PJ2实验报告
### <center> 邹怡 21307130422 吴皓玥 21307110386
## 一.Phong着色模型
### 1.实验要求：
&emsp;&emsp;实现点光源和Phong光照反射模型。Phong着色模型由三部分组成：环境光（Ambient）、漫反射光（Diffuse）和镜面反射（Specular）。实验要求计算出这三个部分并返回累加的光照强度
### 2.实验原理：
#### i. 环境光（ambient）
环境光在Scene类中定义，可以通过调用`getAmbientLight() `函数获取。并通过公式$I_{ambient}=L_{ambient}*k_{diffuse}$计算光强
#### ii. 漫反射光（diffuse）
对于给定光线方向L和面法向量N，若光源在切平面以下，即(L·N<0)时漫反射为0，由公式
$clamp(\mathbf{L},\mathbf{N}) = \begin{cases} 
      \mathbf{L}·\mathbf{N} & \text{if } \mathbf{L}·\mathbf{N} < 0 \\
      0 & \text{otherwise } 
\end{cases}$
漫反射光强公式如下：
$I_{diffuse}=clamp(\mathbf{L}·\mathbf{N} )*L*k_{diffuse}$其中$k_{diffuse}$可以由`_diffuseColor`获得
#### iii. 镜面反射(specular)
假设入射光线的入射方向为 \( \mathbf{L} \)，表面的法向量为 \( \mathbf{N} \)，反射光线的方向为 \( \mathbf{R} \)，那么镜面反射的公式可以表示为：
\[ \mathbf{R} = \mathbf{L} - 2 (\mathbf{L} \cdot \mathbf{N}) \mathbf{N} \]
镜面反射项的公式为：
$I_{specular}=clamp(L,R)^s*L*k_{specular}$ 
其中光泽度s可以由`_shininess`得到，反射率k_specular 可以由`Material`类的`_specularColor`定义。
#### iv.光照强度
将上面所计算的三个部分光强相加，其中漫反射和镜面反射需要累加所有光源的对应光强度，公式如下：
$ I=I_{ambient}+\sum_{i \in {lights}}(I_{diffuse,i}+I_{specular,i})$
在shade函数中进行遍历累加并返回累加值即可
### 3.代码分析
#### i. Light.cpp
`getIllumination`对在空间中的一个点修改以下三个值：
(a)tolight：从场景中一个点指向光源的方向矢量（归一化后的）。
(b)intensity：此时的照明强度（RGB）。
(c)distToLight：场景点与光源之间的距离。
点光源在距离为d的场景点$x_{surf}$处的光强，通过下面公式计算：
$L(x_{surf})=\frac{I}{αd^2}$，其中I为光源光强，α为衰减因子，可由`_falloff`得到，d为距离
```
    void PointLight::getIllumination(const Vector3f &p, 
                                 Vector3f &tolight, 
                                 Vector3f &intensity, 
                                 float &distToLight) const
    {
        // TODO Implement point light source
        // tolight, intensity, distToLight are outputs
        tolight =  (_position - p).normalized();
        distToLight = (_position - p).abs();
        intensity = _color/(_falloff*distToLight*distToLight);
    }
```
#### ii.Material.cpp

```
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
```