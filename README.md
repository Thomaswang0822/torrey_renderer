# Torrey: Physically-Based Path-Tracing Renderer

![dining_room.png](./img_png/hw4/deterMIS/dining_room_MIS.png)

## Introduction

This is the course project of [UCSD CSE 168 Computer Graphics II: Rendering](https://cseweb.ucsd.edu/~tzli/cse168/sp2023/). Throughout the quarter, we built a physically-based renderer (from ray-tracing to path-tracing) almost from scratch.

All the high-level feature descriptions and concept explanations are outlined in our homework specification:

- [Homework 1: Ray Tracing](https://cseweb.ucsd.edu/~tzli/cse168/sp2023/homework1.pdf)
- [Homework 2: Triangles and acceleration structures](https://cseweb.ucsd.edu/~tzli/cse168/sp2023/homework2.pdf)
- [Homework 3: Textures, shading normals, Fresnel, and area lights](https://cseweb.ucsd.edu/~tzli/cse168/sp2023/homework3.pdf)
- [Homework 4: Indirect lighting, BRDFs, and Multiple Importance Sampling](https://cseweb.ucsd.edu/~tzli/cse168/sp2023/homework4.pdf)

## Feature Highlights

My renderer implements many fundamental features and functionalities presented in modern renderers. Here are some selected ones, sorted from their order in homework specifications:

### Antialiasing

Each pixel is not a point but a little rectangle. If we only shoot a camera ray through the center of each pixel, all the shapes in the scene will only have details at the image resolution level. As a result, the smaller a shape is, the more jaggy its boundary will appear to be.

To solve this, we shoot multiple camera rays through each pixel and randomize their direction around the pixel center. At last, we simply take the average of the results from these rays and get a "smoother" image.

![hw_1_6_before.png](./handouts/imgs/hw_1_6_before.png "Before Antialiasing")
![hw_1_6_after.png](./handouts/imgs/hw_1_6_after.png "After Antialiasing")

### Parallelization

Despite many powerful features, this is still a CPU-based renderer. I am currently learning some CUDA programming and moving my renderer to GPU for my final project. But before that, we can still leverage the powerful parallel computing on CPU (though less powerful than GPU parallelization).

The external parallelization API I used is offered by the [PBRT book](https://github.com/mmp/pbrt-v3/blob/master/src/core/parallel.h) which handles all low-level details. On a high level, we aim to utilize all cores of our CPU. We divided the bigger image into smaller regions called tiles, and use a parallel thread to render each tile.

### Defocus Blur

![defocus_blur.png](./img_png/hw1/defocus_blur.png)

The idea behind defocus blur actually is not hard. Instead of a perfect point, the camera lens becomes a small disk. This means we also randomize the origin of our camera ray. Note the difference between this and Antialiasing above. Antialiasing will randomize the point within each pixel, which effectively randomizes the direction of the camera ray slightly. On the other hand, here we randomize the origin of each camera ray.

The larger the camera lens disk radius, the greater defocus blur effect.

### XML Scene Parser

At the beginning stage, configuring a scene via cpp code directly is convenient. But as we move on and add more features, we need more complex scenes and it becomes ineffective or infeasible to create scenes programmatically.

Fortunately, there are plenty of alternatives coming to the rescue. We chose one that balances ease of use and feature richness: [Mitsuba Renderer XML scene format](https://mitsuba.readthedocs.io/en/latest/src/key_topics/scene_format.html). The XML scene description files are highly readable and [Blender](https://www.blender.org/) has [an exporter plug-in](https://github.com/mitsuba-renderer/mitsuba-blender) for it. In this way, we can build a custom scene, export an XML scene description, render it, and compare the result with the Blender renderer.

The low-level code that parses each node in an XML is provided in Professor's starter code `pare_scene.h` and `parse_scene.cpp`. But it simply "translates" scene definition into corresponding cpp struct and class. But there is more parsing to be done for the scene to better work for the renderer. For example, the scene defines a triangle mesh with arrays (vertex, position, normal, uv) plus some additional global information about the mesh. While, for code efficiency, we want a Triangle struct that stores information for individual triangles. I implemented the scene customization parsing part, which locates in `Scene.h` and `Scene.cpp`.

### Acceleration Structure

This is arguably the most important part of any ray-tracing or path-tracing render. The most frequently called function is ray-scene-intersection, which answers the question "Which object (triangle, sphere, or whatever primitive shape) will this ray hit?" Model/mesh with fine details usually has a triangle count in million magnitudes. Without an acceleration structure, rendering scenes with complex meshes becomes practically impossible.

An acceleration structure solves the following problem: how to test the intersection between a ray and EVERY object in the scene. A brute-force checking has complexity O(N) because it tests ray-shape-intersection against all N shapes in the scene one by one. An acceleration structure can achieve O(log(N)) complexity.

There are 2 common types of acceleration structure: ***spatial subdivision*** and ***bounding volume hierarchies***. Essentially, spatial subdivision divides the 3D scene into cells (3D grids). They will be subdivided repeatedly until a small cell only contains one or two shapes. Bounding volume hierarchies (usually referred to as 'BVH') creates bounding volumes around shapes, and bigger bounding volumes will be created to enclose smaller bounding volumes. I will borrow 2 images from our slides to illustrate:

![spatial_subdiv.png](./img_png/writeup/spatial_subdiv.png "Spatial Subdivision")
![hw_1_6_after.png](./img_png/writeup/BVH.png "BVH")

Both techniques utilize the fact that 3D grids and bounding volumes (usually boxes) can be easily merged into bigger ones. If a ray misses a big box entirely, we save the ray-shape-intersection check against all shapes enclosed by the box. With some spatial short-circuiting trick, it achieves ray-scene-intersection in O(log(N)) complexity.

Our renderer implements BVH. And the bounding volume we chose is the most commonly used axis-aligned bounding box, usually referred to as 'AABB'.

### Motion Blur

![stable.png](./img_png/hw2/stable.png "Stable")
![motion_blur.png](./img_png/hw2/motion_blur.png "Motion Blur")

Motion blur is a phenomenon occurred when a real camera captures photos of an object moving fast. The underlying physics is related to exposure time and photons and is quite involved. For our computer graphics artificial camera, what we need is to set a small start-to-end interval, like 1s, start position, and end position. When an object is hit by each ray, we generate a random time in the interval and use the interpolated position. This will create a blurry effect that suggests "this object is moving" to our brain.

### UV Texture Mapping

Textures are used almost everywhere in computer graphics. They are in image format and allow us to specify the appearance independent of the model complexity. In other words, you can attach a super high-resolution image to a plane consisting of 2 triangles, and you can also extrapolate a very small image across a complex mesh to represent patterns.

A little bit more details: when a ray hit a triangle at position p, we can calculate a ***barycentric coordinate*** of p and use it as the ***UV coordinate*** to query/interpolate a pixel RGB value in the texture image space.

The first image below is a low-polygonal mesh with a high-resolution texture. See `scenes/head`. The second image is a complex mesh with several simple textures that creates the brick pattern of the Sponza Palace. See `scenes/sponza`.

![face_baseline.png](img_png/hw3/face_baseline.png)
![sponza_texture.png](img_png/hw3/sponza_texture.png)

### Shading Normals

This is another brilliant idea invented by Phong and is extensively used in computer graphics. It makes a mesh look smoother and less polygonal, while not increasing the mesh complexity. It uses a similar idea as the UV texture mapping above. This time, the UV coordinate is used to interpolate the 3 vertex normals of the current triangle. This effectively turns a flat triangle in a mesh into a smoothly curved spherical triangle. The discrete change in the normal direction across the edge, which is the source of the polygonal look, will disappear.

![stable.png](./img_png/hw2/stable.png "Without shading normal")
![teapot_with_n.png](./img_png/hw3/teapot_with_n.png "With shading normal")

### Area Light (Monte Carlo Integration)

Our renderer begins with only supporting point light, which is the easiest type of light source in computer graphics. Point lights require easier code and offer faster computation, but they also introduce unreal artifacts, such as hard shadow. You can compare the following image with the other one above. The shadow of the head on the shoulder looks very unreal.

![hard_shadow.png](./handouts/imgs/hw_3_2b.png "Hard Shadow")

To solve this, we introduce the concept of ***area lights***. In essence, the light source becomes an object with volume and surface area. A sphere, a very complex mesh, and whatever being modeled in the scene are capable of being an area light. We assume the light emits from the surface of the light source, so this is why we call them area lights instead of volume lights.

But rendering area-light effects is not easy. In theory, an area light is an infinite collection of infinitesimal points, and we need to consider the radiance contribution from all of them. Mathematically, this is solving an integral, which cannot be done analytically on computers 99% of time.

$$\int_{x \in S} f(x) \mathrm{d}A(x)$$

where $\mathrm{d}A(x)$ means an infinitesimal area around x on the surface, and $f(x)$ is similar to the contribution of a single point light located at $x$:
$$f(x) = \frac{K_d \cdot \max\left(n_s \cdot l, 0\right)}{\pi} \cdot \frac{I \max\left(-n_x \cdot l, 0\right)}{d^2} \cdot \text{visibility}$$

Thus, we need a statistical method, ***Monte Carlo Integration***, to approximate the result. In plain language, it draws random samples in the domain you would integrate over, compute a finite sum from these samples to approximate the integral, an infinite sum.

How to draw these samples is another non-trival question. Statistics API in most programming languages can generate a random sample from any common statistical distribution. But in the rendering context, the area-light shape is a geometry and doesn't fit any distribution. Instead, we need the sampling strategy for shape primitives like sphere and triangle. The formula derivation can be found easily online and we will just include the formula here. Assume we have 2 random numbers, $s$ and $t$, generated from a uniform distribution between 0 and 1, then

- to uniformly sample a point represented by elevation angle $\theta$ and azumith angle $\phi$ on a spehrical surface:
$$ \theta = cos^{-1}(1 - 2s); \phi = 2\pi t $$
- to uniformly sample a point represented by barycentric coordinates ($b_1$, $b_2$) on a triangle:
$$ b_1 = 1 - \sqrt{s}; b_2 = t \sqrt{s} $$
Furthermore, we don't necessarily need to randomly sample the point across the entire surface, because in rendering, some region of a shape will be occuluded. Some improvements on the current sampling strategy are covered in the [Efficient Sampling Strategies](#efficient-sampling-strategies).

At last, the key of Monte Carlo Integration is to scale the sampled value by dividing the probability density of picking this sample. This will give us an unbiased estimate of the integral using one sample points. Here when we randomly pick a position on a sphere or triangle, the probability density is simply (1/area). To use multiple samples to reduce variance, we scale the values first then take the average.

### Efficient Sampling Strategies

#### 1. Sample One Light at a Time

For triangles, the area light property is bound to a trianle mesh instead of single triangles. My baseline area-light implementation suffers when we have a complex mesh as an area light. If the mesh has a million triangle, we need to sample a million times and sum up the radiance contribution from these one million triangles, and this brings the time complexity back to O(N).

The fix isn't hard: we do a 2-step sampling. Before sampling a point on a triangle, we randomly pick several triangles from the complex mesh. A key design choice is to set the probability of being picked proportional to triangle areas. Other implementation details are included in the relative function comments.

#### 2. Cone Sampling a Sphere

![my_cone.png](img_png/hw3/my_cone.png)

If we look closely at how a spherical area light illuminates a shading point, we will find that more than half of the surface region on the back is blocked by that the region on the front. This makes sampling on the entire sphere surface less effetive. Say we draw 8 samples from a spherical light but expect 4~5 of them being occuluded and thus give 0 contribution. We take average over 8 samples but only 3 of them are nonzero. We would like to ensure our sample points are always visible and give nonzero contribution, such that in the same scenario we only need to average over 3 samples.\

And here comes the idea of Cone Sampling. From the shading point of view, the visible region is the spherical cap enclosed by a cone tangent to the sphere. This region is colored red in my figure. The size of the spherical cap is controlled by that $\theta_{max}$ whose cosine is computed by (radius / center_to_shading_point). Note that a sphere is a special spherical cap with $\theta_{max} = \pi$. Thus, the Cone Sampling formula becomes:
$$ \theta = cos^{-1}(1 - (1-cos(\theta_{max}))s); \phi = 2\pi t $$
and the surface area of a spherical cap is
$$ 2\pi r^2 (1-cos(\theta_{max})) $$
If you plug in $\theta_{max} = \pi$, you will get both formulas for a sphere.

### 3. Stratified Sampling

This could be the most extensivel used sampling strategy. The idea is simple: we subdivide the sampling domain and sample a point from each sub-region. This eliminates the rare case where most samples of shape are concentrated in a region and as a result we get a pixel too bright or too dark.

I picked 2 common stratification methods for triangles and spheres. For a triangle, we connect the 3 middle points on the edges and divide the triangle into 4 equal-area parts. For a sphere, we divide the azmumith angle $\phi$, which ranges from 0 to $2\pi$, into 6 or 8 equally-spaced intervals. Intuitively, this works like dividing an orange into 6 or 8 slices.

![face_baseline.png](./img_png/hw3/face_baseline.png "Uniform")
![face_strat6.png](./img_png/hw3/face_strat6.png "Stratified")

![cbox_baseline.png](./img_png/hw3/cbox_baseline.png "Uniform")
![cbox_strat.png](./img_png/hw3/cbox_strat.png "Stratified")

The head is illuminated by a spherical area light. We can clearly observe that those noisy tiny black dots almost disappear. The Cornell Box scene has triangle-mesh area light (that square on the ceiling). Its improvement is not as obvious, but you may still obverse the quality improvement around shadow.

### Path-Tracing

### BRDF Importance Sampling

### BRDF for Different Materials

### Multiple Importance Sampling

### Russian Roulette

## Attribute

All the low-level implementation details and design decisions are made by myself, but the high-level ideas are heavily influenced and supported by the following people and sources:

- [Professor Tzu-mao Li, course instructor](https://cseweb.ucsd.edu/~tzli/)
- [Trevor Hedstrom, course TA](https://www.linkedin.com/in/trevor-hedstrom-9abb6362)
- [Starter code by Professor Li](https://github.com/BachiLi/torrey_public)
- [Course Slides](https://cseweb.ucsd.edu/~tzli/cse168/sp2023/) attached to the course website.
- [Ray Tracing in One Weekend series](https://raytracing.github.io/), the famous ray-tracing tutorial series.
- [Physically-based Rendering: from Theory to Implementation](https://www.pbr-book.org/), the famous PBTR book.
- [Eric Veach's thesis](https://graphics.stanford.edu/papers/veach_thesis/), the famous "Bible" for rendering theory foundation.
- Many utilities used by the starter code, see [starter code README](https://github.com/BachiLi/torrey_public#acknowledgement)
- And many other papers/theses for particular techniques. They are attached to the homework specification files and also in the @ref section of my code implementation.

## Author

- [@thomaswang0822](https://github.com/Thomaswang0822)
