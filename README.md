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

There are 2 common types of acceleration structure: spatial subdivision and bounding volume hierarchies. Essentially, spatial subdivision divides the 3D scene into cells (3D grids). They will be subdivided repeatedly until a small cell only contains one or two shapes. Bounding volume hierarchies (usually referred to as 'BVH') creates bounding volumes around shapes, and bigger bounding volumes will be created to enclose smaller bounding volumes.

Both techniques utilize the fact that 3D grids and bounding volumes (usually boxes) can be easily merged into bigger ones. If a ray misses a big box entirely, we save the ray-shape-intersection check against all shapes enclosed by the box. With some spatial short-circuiting trick, it achieves ray-scene-intersection in O(log(N)) complexity.

Our renderer implements BVH. And the bounding volume we chose is the most commonly used axis-aligned bounding box, usually referred to as 'AABB'.

### Motion Blur

![stable.png](./img_png/hw2/stable.png "Stable")
![motion_blur.png](./img_png/hw2/motion_blur.png "Motion Blur")

Motion blur is a phenomenon occurred when a real camera captures photos of an object moving fast. The underlying physics is related to exposure time and photons and is quite involved. For our computer graphics artificial camera, what we need is to set a small start-to-end interval, like 1s, start position, and end position. When an object is hit by each ray, we generate a random time in the interval and use the interpolated position. This will create a blurry effect that suggests "this object is moving" to our brain.

### UV Texture Mapping

Textures are used almost everywhere in computer graphics. They are in image format and allow us to specify the appearance independent of the model complexity. In other words, you can attach a super high-resolution image to a plane consisting of 2 triangles, and you can also extrapolate a very small image across a complex mesh to represent patterns.

A little bit more details: when a ray hit a triangle at position p, we can calculate a barycentric coordinate of p and use it as the UV coordinate to query/interpolate a pixel RGB value in the texture image space.

The first image below is a low-polygonal mesh with a high-resolution texture. See `scenes/head`. The second image is a complex mesh with several simple textures that creates the brick pattern of the Sponza Palace. See `scenes/sponza`.

![face_baseline.png](img_png/hw3/face_baseline.png)
![sponza_texture.png](img_png/hw3/sponza_texture.png)

### Shading Normals

This is another brilliant idea invented by Phong. It makes a mesh look smoother and less polygonal, while not increasing the mesh complexity. It uses a similar idea as the UV texture mapping above. This time, the UV coordinate is used to interpolate the 3 vertex normals of the current triangle. This effectively turns a flat triangle in a mesh into a smoothly curved spherical triangle. The discrete change in the normal direction across the edge, which is the source of the polygonal look, will disappear.

![stable.png](./img_png/hw2/stable.png "Without shading normal")
![teapot_with_n.png](./img_png/hw3/teapot_with_n.png "With shading normal")

### Area Light (Monte Carlo Importance Sampling)

### Efficient Sampling Strategies

### Path-Tracing

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
