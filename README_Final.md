# Final Project

## Team Members

- [Haoxuan Wang](https://github.com/Thomaswang0822?tab=repositories)
- [David Liu](TODO)

## Table of Contents

1. Watertight ray triangle intersection
2. GPU rendering (very basic)
3. Fourier BSDF

## Watertight ray triangle intersection

The first several sections of PBRT Chapter 3.9 offer very detailed information on floating point error, which includes why floating point operations cannot be precise, how big the error is, how errors accumulate in each step, and so forth.

But before all of these, I'd like to make clear one thing some people may wonder like I do: why don't we use double instead of float to ensure precision. Especially for people who remember some computer science basics well, double generally is as fast as float if we don't consider memory I/O. In fact, it is the memory bound rather than the computational-power bound that restricts the usage of double in certain cases. For a modern renderer, the complexity of the scene passed in could be overwhelming, thus using float can significantly save the memory. Unfortunately, you cannot store the scene with float but using double in path tracing, because the error by float will dominate that by double. Thus, sometimes we have to limit ourselves from using double everywhere.

After all the discussion in PBRT, I want to reiterate one thing. In path tracing, like many other application scenarios, most of the times we effectively only care about those values very close to 0. This include whether the ray hit the bounding box, sphere, triangle, or whatever geometry when close to the boundary. But this doesn't mean that the precision of a large value never matters. A very typical example is when a ray is boucing off toward a complex mesh distant away. We (actually the ray) want to know which triangle in the mesh exactly is first hit by the ray. If the hitting `t` of the current triangle is 40.1, we really want to be 100% sure it's smaller than some `t = 40.12` the ray checked against earlier. In this case, the precision of values far from 0 is also important.

For the implementation, I finished the robust version of ray-bound, ray-sphere, and ray-triangle intersection functions. There isn't much worth discussing in terms of implementation details, since I just follow along the PBRT book and attribute full credit to it.

Let's look at some comparison and have a much clearer idea of (1) how much difference (imprecision) is introduced by switching from double to float and (2) how much compensation is brough by the more robust intersection check. In each group, 3 images are "float with no fix", "float with robust intersection" and "double, our reference image" respectively.

The first is the baseline Cornell Box scene. We can see the huge artifacts on the tall box without the Watertight fix. With the fix, there is no noticable difference at all.

![old_tri_isect.png](./img_png/final_proj/old_tri_isect.png)
![new_tri_isect.png](./img_png/final_proj/new_tri_isect.png)
![hw_4_1b.png](./handouts/imgs/hw_4_1b.png)

The second is the HW0 single sphere area light scene. Still, the artifact is huge for the image without a robust fix, as we can see the sphere looks much darker than the reference image. But this time, we fail to achieve the same precision as the reference image using double. There is a strange "path" going crossing the middle of the image, and the huge sphere ground is also slightly darker.

![sph_no_fix.png](./img_png/final_proj/sph_no_fix.png)
![sph_fix.png](./img_png/final_proj/sph_fix.png)
![hw_4_1b.png](./handouts/imgs/hw_4_1a.png)

## GPU rendering

## Reference

TODO

- [Manage Rounding Error, PBRT](https://www.pbr-book.org/3ed-2018/Shapes/Managing_Rounding_Error#)
- [Type float](https://www.pbr-book.org/3ed-2018/Shapes/Managing_Rounding_Error#)
