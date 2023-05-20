#include "compute_radiance.h"

/* ### BVH-version ### */
bool isVisible(Vector3& shadingPt, Vector3& lightPos, Scene& scene, BVH_node& root) {
    double d = distance(shadingPt, lightPos);
    // shot ray from light to shadingPt
    ray lightRay(lightPos, shadingPt, true);
    // test hitting point with BVH
    Hit_Record rec;
    Shape* hitObj = nullptr;
    // hit => not visible (shadow)
    return !root.hit(lightRay, EPSILON, (1-EPSILON) * d, scene, rec, hitObj);
}

// HW3 Update: deal with ImageTexture Color & Area light
Vector3 diffuse_radiance(Scene& scene, Hit_Record& rec, const Color& refl, 
        BVH_node& root, const Shape* hitObj, pcg32_state& rng)
{
    Vector3 result = Vector3(0.0, 0.0, 0.0);

    // Get Kd: the reflectance of Diffuse
    Vector3 Kd = eval_RGB(refl, rec.u, rec.v);
    // attributes that are different for each light
    Vector3 l;  // normalized shadingPt to light position
    Real dsq;  // distance squared: shadingPt to light position

    // for Area Light usage
    Vector3 light_pos;  // sample position
    Vector3 nx;  // normal at light source (flip toward hitting point for Triangle)
    Vector3 total_contribution;  // to accumulate contribution from a TriangleMesh
    for (Light light : scene.lights) {
        // check point light vs area light
        if (PointLight* ptLight = std::get_if<PointLight>(&light)) {
            l = normalize(ptLight->position - rec.pos);
            dsq = distance_squared(rec.pos, ptLight->position);
            if (isVisible(rec.pos, ptLight->position, scene, root)) {
                result += Kd * std::max( abs(dot(rec.normal, l)), 0.0 ) * 
                    c_INVPI * ptLight->intensity / dsq;
            }
        } 
        else if (DiffuseAreaLight* areaLight = std::get_if<DiffuseAreaLight>(&light)) {
            // std::cout << "Area light not implemented yet; will implement later" 
            //     << std::endl;
            // UNUSED(areaLight);

            // ### HW3: Deal with area light ###
            // Get the actual Object (Shape*) under the hood
            const Shape* lightObj = &scene.shapes[areaLight->shape_id];
            Vector3 lightIntensity = areaLight->radiance;  // I in the formula

            // If a sphere, we only sample once
            if (const Sphere *sph = get_if<Sphere>(lightObj)) {
                UNUSED(sph);
                result += sphereLight_contribution(scene, rec, root, lightObj, Kd, lightIntensity, rng, 1);
            }
            else if (const Triangle *leading_tri = get_if<Triangle>(lightObj)) {
                // deal with the entire mesh:
                // its shape id points to the first triangle in the mesh,
                // so we look at [shape_id, shape_id + mesh count)
                assert(leading_tri->area_light_id >= 0 && 
                    "Area Light points to a mesh, but mesh didn't point back.");
                total_contribution = meshLight_total_contribution(
                    scene, rec, root, leading_tri->mesh_id, areaLight->shape_id,
                    Kd, lightIntensity, rng, false, 64, false
                    // last 3 are: sampleAll, maxSample, stratified
                );

                result += total_contribution;
            }
        }
        
    }
    return result;
}


Vector3 radiance(Scene& scene, ray& localRay, BVH_node& root, 
        pcg32_state& rng, unsigned int recDepth) {
    // Step 1 BVH UPDATE: detect hit. 
    Hit_Record rec;
    Shape* hitObj = nullptr;
    root.hit(localRay, EPSILON, infinity<Real>(), scene, rec, hitObj);
    if (rec.dist > 1e9) {  // no hit
        return scene.background_color;
    }
    assert(hitObj && "Bug: hitObj is a nullptr even when a hit is detected.");

    Vector3 L_emmision(0.0, 0.0, 0.0);
    // HW4 UPDATE: Add Le to rendering equation (instead of return Le directly)
    if (is_light(*hitObj)) {
        int self_light_id = get_area_light_id(*hitObj);
        DiffuseAreaLight& self_light = get<DiffuseAreaLight>(scene.lights[self_light_id]);
        // 2) When a ray hits an area light, return the color of the light directly.
        L_emmision += self_light.radiance;
    }

    if (recDepth == 0) {
        return L_emmision;
    }

    // Step 2: found hit -> get Material of hitObj
    //   to decide which function to call
    Material& currMaterial = scene.materials[rec.mat_id];

    // Step 3 BVH UPDATE: act according to Material (instead of Shape)
    if (Diffuse* diffuseMat = std::get_if<Diffuse>(&currMaterial)) {
        // // no recursion, compute diffuse color
        // return diffuse_radiance(scene, rec, diffuseMat->reflectance, root, hitObj, rng);

        // HW4 UPDATE: path tracing Diffuse
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_dir = dir_cos_sample(rng, basis);
        Vector3 Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);
        ray scatterRay = ray(rec.pos, sample_dir);
        // cout << dot(rec.normal, sample_dir) << endl;
        Real cosTerm = dot(rec.normal, sample_dir);
        return L_emmision + (Kd * std::max(cosTerm, 0.0) * c_INVPI)
            * c_PI / cosTerm  // inverse of cosine hemisphere pdf
            * radiance(scene, scatterRay, root, rng, recDepth-1);
    }
    else if (Mirror* mirrorMat = std::get_if<Mirror>(&currMaterial)) {
        // mirror refect ray and do recursion
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);

        // ######### perfect mirror #########
        // Vector3* mirrorColor = std::get_if<Vector3>(&mirrorMat->reflectance);
        // assert(mirrorColor && "Mirror material has reflectance not Vec3 RGB");
        // return *mirrorColor // color at current hitting pt
        //     * radiance(scene, rayOut, root, recDepth=recDepth-1);   // element-wise mutiply

        // ######### hw3 Fresnel reflection  #########
        double cos_theta = dot(rec.normal, rayOut.direction());
        // Vector3 F * mirror recursion
        return mirror_SchlickFresnel_color(mirrorMat->reflectance, rec.u, rec.v, cos_theta) 
            * radiance(scene, rayOut, root, rng, recDepth=recDepth-1);

    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(&currMaterial)) {
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);
        double cos_theta = dot(rec.normal, rayOut.direction());

        // double F0 = plasticMat->get_F0();
        double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);

        /* // Vector3 F * mirror recursion + (1 − F)diffuse,
        return F * radiance(scene, rayOut, root, rng, recDepth=recDepth-1) +
            (1.0 - F) * diffuse_radiance(scene, rec, plasticMat->reflectance, root, hitObj, rng); */

        // HW4 UPDATE: instead of tracing both,
        // we decide between specular and diffuse with Stochastic Probability F vs (1-F)
        bool traceSpecular = next_pcg32_real<Real>(rng) < F;
        if (traceSpecular) {
            return L_emmision + radiance(scene, rayOut, root, rng, recDepth-1);
        } else {
            Basis basis = Basis::orthonormal_basis(rec.normal);
            Vector3 sample_dir = dir_cos_sample(rng, basis);
            Vector3 Kd = eval_RGB(plasticMat->reflectance, rec.u, rec.v);
            ray scatterRay = ray(rec.pos, sample_dir);
            // cout << dot(rec.normal, sample_dir) << endl;
            Real cosTerm = dot(rec.normal, sample_dir);
            return L_emmision + (Kd * std::max(cosTerm, 0.0) * c_INVPI)
                * c_PI / cosTerm  // inverse of cosine hemisphere pdf
                * radiance(scene, scatterRay, root, rng, recDepth-1);
        }
    }
    else if (Phong* phongMat = get_if<Phong>(&currMaterial)) {
        // sample dir first, this time around mirror ray direction
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);
        Vector3 r = rayOut.direction();
        Basis basis = Basis::orthonormal_basis(r);
        Vector3 sample_dir = dir_Phong_sample(rng, basis, phongMat->exponent);

        // check dot(hitting normal, sample direction)
        if (dot(rec.normal, sample_dir) <=0 ) {
            // This could happen because the sample direction is not around 
            // hitting normal anymore
            return L_emmision;
        }

        // compute Phong BRDF
        Vector3 Ks = eval_RGB(phongMat->reflectance, rec.u, rec.v);
        /* Real cosTerm = dot(r, sample_dir);
        Vector3 phong = Ks * (phongMat->exponent + 1.0) * c_INVTWOPI *
                pow(std::max(cosTerm, 0.0), phongMat->exponent);

        // compute Phong pdf
        Real phongPDF = (phongMat->exponent + 1.0) * c_INVTWOPI *
                pow(cosTerm, phongMat->exponent); */
        // recursion
        ray scatterRay = ray(rec.pos, sample_dir); 

        // ##### But note that phong * (1/ phongPDF) is just Ks #####     
        return L_emmision + Ks  // inverse of Phong pdf
            * radiance(scene, scatterRay, root, rng, recDepth-1);
    }
    else if (BlinnPhong* blphMat = get_if<BlinnPhong>(&currMaterial)) {
        // sample half-vector h around shading normal
        /**
         * We use the Phong sampling function because the procedure,
         * i.e. how to get (theta, phi) from (u1, u2), is the same.
         * What we get is the intermediate h.
         * 
         * The difference is in the pdf used in recursion step.
         */
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_h = dir_Phong_sample(rng, basis, blphMat->exponent);

        // reflect in_dir over h to get out_dir
        ray scatterRay = mirror_ray(localRay, sample_h, rec.pos);
        Vector3 out_dir = scatterRay.direction();
        // check dot(hitting normal, out_dir) 
        if (dot(rec.normal, out_dir) <= 0.0) {
            return L_emmision;
        }
        // compute BlinnPhong BRDF
        Vector3 blphBRDF = blphMat->compute_BRDF(sample_h, out_dir, rec);
        // compute BlinnPhong PDF
        Real blphPDF = blphMat->compute_PDF(sample_h, out_dir, rec);
        // recursion
        return L_emmision + blphBRDF * (1.0 / blphPDF)
            * radiance(scene, scatterRay, root, rng, recDepth-1);
    }
    else if (BlinnPhongMicrofacet* micro_blphMat = get_if<BlinnPhongMicrofacet>(&currMaterial)) {
        // sample half vector h to estimate D(h)
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_h = dir_Phong_sample(rng, basis, micro_blphMat->exponent);

        // reflect in_dir over h to get out_dir
        ray scatterRay = mirror_ray(localRay, sample_h, rec.pos);
        Vector3 out_dir = scatterRay.direction();
        Vector3 in_dir = -localRay.direction();
        
        // check dot(hitting normal, out_dir) 
        if (dot(rec.normal, out_dir) <= 0.0) {
            return L_emmision;
        }

        // compute BlinnPhongMicrofacet BRDF;
        // note: in_dir w_i should align with shading normal
        Vector3 micro_blphBRDF = micro_blphMat->compute_BRDF(
            sample_h, in_dir, out_dir, rec);
        
        // compute BlinnPhongMicrofacet PDF;
        Real micro_blphPDF = micro_blphMat->compute_PDF(sample_h, out_dir, rec);

        // recursion
        return L_emmision + micro_blphBRDF * (1.0 / micro_blphPDF)
            * radiance(scene, scatterRay, root, rng, recDepth-1);
    }
    else {
        std::cout << "Material Unknown; will implement later" 
            << std::endl;
    }


    return Vector3(0.0, 0.0, 0.0);
}


Vector3 meshLight_total_contribution(Scene& scene, Hit_Record& rec, BVH_node& root,
            int mesh_id, int shape_id,
            const Vector3& Kd, const Vector3& I,
            pcg32_state& rng, 
            bool sampleAll, int maxSample, 
            bool stratified) 
{
    // for Area Light usage
    Vector3 light_pos;  // sample position
    Vector3 nx;  // normal at light source (flip toward hitting point for Triangle)
    Vector3 total_contribution = {0.0, 0.0, 0.0};  // to accumulate contribution from a TriangleMesh
    Shape* light_tri;  // store our iteration over scene.shapes
    Triangle* tri;
    int local_idx;  // which tri to choose if sample the mesh

    // decide several variables beforehand
    int meshCt = scene.meshes[mesh_id].size;
    Real meshArea = scene.meshes[mesh_id].totalArea;
    bool all = sampleAll || meshCt < maxSample;  // what really should be done
    // sample some (64) lights only if flag set and > 64 lights present
    int n_sample = all? meshCt : maxSample;
    // stratas to look at
    vector<int> stratas;
    stratified? stratas.assign({0,1,2,3}): stratas.assign({-1});
    for (int local_i=0; local_i<n_sample; ++local_i) {
        // 1. pick a triangle
        if (all) {
            // look at each Triangle sequentially
            light_tri = &scene.shapes[shape_id + local_i];
        } else {
            // sample certain triangle from the cdf
            local_idx = scene.meshes[mesh_id].which_tri(next_pcg32_real<double>(rng));
            light_tri = &scene.shapes[shape_id + local_idx];
        }
        tri = get_if<Triangle>(light_tri);
        assert(tri && "Some shape is not a Traingle in an area-lighted mesh");

        for (int which_part : stratas) {
            // 2. pick a point from the triangle
            light_pos = Triangle_sample(tri, rng, which_part);
            // 3. visibility check        
            if (!isVisible(rec.pos, light_pos, scene, root)) {
                // QUESTION: shall we try to sample again?
                continue;
            }
            // 4. get geometric (instead of interpolated shading)normal 
            // and area from the triangle
            nx = tri->normal;
            // flip: want nx and shading normal against, since we use max(−nx · l, 0)
            nx = (dot(nx, light_pos - rec.pos) < 0.0)? nx : -nx;
            // 5. accumulate
            total_contribution += (all? tri->area : meshArea) *  // p(x)
                1.0 / stratas.size() *   // average over stratas
                areaLight_contribution(light_tri, rec, light_pos, Kd, I, nx);
        }
            
    }
    // See function docstring @note
    return all? total_contribution : total_contribution / Real(n_sample);
}


Vector3 sphereLight_contribution(Scene& scene, Hit_Record& rec, BVH_node& root,
            const Shape* lightObj, 
            const Vector3& Kd, const Vector3& I,
            pcg32_state& rng, 
            int ct)
{
    Vector3 total(0.0, 0.0, 0.0);  // return value
    Vector3 light_pos;
    Vector3 nx;  // normal at light source
    const Sphere* sph = get_if<Sphere>(lightObj);
    assert(sph && "Sampling on sphere but Shape lightObj is not a sphere");

    for (int idx=0; idx<ct; idx++) {
        light_pos = Sphere_sample(sph, rng, idx, ct);

        // uncomment following block if using cone sampling
        #pragma region ConeSampling
        /* // This will cause bug if dist(hit_point, center) < R
        Vector3 cp = rec.pos - sph->position;
        assert(length(cp) > sph->radius
            && "hit position is inside a sphere");
        Real cos_theta_max = sph->radius / distance(rec.pos, sph->position);
        light_pos = Sphere_sample_cone(sph, rng, cos_theta_max, normalize(cp)); */
        #pragma endregion ConeSampling

        if (!isVisible(rec.pos, light_pos, scene, root)) {
            continue;
        }
        nx = normalize(light_pos - sph->position);
        total += areaLight_contribution(lightObj, rec, light_pos, Kd, I, nx) 
                    * c_FOURPI * sph->radius * sph->radius;  // pick sphere
    }

    return total / Real(ct);  // average over all "orange slice" samples
}
