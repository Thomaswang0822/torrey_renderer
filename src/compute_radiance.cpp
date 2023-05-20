#include "compute_radiance.h"

/* ### BVH-version ### */
bool isVisible(const Vector3& shadingPt, Vector3& lightPos, Scene& scene, BVH_node& root) {
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
    for (Light& light : scene.lights) {
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

    // Step 0: end if reaching recursion depth
    if (recDepth == 0) {
        return Vector3(0.0, 0.0, 0.0);
    }
    // Step 1: detect hit. 
    Hit_Record rec;
    Shape* hitObj = nullptr;
    root.hit(localRay, EPSILON, infinity<Real>(), scene, rec, hitObj);
    if (rec.dist > 1e9) {  // no hit
        return scene.background_color;
    }
    assert(hitObj && "Bug: hitObj is a nullptr even when a hit is detected.");
    
    // Step 2: Add Le to rendering equation (if it's a light)
    Vector3 L_emmision(0.0, 0.0, 0.0);
    if (is_light(*hitObj)) {
        int self_light_id = get_area_light_id(*hitObj);
        DiffuseAreaLight& self_light = get<DiffuseAreaLight>(scene.lights[self_light_id]);
        L_emmision += self_light.radiance;
    }

    // Step 3: get Material of hitObj -> handle special case Mirror
    // It's special because we don't need any kind of sampling
    Material& currMaterial = scene.materials[rec.mat_id];
    if (Mirror* mirrorMat = std::get_if<Mirror>(&currMaterial)) {
        // mirror refect ray and do recursion
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);
        double cos_theta = dot(rec.normal, rayOut.dir);
        // Vector3 F * mirror recursion
        return mirror_SchlickFresnel_color(mirrorMat->reflectance, rec.u, rec.v, cos_theta) 
            * radiance(scene, rayOut, root, rng, recDepth=recDepth-1);
    }

    /**
     * Step 4: (if not Mirror) do Multiple Importance Sampling (MIS)
     * @note For hw_4_3, we do one-sample MIS
     *     This means we do recursion whichever BRDF sampling or light sampling we choose.
     * @note For hw_4_9, we do deterministic MIS
     *     This means calculating <radiance, pdf> for both sampling strategies.
     *     So we don't do recursion for light sampling. Instead, we sum of the contribution
     *     of each light, and terminate.
     * @note Thus, instead of letting the sampling function decide which sampling strategy to use,
     *     we "flip a coin" here and call different function for BRDF sampling and light sampling
     */
    #pragma region one_sample_MIS
    // flip a coin
    bool pickLight = next_pcg32_real<double>(rng) <= 0.0;
    Vector3 out_dir, brdfValue;
    Real brdf_PDF, light_PDF;
    Vector3 in_dir = -localRay.dir;
    ray outRay(rec.pos, Vector3(1.0, 0.0, 0.0));  // set dir to out_dir later
    if (pickLight) {
        assert(false);
    }
    else {
        tie(out_dir, brdfValue, brdf_PDF) = BRDF_sample(&currMaterial, rec, rng, in_dir);
        outRay.dir = out_dir;
        // "fake" choose a light
        light_PDF = alternative_light_pdf(outRay, scene, root, rec.pos);
    }
    // do recursion
    return L_emmision + brdfValue * (2.0 / (brdf_PDF + light_PDF)) *
            radiance(scene, outRay, root, rng, recDepth-1);
    #pragma endregion one_sample_MIS

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


Sample BRDF_sample(Material* currMaterial, Hit_Record& rec, 
            pcg32_state& rng, const Vector3& in_dir)
{
    // our return values
    Vector3 out_dir, brdfValue;
    Real pdf;

    if (Diffuse* diffuseMat = get_if<Diffuse>(currMaterial)) {
        // HW4 UPDATE: path tracing Diffuse
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);

        out_dir = dir_cos_sample(rng, basis);
        Real cosTerm = dot(rec.normal, out_dir);
        brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI;
        pdf = cosTerm * c_INVPI;  // cosTerm / PI

        return {out_dir, brdfValue, pdf};
        
    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(currMaterial)) {
        out_dir = mirror_dir(in_dir, rec.normal);
        double cos_theta = dot(rec.normal, out_dir);

        // decide between mirror-like and diffuse-like
        double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);
        bool traceSpecular = next_pcg32_real<Real>(rng) < F;
        if (traceSpecular) {
            return {out_dir, Vector3(1.0, 1.0, 1.0), 1.0};
        } else {
            // same as diffuse
            Basis basis = Basis::orthonormal_basis(rec.normal);
            Vector3 Kd = eval_RGB(plasticMat->reflectance, rec.u, rec.v);
            // update out_dir
            out_dir = dir_cos_sample(rng, basis);
            Real cosTerm = dot(rec.normal, out_dir);
            brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI;
            pdf = cosTerm * c_INVPI;  // cosTerm / PI

            return {out_dir, brdfValue, pdf};
        }
    }
    else if (Phong* phongMat = get_if<Phong>(currMaterial)) {
        // sample dir first, this time around mirror ray direction
        out_dir = mirror_dir(in_dir, rec.normal);
        Basis basis = Basis::orthonormal_basis(out_dir);
        Vector3 sample_dir = dir_Phong_sample(rng, basis, phongMat->exponent);

        // check dot(hitting normal, sample direction)
        if (dot(rec.normal, sample_dir) <=0 ) {
            // This could happen because the sample direction is not around 
            // hitting normal anymore
            return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0};
        }

        // compute Phong BRDF
        Vector3 Ks = eval_RGB(phongMat->reflectance, rec.u, rec.v);
        Real cosTerm = dot(out_dir, sample_dir);
        brdfValue = Ks * (phongMat->exponent + 1.0) * c_INVTWOPI *
                pow(std::max(cosTerm, 0.0), phongMat->exponent);

        // compute Phong pdf
        pdf = (phongMat->exponent + 1.0) * c_INVTWOPI *
                pow(cosTerm, phongMat->exponent);

        return {out_dir, brdfValue, pdf};
    }
    else if (BlinnPhong* blphMat = get_if<BlinnPhong>(currMaterial)) {
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
        out_dir = mirror_dir(in_dir, sample_h);
        // check dot(hitting normal, out_dir) 
        if (dot(rec.normal, out_dir) <= 0.0) {
            return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0};
        }
        // compute BlinnPhong BRDF
        brdfValue = blphMat->compute_BRDF(sample_h, out_dir, rec);
        // compute BlinnPhong PDF
        pdf = blphMat->compute_PDF(sample_h, out_dir, rec);

        return {out_dir, brdfValue, pdf};
    }
    else if (BlinnPhongMicrofacet* micro_blphMat = get_if<BlinnPhongMicrofacet>(currMaterial)) {
        // sample half vector h to get out_dir AND estimate D(h)
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_h = dir_Phong_sample(rng, basis, micro_blphMat->exponent);
        // reflect in_dir over h to get out_dir
        out_dir = mirror_dir(in_dir, sample_h);
        // check dot(hitting normal, out_dir) 
        if (dot(rec.normal, out_dir) <= 0.0) {
            return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0};
        }

        // compute BlinnPhongMicrofacet BRDF;
        // note: in_dir w_i should align with shading normal
        brdfValue = micro_blphMat->compute_BRDF(
            sample_h, in_dir, out_dir, rec);
        
        // compute BlinnPhongMicrofacet PDF;
        pdf = micro_blphMat->compute_PDF(sample_h, out_dir, rec);

        return {out_dir, brdfValue, pdf};
    }
    else {
        Error("Material Unknown; will implement later");
        return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0}; 
    }
}


Real alternative_light_pdf(ray& outRay, Scene& scene, BVH_node& root,  // determine the light
            const Vector3& shadingPos)
{
    // Step 1: detect hit. 
    Hit_Record rec;
    Shape* lightObj = nullptr;
    root.hit(outRay, EPSILON, infinity<Real>(), scene, rec, lightObj);
    if (!lightObj || !is_light(*lightObj)) {  // no hit OR hitObj is not Area Light
        return 0.0;
    }

    Vector3 out_dir = outRay.dir;
    Vector3 light_pos = rec.pos;
    Real dsq = distance_squared(light_pos, shadingPos);
    int n = scene.lights.size();

    if (const Sphere *sph = get_if<Sphere>(lightObj)) {
        // spherical cone area: 2PI * (1-cos_theta)
        Real cos_theta_max = sph->radius / distance(shadingPos, sph->position);
        Real area = c_TWOPI * (1.0 - cos_theta_max);
        Real positive_cosine = abs(dot(out_dir, sph->normal_at(light_pos)));
        // probability of choosing this light is 1/n
        return dsq / (area * positive_cosine * n);
    }
    else if (const Triangle *tri = get_if<Triangle>(lightObj)) {
        Real positive_cosine = abs(dot(out_dir, tri->normal));
        // I. probability of choosing this mesh light is 1/n
        // II. probability of choosing this triangle from the mesh is the area/total_area ratio
        // III. probability of choosing this point from the triangle is 1/area
        // put together, we need 1 / (total area * n)
        Real total_area = scene.meshes[tri->mesh_id].totalArea;
        return dsq / (total_area * positive_cosine * n);   
    }
    
}
