#include "compute_radiance.h"

using namespace std;

// HW3 Update: deal with ImageTexture Color & Area light
Vector3 BVH_DiffuseColor(Scene& scene, Hit_Record& rec, const Color& refl, 
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
                // average
                // cout << "This area light contributes: " << total_contribution << endl;
                result += total_contribution /* / Real(meshCt) */;
            }
        }
        
    }
    return result;
}

// HW4_2 Update: can deal with BRDF sampling of all materials
Vector3 BVH_PixelColor(Scene& scene, ray& localRay, BVH_node& root, 
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
        // return BVH_DiffuseColor(scene, rec, diffuseMat->reflectance, root, hitObj, rng);

        // HW4 UPDATE: path tracing Diffuse
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_dir = dir_cos_sample(rng, basis);
        Vector3 Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);
        ray scatterRay = ray(rec.pos, sample_dir);
        // cout << dot(rec.normal, sample_dir) << endl;
        Real cosTerm = dot(rec.normal, sample_dir);
        return L_emmision + (Kd * std::max(cosTerm, 0.0) * c_INVPI)
            * c_PI / cosTerm  // inverse of cosine hemisphere pdf
            * BVH_PixelColor(scene, scatterRay, root, rng, recDepth-1);
    }
    else if (Mirror* mirrorMat = std::get_if<Mirror>(&currMaterial)) {
        // mirror refect ray and do recursion
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);

        // ######### perfect mirror #########
        // Vector3* mirrorColor = std::get_if<Vector3>(&mirrorMat->reflectance);
        // assert(mirrorColor && "Mirror material has reflectance not Vec3 RGB");
        // return *mirrorColor // color at current hitting pt
        //     * BVH_PixelColor(scene, rayOut, root, recDepth=recDepth-1);   // element-wise mutiply

        // ######### hw3 Fresnel reflection  #########
        double cos_theta = dot(rec.normal, rayOut.dir);
        // Vector3 F * mirror recursion
        return mirror_SchlickFresnel_color(mirrorMat->reflectance, rec.u, rec.v, cos_theta) 
            * BVH_PixelColor(scene, rayOut, root, rng, recDepth=recDepth-1);

    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(&currMaterial)) {
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);
        double cos_theta = dot(rec.normal, rayOut.dir);

        // double F0 = plasticMat->get_F0();
        double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);

        /* // Vector3 F * mirror recursion + (1 − F)diffuse,
        return F * BVH_PixelColor(scene, rayOut, root, rng, recDepth=recDepth-1) +
            (1.0 - F) * BVH_DiffuseColor(scene, rec, plasticMat->reflectance, root, hitObj, rng); */

        // HW4 UPDATE: instead of tracing both,
        // we decide between specular and diffuse with Stochastic Probability F vs (1-F)
        bool traceSpecular = next_pcg32_real<Real>(rng) < F;
        if (traceSpecular) {
            return L_emmision + BVH_PixelColor(scene, rayOut, root, rng, recDepth-1);
        } else {
            Basis basis = Basis::orthonormal_basis(rec.normal);
            Vector3 sample_dir = dir_cos_sample(rng, basis);
            Vector3 Kd = eval_RGB(plasticMat->reflectance, rec.u, rec.v);
            ray scatterRay = ray(rec.pos, sample_dir);
            // cout << dot(rec.normal, sample_dir) << endl;
            Real cosTerm = dot(rec.normal, sample_dir);
            return L_emmision + (Kd * std::max(cosTerm, 0.0) * c_INVPI)
                * c_PI / cosTerm  // inverse of cosine hemisphere pdf
                * BVH_PixelColor(scene, scatterRay, root, rng, recDepth-1);
        }
    }
    else if (Phong* phongMat = get_if<Phong>(&currMaterial)) {
        // sample dir first, this time around mirror ray direction
        ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);
        Vector3 r = rayOut.dir;
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
            * BVH_PixelColor(scene, scatterRay, root, rng, recDepth-1);
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
        Vector3 out_dir = scatterRay.dir;
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
            * BVH_PixelColor(scene, scatterRay, root, rng, recDepth-1);
    }
    else if (Microfacet* micro_blphMat = get_if<Microfacet>(&currMaterial)) {
        // sample half vector h to estimate D(h)
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_h = dir_Phong_sample(rng, basis, micro_blphMat->exponent);

        // reflect in_dir over h to get out_dir
        ray scatterRay = mirror_ray(localRay, sample_h, rec.pos);
        Vector3 out_dir = scatterRay.dir;
        Vector3 in_dir = -localRay.dir;
        
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
            * BVH_PixelColor(scene, scatterRay, root, rng, recDepth-1);
    }
    else {
        Error("Material Unknown; will implement later");
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


#pragma region PATH_TRACING  // HW4_3

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
    if (!hitObj) {  // no hit
        return scene.background_color;
    }
    
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
    if (Mirror* mirrorMat = get_if<Mirror>(&currMaterial)) {
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
    bool pickLight = next_pcg32_real<double>(rng) <= 0.5;
    Vector3 out_dir, brdfValue;
    Real brdf_PDF, light_PDF;
    Vector3 in_dir = -localRay.dir;
    ray outRay(rec.pos, Vector3(1.0, 0.0, 0.0));  // set dir to out_dir later
    if (pickLight) {
        // there is no light to choose from -> nothing to do
        if (scene.lights.size() == 0) {
            return Vector3(0.0, 0.0, 0.0);
        }
        else if (is_light(*hitObj)) {
            return L_emmision;
        }

        tie(out_dir, brdfValue, brdf_PDF, light_PDF) = 
            Light_sample_dir(scene, rec, root, currMaterial, rng, in_dir);
        outRay.dir = out_dir;
        // cout << brdf_PDF << "\t" << light_PDF << "\t" << brdfValue << endl;
    }
    else {
        // BRDF sampling
        /* if (Plastic* plasticMat = get_if<Plastic>(&currMaterial)){
            ray rayOut = mirror_ray(localRay, rec.normal, rec.pos);
            double cos_theta = dot(rec.normal, rayOut.dir);

            // decide between mirror-like and diffuse-like
            double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);
            if (next_pcg32_real<Real>(rng) < F) {
                // We only consider specular component in BRDF sampling, 50% of time
                return 2.0 * radiance(scene, rayOut, root, rng, recDepth=recDepth-1);
            }
        } */
        tie(out_dir, brdfValue, brdf_PDF, light_PDF) = BRDF_sample_dir(currMaterial, rec, rng, in_dir);
        outRay.dir = out_dir;
        if (get_if<Plastic>(&currMaterial) && closeToZero(light_PDF-1.0)) {
            // brdfValue = {1.0, 1.0, 1.0}, pdf_BRDF = 1.0; pdf_Light = 0.0
            light_PDF = 0.0;
        } else {
            // "fake" choose a light: if no light, this is 0
            light_PDF = alternative_light_pdf(outRay, scene, root, rec.pos);
        }

        // cout << brdf_PDF << "\t" << light_PDF << "\t" << brdfValue << endl;
    }
    
    // uncomment this region AND change pickLight to false (by random <= 0.0)
    // to turn off MIS
    #pragma region hw_4_1-2
    // return L_emmision + brdfValue * (1.0 / brdf_PDF) *
    //         radiance(scene, outRay, root, rng, recDepth-1);
    #pragma endregion hw_4_1-2

    // return L_emmision + brdfValue * (1.0 / light_PDF) *
    //         radiance(scene, outRay, root, rng, recDepth-1);

    // do recursion
    return L_emmision + brdfValue * (2.0 / (brdf_PDF + light_PDF)) *
            radiance(scene, outRay, root, rng, recDepth-1);
    #pragma endregion one_sample_MIS

}


Sample BRDF_sample_dir(Material& currMaterial, Hit_Record& rec, 
            pcg32_state& rng, const Vector3& in_dir)
{
    // our return values
    Vector3 out_dir, brdfValue;
    Real pdf;

    // local var
    Vector3 mir_dir;

    if (Diffuse* diffuseMat = get_if<Diffuse>(&currMaterial)) {
        // HW4 UPDATE: path tracing Diffuse
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);

        out_dir = dir_cos_sample(rng, basis);
        Real cosTerm = dot(rec.normal, out_dir);
        brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI;
        pdf = cosTerm * c_INVPI;  // cosTerm / PI

        return {out_dir, brdfValue, pdf, 0.0};
        
    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(&currMaterial)) {
        // compute F, but specular case is handled outside
        double cos_theta = dot(rec.normal, in_dir);
        double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);
        if (next_pcg32_real<Real>(rng) < F) {
            Vector3 mir_dir = mirror_dir(in_dir, rec.normal);
            // We only consider specular component in BRDF sampling, 50% of time
            return {mir_dir, {1.0, 1.0, 1.0}, 1.0, 1.0};
        }
        // sample a dir (diffuse)
        Basis basis = Basis::orthonormal_basis(rec.normal);
        out_dir = dir_cos_sample(rng, basis);

        Real cosTerm = dot(rec.normal, out_dir);
        pdf = cosTerm * c_INVPI * (1.0-F);  // 3
        brdfValue = plasticMat->compute_BRDF_diffuse(cosTerm, rec) * (1.0-F);  // 2
        return {out_dir, brdfValue, pdf, 0.0};
    }
    else if (Phong* phongMat = get_if<Phong>(&currMaterial)) {
        // sample dir first, this time around mirror ray direction
        mir_dir = mirror_dir(in_dir, rec.normal);
        Basis basis = Basis::orthonormal_basis(mir_dir);
        out_dir = dir_Phong_sample(rng, basis, phongMat->exponent);

        // check dot(hitting normal, sample direction)
        if (dot(rec.normal, out_dir) <=0 ) {
            // This could happen because the sample direction is not around 
            // hitting normal anymore
            return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0, 0.0};
        }

        // compute Phong BRDF
        Real cosTerm = dot(mir_dir, out_dir);
        brdfValue = phongMat->compute_BRDF(cosTerm, rec);

        // compute Phong pdf
        pdf = phongMat->compute_PDF(cosTerm);

        return {out_dir, brdfValue, pdf, 0.0};
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
        out_dir = mirror_dir(in_dir, sample_h);
        // check dot(hitting normal, out_dir) 
        if (dot(rec.normal, out_dir) <= 0.0) {
            return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0, 0.0};
        }
        // compute BlinnPhong BRDF
        brdfValue = blphMat->compute_BRDF(sample_h, out_dir, rec);
        // compute BlinnPhong PDF
        pdf = blphMat->compute_PDF(sample_h, out_dir, rec);

        return {out_dir, brdfValue, pdf, 0.0};
    }
    else if (Microfacet* micro_blphMat = get_if<Microfacet>(&currMaterial)) {
        // sample half vector h to get out_dir AND estimate D(h)
        Basis basis = Basis::orthonormal_basis(rec.normal);
        Vector3 sample_h = dir_Phong_sample(rng, basis, micro_blphMat->exponent);
        // Vector3 sample_h = dir_GGX_sample(rng, basis, micro_blphMat->exponent);

        // reflect in_dir over h to get out_dir
        out_dir = mirror_dir(in_dir, sample_h);
        // check dot(hitting normal, out_dir) 
        if (dot(rec.normal, out_dir) <= 0.0) {
            return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0, 0.0};
        }

        // compute BlinnPhongMicrofacet BRDF;
        // note: in_dir w_i should align with shading normal
        brdfValue = micro_blphMat->compute_BRDF(sample_h, in_dir, out_dir, rec);
        // brdfValue = micro_blphMat->compute_BRDF_GGX(sample_h, in_dir, out_dir, rec);
        
        // compute BlinnPhongMicrofacet PDF;
        pdf = micro_blphMat->compute_PDF(sample_h, out_dir, rec);
        // pdf = micro_blphMat->compute_PDF_GGX(sample_h, out_dir, rec);

        return {out_dir, brdfValue, pdf, 0.0};
    }
    else {
        Error("Material Unknown; will implement later");
        return {out_dir, Vector3(0.0, 0.0, 0.0), 1.0, 0.0}; 
    }
}


Sample Light_sample_dir(Scene& scene, Hit_Record& rec, BVH_node& root,
            Material& currMaterial, pcg32_state& rng, const Vector3& in_dir,
            int given_id)
{
    // our return values
    Vector3 out_dir, brdfValue;
    Real pdf_BRDF = 0.0; Real pdf_Light = 0.0;

    // local variables
    Vector3 light_pos;
    Real dsq;  // d^2
    Vector3 Kd, I; // Kd and lightIntensity
    Vector3 h; // half-vector for BlinnPhong and Microfacet useage

    // uniformly pick a light OR use the given light
    int n = scene.lights.size();
    int lightId = (given_id == -1)?
        static_cast<int>(n * next_pcg32_real<double>(rng)) : given_id;
    Light& light = scene.lights[lightId];
    assert(!get_if<PointLight>(&light) && "One-sample MIS doesn't support point Light.");
    DiffuseAreaLight* areaLight = get_if<DiffuseAreaLight>(&light);
    assert(areaLight && "areaLight is a nullptr.");

    // get the shape of the light
    const Shape* lightObj = &scene.shapes[areaLight->shape_id];
    I = areaLight->radiance;  
    // sample a point and compute related values according to Triangle / Sphere
    if (const Sphere* sph = get_if<Sphere>(lightObj)) {
        Vector3 cp = rec.pos - sph->position;
        assert(length(cp) > sph->radius
            && "hit position is inside a sphere");
        // spherical cone area: 2PI * (1-cos_theta)
        Real cos_theta_max = sph->radius / distance(rec.pos, sph->position);
        Real area = c_TWOPI * (1.0 - cos_theta_max) * sph->radius * sph->radius;
        // do cone sampling
        light_pos = Sphere_sample_cone(sph, rng, cos_theta_max, normalize(cp));

        // shadow test
        if (!isVisible(rec.pos, light_pos, scene, root)) {
            // cout << rec.pos << "\t" << light_pos << endl;
            // brdfValue will be 0, pdf will not matter
            return {normalize(light_pos - rec.pos), Vector3(0.0, 0.0, 0.0),
                1.0, 1.0};
        }

        // write return values
        out_dir = normalize(light_pos - rec.pos);  // 1
        Real positive_cosine = abs(dot(out_dir, sph->normal_at(light_pos)));
        dsq = distance_squared(light_pos, rec.pos);
        // probability of choosing this light is 1/n
        pdf_Light = dsq / (area * positive_cosine * n);  // 4
        
    } else if (const Triangle* leading_tri = get_if<Triangle>(lightObj)) {
        // pick a Triangle from the mesh
        TriangleMesh& mesh = scene.meshes[leading_tri->mesh_id];
        int local_idx = mesh.which_tri(next_pcg32_real<double>(rng));  // index in the mesh
        // shape_id points to the first (0-th) triangle in the mesh
        Triangle* tri = get_if<Triangle>(&scene.shapes[areaLight->shape_id + local_idx]);

        // pick a point from the Triangle
        light_pos = Triangle_sample(tri, rng);
        // light_pos = SphTri_sample(tri, rng, rec);

        // shadow test
        if (!isVisible(rec.pos, light_pos, scene, root)) {
            // cout << rec.pos << "\t" << light_pos << endl;
            // brdfValue will be 0, pdf will not matter
            return {normalize(light_pos - rec.pos), Vector3(0.0, 0.0, 0.0),
                1.0, 1.0};
        }

        // write return values
        out_dir = normalize(light_pos - rec.pos);  // 1
        Real positive_cosine = abs(dot(out_dir, tri->normal));
        dsq = distance_squared(light_pos, rec.pos);
        // I. probability of choosing this mesh light is 1/n
        // II. probability of choosing this triangle from the mesh is the area/total_area ratio
        // III. probability of choosing this point from the triangle is 1/area
        // put together, we need 1 / (total area * n)
        Real total_area = scene.meshes[tri->mesh_id].totalArea;
        pdf_Light =  dsq / (total_area * positive_cosine * n);  // 4 
    }

    
    // need dot(rec.normal, sample_dir) check
    Real cosTerm = dot(rec.normal, out_dir);
    bool isPossible = cosTerm > 0;
    // for BlinnPhong and microfacet
    h = normalize(in_dir + out_dir);

    // switch material, calculate brdfValue (light) and pdf_BRDF
    if (Diffuse* diffuseMat = get_if<Diffuse>(&currMaterial)) {
        Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);
        brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI;  // 2
        pdf_BRDF = std::max(cosTerm, 0.0) * c_INVPI;  // 3
    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(&currMaterial)) {
        // compute F
        double cos_theta = dot(rec.normal, in_dir);
        double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);
        pdf_BRDF = cosTerm * c_INVPI * (1.0-F);  // 3
        brdfValue = plasticMat->compute_BRDF_diffuse(cosTerm, rec) * (1.0-F);  // 2
    }
    else if (Phong* phongMat = get_if<Phong>(&currMaterial)) {
        Kd = eval_RGB(phongMat->reflectance, rec.u, rec.v);
        

        // Phong needs special check because it samples around mir_dir
        Vector3 mir_dir = mirror_dir(in_dir, rec.normal);
        cosTerm = dot(mir_dir, out_dir);
        isPossible = isPossible && (cosTerm > 0);
        brdfValue = phongMat->compute_BRDF(cosTerm, rec);  // 2
        pdf_BRDF = isPossible? phongMat->compute_PDF(cosTerm) : 0.0;  // 3
    }
    else if (BlinnPhong* blphMat = get_if<BlinnPhong>(&currMaterial)) {
        Kd = eval_RGB(blphMat->reflectance, rec.u, rec.v);
        brdfValue = blphMat->compute_BRDF(h, out_dir, rec);  // 2
        pdf_BRDF = isPossible? blphMat->compute_PDF(h, out_dir, rec) : 0.0;  // 3
    }
    else if (Microfacet* micro_blphMat = get_if<Microfacet>(&currMaterial)) {
        Kd = eval_RGB(micro_blphMat->reflectance, rec.u, rec.v);
        brdfValue = micro_blphMat->compute_BRDF(h, in_dir, out_dir, rec);  // 2
        pdf_BRDF = isPossible? micro_blphMat->compute_PDF(h, out_dir, rec) : 0.0;
        // brdfValue = micro_blphMat->compute_BRDF_GGX(h, in_dir, out_dir, rec);  // 2
        // pdf_BRDF = isPossible? micro_blphMat->compute_PDF_GGX(h, out_dir, rec) : 0.0;  // 3
    }
    else {
        Error("Shading point UNKNOWN material.");
    }


    return {out_dir, brdfValue, pdf_BRDF, pdf_Light};
}


Real alternative_light_pdf(ray& outRay, Scene& scene, BVH_node& root,  // determine the light
            const Vector3& shadingPos)
{
    // detect hit. 
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
        Real area = c_TWOPI * (1.0 - cos_theta_max) * sph->radius * sph->radius;
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
    
    Error("Should never reach here: unknown lightObj");
    return 0.0;
}


Vector3 radiance_iterative(Scene& scene, ray& localRay, BVH_node& root, 
                        pcg32_state& rng, unsigned int recDepth)
{
    // Step 0. Init variables
    Vector3 L(0.0, 0.0, 0.0);
    Vector3 Beta(1.0, 1.0, 1.0); // accumulated brdf_value * cosine_term
    bool specularBounce = false;
    double nLights = static_cast<double>(scene.lights.size());
    
    for (unsigned int bounces=0; bounces <recDepth; bounces++) {
        #pragma region in_loop
// Step 1. Intersect ray with scene and store intersection in rec
Hit_Record rec;
Shape* hitObj = nullptr;
root.hit(localRay, EPSILON, infinity<Real>(), scene, rec, hitObj);

// Step 2. <<Terminate path if ray escaped or maxDepth was reached>>= 
if (hitObj == nullptr) {
    L += Beta * scene.background_color;
    break;
}

// Step 3. Possibly add emitted light at intersection if
//     a. this is camera ray
//     b. current bounce is a specular (mirror) which cannot account for light
if (bounces == 0 || specularBounce) {
    // PBRT will add contribution from infinite area light to 
    //     no-hit ray, but we don't support it. 
    if (is_light(*hitObj)) {
        int self_light_id = get_area_light_id(*hitObj);
        DiffuseAreaLight& self_light = get<DiffuseAreaLight>(scene.lights[self_light_id]);
        L += Beta * self_light.radiance;
    }
}

// Step 4. Sample illumination from lights to find path contribution
Material& currMaterial = scene.materials[rec.mat_id];
// if hit && self not a light && not a mirror
if (!is_light(*hitObj) && !get_if<Mirror>(&currMaterial))
    L += Beta * nLights *
        sample_oneLight_contribution(scene, rec, root, rng, currMaterial, -localRay.dir);

// Step 5. Sample BSDF to get new path direction
Vector3 in_dir = -localRay.dir;
Vector3 out_dir, brdfValue;
Real brdf_PDF, light_PDF;
// special care for specular/mirror material
if (Mirror* mirrorMat = get_if<Mirror>(&currMaterial)) {
    // mirror refect ray
    localRay = mirror_ray(localRay, rec.normal, rec.pos);
    double cos_theta = dot(rec.normal, localRay.dir);
    // Vector3 F * mirror recursion
    Beta *= mirror_SchlickFresnel_color(mirrorMat->reflectance, rec.u, rec.v, cos_theta);
    specularBounce = true;
}
else {
    specularBounce = false;
    tie(out_dir, brdfValue, brdf_PDF, light_PDF) = BRDF_sample_dir(currMaterial, rec, rng, in_dir);
    // don't bother with too-little contribution OR too-low pdf (for numerical stability)
    if(closeToZero(brdfValue) || closeToZero(brdf_PDF)) {break;}
    Beta *= brdfValue / brdf_PDF;  // our brdfValue includes the cosine term.
    localRay = ray(rec.pos, out_dir);  // update to next ray
}


// Step 6. Possibly terminate the path with Russian roulette
if (bounces > 3) {
    // A minimum termination probability 0.05 
    // ensures termination is possible even if Beta is large
    Real q = std::max(0.05, 1.0-Luminance(Beta));
    if (next_pcg32_real<Real>(rng) < q)
        break;
    Beta /= 1.0 - q;
}
        #pragma endregion in_loop
    }  // end of for loop
    
    return L;

}


Vector3 sample_oneLight_contribution(Scene& scene, Hit_Record& rec, 
        BVH_node& root, pcg32_state& rng,
        Material& mat, const Vector3& in_dir)
{
    // our return values: brdf * L(deterministic) / pdf_light
    Vector3 Ld(0.0, 0.0, 0.0);
    Vector3 brdfValue(0.0, 0.0, 0.0);  // safely return 0 when occuluded
    Vector3 Li(0.0, 0.0, 0.0);
    Real pdf_Light = 1.0; Real pdf_BRDF = 1.0;
    Real weight;  // balance heuristic

    // uniformly pick a light, which can be 
    // [point light, Sphere area light, TriangleMesh area light]
    int nLights = scene.lights.size();
    int pick_id = static_cast<int>(nLights * next_pcg32_real<double>(rng));
    Light& light = scene.lights[pick_id];

    // compute accordingly
    Vector3 out_dir;
    Real dsq;  // distance squared: shadingPt to light position
    Real abs_cos;  // | <n_x, out_dir> |

    
    // contribution is:
    //    BRDF * L when we are doing importance sampling and tracing ray
    //    BRDF * I * max(dot(-n_x, l), 0) / d^2 * visibility
    if (PointLight* ptLight = std::get_if<PointLight>(&light)) {
        out_dir = normalize(ptLight->position - rec.pos);
        dsq = distance_squared(rec.pos, ptLight->position);
        if (isVisible(rec.pos, ptLight->position, scene, root)) { // 2
            Li =  ptLight->intensity * abs(dot(rec.normal, out_dir)) / dsq;
        }
        brdfValue = compute_f_ptLight(mat, in_dir, ptLight->position, rec);  // 1
        pdf_Light = 1.0 / nLights;  // 3
        Ld += brdfValue * Li / pdf_Light;
    }
    else if (DiffuseAreaLight* areaLight = get_if<DiffuseAreaLight>(&light)) {
        if (scene.lights.size() == 0) {
            return Vector3(0.0, 0.0, 0.0);
        }
        #pragma region lightSample
        // NOTE: it gives us 2 pdfs in dir(solid angle) measurement. Should turn back to area measurement
        tie(out_dir, brdfValue, pdf_BRDF, pdf_Light) = 
            Light_sample_dir(scene, rec, root, mat, rng, in_dir, pick_id);  // 1 & 3
        
        // with a dir instead of a position, we use hit() to do visibility check
        Hit_Record rec_light;
        Shape* lightObj = nullptr;
        ray lightRay(rec.pos, out_dir);
        root.hit(lightRay, EPSILON, infinity<Real>(), scene, rec_light, lightObj);
        if (lightObj != nullptr) {
            // cout << rec_light.normal << length(rec_light.normal) << endl;
            dsq = distance_squared(rec.pos, rec_light.pos);
            abs_cos = abs(dot(out_dir, rec_light.normal));
            abs_cos = std::max(dot(out_dir, -rec_light.normal), 0.0);
            Li = areaLight->radiance * abs_cos / dsq;  // 2

            // convert pdfs back to area measurement
            pdf_dir2pos(pdf_BRDF, pdf_Light, dsq, abs_cos);
            // the sample_dir() function includes probability of choosing
            // from n lights, but it shouldn't be included there.
            pdf_Light *= nLights;

            // actually it's f * Li / (pdf_L + pdf_BRDF), but it's good to keep a clear formula structure.
            weight = pdf_Light / (pdf_Light + pdf_BRDF);
            Ld += brdfValue * Li * weight / pdf_Light;
        }
        #pragma endregion lightSample


        #pragma region BRDFSample
        // Do BRDF sampling only for area light; now we have a new out_dir
        // NOTE: it gives us 2 pdfs in dir(solid angle) measurement. Should turn back to area measurement
        tie(out_dir, brdfValue, pdf_BRDF, pdf_Light) = BRDF_sample_dir(mat, rec, rng, in_dir);
            ray outRay(rec.pos, out_dir);

        if (get_if<Plastic>(&mat) && closeToZero(pdf_Light-1.0)) {
            // brdfValue = {1.0, 1.0, 1.0}, pdf_BRDF = 1.0; pdf_Light = 0.0
            pdf_Light = 0.0;
        } else {
            // "fake" choose a light: if no light, this is 0
            pdf_Light = alternative_light_pdf(outRay, scene, root, rec.pos);
        }

        Hit_Record rec_brdf;
        Shape* brdfObj = nullptr;
        ray brdfRay(rec.pos, out_dir);
        root.hit(brdfRay, EPSILON, infinity<Real>(), scene, rec_brdf, brdfObj);
        if (brdfObj != nullptr) {
            Triangle* tri = get_if<Triangle>(brdfObj);
            Sphere* sph = get_if<Sphere>(brdfObj);
            // only accumulate BRDF sampling contribution because this function is
            // "oneLight_contribution"
            if ((tri != nullptr && tri->area_light_id == pick_id) || 
                (sph != nullptr && sph->area_light_id == pick_id)) 
            {
                dsq = distance_squared(rec.pos, rec_brdf.pos);
                abs_cos = abs(dot(out_dir, rec_brdf.normal));
                abs_cos = std::max(dot(out_dir, -rec_brdf.normal), 0.0);
                // std::cout << dsq << "\t" << abs_cos << endl;
                Li = areaLight->radiance * abs_cos / dsq;  // 2

                // convert pdfs back to area measurement
                pdf_dir2pos(pdf_BRDF, pdf_Light, dsq, abs_cos);
                // the sample_dir() function includes probability of choosing
                // from n lights, but it shouldn't be included there.
                pdf_Light *= nLights;
                weight = pdf_BRDF / (pdf_Light + pdf_BRDF);
                Ld += brdfValue * Li * weight / pdf_BRDF;
                // Ld += brdfValue * Li / pdf_BRDF;
            }
        }
        #pragma endregion BRDFSample        
    }

    // cout << "contrib from one light: " << brdfValue * L / pdf_Light << endl;
    return Ld;
}



Vector3 compute_f_ptLight(Material& mat, const Vector3& in_dir,
            const Vector3& light_pos, Hit_Record& rec)
{
    
    // our return values
    Vector3 brdfValue;

    // local variables
    Vector3 mir_dir = mirror_dir(in_dir, rec.normal);
    Vector3 out_dir = normalize(light_pos - rec.pos);
    Vector3 Kd;

    // need dot(rec.normal, sample_dir) check
    Real cosTerm = dot(rec.normal, out_dir);
    bool isPossible = cosTerm > 0;
    // for BlinnPhong and microfacet
    Vector3 h = normalize(in_dir + out_dir);

    // switch material, calculate brdfValue (light) and pdf_BRDF
    if (Diffuse* diffuseMat = get_if<Diffuse>(&mat)) {
        Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);
        brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI;  // 2
    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(&mat)) {
        /**
         * @brief When treating plastic as specular, there is 0 probability
         * that our light sample could contribute to the shading point.
         * 
         * Thus, we will scale down the brdfValue by 1-F (prob of treaing as diffuse)
         */
        double cos_theta = dot(rec.normal, mir_dir);
        double F = compute_SchlickFresnel(plasticMat->get_F0(), cos_theta);
        Kd = eval_RGB(plasticMat->reflectance, rec.u, rec.v);
        brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI * (1.0-F);  // 2
    }
    else if (Phong* phongMat = get_if<Phong>(&mat)) {
        Kd = eval_RGB(phongMat->reflectance, rec.u, rec.v);
        // Phong needs special check because it samples around mir_dir
        Vector3 mir_dir = mirror_dir(in_dir, rec.normal);
        cosTerm = dot(mir_dir, out_dir);
        isPossible = isPossible && (cosTerm > 0);
        brdfValue = phongMat->compute_BRDF(cosTerm, rec);  // 2
    }
    else if (BlinnPhong* blphMat = get_if<BlinnPhong>(&mat)) {
        Kd = eval_RGB(blphMat->reflectance, rec.u, rec.v);
        brdfValue = blphMat->compute_BRDF(h, out_dir, rec);  // 2
    }
    else if (Microfacet* micro_blphMat = get_if<Microfacet>(&mat)) {
        Kd = eval_RGB(micro_blphMat->reflectance, rec.u, rec.v);
        brdfValue = micro_blphMat->compute_BRDF(h, in_dir, out_dir, rec);  // 2
    }
    else {
        Error("Shading point UNKNOWN material.");
    }

    return brdfValue;
}

#pragma endregion PATH_TRACING