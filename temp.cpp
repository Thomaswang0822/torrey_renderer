Sample arealight_sample(Scene& scene, Hit_Record& rec, BVH_node& root, DiffuseAreaLight* areaLight,
            Material& currMaterial, pcg32_state& rng, const Vector3& in_dir)
{
    // our return values
    Vector3 brdfValue(0.0, 0.0, 0.0);
    Real pdf_Light = 0.0; Real pdf_BRDF = 1.0;

    // local variables
    int n = scene.lights.size();
    Vector3 light_pos;
    Vector3 Kd, I; // Kd and lightIntensity
    Vector3 h; // half-vector for BlinnPhong and Microfacet useage

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
            return {Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), 1.0};
        }

        // probability of choosing this light is 1/n
        pdf_Light = 1.0 / (area * n);  // 4
        
    } else if (const Triangle* leading_tri = get_if<Triangle>(lightObj)) {
        // pick a Triangle from the mesh
        TriangleMesh& mesh = scene.meshes[leading_tri->mesh_id];
        int local_idx = mesh.which_tri(next_pcg32_real<double>(rng));  // index in the mesh
        // shape_id points to the first (0-th) triangle in the mesh
        Triangle* tri = get_if<Triangle>(&scene.shapes[areaLight->shape_id + local_idx]);

        // pick a point from the Triangle
        light_pos = Triangle_sample(tri, rng);

        // shadow test
        if (!isVisible(rec.pos, light_pos, scene, root)) {
            // cout << rec.pos << "\t" << light_pos << endl;
            // brdfValue will be 0, pdf will not matter
            return {Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), 1.0};
        }

        // write return values
        // I. probability of choosing this mesh light is 1/n
        // II. probability of choosing this triangle from the mesh is the area/total_area ratio
        // III. probability of choosing this point from the triangle is 1/area
        // put together, we need 1 / (total area * n)
        Real total_area = scene.meshes[tri->mesh_id].totalArea;
        pdf_Light =  1.0 / (total_area * n);  // 4 
    }

    
    // need dot(rec.normal, sample_dir) check
    Vector3 out_dir = normalize(light_pos - rec.pos);
    Vector3 mir_dir = mirror_dir(in_dir, rec.normal);
    Real cosTerm = dot(rec.normal, out_dir);
    bool isPossible = cosTerm > 0;
    // for BlinnPhong and microfacet
    h = normalize(in_dir + out_dir);

    // switch material, calculate brdfValue (light) and pdf_BRDF
    if (Diffuse* diffuseMat = get_if<Diffuse>(&currMaterial)) {
        Kd = eval_RGB(diffuseMat->reflectance, rec.u, rec.v);
        brdfValue = Kd * std::max(cosTerm, 0.0) * c_INVPI;  // 2
    }
    else if (Plastic* plasticMat = std::get_if<Plastic>(&currMaterial)) {
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
    else if (Phong* phongMat = get_if<Phong>(&currMaterial)) {
        Kd = eval_RGB(phongMat->reflectance, rec.u, rec.v);

        // Phong needs special check because it samples around mir_dir
        cosTerm = dot(mir_dir, out_dir);
        isPossible = isPossible && (cosTerm > 0);
        brdfValue = phongMat->compute_BRDF(cosTerm, rec);  // 2
    }
    else if (BlinnPhong* blphMat = get_if<BlinnPhong>(&currMaterial)) {
        Kd = eval_RGB(blphMat->reflectance, rec.u, rec.v);
        brdfValue = blphMat->compute_BRDF(h, out_dir, rec);  // 2
    }
    else if (BlinnPhongMicrofacet* micro_blphMat = get_if<BlinnPhongMicrofacet>(&currMaterial)) {
        Kd = eval_RGB(micro_blphMat->reflectance, rec.u, rec.v);
        brdfValue = micro_blphMat->compute_BRDF(h, in_dir, out_dir, rec);  // 2
    }
    else {
        Error("Shading point UNKNOWN material.");
    }


    return {out_dir, brdfValue, pdf_Light};
}

