#pragma once

#include "parse_scene.h"

using namespace std;

/**
 * @brief One ImageTexture maps to one RGB img
 * 
 * @note: image data is written when the (only)
 * constructor is called.
 * 
 */
struct ImageTexture {
    Image3 img3;
    Real uscale = 1, vscale = 1;
    Real uoffset = 0, voffset = 0;

    ImageTexture(const ParsedImageTexture& pImgTex) :
        img3(imread3(pImgTex.filename)), 
        uscale(pImgTex.uscale), vscale(pImgTex.vscale),
        uoffset(pImgTex.uoffset), voffset(pImgTex.voffset) 
        {}


    /**
     * @brief Given a uv position, return a Vector3 RGB on the texture image
     * We do bilinear interpolation with the weighted mean approach.
     * @ref https://en.wikipedia.org/wiki/Bilinear_interpolation#Weighted_mean
     * 
     */
    Vector3 blerp_color(double u, double v) const {
        // null guardian
        if (img3.data.size() == 0) {
            // TODO: change to scene background color
            return Vector3(0.5, 0.5, 0.5);
        }
        
        // convert
        double x = img3.width  * modulo(uscale * u + uoffset, 1.0) - 1e-9;
        double y = img3.height * modulo(vscale * v + voffset, 1.0) - 1e-9;

        // obtain x1 y1 x2 y2
        int x1 = static_cast<int>(x); int y1 = static_cast<int>(y);
        int x2 = x1 + 1; int y2 = y1 + 1;
        // do bilinear interpolation, Weighted Mean approach
        // (x2 - x1) * (y2 - y1) = 1
        double w11 = (x2-x) * (y2-y);
        double w12 = (x2-x) * (y-y1);
        double w21 = (x-x1) * (y2-y);
        double w22 = (x-x1) * (y-y1);
        // wrap around
        return  w11 * img3(x1, y1) + 
                w12 * img3(x1, y2%img3.height) + 
                w21 * img3(x2%img3.width, y1) + 
                w22 * img3(x2%img3.width, y2%img3.height);

    }
};

using Color = std::variant<Vector3 /* RGB */, ImageTexture>;


/**
 * @brief Given a Color, return a Vector3 RGB
 * if Color is Vector3, then trivial case
 * if Color is ImageTexture, call funciton
 * 
 * @param refl 
 * @param u,v uv coordinate of hit record
 * @return Vector3 
 */
inline Vector3 eval_RGB(const Color& refl, const double u, const double v) {
    if (std::get_if<Vector3>(&refl)) {
        return std::get<Vector3>(refl);
    } 
    else if (const ImageTexture* txPtr = std::get_if<ImageTexture>(&refl)) {
        return txPtr->blerp_color(u, v);
    }
    else {
        Error("Diffuse material has reflectance not Vec3 RGB or ImageTexture");
    }
}

struct Diffuse {
    Color reflectance;
};

struct Mirror {
    Color reflectance;
};

inline double compute_SchlickFresnel(double F0, double cos_theta) {
    return F0 + (1.0 - F0) * pow(1.0 - cos_theta, 5.0);
}

/**
 * @brief Special handle for Mirror "component" or a Material.
 * Instead of directly multiplying the color with the reflectance, 
 * we multiply it with the Schlick Fresnel
 * Formula: F = F0 + (1 − F0) (1 − n · l)^5
 * 
 * @note We use the RGB value of a color as F0
 * 
 * @param refl reflectance attribute of any Material
 * @param u,v uv coordinate of hit record
 * @param cos_theta dot product between outgoing ray (to light) direction and hitting normal
 * @return Vector3 
 */
inline Vector3 mirror_SchlickFresnel_color(const Color& refl, 
        const double u, const double v, double cos_theta) {
    Vector3 F0 = eval_RGB(refl, u, v);
    return F0 + (-F0 + 1.0) * pow(1.0 - cos_theta, 5.0);
}

struct Plastic {
    Real eta; // index of refraction
    Color reflectance;

    inline Real get_F0() {
        return pow( (eta - 1.0) / (eta + 1.0), 2.0 );
    }
};

struct Phong {
    Color reflectance; // Ks
    Real exponent; // alpha
};

struct BlinnPhong {
    Color reflectance; // Ks
    Real exponent; // alpha

    /**
     * @brief Get the Fresnel calculated with half-vector h
     * 
     * @param h half-vector from random sampling
     * @param out_dir ray_in direction mirror-reflected over h
     * @param rec
     * @return Vector3 
     */
    Vector3 get_Fh(const Vector3& h, const Vector3& out_dir, Hit_Record& rec) {
        Vector3 Ks = eval_RGB(reflectance, rec.u, rec.v);
        return Ks + (1.0 - Ks) * pow(1.0-dot(h, out_dir), 5.0);
    }

    Vector3 BlinnPhong_BRDF(const Vector3& h, const Vector3& out_dir, Hit_Record& rec) {
        // check dot(hitting normal, out_dir) is outside:
        
        Real c = (exponent + 2.0) * c_INVFOURPI / (2.0 - pow(2.0, -exponent/2));
        Vector3 Fh = get_Fh(h, out_dir, rec);
        return c * Fh * pow(dot(rec.normal, h), exponent);
    }

    Real BlinnPhong_PDF(const Vector3& h, const Vector3& out_dir, Hit_Record& rec) {
        return (exponent + 1.0) * pow(dot(rec.normal, h), exponent) *
            c_INVTWOPI / (4.0 * dot(out_dir, h));
    }
};

struct BlinnPhongMicrofacet {
    Color reflectance; // Ks
    Real exponent; // alpha
};

using Material = std::variant<Diffuse,
                                    Mirror,
                                    Plastic,
                                    Phong,
                                    BlinnPhong,
                                    BlinnPhongMicrofacet>;


