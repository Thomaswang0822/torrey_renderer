#pragma once

#include "parse_scene.h"

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
        img3(imread3(pImgTex.filename)) {}


    /**
     * @brief Given a uv position, return a Vector3 RGB on the texture image
     * 
     */
    Vector3 color_value(double u, double v) {
        // null guardian
        if (img3.data.size() == 0) {
            // TODO: change to scene background color
            return Vector3(0.5, 0.5, 0.5);
        }
        
        // convert
        int x = img3.width * static_cast<int>( modulo(uscale * u + uoffset, 1.0) );
        int y = img3.height * static_cast<int>( modulo(vscale * v + voffset, 1.0) );
        // Very rare case, since actual coordinates should be less than 1.0
        if (x == img3.width)  { x--; }
        if (y == img3.height) { y--; }

        // return color
        const Real color_scale = 1.0 / 255.0;
        return img3(x, y) * color_scale;
    }
};

using Color = std::variant<Vector3 /* RGB */, ImageTexture>;

struct Diffuse {
    Color reflectance;
};

struct Mirror {
    Color reflectance;
};

struct Plastic {
    Real eta; // index of refraction
    Color reflectance;
};

struct Phong {
    Color reflectance; // Ks
    Real exponent; // alpha
};

struct BlinnPhong {
    Color reflectance; // Ks
    Real exponent; // alpha
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


