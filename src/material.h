#pragma once

#include "parse_scene.h"
#include "Scene.h"



struct ImageTexture {
    fs::path filename;
    Real uscale = 1, vscale = 1;
    Real uoffset = 0, voffset = 0;

    ImageTexture(const ParsedImageTexture& pImgTex) :
        filename(pImgTex.filename) {}
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


