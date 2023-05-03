#pragma once

#include "image.h"
#include "ray.h"
#include "Scene.h"
#include "pcg.h"
#include "BVH_node.h"
#include "progressreporter.h"

#include <filesystem>
#include <variant>
#include <vector>

Image3 hw_3_1(const std::vector<std::string> &params);
Image3 hw_3_2(const std::vector<std::string> &params);
Image3 hw_3_3(const std::vector<std::string> &params);
Image3 hw_3_4(const std::vector<std::string> &params);
