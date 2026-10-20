#pragma once

#include "rig.hpp"
#include <string_view>

namespace orbvi_camera_contract {

enum class Product { Unknown, FourV, FourVL, FourVLite, FourP };

inline constexpr const char* ToString(Product product) {
  switch (product) {
    case Product::FourV: return "orbvi-4v";
    case Product::FourVL: return "orbvi-4vl";
    case Product::FourVLite: return "orbvi-4v-lite";
    case Product::FourP: return "orbvi-4p";
    default: return "";
  }
}

inline constexpr Product ProductFromModel(std::string_view model) {
  for (const auto product : {Product::FourV, Product::FourVL,
                             Product::FourVLite, Product::FourP})
    if (model == ToString(product)) return product;
  return Product::Unknown;
}

// A hardware platform alone cannot establish the mounting roles (X5 also has 4P).
inline constexpr std::optional<Rig> DefaultRig(Product product) {
  switch (product) {
    case Product::FourV:
    case Product::FourVL: return kRk3588Rig;
    case Product::FourVLite: return kX5Rig;
    default: return std::nullopt;
  }
}

inline constexpr std::optional<Role> DefaultCameraRole(Product product, int physical_id) {
  const auto rig = DefaultRig(product);
  if (rig)
    for (std::size_t i = 0; i < kRoles.size(); ++i)
      if (rig->camera_ids[i] == physical_id) return kRoles[i];
  return std::nullopt;
}
}  // namespace orbvi_camera_contract
