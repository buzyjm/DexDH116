#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace ethercat_iomap {

inline bool regionWithinMap(const void* map_begin,
                            std::size_t mapped_bytes,
                            const void* region_begin,
                            std::size_t region_bytes) {
  if (region_bytes == 0) {
    return true;
  }
  if (!map_begin || !region_begin) {
    return false;
  }

  const auto map_start = reinterpret_cast<std::uintptr_t>(map_begin);
  const auto region_start = reinterpret_cast<std::uintptr_t>(region_begin);
  if (region_start < map_start) {
    return false;
  }

  constexpr auto kMaxAddress = (std::numeric_limits<std::uintptr_t>::max)();
  if (mapped_bytes > kMaxAddress - map_start) {
    return false;
  }
  if (region_bytes > kMaxAddress - region_start) {
    return false;
  }

  const auto map_end = map_start + mapped_bytes;
  const auto region_end = region_start + region_bytes;
  return region_end <= map_end;
}

inline bool layoutWithinMapBounds(const void* map_begin,
                                  std::size_t map_capacity,
                                  std::size_t mapped_bytes,
                                  const void* outputs,
                                  std::size_t output_bytes,
                                  const void* inputs,
                                  std::size_t input_bytes) {
  if (mapped_bytes > map_capacity) {
    return false;
  }
  if (mapped_bytes > 0 && !map_begin) {
    return false;
  }

  return regionWithinMap(map_begin, mapped_bytes, outputs, output_bytes) &&
         regionWithinMap(map_begin, mapped_bytes, inputs, input_bytes);
}

}  // namespace ethercat_iomap
