#include <tf2/utils.h>

#ifndef FUSE_MODELS_COMMON_STD_UTILS_HPP_
#define FUSE_MODELS_COMMON_STD_UTILS_HPP_

namespace std
{

inline bool isfinite(const tf2_2d::Vector2 & vector)
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y());
}

inline bool isfinite(const tf2_2d::Transform & transform)
{
  return std::isfinite(transform.x()) && std::isfinite(transform.y()) && std::isfinite(
    transform.yaw());
}

inline std::string to_string(const tf2_2d::Vector2 & vector)
{
  std::ostringstream oss;
  oss << vector;
  return oss.str();
}

inline std::string to_string(const tf2_2d::Transform & transform)
{
  std::ostringstream oss;
  oss << transform;
  return oss.str();
}

}  // namespace std

#endif  // FUSE_MODELS_COMMON_STD_UTILS_HPP_