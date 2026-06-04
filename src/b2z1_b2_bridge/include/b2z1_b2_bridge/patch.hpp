#pragma once

#include <cstring>
#include <string>

inline void ReplaceAll(std::string& str, const std::string& from, const std::string& to)
{
  if (from.empty()) {
    return;
  }
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

template<typename T>
inline void PatchFrameId(T& msg, const std::string& frame_id)
{
  ReplaceAll(msg.header.frame_id, "odom", frame_id);
}
