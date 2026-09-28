#pragma once

#include <XPLMDataAccess.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace ogr {

class FloatDataRef {
public:
  FloatDataRef(std::string name, float initial = 0.0f, bool writable = false,
               float minimum = -1.0f, float maximum = 1.0f);
  ~FloatDataRef();
  FloatDataRef(const FloatDataRef&) = delete;
  FloatDataRef& operator=(const FloatDataRef&) = delete;
  void set(float value);
  float value() const { return value_; }

private:
  static float read(void* refcon);
  static void write(void* refcon, float value);
  std::string name_;
  float value_{};
  bool writable_{};
  float minimum_{};
  float maximum_{};
  XPLMDataRef handle_{};
};

class DataRefRegistry {
public:
  FloatDataRef& create(const std::string& name, float initial = 0.0f,
                       bool writable = false, float minimum = -1.0f,
                       float maximum = 1.0f);
  void clear();

private:
  std::unordered_map<std::string, std::unique_ptr<FloatDataRef>> refs_;
};

} // namespace ogr
