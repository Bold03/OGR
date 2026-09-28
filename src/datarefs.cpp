#include "ogr/datarefs.hpp"
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ogr {

FloatDataRef::FloatDataRef(std::string name, float initial, bool writable,
                           float minimum, float maximum)
    : name_(std::move(name)), writable_(writable),
      minimum_(std::min(minimum, maximum)), maximum_(std::max(minimum, maximum)) {
  value_ = std::clamp(initial, minimum_, maximum_);
  handle_ = XPLMRegisterDataAccessor(
      name_.c_str(), xplmType_Float, writable_ ? 1 : 0,
      nullptr, nullptr, read, writable_ ? write : nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      this, this);
  if (!handle_) throw std::runtime_error("Cannot register OGR dataref: " + name_);
}

FloatDataRef::~FloatDataRef() {
  if (handle_) XPLMUnregisterDataAccessor(handle_);
}

float FloatDataRef::read(void* refcon) {
  return static_cast<FloatDataRef*>(refcon)->value_;
}

void FloatDataRef::write(void* refcon, float value) {
  auto* self = static_cast<FloatDataRef*>(refcon);
  if (self->writable_) self->set(value);
}

void FloatDataRef::set(float value) {
  value_ = std::clamp(value, minimum_, maximum_);
}

FloatDataRef& DataRefRegistry::create(const std::string& name, float initial,
                                      bool writable, float minimum, float maximum) {
  auto it = refs_.find(name);
  if (it != refs_.end()) return *it->second;
  auto ref = std::make_unique<FloatDataRef>(name, initial, writable, minimum, maximum);
  auto* raw = ref.get();
  refs_.emplace(name, std::move(ref));
  return *raw;
}

void DataRefRegistry::clear() { refs_.clear(); }

} // namespace ogr
