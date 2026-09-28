#include "ogr/datarefs.hpp"
#include "ogr/dsf_bridge.hpp"
#include "ogr/runtime.hpp"

#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>

namespace {
std::unique_ptr<ogr::Runtime> runtime;
ogr::DataRefRegistry refs;
XPLMDataRef heading_ref{};
XPLMDataRef agl_ref{};
XPLMMenuID menu{};
int menu_item{-1};
bool flightloop_registered{};
std::string xplane_root;

float flightloop(float elapsed_since_last_call, float, int, void*) {
  if (!runtime) return -1.0f;
  const float heading = heading_ref ? XPLMGetDataf(heading_ref) : 0.0f;
  const float agl_m = agl_ref ? XPLMGetDataf(agl_ref) : 0.0f;
  runtime->update(elapsed_since_last_call, heading, agl_m);
  return -1.0f;
}

void refresh_direct_dsf() {
  if (xplane_root.empty()) return;
  const auto stats = ogr::refresh_direct_dsf_areas(xplane_root);
  for (const auto& text : stats.messages) {
    const std::string line = "[OGR] " + text + "\n";
    XPLMDebugString(line.c_str());
  }
}

void menu_handler(void*, void* item_ref) {
  if (!runtime || item_ref != reinterpret_cast<void*>(1)) return;
  XPLMDebugString("[OGR] Manual reload requested\n");
  refresh_direct_dsf();
  runtime->reload();
}

void create_menu() {
  const int plugins_item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Oafish Grass Runtime", nullptr, 1);
  menu = XPLMCreateMenu("Oafish Grass Runtime", XPLMFindPluginsMenu(), plugins_item,
                        menu_handler, nullptr);
  if (menu) menu_item = XPLMAppendMenuItem(menu, "Reload WED/DSF grass areas", reinterpret_cast<void*>(1), 1);
}

void destroy_menu() {
  if (menu) XPLMDestroyMenu(menu);
  menu = nullptr;
  menu_item = -1;
}
} // namespace

PLUGIN_API int XPluginStart(char* name, char* signature, char* description) {
  std::snprintf(name, 256, "%s", "Oafish Grass Runtime");
  std::snprintf(signature, 256, "%s", "oafish.ogr.runtime");
  std::snprintf(description, 256, "%s", "WED-authored animated grass runtime for X-Plane 11/12");

  try {
    // These must exist before an OBJ using them is loaded.
    for (int group = 0; group < 4; ++group) {
      refs.create("oafish/ogr/grass/bend_x_" + std::to_string(group), 0.0f, false, -1.0f, 1.0f);
      refs.create("oafish/ogr/grass/bend_z_" + std::to_string(group), 0.0f, false, -1.0f, 1.0f);
    }

    heading_ref = XPLMFindDataRef("sim/flightmodel/position/psi");
    agl_ref = XPLMFindDataRef("sim/flightmodel/position/y_agl");

    char root[2048]{};
    XPLMGetSystemPath(root);
    xplane_root = root;
    refresh_direct_dsf();
    runtime = std::make_unique<ogr::Runtime>();
    runtime->load_all(root);
    create_menu();
    return 1;
  } catch (const std::exception& e) {
    const std::string message = std::string("[OGR] XPluginStart failed: ") + e.what() + "\n";
    XPLMDebugString(message.c_str());
    runtime.reset();
    refs.clear();
    return 0;
  }
}

PLUGIN_API void XPluginStop(void) {
  if (flightloop_registered) {
    XPLMUnregisterFlightLoopCallback(flightloop, nullptr);
    flightloop_registered = false;
  }
  destroy_menu();
  runtime.reset();
  xplane_root.clear();
  refs.clear();
}

PLUGIN_API int XPluginEnable(void) {
  if (!flightloop_registered) {
    XPLMRegisterFlightLoopCallback(flightloop, -1.0f, nullptr);
    flightloop_registered = true;
  }
  return 1;
}

PLUGIN_API void XPluginDisable(void) {
  if (flightloop_registered) {
    XPLMUnregisterFlightLoopCallback(flightloop, nullptr);
    flightloop_registered = false;
  }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int message, void*) {
  if (!runtime) return;
  if (from == XPLM_PLUGIN_XPLANE && message == XPLM_MSG_SCENERY_LOADED) {
    XPLMDebugString("[OGR] X-Plane scenery reload detected; refreshing direct DSF grass areas\n");
    refresh_direct_dsf();
    runtime->reload();
  }
}
