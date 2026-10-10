#pragma once
// Debug builds only. No VM on the board: bounded native operations and a
// snapshot taken under the same lock as rendering and input dispatch.
#include "cJSON.h"
#include "esp_random.h"
#include "esp_app_desc.h"
#include "pixel.h"
#include "services/app_state.h"
#include "services/frame_scheduler.h"
#include "ui/tree_internal.h"
#include "ui/tree_state.h"
#include "ui/debugger_overlay.h"
#include "ui/debugger_picker.h"
#include "ui/debugger_listeners.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace gea::debugger {
using namespace gea::embedded::ui;
// Command dispatch is serialized by the device-control mutex. Expected
// validation failures use explicit status, never C++ exception unwinding.
inline const char *lastError = nullptr;
inline unsigned bootToken() {
  static const unsigned token = esp_random();
  return token;
}
inline std::string elfHash() {
  const auto *description = esp_app_get_description();
  static constexpr char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(64);
  for (unsigned char byte : description->app_elf_sha256) {
    out += digits[byte >> 4]; out += digits[byte & 15];
  }
  return out;
}
struct Lock {
  Lock() { gea::framework::services::AppState::lock(); }
  ~Lock() { gea::framework::services::AppState::unlock(); }
};
struct Json {
  cJSON *value;
  explicit Json(cJSON *v) : value(v) {
    if (!v)
      lastError = "JSON allocation failed";
  }
  ~Json() { cJSON_Delete(value); }
};
inline void string(cJSON *o, const char *k, const std::string &v) {
  if (!cJSON_AddStringToObject(o, k, v.c_str()))
    lastError = "JSON allocation failed";
}
inline void number(cJSON *o, const char *k, double v) {
  if (!cJSON_AddNumberToObject(o, k, v))
    lastError = "JSON allocation failed";
}
inline std::string encode(cJSON *o) {
  char *p = cJSON_PrintUnformatted(o);
  if (!p) {
    lastError = "JSON allocation failed";
    return "";
  }
  std::string s(p);
  cJSON_free(p);
  return s;
}
inline int identity(int slot) {
  return int(Tree::instance().node(slot).debugger_identity * 2 + 2);
}
inline int slot(int id) {
  auto &t = Tree::instance();
  if (id < 4 || id % 2) {
    lastError = "invalid element identity";
    return -1;
  }
  for (int i = 0; i < t.nodeCount(); ++i)
    if (identity(i) == id && treeState().nodeActive[i])
      return i;
  lastError = "stale element identity";
  return -1;
}
inline std::string px(int n) { return std::to_string(n) + "px"; }
inline std::string color(gea::framework::graphics::pixel::native_t value) {
#if GEA_EMBEDDED_PIXEL_FORMAT == GEA_PIXEL_ARGB8888
  unsigned v = value;
  int r = (v >> 16) & 255, g = (v >> 8) & 255, b = v & 255;
#elif GEA_EMBEDDED_PIXEL_FORMAT == GEA_PIXEL_RGBA8888
  unsigned v = value;
  int r = v & 255, g = (v >> 8) & 255, b = (v >> 16) & 255;
#else
  auto v = gea::framework::graphics::pixel::toRgb565(value);
  int r = ((v >> 11) & 31) * 255 / 31, g = ((v >> 5) & 63) * 255 / 63,
      b = (v & 31) * 255 / 31;
#endif
  return "rgb(" + std::to_string(r) + ", " + std::to_string(g) + ", " +
         std::to_string(b) + ")";
}
inline cJSON *computed(int id) {
  auto &n = Tree::instance().node(id);
  const auto &s = n.computedStyle();
  Json j(cJSON_CreateObject());
  string(j.value, "width", px(n.layout.width));
  string(j.value, "height", px(n.layout.height));
  string(j.value, "left",
         px(GEA_CSS_POSITION_PX_3(s) == kUnset ? 0 : GEA_CSS_POSITION_PX_3(s)));
  string(j.value, "top",
         px(GEA_CSS_POSITION_PX_0(s) == kUnset ? 0 : GEA_CSS_POSITION_PX_0(s)));
  string(j.value, "position",
         s.position == 1   ? "absolute"
         : s.position == 2 ? "relative"
         : s.position == 3 ? "fixed"
                           : "static");
  string(j.value, "color", color(s.text_color));
  string(j.value, "background-color",
         s.has_bg ? color(s.bg_color) : "rgba(0, 0, 0, 0)");
  string(j.value, "opacity", std::to_string(s.opacity / 255.0));
  string(j.value, "font-size", px(s.font_size));
  string(j.value, "font-weight", std::to_string(s.font_weight));
  string(j.value, "display",
         s.display == kDisplayNone   ? "none"
         : s.display == kDisplayFlex ? "flex"
         : s.display == kDisplayGrid ? "grid"
                                     : "block");
  const char *sides[] = {"top", "right", "bottom", "left"};
  for (int i = 0; i < 4; ++i) {
    string(j.value, (std::string("padding-") + sides[i]).c_str(),
           px(s.padding[i]));
    string(j.value, (std::string("margin-") + sides[i]).c_str(),
           px(s.margin[i]));
    string(j.value, (std::string("border-") + sides[i] + "-width").c_str(),
           px(computedBorderWidth(s, i)));
  }
  cJSON *result = j.value;
  j.value = nullptr;
  return result;
}
inline cJSON *inlineStyle(int id) {
  Json result(cJSON_CreateObject());
  const auto *rare = rareDataFor(id);
  if (!rare) {
    auto *v = result.value;
    result.value = nullptr;
    return v;
  }
  Json used(computed(id));
  for (size_t i = 0; i < rare->inlineStyles.size(); ++i) {
    const auto &e = rare->inlineStyles.at(i);
    const char *name = nullptr;
    bool length = true, percent = false;
    switch (e.property) {
    case Property::Width:
      name = "width";
      break;
    case Property::Height:
      name = "height";
      break;
    case Property::WidthPercent:
      name = "width";
      percent = true;
      break;
    case Property::HeightPercent:
      name = "height";
      percent = true;
      break;
    case Property::Left:
      name = "left";
      break;
    case Property::Top:
      name = "top";
      break;
    case Property::Right:
      name = "right";
      break;
    case Property::Bottom:
      name = "bottom";
      break;
    case Property::PaddingTop:
      name = "padding-top";
      break;
    case Property::PaddingRight:
      name = "padding-right";
      break;
    case Property::PaddingBottom:
      name = "padding-bottom";
      break;
    case Property::PaddingLeft:
      name = "padding-left";
      break;
    case Property::MarginTop:
      name = "margin-top";
      break;
    case Property::MarginRight:
      name = "margin-right";
      break;
    case Property::MarginBottom:
      name = "margin-bottom";
      break;
    case Property::MarginLeft:
      name = "margin-left";
      break;
    case Property::FontSize:
      name = "font-size";
      break;
    case Property::BackgroundColor:
      name = "background-color";
      length = false;
      break;
    case Property::Color:
      name = "color";
      length = false;
      break;
    case Property::Opacity:
      name = "opacity";
      length = false;
      break;
    case Property::Display:
      name = "display";
      length = false;
      break;
    default:
      break;
    }
    if (!name || e.value == kUnset)
      continue;
    char buffer[48];
    if (percent) {
      std::snprintf(buffer, sizeof(buffer), "%.5g%%", e.value / 10.0);
      string(result.value, name, buffer);
    } else if (length) {
      float pixels;
      if (rare->inlineStyles.getCssPixels(e.property, pixels)) {
        std::snprintf(buffer, sizeof(buffer), "%.5gpx", double(pixels));
        string(result.value, name, buffer);
      } else
        string(result.value, name, px(e.value));
    } else {
      auto *v = cJSON_GetObjectItemCaseSensitive(used.value, name);
      if (cJSON_IsString(v))
        string(result.value, name, v->valuestring);
    }
  }
  auto *v = result.value;
  result.value = nullptr;
  return v;
}
using Writer = bool (*)(const void *, std::size_t);
inline void snapshot(unsigned sequence, Writer writer) {
  std::vector<std::string> lines;
  int root = -1, width = 0, height = 0;
  std::string pickerStatus;
  {
    Lock lock;
    auto &t = Tree::instance();
    root = t.mountedRoot();
    width = t.mountedWidth();
    height = t.mountedHeight();
    debuggerPicker.expire();
    pickerStatus = " picker=1 inspectToken=" + std::to_string(debuggerPicker.token) +
                   " inspectActive=" + std::to_string(debuggerPicker.active) +
                   " inspectHover=" + std::to_string(debuggerPicker.hoverId()) +
                   " inspectNode=" + std::to_string(debuggerPicker.selectedId()) +
                   " inspectRevision=" + std::to_string(debuggerPicker.revision);
    std::vector<int> pending;
    for (int i = t.nodeCount() - 1; i >= 0; --i)
      if (treeState().nodeActive[i])
        pending.push_back(i);
    while (!pending.empty()) {
      if (lines.size() >= 2048) {
        lastError = "tree exceeds debugger limit of 2048 nodes";
        return;
      }
      int i = pending.back();
      pending.pop_back();
      auto &n = t.node(i);
      Json j(cJSON_CreateObject());
      number(j.value, "id", identity(i));
      number(j.value, "parent", n.parent >= 0 ? identity(n.parent) : 2);
      string(j.value, "tag", t.tagName(i) ? t.tagName(i) : "div");
      string(j.value, "text", n.text.c_str());
      number(j.value, "type", n.type == NodeType::Text ? 3 : 1);
      number(j.value, "x", n.layout.x);
      number(j.value, "y", n.layout.y);
      number(j.value, "width", n.layout.width);
      number(j.value, "height", n.layout.height);
      auto *attrs = cJSON_AddObjectToObject(j.value, "attributes");
#if GEA_UI_NODE_ATTRIBUTES
      const auto *rd = rareDataFor(i);
      if (rd)
        for (auto *entry = rd->attributes.values.get(); entry;
             entry = entry->next.get())
          string(attrs, entry->name(), entry->value());
#endif
      string(attrs, "class", t.className(i));
      auto *children = cJSON_AddArrayToObject(j.value, "children");
      for (int child = n.first_child; child >= 0;
           child = t.node(child).next_sibling) {
        cJSON_AddItemToArray(children, cJSON_CreateNumber(identity(child)));
      }
      cJSON_AddItemToObject(j.value, "computed", computed(i));
      cJSON_AddItemToObject(j.value, "inline", inlineStyle(i));
      auto *rules = cJSON_AddArrayToObject(j.value, "rules");
      for (const auto &r : debuggerMatchedCssRules(i)) {
        auto *rule = cJSON_CreateObject();
        string(rule, "selector", r.selector);
        string(rule, "property", r.property);
        string(rule, "value", r.value);
        string(rule, "media", r.media);
        cJSON_AddBoolToObject(rule, "userAgent", r.userAgent);
        cJSON_AddItemToArray(rules, rule);
      }
      lines.push_back(encode(j.value));
      if (lastError)
        return;
    }
    if (root >= 0)
      root = identity(root);
  }
  // Release rendering lock before USB output. Console command mutex serializes
  // these replies; host uses one in-flight request for the physical connection.
  // Never hold the render lock while streaming. The USB driver write-all
  // callback handles partial writes; stdio's per-character VFS path does not.
  flockfile(stdout);
  std::fflush(stdout);
  auto write = [&](const std::string &text) {
    return writer ? writer(text.data(), text.size())
                  : std::fwrite(text.data(), 1, text.size(), stdout) ==
                        text.size();
  };
  unsigned checksum = 2166136261u;
  bool ok =
      write("GEADEV:DEBUG BEGIN sequence=" + std::to_string(sequence) + "\n");
  for (const auto &line : lines) {
    for (unsigned char c : line) {
      checksum ^= c;
      checksum *= 16777619u;
    }
    checksum ^= '\n';
    checksum *= 16777619u;
    if (ok)
      ok = write("GEADEV:DEBUG NODE " + line + "\n");
  }
  if (ok)
    ok = write(
        "GEADEV:DEBUG END root=" + std::to_string(root) + " width=" +
        std::to_string(width) + " height=" + std::to_string(height) + " boot=" +
        std::to_string(bootToken()) + " sequence=" + std::to_string(sequence) +
        " count=" + std::to_string(lines.size()) + " fps=" +
        std::to_string(
            gea::framework::services::FrameScheduler::debuggerFrameRate()) +
        " overlay=1 listeners=1" + pickerStatus + " checksum=" + std::to_string(checksum) + " elf=" + elfHash() + "\n");
  funlockfile(stdout);
  if (!ok)
    lastError = "snapshot transport write failed";
}
// JSX listeners of the given elements, with their registering call sites.
// Reply: BEGIN, one LISTENER line per handler, END with count and boot.
inline void listeners(const char *args, Writer writer) {
  char *end = nullptr;
  const unsigned long sequence = std::strtoul(args, &end, 10);
  if (end == args || *end != ' ') {
    lastError = "invalid listeners request";
    return;
  }
  std::vector<std::uint32_t> ids;
  for (const char *cursor = end + 1; *cursor;) {
    const unsigned long id = std::strtoul(cursor, &end, 10);
    if (end == cursor || id < 4 || id % 2 || ids.size() >= 64 || (*end && *end != ',')) {
      lastError = "invalid listener identity list";
      return;
    }
    ids.push_back(std::uint32_t(id));
    cursor = *end ? end + 1 : end;
  }
  std::vector<std::string> lines;
  {
    Lock lock;
    auto &registry = debuggerListeners();
    for (auto id : ids) {
      auto found = registry.find((id - 2) / 2);
      if (found == registry.end())
        continue;
      for (const auto &listener : found->second) {
        Json j(cJSON_CreateObject());
        number(j.value, "id", id);
        string(j.value, "type", listener.type);
        auto *sites = cJSON_AddArrayToObject(j.value, "sites");
        for (int i = 0; i < listener.count; ++i)
          cJSON_AddItemToArray(sites, cJSON_CreateNumber(listener.sites[i]));
        lines.push_back(encode(j.value));
        if (lastError)
          return;
      }
    }
  }
  flockfile(stdout);
  std::fflush(stdout);
  auto write = [&](const std::string &text) {
    return writer ? writer(text.data(), text.size())
                  : std::fwrite(text.data(), 1, text.size(), stdout) == text.size();
  };
  bool ok = write("GEADEV:DEBUG LISTENERS BEGIN sequence=" + std::to_string(sequence) + "\n");
  for (const auto &line : lines)
    if (ok)
      ok = write("GEADEV:DEBUG LISTENER " + line + "\n");
  if (ok)
    ok = write("GEADEV:DEBUG LISTENERS END sequence=" + std::to_string(sequence) +
               " count=" + std::to_string(lines.size()) + " boot=" + std::to_string(bootToken()) + "\n");
  funlockfile(stdout);
  if (!ok)
    lastError = "listeners transport write failed";
}
inline const char *field(cJSON *j, const char *key) {
  auto *v = cJSON_GetObjectItemCaseSensitive(j, key);
  return cJSON_IsString(v) ? v->valuestring : "";
}
inline int integer(cJSON *j, const char *key) {
  auto *v = cJSON_GetObjectItemCaseSensitive(j, key);
  if (!cJSON_IsNumber(v) || v->valuedouble != v->valueint) {
    lastError = "missing or invalid numeric field";
    return -1;
  }
  return v->valueint;
}
inline std::string decode(const char *text) {
  const std::string alphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  unsigned bits = 0;
  int count = 0;
  const size_t length = std::strlen(text);
  if (length > 480 || length % 4) {
    lastError = "invalid request encoding";
    return "";
  }
  for (size_t i = 0; i < length; ++i) {
    char c = text[i];
    if (c == '=') {
      for (; i < length; ++i)
        if (text[i] != '=') {
          lastError = "invalid base64 padding";
          return "";
        }
      break;
    }
    auto v = alphabet.find(c);
    if (v == std::string::npos) {
      lastError = "invalid base64";
      return "";
    }
    bits = (bits << 6) | unsigned(v);
    count += 6;
    if (count >= 8) {
      count -= 8;
      out += char((bits >> count) & 255);
    }
  }
  return out;
}
inline void mutate(const char *encoded) {
  const auto decoded = decode(encoded);
  if (lastError)
    return;
  Json j(cJSON_Parse(decoded.c_str()));
  if (!cJSON_IsObject(j.value)) {
    lastError = "invalid JSON request";
    return;
  }
  const char *requestBoot = field(j.value, "boot");
  char *end = nullptr;
  const unsigned long token = std::strtoul(requestBoot, &end, 10);
  if (!*requestBoot || !end || *end || token != bootToken()) {
    lastError = "device rebooted or debugger boot token missing; reconnect";
    return;
  }
  const std::string op = field(j.value, "op"), key = field(j.value, "key"),
                    value = field(j.value, "value");
  int result = 0;
  {
    Lock lock;
    auto &t = Tree::instance();
    if (op == "fps") {
      const int fps = integer(j.value, "fps");
      if (lastError || fps < 0 || fps > 120) {
        lastError = "debug FPS must be between 0 and 120";
        return;
      }
      gea::framework::services::FrameScheduler::setDebuggerFrameRate(fps);
      result = fps;
    } else if (op == "inspect" || op == "inspectPoint") {
      const int owner = integer(j.value, "token");
      if (lastError || owner <= 0) {
        lastError = "invalid inspector token";
        return;
      }
      const std::string mode = field(j.value, "mode");
      if (op == "inspectPoint") {
        if (mode == "leave") {
          debuggerPicker.expire();
          if (debuggerPicker.active && debuggerPicker.token == unsigned(owner)) {
            debuggerPicker.hovered = -1;
            debuggerPicker.hoverIdentity = 0;
            debuggerPicker.outline(-1);
          }
        } else if (mode == "hover" || mode == "select") {
          const int x = integer(j.value, "x"), y = integer(j.value, "y");
          if (lastError) return;
          debuggerPicker.point(unsigned(owner), x, y, mode == "select");
        } else { lastError = "invalid inspector point mode"; return; }
      } else if (mode == "searchForNode" || mode == "searchForUAShadowDOM") {
        debuggerPicker.begin(unsigned(owner));
      } else if (mode == "keepAlive") {
        debuggerPicker.renew(unsigned(owner));
      } else if (mode == "none") {
        debuggerPicker.cancel(unsigned(owner));
      } else { lastError = "unsupported inspect mode"; return; }
      result = int(debuggerPicker.token);
    } else if (op == "highlight") {
      const int id = integer(j.value, "id");
      if (lastError) return;
      int node = id ? slot(id) : -1;
      if (lastError) return;
      int red = integer(j.value, "red"), green = integer(j.value, "green"),
          blue = integer(j.value, "blue");
      if (lastError || red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255) {
        lastError = "invalid overlay color";
        return;
      }
      debuggerPicker.expire();
      if (!debuggerPicker.active)
        debuggerOverlay.set(node, gea::framework::graphics::pixel::nativeColor(red, green, blue));
    } else if (op == "rule") {
      if (!debuggerSetCssRule(field(j.value, "selector"), field(j.value, "media"), key, value)) {
        lastError = "stale or read-only stylesheet rule";
        return;
      }
    } else if (op == "create") {
      int i = key == "button" ? t.createButton()
              : key == "span" ? t.createText()
                              : t.createView();
      if (i < 0) {
        lastError = "node allocation failed";
        return;
      }
      t.setTagName(i, key.c_str());
      result = identity(i);
    } else {
      int i = slot(integer(j.value, "id"));
      if (lastError || i < 0)
        return;
      if (op == "style") {
        if (value.empty())
          Style(i).removeProperty(key);
        else
          Style(i).setProperty(key, value);
      } else if (op == "attribute")
        t.setAttribute(i, key.c_str(), value.c_str());
      else if (op == "removeAttribute")
        t.removeAttribute(i, key.c_str());
      else if (op == "text") {
        if (t.node(i).type == NodeType::Text)
          t.setText(i, value.c_str());
        else {
          while (t.node(i).first_child >= 0)
            t.removeNode(t.node(i).first_child);
          t.setText(i, "");
          if (!value.empty()) {
            int child = t.createText();
            if (child < 0) {
              lastError = "node allocation failed";
              return;
            }
            t.setText(child, value.c_str());
            t.setParent(child, i);
          }
        }
      } else if (op == "remove") {
        if (i == t.mountedRoot()) {
          lastError = "cannot remove app root";
          return;
        }
        t.removeNode(i);
      } else if (op == "append") {
        int parent = slot(integer(j.value, "parent"));
        if (lastError || parent < 0)
          return;
        if (t.containsNode(i, parent)) {
          lastError = "tree cycle";
          return;
        }
        t.setParent(i, parent);
      } else if (op == "scroll")
        t.scrollIntoView(i);
      else if (op == "click") {
        for (auto type : {gea::framework::events::PointerEventType::TouchStart,
                          gea::framework::events::PointerEventType::TouchEnd,
                          gea::framework::events::PointerEventType::Click}) {
          int current = slot(integer(j.value, "id"));
          if (lastError || current < 0)
            return;
          gea::framework::events::PointerEvent event;
          event.type = type;
          event.targetId = current;
          t.dispatchEvent(event);
        }
      } else {
        lastError = "unsupported native operation";
        return;
      }
    }
  }
  std::printf("GEADEV:DEBUG OK id=%d\n", result);
}
inline void handle(char *args, Writer writer = nullptr) {
  lastError = nullptr;
  while (*args == ' ')
    ++args;
  if (std::strncmp(args, "SNAPSHOT ", 9) == 0) {
    char *end = nullptr;
    unsigned long sequence = std::strtoul(args + 9, &end, 10);
    if (!*(args + 9) || !end || *end) {
      lastError = "invalid snapshot sequence";
    } else
      snapshot(unsigned(sequence), writer);
  } else if (std::strncmp(args, "LISTENERS ", 10) == 0)
    listeners(args + 10, writer);
  else
    mutate(args);
  if (lastError)
    std::printf("GEADEV:ERR DEBUG %s\n", lastError);
}
} // namespace gea::debugger
