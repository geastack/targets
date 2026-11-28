#pragma once

#include "geaos.h"

namespace gea::platform::esp32::apps {

// The GeaOS host for boards whose apps are whole firmware images in OTA
// slots: launching an app selects its slot and restarts. It serves until the
// board moves to loadable apps (geapp/), which replace it as the host.
gea::geaos::AppHost &launcherPlatform();
void startLauncherButtonTask();

}  // namespace gea::platform::esp32::apps
