#pragma once

#include "apps.h"

namespace gea::platform::esp32::apps {

gea::framework::apps::AppLauncherPlatform &launcherPlatform();
void startLauncherButtonTask();

}  // namespace gea::platform::esp32::apps
