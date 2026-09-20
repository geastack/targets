#include "services/battery_service.h"

#include <cstdint>

namespace gea::framework::services {

namespace {

constexpr int kPollIntervalMs = 60000;
std::uint32_t nextPollMs = 0;

}  // namespace

void BatteryService::start()
{
	nextPollMs = 0;
}

void BatteryService::poll(int nowMs)
{
	const auto now = static_cast<std::uint32_t>(nowMs);
	if (nextPollMs == 0) {
		nextPollMs = now + kPollIntervalMs;
		return;
	}
	if (static_cast<std::int32_t>(now - nextPollMs) < 0) return;
	BatteryService::update();
	nextPollMs = now + kPollIntervalMs;
}

}  // namespace gea::framework::services
