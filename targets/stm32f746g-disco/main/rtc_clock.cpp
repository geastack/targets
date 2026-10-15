#include "rtc_clock.h"

#include "clock.h"
#include "stm32f7xx.h"

namespace gea::stm32::rtc_clock {

namespace {

constexpr std::uint64_t kLseStartTimeoutUs = 3'000'000;
constexpr std::uint32_t kAsyncPrescaler = 127;
constexpr std::uint32_t kSyncPrescaler = 255;  // 32768 / 128 / 256 = 1 Hz
constexpr std::int64_t kSecondsPerDay = 86'400;

std::int64_t gEpochOffsetUs = 0;
bool gRunning = false;

struct Civil {
	int year;
	unsigned month;
	unsigned day;
};

// Howard Hinnant's days_from_civil / civil_from_days (proleptic Gregorian).
std::int64_t daysFromCivil(Civil date) {
	const int y = date.year - (date.month <= 2 ? 1 : 0);
	const int era = (y >= 0 ? y : y - 399) / 400;
	const unsigned yoe = static_cast<unsigned>(y - era * 400);
	const unsigned doy = (153 * (date.month + (date.month > 2 ? -3 : 9)) + 2) / 5 + date.day - 1;
	const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return static_cast<std::int64_t>(era) * 146'097 + doe - 719'468;
}

Civil civilFromDays(std::int64_t days) {
	days += 719'468;
	const std::int64_t era = (days >= 0 ? days : days - 146'096) / 146'097;
	const unsigned doe = static_cast<unsigned>(days - era * 146'097);
	const unsigned yoe = (doe - doe / 1460 + doe / 36'524 - doe / 146'096) / 365;
	const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	const unsigned mp = (5 * doy + 2) / 153;
	const unsigned day = doy - (153 * mp + 2) / 5 + 1;
	const unsigned month = mp < 10 ? mp + 3 : mp - 9;
	const int year = static_cast<int>(yoe + era * 400) + (month <= 2 ? 1 : 0);
	return {year, month, day};
}

std::uint32_t fromBcd(std::uint32_t value) {
	return (value >> 4) * 10 + (value & 0xF);
}

std::uint32_t toBcd(std::uint32_t value) {
	return ((value / 10) << 4) | (value % 10);
}

void unlock() {
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
}

void lock() {
	RTC->WPR = 0xFF;
}

// The shadow registers resync on the next RTCCLK edge after RSF is cleared;
// SSR then TR then DR is the order that keeps one consistent snapshot.
std::int64_t readEpochMicros() {
	RTC->ISR &= ~RTC_ISR_RSF;
	while (!(RTC->ISR & RTC_ISR_RSF)) {}

	const std::uint32_t ssr = RTC->SSR;
	const std::uint32_t tr = RTC->TR;
	const std::uint32_t dr = RTC->DR;

	const Civil date{2000 + static_cast<int>(fromBcd((dr >> RTC_DR_YU_Pos) & 0xFF)), fromBcd((dr >> RTC_DR_MU_Pos) & 0x1F),
	                 fromBcd(dr & 0x3F)};
	const std::int64_t seconds = daysFromCivil(date) * kSecondsPerDay + fromBcd((tr >> RTC_TR_HU_Pos) & 0x3F) * 3600 +
	                             fromBcd((tr >> RTC_TR_MNU_Pos) & 0x7F) * 60 + fromBcd(tr & 0x7F);
	const std::int64_t fractionUs = static_cast<std::int64_t>(kSyncPrescaler - ssr) * 1'000'000 / (kSyncPrescaler + 1);
	return seconds * 1'000'000 + fractionUs;
}

void syncOffset() {
	gEpochOffsetUs = isSet() ? readEpochMicros() - static_cast<std::int64_t>(clock::micros()) : 0;
}

}  // namespace

bool init() {
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;
	PWR->CR1 |= PWR_CR1_DBP;

	if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
		RCC->BDCR |= RCC_BDCR_LSEON;
		const std::uint64_t deadline = clock::micros() + kLseStartTimeoutUs;
		while (!(RCC->BDCR & RCC_BDCR_LSERDY)) {
			if (clock::micros() > deadline) return false;
		}

		RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCEN;
	}

	gRunning = true;
	syncOffset();
	return true;
}

bool isSet() {
	return gRunning && (RTC->ISR & RTC_ISR_INITS) != 0;
}

std::int64_t epochMicros() {
	return gEpochOffsetUs + static_cast<std::int64_t>(clock::micros());
}

// The RTC calendar covers 2000-2099.
bool setEpochSeconds(std::int64_t seconds) {
	const std::int64_t days = seconds / kSecondsPerDay;
	const std::int64_t secondOfDay = seconds % kSecondsPerDay;
	const Civil date = civilFromDays(days);
	if (!gRunning || seconds < 0 || date.year < 2000 || date.year > 2099) return false;

	const std::uint32_t weekday = static_cast<std::uint32_t>((days + 3) % 7) + 1;  // 1970-01-01 was a Thursday; 1 = Monday
	const std::uint32_t tr = (toBcd(static_cast<std::uint32_t>(secondOfDay / 3600)) << RTC_TR_HU_Pos) |
	                         (toBcd(static_cast<std::uint32_t>(secondOfDay / 60 % 60)) << RTC_TR_MNU_Pos) |
	                         toBcd(static_cast<std::uint32_t>(secondOfDay % 60));
	const std::uint32_t dr = (toBcd(static_cast<std::uint32_t>(date.year - 2000)) << RTC_DR_YU_Pos) | (weekday << RTC_DR_WDU_Pos) |
	                         (toBcd(date.month) << RTC_DR_MU_Pos) | toBcd(date.day);

	unlock();
	RTC->ISR |= RTC_ISR_INIT;
	while (!(RTC->ISR & RTC_ISR_INITF)) {}

	RTC->PRER = kSyncPrescaler;
	RTC->PRER |= kAsyncPrescaler << RTC_PRER_PREDIV_A_Pos;
	RTC->CR &= ~RTC_CR_FMT;
	RTC->TR = tr;
	RTC->DR = dr;
	RTC->ISR &= ~RTC_ISR_INIT;
	lock();

	syncOffset();
	return true;
}

}  // namespace gea::stm32::rtc_clock
