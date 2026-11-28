// Permanent-offline gea::host::fetch surface for the RP2350 targets: the
// board has no radio, so every request resolves immediately as a failed
// response (ok=false, status 0). Networked apps (weather, maps, ...) link and
// run in their offline/placeholder states — most already gate their fetch
// loops on WiFi.connected(), which never turns true here.
//
// The full implementation in core/packages/host/host/fetch.cpp only has
// ESP-IDF, Emscripten, and std::thread/socket backends, none of which build
// on the Pico SDK.

// fetch.h grows a gea_cpp_value bridge when GEA_CPP_VALUE_AVAILABLE is set;
// the stub doesn't need it and must not drag the value runtime in (mirrors
// the include dance in host/fetch.cpp).
#ifdef GEA_CPP_VALUE_AVAILABLE
#define GEA_RP2350_FETCH_RESTORE_CPP_VALUE 1
#undef GEA_CPP_VALUE_AVAILABLE
#endif
#include "host/fetch.h"
#ifdef GEA_RP2350_FETCH_RESTORE_CPP_VALUE
#define GEA_CPP_VALUE_AVAILABLE 1
#undef GEA_RP2350_FETCH_RESTORE_CPP_VALUE
#endif

namespace gea::host {

namespace {

FetchResponse offlineResponse()
{
	FetchResponse response;
	response.ok = false;
	response.status = 0;
	response.status_text = "offline";
	return response;
}

}  // namespace

FetchResponse fetch(const std::string &)
{
	return offlineResponse();
}

FetchResponse fetch(const std::string &, const FetchRequestInit &)
{
	return offlineResponse();
}

double fetchAsync(const std::string &)
{
	return -1;
}

double fetchAsync(const std::string &, const FetchRequestInit &)
{
	return -1;
}

FetchResponse fetchResult(double)
{
	return offlineResponse();
}

}  // namespace gea::host
