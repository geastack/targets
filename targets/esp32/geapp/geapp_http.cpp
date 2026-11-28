// SPDX-License-Identifier: GPL-3.0-only
// Remote install over Wi-Fi: routes on the device's HTTP server (services/ota.cpp,
// port 8080) that list, install and remove apps.
//
//   GET    /apps                          installed apps and volumes, as JSON
//   POST   /apps/install[?volume=<id>]    body: a .geapp package
//   GET    /apps/install/status           progress of the install in flight
//   DELETE /apps/<id>                      uninstall
//
// An upload streams to a hidden staging name in the target volume's apps
// folder and is committed by the same verified install as a USB push
// (Launcher::installStaged): a dropped connection, a full volume or a
// package for another device leaves nothing behind and the old app in place.
#include "geapp_host.h"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <new>
#include <optional>
#include <string>

#include "esp_http_server.h"
#include "esp_log.h"
#if __has_include("mdns.h")
#include "esp_mac.h"
#include "mdns.h"
#define GEA_GEAPP_MDNS 1
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace gea::platform::geapp {

namespace {

using gea::geaos::Launcher;

constexpr const char *kTag = "geapp_http";
constexpr std::size_t kChunk = 4096;
constexpr int kMaxTimeouts = 20;  // ~5 s each with the server's default recv timeout

std::atomic<std::uint32_t> g_received{0};
std::atomic<std::uint32_t> g_expected{0};
std::atomic<bool> g_busy{false};
std::atomic<std::uint32_t> g_sequence{0};

void appendJsonString(std::string &out, const std::string &value)
{
	out += '"';
	for (const char c : value) {
		switch (c) {
		case '"': out += "\\\""; break;
		case '\\': out += "\\\\"; break;
		case '\n': out += "\\n"; break;
		case '\r': out += "\\r"; break;
		case '\t': out += "\\t"; break;
		default:
			if (static_cast<unsigned char>(c) < 0x20) {
				char escaped[8];
				std::snprintf(escaped, sizeof(escaped), "\\u%04x", c);
				out += escaped;
			} else {
				out += c;
			}
		}
	}
	out += '"';
}

esp_err_t sendJson(httpd_req_t *req, const char *status, const std::string &body)
{
	httpd_resp_set_status(req, status);
	httpd_resp_set_type(req, "application/json");
	return httpd_resp_send(req, body.data(), static_cast<ssize_t>(body.size()));
}

esp_err_t sendError(httpd_req_t *req, const char *status, const std::string &message)
{
	std::string body = "{\"ok\":false,\"error\":";
	appendJsonString(body, message);
	body += "}\n";
	return sendJson(req, status, body);
}

std::string queryValue(httpd_req_t *req, const char *key)
{
	char query[96] = {};
	char value[48] = {};
	if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return {};
	if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return {};
	return value;
}

esp_err_t handleList(httpd_req_t *req)
{
	rescan();
	const auto result = Launcher::apps();
	const auto install = Launcher::installVolume();
	std::string body = "{\"ok\":true,\"abi\":";
	appendJsonString(body, firmwareAbi());
	body += ",\"target\":";
	appendJsonString(body, kTargetKey);
	body += ",\"volumes\":[";
	bool first = true;
	for (const auto &volume : Launcher::volumes()) {
		if (!first) body += ',';
		first = false;
		body += "{\"id\":";
		appendJsonString(body, volume.id);
		body += ",\"label\":";
		appendJsonString(body, volume.label);
		body += ",\"removable\":";
		body += volume.removable ? "true" : "false";
		body += ",\"readOnly\":";
		body += volume.readOnly ? "true" : "false";
		body += ",\"install\":";
		body += install && install->id == volume.id ? "true" : "false";
		body += '}';
	}
	body += "],\"apps\":[";
	first = true;
	for (const auto &app : result.apps) {
		if (!first) body += ',';
		first = false;
		body += "{\"id\":";
		appendJsonString(body, app.manifest.id);
		body += ",\"name\":";
		appendJsonString(body, app.manifest.name);
		body += ",\"version\":";
		appendJsonString(body, app.manifest.version.toString());
		body += ",\"volume\":";
		appendJsonString(body, app.volumeId);
		body += '}';
	}
	body += "],\"broken\":[";
	first = true;
	for (const auto &broken : result.broken) {
		if (!first) body += ',';
		first = false;
		body += "{\"location\":";
		appendJsonString(body, broken.location);
		body += ",\"reason\":";
		appendJsonString(body, broken.reason);
		body += '}';
	}
	body += "]}\n";
	return sendJson(req, "200 OK", body);
}

esp_err_t handleStatus(httpd_req_t *req)
{
	char body[96];
	std::snprintf(body, sizeof(body), "{\"ok\":true,\"busy\":%s,\"received\":%lu,\"expected\":%lu}\n", g_busy.load() ? "true" : "false",
	              static_cast<unsigned long>(g_received.load()), static_cast<unsigned long>(g_expected.load()));
	return sendJson(req, "200 OK", body);
}

// Receives the body into `path`. False with the reason (and the file removed)
// on a dropped connection, a write that fails, or a short body.
bool receiveTo(httpd_req_t *req, const std::string &path, std::string &error)
{
	FILE *file = std::fopen(path.c_str(), "wb");
	if (!file) {
		error = "cannot create " + path + " (" + std::strerror(errno) + ")";
		return false;
	}
	char *buffer = new (std::nothrow) char[kChunk];
	if (!buffer) {
		std::fclose(file);
		std::remove(path.c_str());
		error = "out of memory";
		return false;
	}
	std::size_t total = 0;
	int timeouts = 0;
	bool ok = true;
	while (total < req->content_len) {
		const std::size_t remaining = req->content_len - total;
		const int got = httpd_req_recv(req, buffer, remaining < kChunk ? remaining : kChunk);
		if (got == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < kMaxTimeouts) continue;
		if (got <= 0) {
			error = "connection lost after " + std::to_string(total) + " of " + std::to_string(req->content_len) + " bytes";
			ok = false;
			break;
		}
		timeouts = 0;
		if (std::fwrite(buffer, 1, static_cast<std::size_t>(got), file) != static_cast<std::size_t>(got)) {
			error = std::string("write failed (") + std::strerror(errno) + "): the volume may be full";
			ok = false;
			break;
		}
		total += static_cast<std::size_t>(got);
		g_received.store(static_cast<std::uint32_t>(total), std::memory_order_relaxed);
	}
	delete[] buffer;
	if (std::fclose(file) != 0 && ok) {
		error = std::string("write failed (") + std::strerror(errno) + "): the volume may be full";
		ok = false;
	}
	if (!ok) std::remove(path.c_str());
	return ok;
}

esp_err_t handleInstall(httpd_req_t *req)
{
	if (req->content_len == 0) return sendError(req, "400 Bad Request", "empty body: send a .geapp package");
	bool expected = false;
	if (!g_busy.compare_exchange_strong(expected, true)) return sendError(req, "409 Conflict", "another install is in progress");
	struct Release {
		~Release() { g_busy.store(false); }
	} release;
	g_received.store(0);
	g_expected.store(static_cast<std::uint32_t>(req->content_len));

	rescan();
	const std::string volumeId = queryValue(req, "volume");
	std::optional<gea::geaos::Volume> volume;
	if (volumeId.empty()) {
		volume = Launcher::installVolume();
	} else {
		for (const auto &v : Launcher::volumes())
			if (v.id == volumeId) volume = v;
	}
	if (!volume) return sendError(req, "404 Not Found", volumeId.empty() ? "no volume to install to" : "no volume \"" + volumeId + "\"");
	if (volume->readOnly) return sendError(req, "409 Conflict", volume->label + " is read-only");

	const std::string apps = gea::geaos::appsDirectory(*volume);
	gea::geaos::posixFs().makeDir(apps);
	const std::string staged = apps + "/.http-" + std::to_string(g_sequence.fetch_add(1)) + ".geapp.partial";
	std::string error;
	if (!receiveTo(req, staged, error)) {
		ESP_LOGW(kTag, "install refused: %s", error.c_str());
		return sendError(req, "400 Bad Request", error);
	}
	const auto result = Launcher::installStaged(staged, volume->id);
	gea::geaos::posixFs().remove(staged);  // gone on success; on refusal, never left behind
	if (!result.ok) {
		ESP_LOGW(kTag, "install refused: %s", result.error.c_str());
		return sendError(req, "422 Unprocessable Entity", result.error);
	}
	ESP_LOGI(kTag, "installed %s at %s", result.id.c_str(), result.location.c_str());
	std::string body = "{\"ok\":true,\"id\":";
	appendJsonString(body, result.id);
	body += ",\"location\":";
	appendJsonString(body, result.location);
	body += ",\"volume\":";
	appendJsonString(body, volume->id);
	body += "}\n";
	return sendJson(req, "200 OK", body);
}

esp_err_t handleRemove(httpd_req_t *req)
{
	const char *uri = req->uri;
	constexpr char kPrefix[] = "/apps/";
	if (std::strncmp(uri, kPrefix, sizeof(kPrefix) - 1) != 0) return sendError(req, "404 Not Found", "not an app path");
	std::string id(uri + sizeof(kPrefix) - 1);
	const std::size_t query = id.find('?');
	if (query != std::string::npos) id.resize(query);
	if (!gea::geaos::isValidAppId(id)) return sendError(req, "400 Bad Request", "not an app id: " + id);
	if (g_busy.load()) return sendError(req, "409 Conflict", "an install is in progress");
	if (!Launcher::uninstall(id)) return sendError(req, "409 Conflict", Launcher::lastError());
	std::string body = "{\"ok\":true,\"id\":";
	appendJsonString(body, id);
	body += "}\n";
	return sendJson(req, "200 OK", body);
}

// Announces Home on the local network as _geaos._tcp (port 8080), named
// geaos-<board>-<last 2 MAC bytes>, with the board, target and ABI in TXT:
// what `gea app ... --wifi` and a companion app look for.
void advertise()
{
#if GEA_GEAPP_MDNS
	static bool started = false;
	if (started) return;
	if (mdns_init() != ESP_OK) {
		ESP_LOGW(kTag, "mDNS unavailable; install with --host <ip>");
		return;
	}
	started = true;
	std::uint8_t mac[6] = {};
	esp_read_mac(mac, ESP_MAC_WIFI_STA);
	char host[48];
#ifdef GEA_EMBEDDED_BOARD_ID
	const char *board = GEA_EMBEDDED_BOARD_ID;
#else
	const char *board = "esp32";
#endif
	std::snprintf(host, sizeof(host), "geaos-%02x%02x", mac[4], mac[5]);
	mdns_hostname_set(host);
	mdns_instance_name_set("GeaOS Home");
	mdns_txt_item_t txt[] = {{"board", board}, {"target", kTargetKey}, {"abi", firmwareAbi()}};
	mdns_service_add("GeaOS Home", "_geaos", "_tcp", 8080, txt, sizeof(txt) / sizeof(txt[0]));
	ESP_LOGI(kTag, "advertising %s.local as _geaos._tcp", host);
#endif
}

}  // namespace

int registerHttpRoutes(void *server)
{
	advertise();
	auto *handle = static_cast<httpd_handle_t>(server);
	struct Route {
		const char *uri;
		httpd_method_t method;
		esp_err_t (*handler)(httpd_req_t *);
	};
	static const Route routes[] = {
		{"/apps", HTTP_GET, handleList},
		{"/apps/install", HTTP_POST, handleInstall},
		{"/apps/install/status", HTTP_GET, handleStatus},
		{"/apps/*", HTTP_DELETE, handleRemove},
	};
	int registered = 0;
	for (const Route &route : routes) {
		httpd_uri_t uri = {};
		uri.uri = route.uri;
		uri.method = route.method;
		uri.handler = route.handler;
		if (httpd_register_uri_handler(handle, &uri) == ESP_OK) registered++;
		else ESP_LOGE(kTag, "cannot register %s", route.uri);
	}
	return registered;
}

}  // namespace gea::platform::geapp
