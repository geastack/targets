#include "connectivity/ble_hid.h"

#include "ble.h"
#include "bluetooth/hid_profile.h"
#include "imu.h"
#include "services/bluetooth_service.h"
#include "services/ota.h"
#include "wifi.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"

#include "esp_memory_utils.h"
#include "esp_heap_caps.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "sdkconfig.h"

// Central + observer roles (scanning, ble_gap_connect, the GATT client used to
// host a remote HID keyboard and drive a BLE-MIDI pedal) are compiled only on
// targets whose sdkconfig enables them (amoled 2.06). Peripheral-only targets
// keep their exact NimBLE footprint; the base-class no-op defaults serve.
#if defined(CONFIG_BT_NIMBLE_ROLE_CENTRAL) && defined(CONFIG_BT_NIMBLE_ROLE_OBSERVER)
#define GEA_BLE_CENTRAL 1
#else
#define GEA_BLE_CENTRAL 0
#endif

extern "C" void ble_store_config_init(void);

namespace gea::targets::esp32::wifi::wifi_debug {
int state();
int lastError();
std::uint32_t dmaFree();
std::uint32_t dmaLargest();
}  // namespace wifi_debug

namespace gea::targets::esp32::ota_debug {
int startError();
bool started();
std::uint32_t dmaFree();
std::uint32_t dmaLargest();
}  // namespace gea::targets::esp32::ota_debug

namespace gea::targets::esp32::ble {

namespace hid = gea::framework::bluetooth::hid;
using std::intptr_t;
using std::uint16_t;
using std::uint8_t;

class HidServer final : public gea::framework::bluetooth::BluetoothHidDriver {
public:
	static HidServer &instance()
	{
		static HidServer server;
		return server;
	}

	void preinit() override;
	void init(const char *device_name, uint16_t appearance, const char *mac_address) override;
	bool enabled() const override { return enabled_; }
	void setEnabled(bool enabled) override;
	void startAdvertising() override;
	void stopAdvertising() override;
	bool connected() const override { return connected_; }
	bool bound() const override { return kbdConn_ != kNoConn || mouseConn_ != kNoConn; }
	uint8_t batteryLevel() const override { return batteryLevel_; }
	const char *mac() override;
	const char *deviceName() const override { return deviceName_[0] ? deviceName_ : hid::kDefaultDeviceName; }
	void keyTap(int hid_code) override;
	void keyDown(int modifier, int hid_code) override;
	void keyUp() override;
	void mouseMove(int dx, int dy, int buttons, int wheel) override;
	void mouseClick(int button) override;
	void setBatteryLevel(uint8_t level) override;

	// BLE-MIDI peripheral. The MIDI service is registered unconditionally AFTER
	// the HID/battery/IMU services (bonded HID handles stay put); midiEnabled_
	// gates the advertising payload, not the registration.
	void midiEnable() override;
	bool midiBound() const override;
	void midiSend(const uint8_t *packet, int length) override;

	// Live registry of every link this device holds, across all roles.
	int connectionCount() const override;
	int connectionKindAt(int index) const override;
	const char *connectionNameAt(int index) override;

	// Config service (custom GATT). A Web Bluetooth portal reads the app's
	// config blob and, once paired with the 4-digit code shown on-device,
	// writes a new one. Registered unconditionally AFTER every other service
	// (additive rule: handles ahead of it stay stable) — it is cheap, so
	// peripheral-only targets keep it too.
	void configSetDocument(const uint8_t *bytes, int length) override;
	int configPendingLength() const override;
	int configPendingByteAt(int index) const override;
	void configConsumePending() override;
	bool configPairing() const override;
	const char *configPairCode() override;
	void configDismissPairing() override;
	// Device -> portal activity push. Stashes the fired control's index and
	// drains it on the NimBLE host task (ble_gatts_notify_custom is host-task
	// only), sending a CTRL notify (0xA0 + index) to a subscribed portal.
	void configPushActivity(int index) override;
	void drainActivityPush();

#if GEA_BLE_CENTRAL
	// One scan engine, two filters: the scan-result list is shared and cleared
	// on every scan start, so midi*/hidHost* getters read whichever scan ran last.
	// App-facing GAP ops just queue a request onto the host task (see requestBleCmd).
	void midiStartScan() override { reqScanMode_ = static_cast<int>(ScanMode::Midi); requestBleCmd(); }
	void midiStopScan() override { reqScanMode_ = -1; requestBleCmd(); }
	bool midiScanning() const override { return scanMode_ == ScanMode::Midi; }
	int midiScanCount() const override { return scanCount(); }
	const char *midiScanNameAt(int index) override { return scanNameAt(index); }
	void midiConnect(int index) override { reqConnectIndex_ = index; reqConnectKind_ = kLinkMidiPeer; requestBleCmd(); }
	void midiDisconnect() override { reqDisconnectMidi_ = true; requestBleCmd(); }

	void hidHostStartScan() override { reqScanMode_ = static_cast<int>(ScanMode::HidHost); requestBleCmd(); }
	void hidHostStopScan() override { reqScanMode_ = -1; requestBleCmd(); }
	bool hidHostScanning() const override { return scanMode_ == ScanMode::HidHost; }
	int hidHostScanCount() const override { return scanCount(); }
	const char *hidHostScanNameAt(int index) override { return scanNameAt(index); }
	void hidHostConnect(int index) override { reqConnectIndex_ = index; reqConnectKind_ = kLinkHidHosted; requestBleCmd(); }
	void hidHostDisconnect() override { reqDisconnectHid_ = true; requestBleCmd(); }
	bool hidHostBound() const override { return hidHostConn_ != kNoConn && hostSubscribed_ > 0; }

	int hidHostReportCount() const override;
	int hidHostReportIdAt(int index) const override;
	int hidHostReportLenAt(int index) const override;
	int hidHostReportByteAt(int index, int byteIndex) const override;
	void hidHostClearReports() override;
#endif

	static int gapEventCallback(struct ble_gap_event *event, void *arg)
	{
		(void)arg;
		return instance().handleGapEvent(event);
	}

	static int hidAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                             struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleHidAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int batteryAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleBatteryAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int imuAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                             struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleImuAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int midiAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                              struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleMidiAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int configCtrlAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleConfigCtrlAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int configDataAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleConfigDataAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int otaCtrlAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleOtaCtrlAccess(conn_handle, attr_handle, ctxt, arg);
	}

	static int otaDataAccessCallback(uint16_t conn_handle, uint16_t attr_handle,
	                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
	{
		return instance().handleOtaDataAccess(conn_handle, attr_handle, ctxt, arg);
	}

#if GEA_BLE_CENTRAL
	// GATT-client procedure callbacks (run on the NimBLE host task).
	static int midiSvcDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                               const struct ble_gatt_svc *service, void *arg);
	static int midiChrDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                               const struct ble_gatt_chr *chr, void *arg);
	static int midiDscDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                               uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
	static int hidSvcDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                              const struct ble_gatt_svc *service, void *arg);
	static int hidChrDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                              const struct ble_gatt_chr *chr, void *arg);
	static int hidDscDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                              uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
	static int hidRefReadCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                              struct ble_gatt_attr *attr, void *arg);
	static int hidCccdWriteCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
	                                struct ble_gatt_attr *attr, void *arg);
#endif

	static void onSyncCallback() { instance().onSync(); }
	static void onResetCallback(int reason) { instance().onReset(reason); }

	static void hostTask(void *param)
	{
		(void)param;
		ESP_LOGI(kTag, "NimBLE host task started");
		nimble_port_run();
		nimble_port_freertos_deinit();
		vTaskDelete(nullptr);
	}

	// Air-mouse simulator feed: while a central is subscribed to the custom IMU
	// characteristic, sample the QMI8658 at ~100 Hz and notify a packed sample.
	static void imuNotifyTask(void *param);

private:
	enum class HidAccessId : intptr_t {
		HidInformation = 0,
		ReportMap = 1,
		ControlPoint = 2,
		ProtocolMode = 3,
		KeyboardReport = 4,
		MouseReport = 5,
		KeyboardReportReference = 10,
		MouseReportReference = 11,
	};

	HidServer();

	void configureGattTable();
	int handleHidAccess(uint16_t conn_handle, uint16_t attr_handle,
	                    struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleBatteryAccess(uint16_t conn_handle, uint16_t attr_handle,
	                        struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleImuAccess(uint16_t conn_handle, uint16_t attr_handle,
	                    struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleMidiAccess(uint16_t conn_handle, uint16_t attr_handle,
	                     struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleConfigCtrlAccess(uint16_t conn_handle, uint16_t attr_handle,
	                           struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleConfigDataAccess(uint16_t conn_handle, uint16_t attr_handle,
	                           struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleOtaCtrlAccess(uint16_t conn_handle, uint16_t attr_handle,
	                      struct ble_gatt_access_ctxt *ctxt, void *arg);
	int handleOtaDataAccess(uint16_t conn_handle, uint16_t attr_handle,
	                      struct ble_gatt_access_ctxt *ctxt, void *arg);
	void otaCtrlNotify(uint16_t conn, uint8_t opcode,
	                   gea::framework::services::OtaServer::Result result);
	void wifiStatusNotify(uint16_t conn);
	// Config-service helpers (host-task write handler + app-task getters share
	// the config buffers; all guarded by stateMux_). configCtrlNotify() sends a
	// CTRL status notification; NimBLE calls are made OUTSIDE the critical
	// section (never notify/alloc mbufs while holding the spinlock).
	void configCtrlNotify(uint16_t conn, const uint8_t *data, int len);
	bool configIsAuthed(uint16_t conn) const;   // caller holds stateMux_
	void configAddAuthed(uint16_t conn);
	void configClearConn(uint16_t conn);         // drop authed slot on disconnect
	int handleGapEvent(struct ble_gap_event *event);
	void notifyKeyboard();
	void notifyMouse();
	void onSync();
	void onReset(int reason);
	int parseMacLe(const char *str, uint8_t out[6]) const;

	// Connection registry (NimBLE host task writes, app task reads; spinlocked).
	void linkAdd(uint16_t handle, uint8_t kind, const char *name);
	void linkRemove(uint16_t handle);
	void linkSetKind(uint16_t handle, uint8_t kind);
	int inboundLinkCount() const;

#if GEA_BLE_CENTRAL
	enum class ScanMode : uint8_t { None, Midi, HidHost };

	void startScanInternal(ScanMode mode);
	void stopScanInternal(bool restart_adv);
	int scanCount() const;
	const char *scanNameAt(int index);
	void connectTo(int index, uint8_t kind);
	bool issueCentralConnect();   // fire ble_gap_connect for the stored retry target
	bool retryCentralConnect();
	void handleCentralConnected(uint16_t conn_handle);
	void handleDisc(const struct ble_gap_disc_desc *disc);
	void handleHostNotify(const struct ble_gap_event *event);
	void hidHostNextReport(uint16_t conn_handle);
	void hidHostSubscribeCurrent(uint16_t conn_handle, bool input);

	// GAP ops (scan/connect/terminate) MUST run on the NimBLE host task, not
	// the gea app/touch task — calling ble_gap_* from the app task corrupts the
	// register window and crashes (LoadProhibited in ble_gap_adv_active). The
	// app-facing entry points below only stash a request and post this event to
	// the default NimBLE event queue; bleCmdTrampoline drains it on the host
	// task and calls the real startScanInternal/connectTo/stopScanInternal.
	void requestBleCmd();
	void runBleCmd();
	static void bleCmdTrampoline(struct ble_npl_event *ev);
	struct ble_npl_event bleCmdEvent_ = {};
	bool bleCmdInited_ = false;
	// req codes: -2 none, -1 stop-scan; else ScanMode int (1=Midi,2=HidHost)
	volatile int reqScanMode_ = -2;
	volatile int reqConnectIndex_ = -1;
	volatile uint8_t reqConnectKind_ = 0;
	volatile bool reqDisconnectMidi_ = false;
	volatile bool reqDisconnectHid_ = false;
#endif

	static void *accessArg(HidAccessId id)
	{
		return reinterpret_cast<void *>(static_cast<intptr_t>(id));
	}

	static const char kTag[];
	static const ble_uuid16_t kHidServiceUuid;
	static const ble_uuid16_t kHidInformationUuid;
	static const ble_uuid16_t kReportMapUuid;
	static const ble_uuid16_t kHidControlPointUuid;
	static const ble_uuid16_t kProtocolModeUuid;
	static const ble_uuid16_t kReportUuid;
	static const ble_uuid16_t kReportReferenceUuid;
	static const ble_uuid16_t kBatteryServiceUuid;
	static const ble_uuid16_t kBatteryLevelUuid;
	// Custom raw-IMU stream service for the off-device air-mouse simulator (128-bit).
	static const ble_uuid128_t kImuServiceUuid;
	static const ble_uuid128_t kImuCharUuid;
	// BLE-MIDI (MIDI over GATT) service + I/O characteristic (128-bit).
	static const ble_uuid128_t kMidiServiceUuid;
	static const ble_uuid128_t kMidiCharUuid;
	// Config service (custom 128-bit): CTRL command/status + DATA blob transfer.
	static const ble_uuid128_t kConfigServiceUuid;
	static const ble_uuid128_t kConfigCtrlUuid;
	static const ble_uuid128_t kConfigDataUuid;
	static const ble_uuid128_t kOtaServiceUuid;
	static const ble_uuid128_t kOtaCtrlUuid;
	static const ble_uuid128_t kOtaDataUuid;

	// connectionKindAt() values (contract in core ble.h).
	static constexpr uint8_t kLinkHidServed = 0;   // central on our HID service (desktop)
	static constexpr uint8_t kLinkMidiServed = 1;  // central on our MIDI service (DAW)
	static constexpr uint8_t kLinkHidHosted = 2;   // HID peripheral we host (XPPen)
	static constexpr uint8_t kLinkMidiPeer = 3;    // BLE-MIDI peripheral we drive (pedal)
	static constexpr uint16_t kNoConn = BLE_HS_CONN_HANDLE_NONE;

	char deviceName_[32] = {};
	uint16_t appearance_ = hid::kDefaultAppearance;
	char macAddress_[18] = {};
	char formattedMac_[18] = "";

	bool hostSynced_ = false;
	bool controllerInited_ = false;
	bool hostInited_ = false;
	bool enabled_ = true;

	bool connected_ = false;  // >=1 inbound (peripheral-role) link

	// Per-characteristic subscriber conn handles: with MAX_CONNECTIONS>1 the
	// desktop (HID) and a DAW (MIDI) can be connected at once, so each notify
	// target is tracked per subscription instead of one global connHandle_.
	uint16_t kbdConn_ = kNoConn;
	uint16_t mouseConn_ = kNoConn;
	uint16_t batteryConn_ = kNoConn;
	uint16_t imuConn_ = kNoConn;
	uint16_t midiSrvConn_ = kNoConn;  // central subscribed to our MIDI I/O char

	uint16_t keyboardAttrHandle_ = 0;
	uint16_t mouseAttrHandle_ = 0;
	uint16_t batteryAttrHandle_ = 0;
	uint16_t imuAttrHandle_ = 0;
	uint16_t midiAttrHandle_ = 0;
	uint8_t imuLast_[16] = {};   // last packed sample (served on a plain read)

	bool midiEnabled_ = false;

	uint8_t protocolMode_ = hid::kReportProtocol;
	hid::KeyboardMouseReports reports_;
	uint8_t batteryLevel_ = 100;

	struct ble_gatt_dsc_def keyboardReportDescriptors_[2] = {};
	struct ble_gatt_dsc_def mouseReportDescriptors_[2] = {};
	struct ble_gatt_chr_def hidCharacteristics_[7] = {};
	struct ble_gatt_chr_def batteryCharacteristics_[2] = {};
	struct ble_gatt_chr_def imuCharacteristics_[2] = {};
	struct ble_gatt_chr_def midiCharacteristics_[2] = {};
	struct ble_gatt_chr_def configCharacteristics_[3] = {};  // CTRL, DATA, terminator
	struct ble_gatt_chr_def otaCharacteristics_[3] = {};  // CTRL, DATA, terminator
	struct ble_gatt_svc_def gattServices_[7] = {};

	// ---- config service state (host-task callbacks vs app-task getters) -------
	// All config buffers are guarded by stateMux_. The doc is what the portal
	// reads back; inbound accumulates a portal write; pending holds a committed
	// blob for the app to drain.
	static constexpr int kConfigDocMax = 4096;
	uint16_t configCtrlAttrHandle_ = 0;
	uint16_t configDataAttrHandle_ = 0;
	uint16_t configCtrlConn_ = kNoConn;  // central subscribed to CTRL notifications
	uint16_t otaCtrlAttrHandle_ = 0;
	uint16_t otaDataAttrHandle_ = 0;
	uint16_t otaCtrlConn_ = kNoConn;
	// Latest control index the app asked to push to the portal (-1 = none).
	// Written on the app task, drained on the NimBLE host task; only the most
	// recent index is kept (a queued race just supersedes the older one).
	volatile int reqActivityIndex_ = -1;

	// PSRAM-backed (allocated in preinit) — three 4 KB blobs would be 12 KB of
	// scarce internal .bss otherwise, which starved NimBLE's scan allocations
	// and intermittently crashed the central scan (LoadProhibited). Guarded:
	// if allocation ever fails, the config service is inert.
	uint8_t *configDoc_ = nullptr;   // app-published blob (portal READ)
	int configDocLen_ = 0;
	int configReadCursor_ = 0;                  // DATA read cursor (0x01 resets)

	uint8_t *configInbound_ = nullptr;  // portal WRITE accumulator
	int configInboundExpected_ = 0;              // length from 0x02
	int configInboundWrite_ = 0;                 // DATA write cursor

	uint8_t *configPending_ = nullptr;  // committed blob, app drains
	int configPendingLen_ = 0;

	// Connections that completed AUTH (0x93) since they connected; a 0x02
	// begin-WRITE is rejected unless the writing conn is in this set.
	static constexpr int kMaxAuthed = CONFIG_BT_NIMBLE_MAX_CONNECTIONS;
	uint16_t configAuthed_[kMaxAuthed] = {};
	int configAuthedN_ = 0;

	// Pairing overlay state. The code is random per 0x10 and cleared on
	// ok/fail/dismiss/timeout (~60s). Mutable so the const pairing() getter can
	// expire it.
	mutable bool configPairingActive_ = false;
	char configPairCode_[5] = {};              // 4 ASCII digits + NUL
	char configPairCodeOut_[5] = {};           // stable copy for pairCode() getter
	mutable uint32_t configPairStartMs_ = 0;

	// ---- multi-role link state (host-task callbacks vs app-task getters) ----
	// One spinlock guards the registry, scan list and report FIFO; every
	// critical section is a bounded copy of a few dozen bytes.
	mutable portMUX_TYPE stateMux_ = portMUX_INITIALIZER_UNLOCKED;

	struct LinkEntry {
		uint16_t handle;
		uint8_t kind;
		char name[24];
	};
	static constexpr int kMaxLinks = 4;
	LinkEntry links_[kMaxLinks] = {};
	int linkCount_ = 0;
	char linkNameOut_[24] = {};

#if GEA_BLE_CENTRAL
	struct ScanEntry {
		uint8_t addrType;
		uint8_t addr[6];
		char name[24];
	};
	static constexpr int kMaxScan = 16;
	ScanMode scanMode_ = ScanMode::None;
	ScanEntry scan_[kMaxScan] = {};
	int scanN_ = 0;
	char scanNameOut_[24] = {};
	uint8_t pendingCentralKind_ = 0;  // kLinkHidHosted/kLinkMidiPeer while a connect is in flight
	char pendingName_[24] = {};
	// Central connect retry: cheap BLE HID remotes (e.g. "Shortcut Remote")
	// routinely fail the first LL connection with reason 0x3E (0x23E =
	// "connection failed to be established"); retry the same address a few
	// times before giving up.
	ble_addr_t centralRetryAddr_{};
	uint8_t centralRetryKind_ = 0;
	int centralConnectAttempts_ = 0;
	static constexpr int kMaxCentralConnectAttempts = 6;
	// A central HID connect is deferred until the radio is free: the ESP32 fails
	// LL establishment (reason 574 / HCI 0x3E) when it initiates a fresh link
	// while it still services another connection. connectTo() terminates every
	// live link, then the DISCONNECT handler fires issueCentralConnect() once the
	// last one is gone.
	bool centralConnectDeferred_ = false;

	// BLE-MIDI central link (pedal / WIDI adapter).
	uint16_t midiPeerConn_ = kNoConn;
	uint16_t midiSvcStart_ = 0;
	uint16_t midiSvcEnd_ = 0;
	uint16_t midiPeerValHandle_ = 0;
	uint16_t midiPeerCccd_ = 0;

	// Hosted HID peripheral link (XPPen ACK05).
	struct HostReportChr {
		uint16_t defHandle;
		uint16_t valHandle;
		uint16_t endHandle;
		uint16_t cccdHandle;
		uint16_t rptRefHandle;
		uint8_t reportId;
		uint8_t reportType;  // 1=input 2=output 3=feature (from the 0x2908 descriptor)
		bool subscribed;
	};
	static constexpr int kMaxHostReports = 8;
	uint16_t hidHostConn_ = kNoConn;
	uint16_t hidSvcStart_ = 0;
	uint16_t hidSvcEnd_ = 0;
	uint16_t hidProtoModeHandle_ = 0;
	HostReportChr hostReports_[kMaxHostReports] = {};
	int hostReportN_ = 0;
	int hostDiscIdx_ = 0;         // report whose descriptors are being walked
	int hostSubscribed_ = 0;
	bool hostRedoAfterEnc_ = false;  // a secure op failed pre-bond; retry on ENC_CHANGE

	// Raw input-report FIFO (oldest first, drop-oldest on overflow).
	struct HostReportMsg {
		uint8_t id;
		uint8_t len;
		uint8_t data[16];
	};
	static constexpr int kFifoSize = 32;
	HostReportMsg fifo_[kFifoSize] = {};
	int fifoHead_ = 0;
	int fifoCount_ = 0;
#endif

	// Arguments for the transient internal-RAM-stacked worker that performs the
	// flash-touching first-time BLE bring-up off the PSRAM-stacked gea_init task
	// (see init()).
	struct BleFirstInitWork {
		HidServer *self;
		const char *name;
		uint16_t appearance;
		const char *mac;
		SemaphoreHandle_t done;
	};
	static void bleFirstInitTrampoline(void *arg);
};

const char HidServer::kTag[] = "gea_embedded_ble";

const ble_uuid16_t HidServer::kHidServiceUuid = BLE_UUID16_INIT(hid::kServiceUuid);
const ble_uuid16_t HidServer::kHidInformationUuid = BLE_UUID16_INIT(hid::kInformationUuid);
const ble_uuid16_t HidServer::kReportMapUuid = BLE_UUID16_INIT(hid::kReportMapUuid);
const ble_uuid16_t HidServer::kHidControlPointUuid = BLE_UUID16_INIT(hid::kControlPointUuid);
const ble_uuid16_t HidServer::kProtocolModeUuid = BLE_UUID16_INIT(hid::kProtocolModeUuid);
const ble_uuid16_t HidServer::kReportUuid = BLE_UUID16_INIT(hid::kReportUuid);
const ble_uuid16_t HidServer::kReportReferenceUuid = BLE_UUID16_INIT(hid::kReportReferenceUuid);
const ble_uuid16_t HidServer::kBatteryServiceUuid = BLE_UUID16_INIT(hid::kBatteryServiceUuid);
const ble_uuid16_t HidServer::kBatteryLevelUuid = BLE_UUID16_INIT(hid::kBatteryLevelUuid);

// Air-mouse simulator IMU stream (custom 128-bit, little-endian byte order):
//   service 19B10000-E8F2-537E-4F6C-D104768A1214
//   char    19B10001-E8F2-537E-4F6C-D104768A1214  (READ + NOTIFY, unencrypted)
// Notify payload = 16 bytes LE: u32 tms, then 6×i16 (ax,ay,az ×100 m/s²; gx,gy,gz ×10 °/s).
const ble_uuid128_t HidServer::kImuServiceUuid = BLE_UUID128_INIT(
    0x14, 0x12, 0x8a, 0x76, 0x04, 0xd1, 0x6c, 0x4f,
    0x7e, 0x53, 0xf2, 0xe8, 0x00, 0x00, 0xb1, 0x19);
const ble_uuid128_t HidServer::kImuCharUuid = BLE_UUID128_INIT(
    0x14, 0x12, 0x8a, 0x76, 0x04, 0xd1, 0x6c, 0x4f,
    0x7e, 0x53, 0xf2, 0xe8, 0x01, 0x00, 0xb1, 0x19);

// BLE-MIDI (MIDI over GATT), bytes little-endian as transmitted on-air:
//   service 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
//   I/O char 7772E5DB-3868-4112-A1A9-F2669D106BF3 (read empty + write-no-rsp + notify)
const ble_uuid128_t HidServer::kMidiServiceUuid = BLE_UUID128_INIT(
    0x00, 0xc7, 0xc4, 0x4e, 0xe3, 0x6c, 0x51, 0xa7,
    0x33, 0x4b, 0xe8, 0xed, 0x5a, 0x0e, 0xb8, 0x03);
const ble_uuid128_t HidServer::kMidiCharUuid = BLE_UUID128_INIT(
    0xf3, 0x6b, 0x10, 0x9d, 0x66, 0xf2, 0xa9, 0xa1,
    0x12, 0x41, 0x68, 0x38, 0xdb, 0xe5, 0x72, 0x77);

// Config service (custom 128-bit), bytes little-endian as BLE_UUID128_INIT
// wants (reverse of the printed UUID):
//   service A1E0F000-1B2C-4D3E-9F80-1234567890AB
//   CTRL    A1E0F001-... (write-with-rsp + notify: command/status channel)
//   DATA    A1E0F002-... (read + write-no-rsp: chunked blob transfer)
const ble_uuid128_t HidServer::kConfigServiceUuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x80, 0x9f,
    0x3e, 0x4d, 0x2c, 0x1b, 0x00, 0xf0, 0xe0, 0xa1);
const ble_uuid128_t HidServer::kConfigCtrlUuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x80, 0x9f,
    0x3e, 0x4d, 0x2c, 0x1b, 0x01, 0xf0, 0xe0, 0xa1);
const ble_uuid128_t HidServer::kConfigDataUuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x80, 0x9f,
    0x3e, 0x4d, 0x2c, 0x1b, 0x02, 0xf0, 0xe0, 0xa1);

// Geastack BLE OTA service. The CONTROL characteristic accepts BEGIN (0x01 +
// u32 image bytes), FINISH (0x02), and ABORT (0x03). Its notifications are
// opcode|0x80, result, received-u32, expected-u32. DATA is an unacknowledged,
// flow-controlled stream written directly to the inactive ESP-IDF OTA
// partition. CONTROL alone acknowledges the begin and finish boundaries.
//   service 7F2E1001-6D8F-4A4F-A0E9-5B8892140001
//   control 7F2E1001-6D8F-4A4F-A0E9-5B8892140002
//   data    7F2E1001-6D8F-4A4F-A0E9-5B8892140003
const ble_uuid128_t HidServer::kOtaServiceUuid = BLE_UUID128_INIT(
    0x01, 0x00, 0x14, 0x92, 0x88, 0x5b, 0xe9, 0xa0,
    0x4f, 0x4a, 0x8f, 0x6d, 0x01, 0x10, 0x2e, 0x7f);
const ble_uuid128_t HidServer::kOtaCtrlUuid = BLE_UUID128_INIT(
    0x02, 0x00, 0x14, 0x92, 0x88, 0x5b, 0xe9, 0xa0,
    0x4f, 0x4a, 0x8f, 0x6d, 0x01, 0x10, 0x2e, 0x7f);
const ble_uuid128_t HidServer::kOtaDataUuid = BLE_UUID128_INIT(
    0x03, 0x00, 0x14, 0x92, 0x88, 0x5b, 0xe9, 0xa0,
    0x4f, 0x4a, 0x8f, 0x6d, 0x01, 0x10, 0x2e, 0x7f);

#if GEA_BLE_CENTRAL
namespace {
// ATT security errors surface as BLE_HS_ERR_ATT_BASE + att code. A HID
// peripheral commonly rejects report-reference reads / CCCD writes until the
// link encrypts; those ops re-run after ENC_CHANGE.
bool attSecurityError(int status)
{
	return status == BLE_HS_ERR_ATT_BASE + BLE_ATT_ERR_INSUFFICIENT_AUTHEN ||
	       status == BLE_HS_ERR_ATT_BASE + BLE_ATT_ERR_INSUFFICIENT_ENC ||
	       status == BLE_HS_ERR_ATT_BASE + BLE_ATT_ERR_INSUFFICIENT_AUTHOR;
}

// Lenient initiator parameters for cheap / finicky HID remotes (e.g. the
// "Shortcut Remote"). Passing nullptr to ble_gap_connect uses NimBLE's stock
// initiate params, whose narrow scan window and short (~720 ms) supervision
// timeout let these remotes fail LL establishment with HCI 0x3E ("Connection
// Failed to be Established" -> reason 0x23E / 574). A continuous 60 ms scan
// window catches the advertiser reliably, and a 6 s supervision timeout keeps
// the fragile link alive through the first few connection events. CE lengths
// stay 0 = controller default. Interval/timeout in their native BLE units
// (itvl 1.25 ms, timeout 10 ms, scan 0.625 ms).
const struct ble_gap_conn_params kCentralConnParams = {
    /* scan_itvl            */ 0x0060,  // 96  * 0.625 ms = 60 ms
    /* scan_window          */ 0x0060,  // 96  * 0.625 ms = 60 ms (continuous while initiating)
    /* itvl_min             */ 0x0018,  // 24  * 1.25 ms  = 30 ms
    /* itvl_max             */ 0x0028,  // 40  * 1.25 ms  = 50 ms
    /* latency              */ 0x0000,
    /* supervision_timeout  */ 0x0258,  // 600 * 10 ms    = 6000 ms
    /* min_ce_len           */ 0x0000,
    /* max_ce_len           */ 0x0000,
};
}  // namespace
#endif

HidServer::HidServer()
{
	std::snprintf(deviceName_, sizeof(deviceName_), "%s", hid::kDefaultDeviceName);
	configureGattTable();
}

void HidServer::configureGattTable()
{
	keyboardReportDescriptors_[0] = {
		.uuid = &kReportReferenceUuid.u,
		.att_flags = BLE_ATT_F_READ,
		.min_key_size = 0,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::KeyboardReportReference),
	};

	mouseReportDescriptors_[0] = {
		.uuid = &kReportReferenceUuid.u,
		.att_flags = BLE_ATT_F_READ,
		.min_key_size = 0,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::MouseReportReference),
	};

	hidCharacteristics_[0] = {
		.uuid = &kHidInformationUuid.u,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::HidInformation),
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
	};
	hidCharacteristics_[1] = {
		.uuid = &kReportMapUuid.u,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::ReportMap),
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
	};
	hidCharacteristics_[2] = {
		.uuid = &kHidControlPointUuid.u,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::ControlPoint),
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
	};
	hidCharacteristics_[3] = {
		.uuid = &kProtocolModeUuid.u,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::ProtocolMode),
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
		       | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
	};
	hidCharacteristics_[4] = {
		.uuid = &kReportUuid.u,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::KeyboardReport),
		.descriptors = keyboardReportDescriptors_,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &keyboardAttrHandle_,
	};
	hidCharacteristics_[5] = {
		.uuid = &kReportUuid.u,
		.access_cb = hidAccessCallback,
		.arg = accessArg(HidAccessId::MouseReport),
		.descriptors = mouseReportDescriptors_,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &mouseAttrHandle_,
	};

	batteryCharacteristics_[0] = {
		.uuid = &kBatteryLevelUuid.u,
		.access_cb = batteryAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &batteryAttrHandle_,
	};

	// Custom IMU stream characteristic — READ + NOTIFY, intentionally UNENCRYPTED so the
	// browser sim can subscribe over Web Bluetooth without bonding (the HID chars stay
	// encrypted for the OS). Notify-only payload, served on read from imuLast_.
	imuCharacteristics_[0] = {
		.uuid = &kImuCharUuid.u,
		.access_cb = imuAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &imuAttrHandle_,
	};

	gattServices_[0] = {
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &kHidServiceUuid.u,
		.includes = nullptr,
		.characteristics = hidCharacteristics_,
	};
	gattServices_[1] = {
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &kBatteryServiceUuid.u,
		.includes = nullptr,
		.characteristics = batteryCharacteristics_,
	};
	// BLE-MIDI I/O characteristic — read (returns empty per the BLE-MIDI spec) +
	// write-without-response (inbound MIDI) + notify (outbound to a subscribed
	// DAW). Unencrypted like the IMU char so a MIDI host connects without bonding.
	midiCharacteristics_[0] = {
		.uuid = &kMidiCharUuid.u,
		.access_cb = midiAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &midiAttrHandle_,
	};

	// Registered AFTER hid/battery so their attribute handles stay stable (the bonded
	// presentation-clicker host keeps working); the IMU service is purely additive.
	gattServices_[2] = {
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &kImuServiceUuid.u,
		.includes = nullptr,
		.characteristics = imuCharacteristics_,
	};
	// MIDI last, same additive rule: every handle in front of it is unchanged.
	// Registration is unconditional (re-registering NimBLE services after init
	// is not supported); midiEnabled_ gates advertising + app behavior instead.
	gattServices_[3] = {
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &kMidiServiceUuid.u,
		.includes = nullptr,
		.characteristics = midiCharacteristics_,
	};

	// Config service — CTRL (write-with-response + notify: opcode/status
	// channel) and DATA (read + write-no-response: chunked blob transfer).
	// Unencrypted so a Web Bluetooth portal connects without bonding; write
	// access is gated by the 4-digit pairing code, not link encryption.
	configCharacteristics_[0] = {
		.uuid = &kConfigCtrlUuid.u,
		.access_cb = configCtrlAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &configCtrlAttrHandle_,
	};
	configCharacteristics_[1] = {
		.uuid = &kConfigDataUuid.u,
		.access_cb = configDataAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		// F_WRITE (with response) added alongside F_WRITE_NO_RSP: the portal must
		// write DATA chunks WITH response so each chunk is acknowledged before the
		// next and, crucially, before the CTRL COMMIT (0x03). With write-no-response
		// the browser resolves once the chunk is locally queued, so a COMMIT on the
		// CTRL char can overtake the tail DATA chunks — the device then commits with
		// configInboundWrite_ < configInboundExpected_ and rejects with code 3
		// ("parse failed"), which is really a length mismatch. Kept F_WRITE_NO_RSP
		// for back-compat with older portal builds.
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
		.min_key_size = 0,
		.val_handle = &configDataAttrHandle_,
	};

	// Config last, same additive rule: every handle ahead of it is unchanged.
	gattServices_[4] = {
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &kConfigServiceUuid.u,
		.includes = nullptr,
		.characteristics = configCharacteristics_,
	};

#ifdef GEA_EMBEDDED_BLE_OTA
	otaCharacteristics_[0] = {
		.uuid = &kOtaCtrlUuid.u,
		.access_cb = otaCtrlAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
		       | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC
		       | BLE_GATT_CHR_F_NOTIFY,
		.min_key_size = 0,
		.val_handle = &otaCtrlAttrHandle_,
	};
	otaCharacteristics_[1] = {
		.uuid = &kOtaDataUuid.u,
		.access_cb = otaDataAccessCallback,
		.arg = nullptr,
		.descriptors = nullptr,
		// OTA data has no per-chunk application acknowledgement. CoreBluetooth
		// and NimBLE apply link-level flow control; CONTROL acknowledges only
		// BEGIN and FINISH. Advertising WRITE here lets clients accidentally pick
		// the one-round-trip-per-chunk path and turns a firmware update into a
		// many-minute transfer.
		.flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
		.min_key_size = 0,
		.val_handle = &otaDataAttrHandle_,
	};
	gattServices_[5] = {
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &kOtaServiceUuid.u,
		.includes = nullptr,
		.characteristics = otaCharacteristics_,
	};
#endif
}

int HidServer::handleHidAccess(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)conn_handle;
	(void)attr_handle;
	const auto id = static_cast<HidAccessId>(reinterpret_cast<intptr_t>(arg));
	int rc;

	switch (id) {
	case HidAccessId::HidInformation:
		rc = os_mbuf_append(ctxt->om, hid::kInformation, sizeof(hid::kInformation));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

	case HidAccessId::ReportMap:
		rc = os_mbuf_append(ctxt->om, hid::kReportMap, sizeof(hid::kReportMap));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

	case HidAccessId::ControlPoint:
		return 0;

	case HidAccessId::ProtocolMode:
		if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
			rc = os_mbuf_append(ctxt->om, &protocolMode_, 1);
			return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;

	case HidAccessId::KeyboardReport:
	{
		const auto report = reports_.keyboard();
		rc = os_mbuf_append(ctxt->om, report.data, report.size);
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}

	case HidAccessId::MouseReport:
	{
		const auto report = reports_.mouse();
		rc = os_mbuf_append(ctxt->om, report.data, report.size);
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}

	case HidAccessId::KeyboardReportReference:
		rc = os_mbuf_append(ctxt->om, hid::kKeyboardReportReference, sizeof(hid::kKeyboardReportReference));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

	case HidAccessId::MouseReportReference:
		rc = os_mbuf_append(ctxt->om, hid::kMouseReportReference, sizeof(hid::kMouseReportReference));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}

	return BLE_ATT_ERR_UNLIKELY;
}

int HidServer::handleBatteryAccess(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)conn_handle;
	(void)attr_handle;
	(void)arg;

	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		int rc = os_mbuf_append(ctxt->om, &batteryLevel_, 1);
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

int HidServer::handleImuAccess(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)conn_handle;
	(void)attr_handle;
	(void)arg;

	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		int rc = os_mbuf_append(ctxt->om, imuLast_, sizeof(imuLast_));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

int HidServer::handleMidiAccess(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)conn_handle;
	(void)attr_handle;
	(void)arg;

	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		return 0;  // BLE-MIDI reads return no payload
	}
	if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
		// Inbound BLE-MIDI from a connected DAW: no consumer yet — absorb.
		return 0;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

// ---- config service --------------------------------------------------------

void HidServer::configCtrlNotify(uint16_t conn, const uint8_t *data, int len)
{
	if (conn == kNoConn || configCtrlAttrHandle_ == 0 || len <= 0) return;
	struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
	if (om) ble_gatts_notify_custom(conn, configCtrlAttrHandle_, om);
}

bool HidServer::configIsAuthed(uint16_t conn) const
{
	for (int i = 0; i < configAuthedN_; i++) {
		if (configAuthed_[i] == conn) return true;
	}
	return false;
}

void HidServer::configAddAuthed(uint16_t conn)
{
	portENTER_CRITICAL(&stateMux_);
	bool present = false;
	for (int i = 0; i < configAuthedN_; i++) {
		if (configAuthed_[i] == conn) { present = true; break; }
	}
	if (!present && configAuthedN_ < kMaxAuthed) configAuthed_[configAuthedN_++] = conn;
	portEXIT_CRITICAL(&stateMux_);
}

void HidServer::configClearConn(uint16_t conn)
{
	portENTER_CRITICAL(&stateMux_);
	for (int i = 0; i < configAuthedN_; i++) {
		if (configAuthed_[i] != conn) continue;
		for (int j = i; j + 1 < configAuthedN_ && j + 1 < kMaxAuthed; j++) {
			configAuthed_[j] = configAuthed_[j + 1];
		}
		configAuthedN_--;
		break;
	}
	portEXIT_CRITICAL(&stateMux_);
}

int HidServer::handleConfigCtrlAccess(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)attr_handle;
	(void)arg;
	if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

	uint8_t in[8] = {};
	uint16_t inLen = 0;
	ble_hs_mbuf_to_flat(ctxt->om, in, sizeof(in), &inLen);
	if (inLen < 1) return 0;

	uint8_t note[3] = {};
	int noteLen = 0;

	switch (in[0]) {
	case 0x01: {  // begin READ: reset DATA read cursor, report current length
		portENTER_CRITICAL(&stateMux_);
		configReadCursor_ = 0;
		const int len = configDocLen_;
		portEXIT_CRITICAL(&stateMux_);
		note[0] = 0x81;
		note[1] = static_cast<uint8_t>((len >> 8) & 0xFF);
		note[2] = static_cast<uint8_t>(len & 0xFF);
		noteLen = 3;
		break;
	}
	case 0x02: {  // begin WRITE: allocate inbound length (gated by AUTH)
		if (inLen < 3) { note[0] = 0x84; note[1] = 2; noteLen = 2; break; }
		bool authed;
		portENTER_CRITICAL(&stateMux_);
		authed = configIsAuthed(conn_handle);
		portEXIT_CRITICAL(&stateMux_);
		if (!authed) { note[0] = 0x84; note[1] = 1; noteLen = 2; break; }
		const int len = (in[1] << 8) | in[2];
		if (len <= 0 || len > kConfigDocMax) { note[0] = 0x84; note[1] = 2; noteLen = 2; break; }
		portENTER_CRITICAL(&stateMux_);
		configInboundExpected_ = len;
		configInboundWrite_ = 0;
		portEXIT_CRITICAL(&stateMux_);
		// No status for 0x02: the portal streams DATA, then commits with 0x03.
		break;
	}
	case 0x03: {  // COMMIT: validate length, hand inbound to the app-pending buffer
		bool authed;
		portENTER_CRITICAL(&stateMux_);
		authed = configIsAuthed(conn_handle);
		portEXIT_CRITICAL(&stateMux_);
		if (!authed) { note[0] = 0x84; note[1] = 1; noteLen = 2; break; }
		bool ok = false;
		portENTER_CRITICAL(&stateMux_);
		if (configInboundExpected_ > 0 && configInboundWrite_ == configInboundExpected_ && configPending_ && configInbound_) {
			std::memcpy(configPending_, configInbound_, configInboundExpected_);
			configPendingLen_ = configInboundExpected_;
			ok = true;
		}
		configInboundExpected_ = 0;
		configInboundWrite_ = 0;
		portEXIT_CRITICAL(&stateMux_);
		if (ok) { note[0] = 0x83; noteLen = 1; }
		else { note[0] = 0x84; note[1] = 3; noteLen = 2; }  // 3 = parse/length fail
		break;
	}
	case 0x10: {  // AUTH begin: generate a random 4-digit code, show the overlay
		const uint32_t code = esp_random() % 10000u;
		char codeBuf[5];
		std::snprintf(codeBuf, sizeof(codeBuf), "%04u", static_cast<unsigned>(code));
		const uint32_t nowMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
		portENTER_CRITICAL(&stateMux_);
		std::memcpy(configPairCode_, codeBuf, sizeof(codeBuf));
		configPairingActive_ = true;
		configPairStartMs_ = nowMs;
		portEXIT_CRITICAL(&stateMux_);
		ESP_LOGI(kTag, "config: pairing code %s", codeBuf);
		note[0] = 0x91;
		noteLen = 1;
		break;
	}
	case 0x12: {  // AUTH submit: verify the 4 ASCII digits
		bool ok = false;
		portENTER_CRITICAL(&stateMux_);
		if (inLen >= 5 && configPairingActive_ &&
		    in[1] == static_cast<uint8_t>(configPairCode_[0]) &&
		    in[2] == static_cast<uint8_t>(configPairCode_[1]) &&
		    in[3] == static_cast<uint8_t>(configPairCode_[2]) &&
		    in[4] == static_cast<uint8_t>(configPairCode_[3])) {
			ok = true;
		}
		configPairingActive_ = false;  // cleared on ok OR fail
		configPairCode_[0] = '\0';
		portEXIT_CRITICAL(&stateMux_);
		if (ok) configAddAuthed(conn_handle);
		note[0] = ok ? 0x93 : 0x94;
		noteLen = 1;
		break;
	}
	default:
		return 0;
	}

	configCtrlNotify(conn_handle, note, noteLen);
	return 0;
}

int HidServer::handleConfigDataAccess(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)attr_handle;
	(void)arg;

	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		// Serve the next chunk of the config doc, capped at min(ATT_MTU-3, 20).
		const int mtu = ble_att_mtu(conn_handle);
		int cap = mtu > 3 ? mtu - 3 : 20;
		if (cap > 20) cap = 20;
		uint8_t chunk[20];
		int n = 0;
		portENTER_CRITICAL(&stateMux_);
		int remaining = configDocLen_ - configReadCursor_;
		if (remaining < 0) remaining = 0;
		n = remaining < cap ? remaining : cap;
		if (n > 0 && configDoc_) {
			std::memcpy(chunk, configDoc_ + configReadCursor_, n);
			configReadCursor_ += n;
		} else {
			n = 0;
		}
		portEXIT_CRITICAL(&stateMux_);
		if (n > 0) {
			int rc = os_mbuf_append(ctxt->om, chunk, n);
			return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;  // cursor exhausted: empty read
	}

	if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
		// Append this chunk to the inbound buffer (bounded by the 0x02 length).
		uint8_t buf[512];
		uint16_t len = 0;
		ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
		if (len > 0 && configInbound_) {
			portENTER_CRITICAL(&stateMux_);
			int cursor = configInboundWrite_;
			for (int i = 0; i < len && cursor < kConfigDocMax && cursor < configInboundExpected_; i++) {
				configInbound_[cursor++] = buf[i];
			}
			configInboundWrite_ = cursor;
			portEXIT_CRITICAL(&stateMux_);
		}
		return 0;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

// ---- BLE OTA service -------------------------------------------------------

void HidServer::otaCtrlNotify(uint16_t conn, uint8_t opcode,
                              gea::framework::services::OtaServer::Result result)
{
	if (conn == kNoConn || otaCtrlAttrHandle_ == 0) return;
	const uint32_t received = static_cast<uint32_t>(gea::framework::services::OtaServer::receivedBytes());
	const uint32_t expected = static_cast<uint32_t>(gea::framework::services::OtaServer::expectedBytes());
	uint8_t note[10] = {
		static_cast<uint8_t>(opcode | 0x80),
		static_cast<uint8_t>(result),
		static_cast<uint8_t>(received), static_cast<uint8_t>(received >> 8),
		static_cast<uint8_t>(received >> 16), static_cast<uint8_t>(received >> 24),
		static_cast<uint8_t>(expected), static_cast<uint8_t>(expected >> 8),
		static_cast<uint8_t>(expected >> 16), static_cast<uint8_t>(expected >> 24),
	};
	struct os_mbuf *om = ble_hs_mbuf_from_flat(note, sizeof(note));
	if (om) ble_gatts_notify_custom(conn, otaCtrlAttrHandle_, om);
}

void HidServer::wifiStatusNotify(uint16_t conn)
{
	if (conn == kNoConn || otaCtrlAttrHandle_ == 0) return;
	const int state = gea::targets::esp32::wifi::wifi_debug::state();
	const int error = gea::targets::esp32::wifi::wifi_debug::lastError();
	const bool enabled = gea::framework::network::wifi().enabled();
	const bool connected = gea::framework::network::wifi().connected();
	const int otaError = gea::targets::esp32::ota_debug::startError();
	const std::uint32_t liveDmaFree = gea::targets::esp32::ota_debug::dmaFree();
	// Stay within the 18-byte status notification that works even before the
	// central and peripheral have negotiated a larger ATT MTU. Byte 9 is a
	// format marker so the CLI can still decode older diagnostic firmware.
	const uint8_t note[18] = {
		0x85,
		static_cast<uint8_t>(state),
		static_cast<uint8_t>(error), static_cast<uint8_t>(error >> 8),
		static_cast<uint8_t>(error >> 16), static_cast<uint8_t>(error >> 24),
		static_cast<uint8_t>(enabled),
		static_cast<uint8_t>(connected),
		static_cast<uint8_t>(gea::targets::esp32::ota_debug::started()),
		0xa5,
		static_cast<uint8_t>(otaError), static_cast<uint8_t>(otaError >> 8),
		static_cast<uint8_t>(otaError >> 16), static_cast<uint8_t>(otaError >> 24),
		static_cast<uint8_t>(liveDmaFree), static_cast<uint8_t>(liveDmaFree >> 8),
		static_cast<uint8_t>(liveDmaFree >> 16), static_cast<uint8_t>(liveDmaFree >> 24),
	};
	struct os_mbuf *om = ble_hs_mbuf_from_flat(note, sizeof(note));
	if (om) ble_gatts_notify_custom(conn, otaCtrlAttrHandle_, om);
}

int HidServer::handleOtaCtrlAccess(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)attr_handle;
	(void)arg;
	using OtaServer = gea::framework::services::OtaServer;

	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		const uint32_t received = static_cast<uint32_t>(OtaServer::receivedBytes());
		const uint32_t expected = static_cast<uint32_t>(OtaServer::expectedBytes());
		const uint8_t status[10] = {
			0x80, static_cast<uint8_t>(OtaServer::Result::Ok),
			static_cast<uint8_t>(received), static_cast<uint8_t>(received >> 8),
			static_cast<uint8_t>(received >> 16), static_cast<uint8_t>(received >> 24),
			static_cast<uint8_t>(expected), static_cast<uint8_t>(expected >> 8),
			static_cast<uint8_t>(expected >> 16), static_cast<uint8_t>(expected >> 24),
		};
		return os_mbuf_append(ctxt->om, status, sizeof(status)) == 0
		       ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

	uint8_t input[8] = {};
	uint16_t length = 0;
	if (ble_hs_mbuf_to_flat(ctxt->om, input, sizeof(input), &length) != 0 || length < 1) {
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}

	OtaServer::Result result = OtaServer::Result::Ok;
	switch (input[0]) {
	case 0x01: {
		if (length != 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
		// Ask the central for a tight interval while OTA owns the link. The
		// central remains authoritative and may clamp this request, but macOS
		// normally accepts it and gives the no-response stream substantially
		// more connection events per second.
		const struct ble_gap_upd_params otaParams = {
			.itvl_min = 6,             // 7.5 ms
			.itvl_max = 12,            // 15 ms
			.latency = 0,
			.supervision_timeout = 600, // 6 s
			.min_ce_len = 0,
			.max_ce_len = 0,
		};
		const int updateResult = ble_gap_update_params(conn_handle, &otaParams);
		if (updateResult != 0) ESP_LOGW(kTag, "OTA connection-parameter request failed: %d", updateResult);
		const int phyResult = ble_gap_set_prefered_le_phy(
			conn_handle, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
		if (phyResult != 0) ESP_LOGW(kTag, "OTA 2M PHY request failed: %d", phyResult);
		const uint32_t imageSize = static_cast<uint32_t>(input[1])
		                         | (static_cast<uint32_t>(input[2]) << 8)
		                         | (static_cast<uint32_t>(input[3]) << 16)
		                         | (static_cast<uint32_t>(input[4]) << 24);
		result = OtaServer::beginImage(imageSize);
		break;
	}
	case 0x02:
		result = OtaServer::finishImage();
		break;
	case 0x03:
		OtaServer::abortImage();
		break;
	case 0x05:
		wifiStatusNotify(conn_handle);
		return 0;
	default:
		return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
	}

	otaCtrlNotify(conn_handle, input[0], result);
	if (input[0] == 0x02 && result == OtaServer::Result::Ok) OtaServer::rebootSoon();
	return 0;
}

int HidServer::handleOtaDataAccess(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	(void)attr_handle;
	(void)arg;
	if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
	if (OS_MBUF_PKTLEN(ctxt->om) > 512) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

	uint8_t data[512];
	uint16_t length = 0;
	if (ble_hs_mbuf_to_flat(ctxt->om, data, sizeof(data), &length) != 0 || length == 0) {
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}
	const auto result = gea::framework::services::OtaServer::writeImage(data, length);
	if (result != gea::framework::services::OtaServer::Result::Ok) {
		otaCtrlNotify(conn_handle, 0x04, result);
		return BLE_ATT_ERR_UNLIKELY;
	}
	return 0;
}

void HidServer::configSetDocument(const uint8_t *bytes, int length)
{
	if (length < 0) length = 0;
	if (length > kConfigDocMax) length = kConfigDocMax;
	portENTER_CRITICAL(&stateMux_);
	if (length > 0 && bytes && configDoc_) std::memcpy(configDoc_, bytes, length);
	else length = 0;
	configDocLen_ = length;
	configReadCursor_ = 0;
	portEXIT_CRITICAL(&stateMux_);
}

int HidServer::configPendingLength() const
{
	portENTER_CRITICAL(&stateMux_);
	const int n = configPendingLen_;
	portEXIT_CRITICAL(&stateMux_);
	return n;
}

int HidServer::configPendingByteAt(int index) const
{
	portENTER_CRITICAL(&stateMux_);
	const int v = (configPending_ && index >= 0 && index < configPendingLen_) ? configPending_[index] : 0;
	portEXIT_CRITICAL(&stateMux_);
	return v;
}

void HidServer::configConsumePending()
{
	portENTER_CRITICAL(&stateMux_);
	configPendingLen_ = 0;
	portEXIT_CRITICAL(&stateMux_);
}

bool HidServer::configPairing() const
{
	portENTER_CRITICAL(&stateMux_);
	if (configPairingActive_) {
		const uint32_t nowMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
		if (nowMs - configPairStartMs_ > 60000u) configPairingActive_ = false;  // ~60s expiry
	}
	const bool active = configPairingActive_;
	portEXIT_CRITICAL(&stateMux_);
	return active;
}

const char *HidServer::configPairCode()
{
	portENTER_CRITICAL(&stateMux_);
	if (configPairingActive_) std::memcpy(configPairCodeOut_, configPairCode_, sizeof(configPairCodeOut_));
	else configPairCodeOut_[0] = '\0';
	portEXIT_CRITICAL(&stateMux_);
	return configPairCodeOut_;
}

void HidServer::configDismissPairing()
{
	portENTER_CRITICAL(&stateMux_);
	configPairingActive_ = false;
	configPairCode_[0] = '\0';
	portEXIT_CRITICAL(&stateMux_);
}

// App task: stash the fired control's index and hand off to the host task. On
// central builds the existing command queue carries it; peripheral-only builds
// have no queue, but ble_gatts_notify_custom is safe enough to issue inline
// there (the config service is registered on those targets too).
void HidServer::configPushActivity(int index)
{
	reqActivityIndex_ = index;
#if GEA_BLE_CENTRAL
	requestBleCmd();
#else
	drainActivityPush();
#endif
}

// Host task: send the pending activity index to a subscribed portal as a CTRL
// notification (opcode 0xA0 + index). No-op when nothing is pending or when no
// portal is subscribed to the CTRL characteristic.
void HidServer::drainActivityPush()
{
	const int idx = reqActivityIndex_;
	reqActivityIndex_ = -1;
	if (idx < 0) return;
	if (configCtrlConn_ == kNoConn) return;
	uint8_t note[2] = {0xA0, static_cast<uint8_t>(idx)};
	configCtrlNotify(configCtrlConn_, note, 2);
}

namespace {
// Clamp a scaled double to int16 for the packed IMU sample.
int16_t packImu(double value, double scale)
{
	double s = value * scale;
	if (s > 32767.0) s = 32767.0;
	if (s < -32768.0) s = -32768.0;
	return static_cast<int16_t>(s >= 0 ? s + 0.5 : s - 0.5);
}
}  // namespace

void HidServer::imuNotifyTask(void *param)
{
	(void)param;
	HidServer &self = instance();
	bool accelReady = false;
	TickType_t last = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&last, pdMS_TO_TICKS(10));  // ~100 Hz cap
		if (self.imuConn_ == kNoConn || self.imuAttrHandle_ == 0) continue;
		if (!accelReady) {
			gea::platform::sensors::Accelerometer::init();  // idempotent; shares the QMI8658 mutex
			accelReady = true;
		}
		const int16_t vals[6] = {
			packImu(gea::platform::sensors::Accelerometer::accelerationX(), 100.0),
			packImu(gea::platform::sensors::Accelerometer::accelerationY(), 100.0),
			packImu(gea::platform::sensors::Accelerometer::accelerationZ(), 100.0),
			packImu(gea::platform::sensors::Accelerometer::gyroscopeX(), 10.0),
			packImu(gea::platform::sensors::Accelerometer::gyroscopeY(), 10.0),
			packImu(gea::platform::sensors::Accelerometer::gyroscopeZ(), 10.0),
		};
		uint8_t buf[16];
		const uint32_t tms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
		std::memcpy(buf, &tms, 4);
		std::memcpy(buf + 4, vals, sizeof(vals));
		std::memcpy(self.imuLast_, buf, sizeof(buf));
		struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
		if (om) ble_gatts_notify_custom(self.imuConn_, self.imuAttrHandle_, om);
	}
}

void HidServer::notifyKeyboard()
{
	if (kbdConn_ == kNoConn || keyboardAttrHandle_ == 0) return;

	const auto report = reports_.keyboard();
	struct os_mbuf *om = ble_hs_mbuf_from_flat(report.data, report.size);
	if (om) {
		ble_gatts_notify_custom(kbdConn_, keyboardAttrHandle_, om);
	}
}

void HidServer::notifyMouse()
{
	if (mouseConn_ == kNoConn || mouseAttrHandle_ == 0) return;

	const auto report = reports_.mouse();
	struct os_mbuf *om = ble_hs_mbuf_from_flat(report.data, report.size);
	if (om) {
		ble_gatts_notify_custom(mouseConn_, mouseAttrHandle_, om);
	}
}

int HidServer::handleGapEvent(struct ble_gap_event *event)
{
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		ESP_LOGI(kTag, "connection %s (handle=%d)",
		         event->connect.status == 0 ? "established" : "failed",
		         event->connect.conn_handle);
		if (event->connect.status == 0) {
			struct ble_gap_conn_desc desc;
			if (ble_gap_conn_find(event->connect.conn_handle, &desc) != 0) break;
			if (desc.role == BLE_GAP_ROLE_SLAVE) {
				// Inbound: a central (desktop / DAW / sim) connected to us.
				if (!enabled_) {
					ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
					break;
				}
				char nm[24];
				const uint8_t *v = desc.peer_id_addr.val;
				std::snprintf(nm, sizeof(nm), "%02X:%02X:%02X:%02X:%02X:%02X",
				              v[5], v[4], v[3], v[2], v[1], v[0]);
				linkAdd(event->connect.conn_handle, kLinkHidServed, nm);
				connected_ = true;

				// Do NOT initiate security from the peripheral. Forcing pairing on every
				// connection pops a macOS pairing prompt for the air-mouse simulator, whose
				// only goal is to subscribe to the UNENCRYPTED IMU char (no bonding needed),
				// and the pending SMP handshake blocks that CCCD subscribe -> imu stream
				// never starts. A real HOGP host (macOS) initiates encryption itself the
				// moment it accesses an _ENC HID characteristic (report map, HID info, etc.),
				// so the keyboard/mouse still pair host-driven via BLE_GAP_EVENT_ENC_CHANGE /
				// REPEAT_PAIRING. Host-driven pairing only; the peripheral never asks.
				gea::framework::services::BluetoothService::notifyConnected();
#if CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1
				// Keep advertising so a second host (a DAW after the desktop, or
				// vice versa) can still connect while link slots remain.
				startAdvertising();
#endif
			}
#if GEA_BLE_CENTRAL
			else {
				handleCentralConnected(event->connect.conn_handle);
			}
#endif
		} else {
#if GEA_BLE_CENTRAL
			if (pendingCentralKind_) {
				ESP_LOGW(kTag, "central connect failed (status=%d)", event->connect.status);
				pendingCentralKind_ = 0;
			}
			// A failed LL establishment (HCI 0x3E and friends, common on cheap HID
			// remotes) surfaces here as a CONNECT event with nonzero status — retry
			// the dial rather than giving up and dropping back to advertising.
			if (centralRetryKind_ == kLinkHidHosted && retryCentralConnect()) break;
#endif
			startAdvertising();
		}
		break;

	case BLE_GAP_EVENT_DISCONNECT: {
		const uint16_t h = event->disconnect.conn.conn_handle;
		const bool wasInbound = event->disconnect.conn.role == BLE_GAP_ROLE_SLAVE;
		ESP_LOGI(kTag, "disconnected (handle=%d reason=%d)", h, event->disconnect.reason);
#if GEA_BLE_CENTRAL
		// A central HID connect that dropped BEFORE it finished binding (e.g.
		// reason 0x23E / 0x3E, common on cheap remotes) should be retried rather
		// than reported as a hard failure. A bound link that drops is a real
		// disconnect — clear the retry target so it isn't re-dialed.
		bool retryHidConnect = false;
		if (centralRetryKind_ == kLinkHidHosted && hostSubscribed_ == 0 && !wasInbound &&
		    (h == hidHostConn_ || hidHostConn_ == kNoConn)) {
			// A central HID link dropped before it bound — either an established
			// link that collapsed early (h == hidHostConn_) or an LL establishment
			// that failed before handleCentralConnected ran (hidHostConn_ never set,
			// e.g. reason 0x23E / 574). Retry the dial rather than giving up.
			retryHidConnect = true;
		} else if (h == hidHostConn_) {
			// A bound link genuinely dropped — clear the retry target so it isn't re-dialed.
			centralRetryKind_ = 0;
		}
#endif
		linkRemove(h);
		if (kbdConn_ == h) kbdConn_ = kNoConn;
		if (mouseConn_ == h) mouseConn_ = kNoConn;
		if (batteryConn_ == h) batteryConn_ = kNoConn;
		if (imuConn_ == h) imuConn_ = kNoConn;
		if (midiSrvConn_ == h) midiSrvConn_ = kNoConn;
		if (configCtrlConn_ == h) configCtrlConn_ = kNoConn;
		if (otaCtrlConn_ == h) otaCtrlConn_ = kNoConn;
		configClearConn(h);  // drop this conn's AUTH unlock
#if GEA_BLE_CENTRAL
		if (h == midiPeerConn_) {
			midiPeerConn_ = kNoConn;
			midiSvcStart_ = midiSvcEnd_ = 0;
			midiPeerValHandle_ = midiPeerCccd_ = 0;
		}
		if (h == hidHostConn_) {
			hidHostConn_ = kNoConn;
			hidSvcStart_ = hidSvcEnd_ = hidProtoModeHandle_ = 0;
			hostReportN_ = 0;
			hostDiscIdx_ = 0;
			hostSubscribed_ = 0;
			hostRedoAfterEnc_ = false;
		}
#endif
		if (wasInbound) {
			connected_ = inboundLinkCount() > 0;
			if (!connected_) {
				reports_.clear();
				gea::framework::services::BluetoothService::notifyDisconnected();
			}
		}
#if GEA_BLE_CENTRAL
		// A central HID connect was deferred until the radio cleared. Once the
		// last competing link is gone, initiate it now (advertising stays off
		// through the whole attempt so nothing steals radio time).
		if (centralConnectDeferred_ && linkCount_ == 0) {
			centralConnectDeferred_ = false;
			if (issueCentralConnect()) break;
		}
		// Retry a failed central HID establishment before falling back to advertising.
		if (retryHidConnect && retryCentralConnect()) break;
#endif
		// Keep advertising suppressed while a central connect is deferred or
		// retrying; it resumes when the remote binds or the attempt gives up.
		if (enabled_
#if GEA_BLE_CENTRAL
		    && !centralConnectDeferred_ && centralRetryKind_ == 0
#endif
		) startAdvertising();
		break;
	}

	case BLE_GAP_EVENT_SUBSCRIBE: {
		const uint16_t attr = event->subscribe.attr_handle;
		const uint16_t conn = event->subscribe.conn_handle;
		const bool on = event->subscribe.cur_notify;
		ESP_LOGI(kTag, "subscribe: conn=%d attr_handle=%d, notify=%d", conn, attr, on);
		auto bind = [conn, on](uint16_t &slot) {
			if (on) slot = conn;
			else if (slot == conn) slot = kNoConn;
		};
		if (attr == keyboardAttrHandle_) {
			bind(kbdConn_);
			if (on) linkSetKind(conn, kLinkHidServed);
		} else if (attr == mouseAttrHandle_) {
			bind(mouseConn_);
			if (on) linkSetKind(conn, kLinkHidServed);
		} else if (attr == batteryAttrHandle_) {
			bind(batteryConn_);
		} else if (attr == imuAttrHandle_) {
			bind(imuConn_);
		} else if (attr == midiAttrHandle_) {
			bind(midiSrvConn_);
			if (on) linkSetKind(conn, kLinkMidiServed);  // a DAW, not a HID host
		} else if (attr == configCtrlAttrHandle_) {
			bind(configCtrlConn_);  // portal subscribed to CTRL status notifications
		} else if (attr == otaCtrlAttrHandle_) {
			bind(otaCtrlConn_);
		}
		if (kbdConn_ != kNoConn || mouseConn_ != kNoConn) {
			gea::framework::services::BluetoothService::notifyBound();
		}
		break;
	}

	case BLE_GAP_EVENT_ENC_CHANGE: {
		ESP_LOGI(kTag, "encryption change: status=%d", event->enc_change.status);
		// Confirm on serial whether the encrypted link is actually BONDED (keys stored)
		// vs a transient encryption-only session — the signal that the bond stuck.
		struct ble_gap_conn_desc desc;
		if (event->enc_change.status == 0 &&
		    ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
			ESP_LOGI(kTag, "link encrypted: bonded=%d authenticated=%d",
			         desc.sec_state.bonded, desc.sec_state.authenticated);
		}
#if GEA_BLE_CENTRAL
		// A hosted HID peripheral (XPPen) rejects secure GATT ops until the link
		// encrypts: once it does, re-run the report-reference + subscribe pass.
		if (event->enc_change.status == 0 &&
		    event->enc_change.conn_handle == hidHostConn_ && hostRedoAfterEnc_) {
			hostRedoAfterEnc_ = false;
			hostDiscIdx_ = 0;
			hidHostNextReport(hidHostConn_);
		}
#endif
		break;
	}

#if GEA_BLE_CENTRAL
	case BLE_GAP_EVENT_DISC:
		handleDisc(&event->disc);
		break;

	case BLE_GAP_EVENT_DISC_COMPLETE:
		scanMode_ = ScanMode::None;
		break;

	case BLE_GAP_EVENT_NOTIFY_RX:
		handleHostNotify(event);
		break;
#endif

	case BLE_GAP_EVENT_REPEAT_PAIRING: {
		// macOS re-issues an SMP pairing request on a BONDED reconnect in common HOGP
		// cases. The old handler deleted the bond on EVERY repeat-pairing and forced a
		// fresh pair -> the macOS dialog popped on every connect and the two sides'
		// records diverged, so the only recovery was "forget this device" + re-pair.
		// Fix: if we STILL hold a usable bond for this peer, keep it and let the host
		// encrypt with the stored LTK (IGNORE). Only delete + RETRY when there is
		// genuinely no reusable bond (the peer really wiped its keys).
		struct ble_gap_conn_desc desc;
		if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) != 0) {
			return BLE_GAP_REPEAT_PAIRING_RETRY;
		}
		struct ble_store_key_sec key_sec = {};
		key_sec.peer_addr = desc.peer_id_addr;
		struct ble_store_value_sec value_sec;
		if (ble_store_read_peer_sec(&key_sec, &value_sec) == 0) {
			ESP_LOGI(kTag, "repeat pairing: existing bond found, reusing it (ignore re-pair)");
			return BLE_GAP_REPEAT_PAIRING_IGNORE;
		}
		ble_store_util_delete_peer(&desc.peer_id_addr);
		return BLE_GAP_REPEAT_PAIRING_RETRY;
	}

	case BLE_GAP_EVENT_MTU:
		ESP_LOGI(kTag, "MTU update: conn_handle=%d, mtu=%d",
		         event->mtu.conn_handle, event->mtu.value);
		break;
	}

	return 0;
}

void HidServer::startAdvertising()
{
	if (!enabled_) return;
	if (!hostSynced_) {
		ESP_LOGW(kTag, "BLE host not synced; advertising deferred");
		return;
	}
#if GEA_BLE_CENTRAL
	if (scanMode_ != ScanMode::None) return;  // scan and adv can't coexist; resumes on scan stop
	if (ble_gap_adv_active()) return;
#endif

	struct ble_gap_adv_params adv_params = {0};
	struct ble_hs_adv_fields fields = {0};

	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	fields.name = (uint8_t *)deviceName_;
	fields.name_len = std::strlen(deviceName_);
	fields.name_is_complete = 1;
	fields.appearance = appearance_;
	fields.appearance_is_present = 1;

	ble_uuid16_t uuids16[] = {
		BLE_UUID16_INIT(hid::kServiceUuid),
		BLE_UUID16_INIT(hid::kBatteryServiceUuid),
	};
	ble_uuid16_t batteryUuid[] = {
		BLE_UUID16_INIT(hid::kBatteryServiceUuid),
	};
	// appearance=0 is the app's explicit "not a HID device" contract. Mindy
	// serves BLE-MIDI from this shared driver and must not advertise HOGP:
	// doing so makes iOS attach a stale PaperS3 HID bond before the MIDI peer
	// can own the MIDI characteristic. The HID GATT handles remain registered for ABI
	// stability, but only actual HID apps publish the HID service UUID.
	if (appearance_ == 0) {
		fields.uuids16 = batteryUuid;
		fields.num_uuids16 = 1;
		fields.uuids16_is_complete = 1;
	} else {
		fields.uuids16 = uuids16;
		fields.num_uuids16 = 2;
		fields.uuids16_is_complete = 0;
	}

	int rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		ESP_LOGE(kTag, "adv_set_fields failed: %d", rc);
		return;
	}

	// The AD packet is full (flags+name+appearance+16-bit UUIDs), so the 128-bit
	// BLE-MIDI service UUID rides in the scan response — active scanners
	// (macOS/iOS MIDI browsers) see it and list this device as a MIDI port.
	if (midiEnabled_) {
		ble_uuid128_t midi_uuid[1] = {kMidiServiceUuid};
		struct ble_hs_adv_fields rsp = {0};
		rsp.uuids128 = midi_uuid;
		rsp.num_uuids128 = 1;
		rsp.uuids128_is_complete = 1;
		rc = ble_gap_adv_rsp_set_fields(&rsp);
		if (rc != 0) {
			ESP_LOGE(kTag, "adv_rsp_set_fields failed: %d", rc);
		}
	}

	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

	uint8_t own_addr_type = macAddress_[0]
	                        ? BLE_OWN_ADDR_RANDOM : BLE_OWN_ADDR_PUBLIC;
	rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER,
	                       &adv_params, gapEventCallback, nullptr);
	if (rc == BLE_HS_EALREADY) {
		return;
	}
	if (rc != 0) {
		// ENOMEM = every connection slot is in use; advertising resumes when one frees.
		ESP_LOGW(kTag, "adv_start failed: %d", rc);
	} else {
		ESP_LOGI(kTag, "advertising started as \"%s\"", deviceName_);
	}
}

void HidServer::stopAdvertising()
{
	if (!hostSynced_) return;
	ble_gap_adv_stop();
}

// ---- connection registry --------------------------------------------------

void HidServer::linkAdd(uint16_t handle, uint8_t kind, const char *name)
{
	portENTER_CRITICAL(&stateMux_);
	int i = 0;
	while (i < linkCount_ && links_[i].handle != handle) i++;
	if (i == linkCount_) {
		if (linkCount_ == kMaxLinks) {
			portEXIT_CRITICAL(&stateMux_);
			return;
		}
		linkCount_++;
	}
	links_[i].handle = handle;
	links_[i].kind = kind;
	std::snprintf(links_[i].name, sizeof(links_[i].name), "%s", name ? name : "");
	portEXIT_CRITICAL(&stateMux_);
}

void HidServer::linkRemove(uint16_t handle)
{
	portENTER_CRITICAL(&stateMux_);
	for (int i = 0; i < linkCount_; i++) {
		if (links_[i].handle != handle) continue;
		for (int j = i; j < linkCount_ - 1; j++) links_[j] = links_[j + 1];
		linkCount_--;
		break;
	}
	portEXIT_CRITICAL(&stateMux_);
}

void HidServer::linkSetKind(uint16_t handle, uint8_t kind)
{
	portENTER_CRITICAL(&stateMux_);
	for (int i = 0; i < linkCount_; i++) {
		if (links_[i].handle == handle) {
			links_[i].kind = kind;
			break;
		}
	}
	portEXIT_CRITICAL(&stateMux_);
}

int HidServer::inboundLinkCount() const
{
	portENTER_CRITICAL(&stateMux_);
	int n = 0;
	for (int i = 0; i < linkCount_; i++) {
		if (links_[i].kind == kLinkHidServed || links_[i].kind == kLinkMidiServed) n++;
	}
	portEXIT_CRITICAL(&stateMux_);
	return n;
}

int HidServer::connectionCount() const
{
	portENTER_CRITICAL(&stateMux_);
	const int n = linkCount_;
	portEXIT_CRITICAL(&stateMux_);
	return n;
}

int HidServer::connectionKindAt(int index) const
{
	portENTER_CRITICAL(&stateMux_);
	const int kind = (index >= 0 && index < linkCount_) ? links_[index].kind : 0;
	portEXIT_CRITICAL(&stateMux_);
	return kind;
}

const char *HidServer::connectionNameAt(int index)
{
	portENTER_CRITICAL(&stateMux_);
	if (index >= 0 && index < linkCount_) {
		std::snprintf(linkNameOut_, sizeof(linkNameOut_), "%s", links_[index].name);
	} else {
		linkNameOut_[0] = '\0';
	}
	portEXIT_CRITICAL(&stateMux_);
	return linkNameOut_;
}

// ---- BLE-MIDI peripheral ---------------------------------------------------

void HidServer::midiEnable()
{
	if (midiEnabled_) return;
	midiEnabled_ = true;
	// The service is always registered; enabling only adds the MIDI UUID to
	// the scan response. Restart a live advertisement so it takes effect.
	if (hostSynced_ && enabled_ && ble_gap_adv_active()) {
		ble_gap_adv_stop();
		startAdvertising();
	}
}

bool HidServer::midiBound() const
{
	if (midiSrvConn_ != kNoConn) return true;
#if GEA_BLE_CENTRAL
	if (midiPeerConn_ != kNoConn && midiPeerValHandle_ != 0) return true;
#endif
	return false;
}

void HidServer::midiSend(const uint8_t *packet, int length)
{
	if (!packet || length <= 0) return;
	if (midiSrvConn_ != kNoConn && midiAttrHandle_ != 0) {
		struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, length);
		if (om) ble_gatts_notify_custom(midiSrvConn_, midiAttrHandle_, om);
	}
#if GEA_BLE_CENTRAL
	if (midiPeerConn_ != kNoConn && midiPeerValHandle_ != 0) {
		ble_gattc_write_no_rsp_flat(midiPeerConn_, midiPeerValHandle_, packet, length);
	}
#endif
}

#if GEA_BLE_CENTRAL

// ---- central engine: scan --------------------------------------------------

namespace {
const ble_uuid16_t kCccdUuid = BLE_UUID16_INIT(0x2902);
}  // namespace

void HidServer::startScanInternal(ScanMode mode)
{
	ESP_LOGI(kTag, "startScan enter mode=%d synced=%d curmode=%d adv=%d",
	         static_cast<int>(mode), hostSynced_ ? 1 : 0, static_cast<int>(scanMode_),
	         ble_gap_adv_active() ? 1 : 0);
	if (!hostSynced_) { ESP_LOGW(kTag, "startScan abort: not synced"); return; }
	if (scanMode_ == mode) { ESP_LOGW(kTag, "startScan abort: already in mode %d", static_cast<int>(mode)); return; }
	ble_gap_adv_stop();
	if (ble_gap_disc_active()) ble_gap_disc_cancel();
	portENTER_CRITICAL(&stateMux_);
	scanN_ = 0;
	portEXIT_CRITICAL(&stateMux_);
	scanMode_ = mode;

	// Active scan: names and 128-bit service UUIDs commonly ride in the scan
	// response, and both filters need them.
	struct ble_gap_disc_params params = {};
	params.passive = 0;
	params.filter_duplicates = 0;  // scan responses can complete an earlier bare entry
	const uint8_t own = macAddress_[0] ? BLE_OWN_ADDR_RANDOM : BLE_OWN_ADDR_PUBLIC;
	int rc = ble_gap_disc(own, 15000, &params, gapEventCallback, nullptr);
	if (rc != 0 && rc != BLE_HS_EALREADY) {
		ESP_LOGW(kTag, "scan start failed: %d", rc);
		scanMode_ = ScanMode::None;
		startAdvertising();
	} else {
		ESP_LOGI(kTag, "scanning (%s)", mode == ScanMode::Midi ? "midi" : "hid");
	}
}

void HidServer::stopScanInternal(bool restart_adv)
{
	if (ble_gap_disc_active()) ble_gap_disc_cancel();
	scanMode_ = ScanMode::None;
	if (restart_adv) startAdvertising();
}

// Queue a GAP command for the host task and wake it. Called from the app task;
// the actual ble_gap_* calls happen in runBleCmd() on the host task.
void HidServer::requestBleCmd()
{
	if (!bleCmdInited_) {
		ble_npl_event_init(&bleCmdEvent_, bleCmdTrampoline, this);
		bleCmdInited_ = true;
	}
	ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &bleCmdEvent_);
}

void HidServer::bleCmdTrampoline(struct ble_npl_event *ev)
{
	auto *self = static_cast<HidServer *>(ble_npl_event_get_arg(ev));
	if (self) self->runBleCmd();
}

// Runs on the NimBLE host task: safe to call ble_gap_*.
void HidServer::runBleCmd()
{
	const int scanReq = reqScanMode_;
	reqScanMode_ = -2;
	if (scanReq == -1) stopScanInternal(true);
	else if (scanReq == static_cast<int>(ScanMode::Midi)) startScanInternal(ScanMode::Midi);
	else if (scanReq == static_cast<int>(ScanMode::HidHost)) startScanInternal(ScanMode::HidHost);

	const int connIdx = reqConnectIndex_;
	const uint8_t connKind = reqConnectKind_;
	reqConnectIndex_ = -1;
	if (connIdx >= 0) connectTo(connIdx, connKind);

	if (reqDisconnectMidi_) {
		reqDisconnectMidi_ = false;
		if (midiPeerConn_ != kNoConn) ble_gap_terminate(midiPeerConn_, BLE_ERR_REM_USER_CONN_TERM);
	}
	if (reqDisconnectHid_) {
		reqDisconnectHid_ = false;
		if (hidHostConn_ != kNoConn) ble_gap_terminate(hidHostConn_, BLE_ERR_REM_USER_CONN_TERM);
	}

	// Device -> portal activity push queued by the app task.
	drainActivityPush();
}

int HidServer::scanCount() const
{
	portENTER_CRITICAL(&stateMux_);
	const int n = scanN_;
	portEXIT_CRITICAL(&stateMux_);
	return n;
}

const char *HidServer::scanNameAt(int index)
{
	portENTER_CRITICAL(&stateMux_);
	if (index >= 0 && index < scanN_) {
		if (scan_[index].name[0]) {
			std::snprintf(scanNameOut_, sizeof(scanNameOut_), "%s", scan_[index].name);
		} else {
			const uint8_t *v = scan_[index].addr;
			std::snprintf(scanNameOut_, sizeof(scanNameOut_), "%02X:%02X:%02X:%02X:%02X:%02X",
			              v[5], v[4], v[3], v[2], v[1], v[0]);
		}
	} else {
		scanNameOut_[0] = '\0';
	}
	portEXIT_CRITICAL(&stateMux_);
	return scanNameOut_;
}

void HidServer::handleDisc(const struct ble_gap_disc_desc *disc)
{
	if (scanMode_ == ScanMode::None) return;

	struct ble_hs_adv_fields fields;
	if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0) return;

	char name[24] = {};
	if (fields.name != nullptr && fields.name_len > 0) {
		int n = fields.name_len;
		if (n > (int)sizeof(name) - 1) n = sizeof(name) - 1;
		std::memcpy(name, fields.name, n);
		name[n] = '\0';
	}

	bool match = false;
	if (scanMode_ == ScanMode::Midi) {
		for (int i = 0; i < fields.num_uuids128 && !match; i++) {
			if (ble_uuid_cmp(&fields.uuids128[i].u, &kMidiServiceUuid.u) == 0) match = true;
		}
		if (!match && name[0]) {
			match = std::strncmp(name, "WIDI", 4) == 0 || std::strncmp(name, "PuckCC", 6) == 0;
		}
	} else {
		// HID-host scan. Surface any named connectable advertiser (plus anything
		// announcing the HID service / a HID appearance) — MANY BLE keypads &
		// remotes expose 0x1812 only AFTER connect, so name-only devices must
		// still appear. The APP picks the specific controller by name (e.g.
		// "Shortcut Remote") and ignores everything else, so a loose scan here is
		// fine; the HID service is confirmed at GATT-discovery time.
		for (int i = 0; i < fields.num_uuids16 && !match; i++) {
			if (fields.uuids16[i].value == hid::kServiceUuid) match = true;
		}
		if (!match && fields.appearance_is_present) {
			match = (fields.appearance & 0xFFC0) == 0x03C0;
		}
		if (!match && name[0] &&
		    (disc->event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND ||
		     disc->event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND ||
		     disc->event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP)) {
			match = true;
		}
	}

	// A scan response completing a known entry carries the name even when the
	// filterable payload rode in the earlier ADV_IND — merge by address.
	portENTER_CRITICAL(&stateMux_);
	int existing = -1;
	for (int i = 0; i < scanN_; i++) {
		if (scan_[i].addrType == disc->addr.type &&
		    std::memcmp(scan_[i].addr, disc->addr.val, 6) == 0) {
			existing = i;
			break;
		}
	}
	if (existing >= 0) {
		if (name[0] && !scan_[existing].name[0]) {
			std::memcpy(scan_[existing].name, name, sizeof(name));
		}
	} else if (match && scanN_ < kMaxScan) {
		scan_[scanN_].addrType = disc->addr.type;
		std::memcpy(scan_[scanN_].addr, disc->addr.val, 6);
		std::memcpy(scan_[scanN_].name, name, sizeof(name));
		scanN_++;
	}
	portEXIT_CRITICAL(&stateMux_);

	if (match && existing < 0) {
		ESP_LOGI(kTag, "scan hit: %s", name[0] ? name : "(unnamed)");
	}
}

// ---- central engine: connect + GATT client ---------------------------------

void HidServer::connectTo(int index, uint8_t kind)
{
	if (!hostSynced_) return;
	ble_addr_t addr;
	portENTER_CRITICAL(&stateMux_);
	const bool valid = index >= 0 && index < scanN_;
	if (valid) {
		addr.type = scan_[index].addrType;
		std::memcpy(addr.val, scan_[index].addr, 6);
		std::snprintf(pendingName_, sizeof(pendingName_), "%s", scan_[index].name);
	}
	portEXIT_CRITICAL(&stateMux_);
	if (!valid) return;

	if (ble_gap_disc_active()) ble_gap_disc_cancel();
	if (ble_gap_adv_active()) ble_gap_adv_stop();
	scanMode_ = ScanMode::None;
	// Remember the target so the connect (and any retry) can re-issue it.
	centralRetryAddr_ = addr;
	centralRetryKind_ = kind;
	centralConnectAttempts_ = 1;
	centralConnectDeferred_ = false;
	pendingCentralKind_ = 0;

	// ROOT CAUSE of the 0x3E / reason-574 failure: the ESP32 radio cannot bring
	// up a fresh central link while it is simultaneously servicing an existing
	// connection (the persistent bonded HID host on handle=1). Free the radio
	// first — terminate every live link — then initiate the connect from the
	// DISCONNECT handler once the last one is gone. The bonded host auto-
	// reconnects when we re-advertise after the new remote binds.
	if (kind == kLinkHidHosted && linkCount_ > 0) {
		centralConnectDeferred_ = true;
		uint16_t handles[kMaxLinks];
		int n = 0;
		portENTER_CRITICAL(&stateMux_);
		for (int i = 0; i < linkCount_ && n < kMaxLinks; i++) handles[n++] = links_[i].handle;
		portEXIT_CRITICAL(&stateMux_);
		for (int i = 0; i < n; i++) {
			ESP_LOGI(kTag, "central connect: freeing radio, terminating handle=%d", handles[i]);
			ble_gap_terminate(handles[i], BLE_ERR_REM_USER_CONN_TERM);
		}
		return;  // issueCentralConnect() resumes from handleGapEvent(DISCONNECT)
	}

	issueCentralConnect();
}

// Fire ble_gap_connect for the stored retry target (centralRetryAddr_/Kind_).
// Runs on the NimBLE host task. Returns true if the initiate was accepted.
bool HidServer::issueCentralConnect()
{
	if (centralRetryKind_ == 0) return false;
	pendingCentralKind_ = centralRetryKind_;
	if (ble_gap_adv_active()) ble_gap_adv_stop();
	const uint8_t own = macAddress_[0] ? BLE_OWN_ADDR_RANDOM : BLE_OWN_ADDR_PUBLIC;
	// HID remotes get the lenient params; other kinds keep the stock defaults.
	const struct ble_gap_conn_params *cp =
	    centralRetryKind_ == kLinkHidHosted ? &kCentralConnParams : nullptr;
	ESP_LOGI(kTag, "connecting to \"%s\" (kind=%d own=%d addrType=%d attempt=%d links=%d)",
	         pendingName_, centralRetryKind_, own, centralRetryAddr_.type,
	         centralConnectAttempts_, linkCount_);
	int rc = ble_gap_connect(own, &centralRetryAddr_, 10000, cp, gapEventCallback, nullptr);
	if (rc != 0 && rc != BLE_HS_EALREADY) {
		ESP_LOGW(kTag, "central connect failed to start: %d", rc);
		pendingCentralKind_ = 0;
		centralRetryKind_ = 0;
		centralConnectDeferred_ = false;
		startAdvertising();
		return false;
	}
	return true;
}

// Re-issue the last central connect (called from the host task on an early
// establishment failure). Returns true if a retry was launched.
bool HidServer::retryCentralConnect()
{
	if (centralRetryKind_ == 0) return false;
	if (centralConnectAttempts_ >= kMaxCentralConnectAttempts) {
		ESP_LOGW(kTag, "central connect gave up after %d attempts", centralConnectAttempts_);
		centralRetryKind_ = 0;
		centralConnectDeferred_ = false;
		return false;
	}
	centralConnectAttempts_++;
	ESP_LOGI(kTag, "central connect retry #%d", centralConnectAttempts_);
	return issueCentralConnect();
}

void HidServer::handleCentralConnected(uint16_t conn_handle)
{
	const uint8_t kind = pendingCentralKind_;
	pendingCentralKind_ = 0;
	linkAdd(conn_handle, kind, pendingName_[0] ? pendingName_ : "peer");
	ble_gattc_exchange_mtu(conn_handle, nullptr, nullptr);

	if (kind == kLinkMidiPeer) {
		midiPeerConn_ = conn_handle;
		midiSvcStart_ = midiSvcEnd_ = 0;
		midiPeerValHandle_ = midiPeerCccd_ = 0;
		ble_gattc_disc_svc_by_uuid(conn_handle, &kMidiServiceUuid.u, midiSvcDiscCallback, nullptr);
	} else if (kind == kLinkHidHosted) {
		hidHostConn_ = conn_handle;
		hidSvcStart_ = hidSvcEnd_ = hidProtoModeHandle_ = 0;
		hostReportN_ = 0;
		hostDiscIdx_ = 0;
		hostSubscribed_ = 0;
		hostRedoAfterEnc_ = false;
		ble_gattc_disc_svc_by_uuid(conn_handle, &kHidServiceUuid.u, hidSvcDiscCallback, nullptr);
		// HID peripherals gate report subscription behind an encrypted link;
		// initiate as central so bonding runs while discovery proceeds.
		ble_gap_security_initiate(conn_handle);
	}

	// For a fresh central HID link, keep advertising OFF through MTU exchange,
	// service discovery and pairing — a bonded host reconnecting mid-handshake
	// would steal radio time and can collapse the still-fragile remote link.
	// Advertising resumes once the remote binds (hid host ready) or the link
	// drops. Other central kinds keep serving immediately.
	if (kind != kLinkHidHosted) startAdvertising();
}

int HidServer::midiSvcDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   const struct ble_gatt_svc *service, void *arg)
{
	(void)arg;
	HidServer &self = instance();
	if (error->status == 0 && service != nullptr) {
		self.midiSvcStart_ = service->start_handle;
		self.midiSvcEnd_ = service->end_handle;
		return 0;
	}
	if (error->status == BLE_HS_EDONE && self.midiSvcStart_ != 0) {
		ble_gattc_disc_all_chrs(conn_handle, self.midiSvcStart_, self.midiSvcEnd_,
		                        midiChrDiscCallback, nullptr);
	} else {
		ESP_LOGW(kTag, "midi peer: service discovery failed (%d)", error->status);
	}
	return 0;
}

int HidServer::midiChrDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   const struct ble_gatt_chr *chr, void *arg)
{
	(void)arg;
	HidServer &self = instance();
	if (error->status == 0 && chr != nullptr) {
		if (ble_uuid_cmp(&chr->uuid.u, &kMidiCharUuid.u) == 0) {
			self.midiPeerValHandle_ = chr->val_handle;
		}
		return 0;
	}
	if (error->status == BLE_HS_EDONE && self.midiPeerValHandle_ != 0) {
		ESP_LOGI(kTag, "midi peer: I/O char at %d", self.midiPeerValHandle_);
		ble_gattc_disc_all_dscs(conn_handle, self.midiPeerValHandle_, self.midiSvcEnd_,
		                        midiDscDiscCallback, nullptr);
	} else {
		ESP_LOGW(kTag, "midi peer: I/O characteristic not found (%d)", error->status);
	}
	return 0;
}

int HidServer::midiDscDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
	(void)chr_val_handle;
	(void)arg;
	HidServer &self = instance();
	if (error->status == 0 && dsc != nullptr) {
		if (ble_uuid_cmp(&dsc->uuid.u, &kCccdUuid.u) == 0) {
			self.midiPeerCccd_ = dsc->handle;
		}
		return 0;
	}
	if (self.midiPeerCccd_ != 0) {
		// Subscribe to inbound MIDI from the pedal.
		const uint8_t enable[2] = {0x01, 0x00};
		ble_gattc_write_flat(conn_handle, self.midiPeerCccd_, enable, sizeof(enable), nullptr, nullptr);
	}
	ESP_LOGI(kTag, "midi peer ready (cccd=%d)", self.midiPeerCccd_);
	return 0;
}

int HidServer::hidSvcDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                  const struct ble_gatt_svc *service, void *arg)
{
	(void)arg;
	HidServer &self = instance();
	if (error->status == 0 && service != nullptr) {
		self.hidSvcStart_ = service->start_handle;
		self.hidSvcEnd_ = service->end_handle;
		return 0;
	}
	if (error->status == BLE_HS_EDONE && self.hidSvcStart_ != 0) {
		ble_gattc_disc_all_chrs(conn_handle, self.hidSvcStart_, self.hidSvcEnd_,
		                        hidChrDiscCallback, nullptr);
	} else {
		ESP_LOGW(kTag, "hid host: service discovery failed (%d)", error->status);
	}
	return 0;
}

int HidServer::hidChrDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                  const struct ble_gatt_chr *chr, void *arg)
{
	(void)arg;
	HidServer &self = instance();
	if (error->status == 0 && chr != nullptr) {
		if (ble_uuid_cmp(&chr->uuid.u, &kReportUuid.u) == 0 &&
		    self.hostReportN_ < kMaxHostReports) {
			HostReportChr &r = self.hostReports_[self.hostReportN_++];
			r = HostReportChr{};
			r.defHandle = chr->def_handle;
			r.valHandle = chr->val_handle;
			r.endHandle = self.hidSvcEnd_;
		} else if (ble_uuid_cmp(&chr->uuid.u, &kProtocolModeUuid.u) == 0) {
			self.hidProtoModeHandle_ = chr->val_handle;
		}
		return 0;
	}
	if (error->status != BLE_HS_EDONE || self.hostReportN_ == 0) {
		ESP_LOGW(kTag, "hid host: no report characteristics (%d)", error->status);
		return 0;
	}
	// Bound each report's descriptor range by the next characteristic.
	for (int i = 0; i + 1 < self.hostReportN_; i++) {
		self.hostReports_[i].endHandle = self.hostReports_[i + 1].defHandle - 1;
	}
	if (self.hidProtoModeHandle_ != 0) {
		const uint8_t report_mode = 0x01;
		ble_gattc_write_no_rsp_flat(conn_handle, self.hidProtoModeHandle_, &report_mode, 1);
	}
	ESP_LOGI(kTag, "hid host: %d report chars", self.hostReportN_);
	self.hostDiscIdx_ = 0;
	self.hidHostNextReport(conn_handle);
	return 0;
}

void HidServer::hidHostNextReport(uint16_t conn_handle)
{
	if (hostDiscIdx_ >= hostReportN_) {
		ESP_LOGI(kTag, "hid host ready: %d input reports subscribed", hostSubscribed_);
		// The remote is bound: the connect sequence is done, so clear the retry
		// target and resume advertising to let the bonded host reconnect.
		centralRetryKind_ = 0;
		centralConnectDeferred_ = false;
		startAdvertising();
		return;
	}
	const HostReportChr &r = hostReports_[hostDiscIdx_];
	if (r.valHandle >= r.endHandle) {
		// No room for descriptors — nothing to subscribe on this report.
		hostDiscIdx_++;
		hidHostNextReport(conn_handle);
		return;
	}
	ble_gattc_disc_all_dscs(conn_handle, r.valHandle, r.endHandle, hidDscDiscCallback, nullptr);
}

int HidServer::hidDscDiscCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                  uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
	(void)chr_val_handle;
	(void)arg;
	HidServer &self = instance();
	if (self.hostDiscIdx_ >= self.hostReportN_) return 0;
	HostReportChr &r = self.hostReports_[self.hostDiscIdx_];
	if (error->status == 0 && dsc != nullptr) {
		if (ble_uuid_cmp(&dsc->uuid.u, &kCccdUuid.u) == 0) {
			r.cccdHandle = dsc->handle;
		} else if (ble_uuid_cmp(&dsc->uuid.u, &kReportReferenceUuid.u) == 0) {
			r.rptRefHandle = dsc->handle;
		}
		return 0;
	}
	if (r.rptRefHandle != 0) {
		int rc = ble_gattc_read(conn_handle, r.rptRefHandle, hidRefReadCallback, nullptr);
		if (rc != 0) {
			ESP_LOGW(kTag, "hid host: report-ref read failed to start (%d)", rc);
			self.hidHostSubscribeCurrent(conn_handle, true);
		}
	} else {
		// No report reference: single-report device, assume input.
		self.hidHostSubscribeCurrent(conn_handle, true);
	}
	return 0;
}

int HidServer::hidRefReadCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr, void *arg)
{
	(void)arg;
	HidServer &self = instance();
	if (self.hostDiscIdx_ >= self.hostReportN_) return 0;
	HostReportChr &r = self.hostReports_[self.hostDiscIdx_];
	if (error->status == 0 && attr != nullptr && attr->om != nullptr &&
	    OS_MBUF_PKTLEN(attr->om) >= 2) {
		uint8_t ref[2] = {};
		os_mbuf_copydata(attr->om, 0, 2, ref);
		r.reportId = ref[0];
		r.reportType = ref[1];
		self.hidHostSubscribeCurrent(conn_handle, r.reportType == 0x01);
		return 0;
	}
	if (attSecurityError(error->status)) {
		// Pre-bond rejection: re-run this pass once the link encrypts.
		self.hostRedoAfterEnc_ = true;
		ble_gap_security_initiate(conn_handle);
		return 0;
	}
	ESP_LOGW(kTag, "hid host: report-ref read failed (%d)", error->status);
	self.hidHostSubscribeCurrent(conn_handle, true);
	return 0;
}

void HidServer::hidHostSubscribeCurrent(uint16_t conn_handle, bool input)
{
	if (hostDiscIdx_ >= hostReportN_) return;
	HostReportChr &r = hostReports_[hostDiscIdx_];
	if (input && r.cccdHandle != 0 && !r.subscribed) {
		const uint8_t enable[2] = {0x01, 0x00};
		int rc = ble_gattc_write_flat(conn_handle, r.cccdHandle, enable, sizeof(enable),
		                              hidCccdWriteCallback, nullptr);
		if (rc == 0) return;  // advance from the write callback
		ESP_LOGW(kTag, "hid host: cccd write failed to start (%d)", rc);
	}
	hostDiscIdx_++;
	hidHostNextReport(conn_handle);
}

int HidServer::hidCccdWriteCallback(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    struct ble_gatt_attr *attr, void *arg)
{
	(void)attr;
	(void)arg;
	HidServer &self = instance();
	if (self.hostDiscIdx_ >= self.hostReportN_) return 0;
	HostReportChr &r = self.hostReports_[self.hostDiscIdx_];
	if (error->status == 0) {
		if (!r.subscribed) {
			r.subscribed = true;
			self.hostSubscribed_++;
		}
	} else if (attSecurityError(error->status)) {
		self.hostRedoAfterEnc_ = true;
		ble_gap_security_initiate(conn_handle);
	} else {
		ESP_LOGW(kTag, "hid host: cccd write failed (%d)", error->status);
	}
	self.hostDiscIdx_++;
	self.hidHostNextReport(conn_handle);
	return 0;
}

// ---- central engine: inbound notifications + report FIFO -------------------

void HidServer::handleHostNotify(const struct ble_gap_event *event)
{
	const uint16_t conn = event->notify_rx.conn_handle;
	const uint16_t attr = event->notify_rx.attr_handle;
	struct os_mbuf *om = event->notify_rx.om;
	if (om == nullptr) return;

	if (conn == midiPeerConn_ && attr == midiPeerValHandle_) {
		// Inbound MIDI from the pedal: no consumer yet — absorb.
		return;
	}
	if (conn != hidHostConn_) return;

	uint8_t report_id = 0;
	for (int i = 0; i < hostReportN_; i++) {
		if (hostReports_[i].valHandle == attr) {
			report_id = hostReports_[i].reportId;
			break;
		}
	}

	HostReportMsg msg = {};
	msg.id = report_id;
	int len = OS_MBUF_PKTLEN(om);
	if (len > (int)sizeof(msg.data)) len = sizeof(msg.data);
	msg.len = (uint8_t)len;
	os_mbuf_copydata(om, 0, len, msg.data);

	portENTER_CRITICAL(&stateMux_);
	if (fifoCount_ == kFifoSize) {
		fifo_[fifoHead_] = msg;
		fifoHead_ = (fifoHead_ + 1) % kFifoSize;
	} else {
		fifo_[(fifoHead_ + fifoCount_) % kFifoSize] = msg;
		fifoCount_++;
	}
	portEXIT_CRITICAL(&stateMux_);
}

int HidServer::hidHostReportCount() const
{
	portENTER_CRITICAL(&stateMux_);
	const int n = fifoCount_;
	portEXIT_CRITICAL(&stateMux_);
	return n;
}

int HidServer::hidHostReportIdAt(int index) const
{
	portENTER_CRITICAL(&stateMux_);
	const int v = (index >= 0 && index < fifoCount_)
	              ? fifo_[(fifoHead_ + index) % kFifoSize].id : 0;
	portEXIT_CRITICAL(&stateMux_);
	return v;
}

int HidServer::hidHostReportLenAt(int index) const
{
	portENTER_CRITICAL(&stateMux_);
	const int v = (index >= 0 && index < fifoCount_)
	              ? fifo_[(fifoHead_ + index) % kFifoSize].len : 0;
	portEXIT_CRITICAL(&stateMux_);
	return v;
}

int HidServer::hidHostReportByteAt(int index, int byteIndex) const
{
	portENTER_CRITICAL(&stateMux_);
	int v = 0;
	if (index >= 0 && index < fifoCount_) {
		const HostReportMsg &msg = fifo_[(fifoHead_ + index) % kFifoSize];
		if (byteIndex >= 0 && byteIndex < msg.len) v = msg.data[byteIndex];
	}
	portEXIT_CRITICAL(&stateMux_);
	return v;
}

void HidServer::hidHostClearReports()
{
	portENTER_CRITICAL(&stateMux_);
	fifoHead_ = 0;
	fifoCount_ = 0;
	portEXIT_CRITICAL(&stateMux_);
}

#endif  // GEA_BLE_CENTRAL

int HidServer::parseMacLe(const char *str, uint8_t out[6]) const
{
	unsigned int b[6];
	if (std::sscanf(str, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
		return -1;
	}

	/* User writes MSB-first (AA:BB:CC:DD:EE:FF); NimBLE wants LE. */
	for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[5 - i];
	return 0;
}

void HidServer::onSync()
{
	int rc;

	if (macAddress_[0]) {
		uint8_t addr[6];
		if (parseMacLe(macAddress_, addr) == 0) {
			addr[5] |= 0xC0; /* BLE random static: top 2 bits of MSB must be 11. */
			rc = ble_hs_id_set_rnd(addr);
			if (rc != 0) {
				ESP_LOGE(kTag, "ble_hs_id_set_rnd failed: %d", rc);
			} else {
				ESP_LOGI(kTag, "using custom MAC: %s", macAddress_);
			}
		} else {
			ESP_LOGE(kTag, "invalid macAddress format, expected XX:XX:XX:XX:XX:XX");
		}
	}

	rc = ble_hs_util_ensure_addr(0);
	if (rc != 0) {
		ESP_LOGE(kTag, "ble_hs_util_ensure_addr failed: %d", rc);
		return;
	}

	hostSynced_ = true;
	if (enabled_) startAdvertising();
}

void HidServer::onReset(int reason)
{
	hostSynced_ = false;
	ESP_LOGE(kTag, "host reset: reason=%d", reason);
}

void HidServer::preinit()
{
	if (controllerInited_) return;

	// Config blobs live in PSRAM, not internal .bss — keeps ~12 KB of internal
	// RAM free for the BLE controller/host (the central scan was crashing under
	// internal-heap pressure). PSRAM has megabytes free; if it somehow fails,
	// the config service simply stays inert (all accesses are guarded).
	if (configDoc_ == nullptr) {
		configDoc_ = static_cast<uint8_t *>(heap_caps_calloc(1, kConfigDocMax, MALLOC_CAP_SPIRAM));
		configInbound_ = static_cast<uint8_t *>(heap_caps_calloc(1, kConfigDocMax, MALLOC_CAP_SPIRAM));
		configPending_ = static_cast<uint8_t *>(heap_caps_calloc(1, kConfigDocMax, MALLOC_CAP_SPIRAM));
		if (!configDoc_ || !configInbound_ || !configPending_) {
			ESP_LOGE(kTag, "config buffer PSRAM alloc failed; config service disabled");
		}
	}

	esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_err_t err = esp_bt_controller_init(&cfg);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
		return;
	}

	err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
		esp_bt_controller_deinit();
		return;
	}

	controllerInited_ = true;
}

void HidServer::bleFirstInitTrampoline(void *arg)
{
	auto *work = static_cast<BleFirstInitWork *>(arg);
	// Re-enter init() now that we run on an internal-RAM stack: esp_ptr_external_ram()
	// in init() is false here, so it skips the dispatch and performs the real
	// controller/host bring-up (flash ops are now cache-safe). `work` and the
	// strings it points at stay valid because the caller blocks on `done` below.
	work->self->init(work->name, work->appearance, work->mac);
	xSemaphoreGive(work->done);
	vTaskDeleteWithCaps(nullptr);
}

void HidServer::init(const char *device_name, uint16_t appearance, const char *mac_address)
{
	// The app owns its public BLE identity. This driver is shared by every ESP32
	// target, so hard-coding one board name here makes unrelated applications
	// advertise as that board and breaks first-party discovery (for example,
	// Mindy's BLE-MIDI endpoint must appear as "Mindy MIDI").
	const char *resolvedName =
	    device_name && device_name[0] ? device_name : hid::kDefaultDeviceName;
	std::snprintf(deviceName_, sizeof(deviceName_), "%s", resolvedName);
	appearance_ = appearance;
	if (mac_address && mac_address[0]) {
		std::snprintf(macAddress_, sizeof(macAddress_), "%s", mac_address);
	} else {
		macAddress_[0] = '\0';
	}

	if (hostInited_) {
		ble_svc_gap_device_name_set(deviceName_);
		ble_svc_gap_device_appearance_set(appearance_);
		return;
	}

	// First-time bring-up below touches flash (BT controller PHY-calibration
	// load, NimBLE NVS bonding store). The flash driver disables the CPU cache
	// and asserts esp_task_stack_is_sane_cache_disabled(): with the cache off,
	// PSRAM is unreachable, so the running task's stack MUST be in internal RAM.
	// Applications run their _init (and thus this bring-up) on the gea_init
	// task, whose 64 KB stack lives in PSRAM (it has to — the JSX mount needs
	// 64 KB, which won't fit internal). So run the one-time bring-up on a
	// transient internal-RAM-stacked worker and block until it finishes. The
	// per-app re-init above (hostInited_) is RAM-only and stays inline. Keeps
	// BLE init lazy — no boot-time DRAM cost (see BluetoothService::preinitForApp).
	{
		int stack_probe;
		if (esp_ptr_external_ram(&stack_probe)) {
			// The bring-up below must run on an internal-RAM stack: its flash ops
			// disable the CPU cache, which also makes PSRAM unreachable, so the
			// running task's stack cannot be in PSRAM (esp_task_stack_is_sane_
			// cache_disabled() asserts exactly that). gea_init's stack IS in PSRAM,
			// so run the one-time bring-up on a transient internal-RAM-stacked
			// worker. Measured peak stack use of the full bring-up (controller +
			// host init, NVS bond load, connect/bind) is ~1.9 KB, so 6 KB is ~3x
			// margin and fits even when internal RAM is tight (needs only an ~8 KB
			// free block, vs the old 16 KB). If it still can't be allocated, skip
			// BLE — NEVER run the flash ops inline on the PSRAM stack (the crash).
			constexpr uint32_t kWorkerStackBytes = 6144;
			SemaphoreHandle_t done = xSemaphoreCreateBinary();
			if (!done) {
				ESP_LOGE(kTag, "ble_init: semaphore create failed; skipping BLE");
				return;
			}
			BleFirstInitWork work{this, device_name, appearance, mac_address, done};
			TaskHandle_t worker = nullptr;
			const BaseType_t ok = xTaskCreateWithCaps(
			    &HidServer::bleFirstInitTrampoline, "ble_init", kWorkerStackBytes, &work, 5, &worker,
			    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
			if (ok == pdPASS) {
				xSemaphoreTake(done, portMAX_DELAY);
				vSemaphoreDelete(done);
				return;
			}
			vSemaphoreDelete(done);
			const size_t internalLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
			ESP_LOGE(kTag, "ble_init: worker create failed (stack=%u, internal_largest=%u); skipping BLE",
			         (unsigned)kWorkerStackBytes, (unsigned)internalLargest);
			return;
		}
	}

	if (!controllerInited_) {
		preinit();
		if (!controllerInited_) return;
	}

	hostSynced_ = false;

	esp_err_t err = esp_nimble_init();
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "esp_nimble_init failed: %s", esp_err_to_name(err));
		return;
	}

	ble_hs_cfg.reset_cb = onResetCallback;
	ble_hs_cfg.sync_cb = onSyncCallback;
	ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
	ble_hs_cfg.sm_bonding = 1;
	ble_hs_cfg.sm_mitm = 0;
	ble_hs_cfg.sm_sc = 1;
	ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

	ble_hs_cfg.store_read_cb = ble_store_config_read;
	ble_hs_cfg.store_write_cb = ble_store_config_write;
	ble_hs_cfg.store_delete_cb = ble_store_config_delete;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

	ble_store_config_init();

	ble_svc_gap_init();
	ble_svc_gatt_init();

	int rc = ble_gatts_count_cfg(gattServices_);
	if (rc != 0) {
		ESP_LOGE(kTag, "ble_gatts_count_cfg failed: %d", rc);
		return;
	}
	rc = ble_gatts_add_svcs(gattServices_);
	if (rc != 0) {
		ESP_LOGE(kTag, "ble_gatts_add_svcs failed: %d", rc);
		return;
	}

	ble_svc_gap_device_name_set(deviceName_);
	ble_svc_gap_device_appearance_set(appearance_);

	BaseType_t task_ok = xTaskCreatePinnedToCore(
		hostTask,
		"nimble_host",
		NIMBLE_HS_STACK_SIZE,
		nullptr,
		configMAX_PRIORITIES - 4,
		nullptr,
		NIMBLE_CORE
	);
	if (task_ok != pdPASS) {
		ESP_LOGE(kTag, "failed to create NimBLE host task");
		return;
	}

	hostInited_ = true;

	// Air-mouse simulator IMU stream: idles until a central subscribes to the custom
	// characteristic, then samples the QMI8658 at ~100 Hz and notifies. Harmless on boards
	// without the IMU (reads return defaults). Created once, on first host init.
	//
	// Stack lives in PSRAM (MALLOC_CAP_SPIRAM, allowed by CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM):
	// on a RAM-tight display board the old internal-RAM xTaskCreate(4096) silently failed
	// (NO_MEM) — the task never ran, so the sim saw 0 Hz even though the HID mouse (driven by
	// the app, not this task) worked fine. This task only does I2C reads + ble_gatts_notify
	// (no flash/cache-disable ops), so a PSRAM stack is safe. Log on failure instead of ignoring it.
	TaskHandle_t imuTask = nullptr;
	const BaseType_t imu_ok = xTaskCreateWithCaps(
	    imuNotifyTask, "ble_imu", 4096, nullptr, 5, &imuTask, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (imu_ok != pdPASS) {
		ESP_LOGE(kTag, "failed to create ble_imu task (sim IMU stream disabled)");
	}

	ESP_LOGI(kTag, "BLE initialized: name=\"%s\" appearance=0x%04X", deviceName_, appearance_);
}

void HidServer::setEnabled(bool enabled)
{
	if (enabled_ == enabled) return;

	enabled_ = enabled;
	if (!hostInited_) return;

	if (!enabled_) {
		stopAdvertising();
		// Terminate every live link, inbound and outbound alike.
		uint16_t handles[kMaxLinks];
		int n = 0;
		portENTER_CRITICAL(&stateMux_);
		for (int i = 0; i < linkCount_ && n < kMaxLinks; i++) handles[n++] = links_[i].handle;
		portEXIT_CRITICAL(&stateMux_);
		for (int i = 0; i < n; i++) {
			ble_gap_terminate(handles[i], BLE_ERR_REM_USER_CONN_TERM);
		}
		return;
	}

	startAdvertising();
}

const char *HidServer::mac()
{
	if (macAddress_[0]) return macAddress_;
	if (formattedMac_[0]) return formattedMac_;

	if (!hostSynced_) {
		uint8_t addr[6] = {0};
		if (esp_read_mac(addr, ESP_MAC_BT) != ESP_OK &&
		    esp_read_mac(addr, ESP_MAC_BASE) != ESP_OK) return "";

		std::snprintf(formattedMac_, sizeof(formattedMac_), "%02X:%02X:%02X:%02X:%02X:%02X",
		         addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		return formattedMac_;
	}

	uint8_t own_addr_type = 0;
	uint8_t addr[6] = {0};
	int is_nrpa = 0;
	int rc = ble_hs_id_infer_auto(0, &own_addr_type);
	if (rc != 0) return "";

	rc = ble_hs_id_copy_addr(own_addr_type, addr, &is_nrpa);
	if (rc != 0) return "";

	std::snprintf(formattedMac_, sizeof(formattedMac_), "%02X:%02X:%02X:%02X:%02X:%02X",
	         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
	return formattedMac_;
}

void HidServer::keyTap(int hid_code)
{
	reports_.clearKeyboard();
	reports_.keyDown(0, hid_code);
	notifyKeyboard();

	reports_.keyUp();
	notifyKeyboard();
}

void HidServer::keyDown(int modifier, int hid_code)
{
	reports_.keyDown(modifier, hid_code);
	notifyKeyboard();
}

void HidServer::keyUp()
{
	reports_.keyUp();
	notifyKeyboard();
}

void HidServer::mouseMove(int dx, int dy, int buttons, int wheel)
{
	reports_.mouseMove(dx, dy, buttons, wheel);
	notifyMouse();
}

void HidServer::mouseClick(int button)
{
	mouseMove(0, 0, button, 0);
	mouseMove(0, 0, 0, 0);
}

void HidServer::setBatteryLevel(uint8_t level)
{
	if (level > 100) level = 100;
	if (level == batteryLevel_) return;

	batteryLevel_ = level;
	ESP_LOGI(kTag, "battery level: %d%%", level);

	if (batteryConn_ == kNoConn || batteryAttrHandle_ == 0) return;

	struct os_mbuf *om = ble_hs_mbuf_from_flat(&batteryLevel_, 1);
	if (om) {
		ble_gatts_notify_custom(batteryConn_, batteryAttrHandle_, om);
	}
}

namespace {

struct BluetoothDriverRegistration {
	BluetoothDriverRegistration()
	{
		gea::framework::bluetooth::BluetoothHidDevice::setDriver(&HidServer::instance());
	}
};

BluetoothDriverRegistration bluetoothDriverRegistration;

}  // namespace

void registerHidDriver()
{
	gea::framework::bluetooth::BluetoothHidDevice::setDriver(&HidServer::instance());
}

void startOtaServer()
{
#ifdef GEA_EMBEDDED_BLE_OTA
	auto &server = HidServer::instance();
	server.init("Geastack OTA", 0, nullptr);
	server.setEnabled(true);
	server.startAdvertising();
#endif
}

}  // namespace gea::targets::esp32::ble
