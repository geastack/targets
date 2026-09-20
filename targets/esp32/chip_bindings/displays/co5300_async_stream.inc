// SPDX-License-Identifier: GPL-3.0-only
//
// Autonomous QSPI color stream for the CO5300 — the "new driver path" used ONLY
// by the gea3d half-res present. Included into co5300.cpp so it can reach the
// panel-IO internals (spiPanelIo / LcdSpiTransDescriptor / nextQueuedDescriptor).
//
// Difference from the normal beginColorStream/queueColorStreamData path: it does
// NOT call spi_device_acquire_bus. An acquired bus routes the queued transfers
// through a path that only advances when the present task drives/drains it, so
// the wire and the CPU (raster on the other core) can't overlap — they serialize
// to ~25ms/frame. Without acquire, spi_device_queue_trans is ISR-driven: once the
// opening RAMWR + the chunk data are queued, the GDMA chains through them in the
// background while the present task rasters the next chunk on the worker core.
// The command bytes live in the transaction's inline tx_data (SPI_TRANS_USE_TXDATA)
// because the async DMA reads them AFTER beginColorStreamAsync returns — a stack
// buffer (as the polling path uses) would be gone by then.
//
// Every other app keeps the acquired-bus path untouched; this file adds two new
// Panel methods and changes nothing existing.

esp_err_t Panel::beginColorStreamAsync(int command)
{
  if (!panelIo_ || colorStreamActive_)
    return ESP_ERR_INVALID_STATE;

  auto *io = spiPanelIo(panelIo_);
  // Collect any transactions still queued from the previous present (their DMA
  // already completed — the present tail spun on it) so numTransInflight and the
  // descriptor pool start clean. No acquire_bus: the ISR queue is autonomous.
  esp_err_t ret = drainInflight(io);
  if (ret != ESP_OK)
  {
    static bool w = false; if (!w) { w = true; ESP_LOGE(kTag, "async begin: drainInflight %s", esp_err_to_name(ret)); }
    return ret;
  }

  esp_err_t qret = ESP_OK;
  auto *lcdTrans = nextQueuedDescriptor(io, qret);
  if (qret != ESP_OK)
  {
    static bool w = false; if (!w) { w = true; ESP_LOGE(kTag, "async begin: nextDesc %s", esp_err_to_name(qret)); }
    return qret;
  }
  if (!lcdTrans)
    return ESP_ERR_INVALID_STATE;
  std::memset(lcdTrans, 0, sizeof(LcdSpiTransDescriptor));

  asyncCmdWord_ = gea::chips::co5300::CommandSet::qspiColorCommand(command);
  prepareCommandBuffer(asyncCmdWord_, io->lcdCmdBits);
  lcdTrans->base.user = io;
  lcdTrans->flags.dc_gpio_level = io->flags.dcCmdLevel;
  lcdTrans->base.length = io->lcdCmdBits;
  // Persistent (member) command buffer: the async DMA reads it after we return.
  lcdTrans->base.tx_buffer = &asyncCmdWord_;
  lcdTrans->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  if (io->flags.octalMode)
    lcdTrans->base.flags |= SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_MODE_OCT;

  ret = spi_device_queue_trans(io->spiDev, &lcdTrans->base, portMAX_DELAY);
  if (ret != ESP_OK)
  {
    static bool w = false; if (!w) { w = true; ESP_LOGE(kTag, "async begin: queue_trans %s", esp_err_to_name(ret)); }
    return ret;
  }
  io->numTransInflight++;

  colorStreamActive_ = true;
  return ESP_OK;
}

void Panel::endColorStreamAsync()
{
  if (!panelIo_ || !colorStreamActive_)
    return;
  // Never acquired the bus, so nothing to release. The present's region tail has
  // already spun until the last chunk's DMA completed; leftover descriptors are
  // reclaimed by the next present's drainInflight.
  colorStreamActive_ = false;
}
