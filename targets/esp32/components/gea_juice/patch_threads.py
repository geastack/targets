#!/usr/bin/env python3
"""Apply the reviewed PSRAM task-lifetime changes to the pinned libjuice fork.

Changes are confined to CMake's downloaded dependency; upstream MPL-2.0
copyright/license notices remain intact. Fail if the pinned source drifts.
"""

from pathlib import Path
import sys


def patch(source: str) -> str:
    changes = [
        (
            '#include <freertos/FreeRTOS.h>',
            '#include "esp_heap_caps.h"\n#include <freertos/FreeRTOS.h>',
        ),
        (
            '\txSemaphoreGive(done);\n\tvTaskDelete(NULL);',
            '\txSemaphoreGive(done);\n\tfor (;;) vTaskSuspend(NULL);',
        ),
        (
            'xTaskCreate(juice_task_wrapper, "juice", JUICE_TASK_STACK_SIZE, w, 5, &t->handle)',
            'xTaskCreatePinnedToCoreWithCaps(juice_task_wrapper, "juice", '
            'JUICE_TASK_STACK_SIZE, w, 5, &t->handle, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)',
        ),
        (
            '#define thread_join(t, res) do { (void)(res); '
            'xSemaphoreTake((t).done, portMAX_DELAY); vSemaphoreDelete((t).done); } while(0)',
            '#define thread_join(t, res) do { (void)(res); '
            'xSemaphoreTake((t).done, portMAX_DELAY); '
            'while (eTaskGetState((t).handle) != eSuspended) vTaskDelay(1); '
            'vTaskDeleteWithCaps((t).handle); vSemaphoreDelete((t).done); } while(0)',
        ),
    ]
    for old, new in changes:
        if new in source:
            continue
        if source.count(old) != 1:
            raise ValueError(f"Pinned libjuice thread adapter changed: {old!r}")
        source = source.replace(old, new)
    return source


if __name__ == "__main__":
    path = Path(sys.argv[1])
    original = path.read_text()
    updated = patch(original)
    if updated != original:
        path.write_text(updated)
