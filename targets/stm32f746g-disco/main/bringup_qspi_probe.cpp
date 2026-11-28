#include "bringup_qspi_probe.h"

#include <stdexcept>

// Linked from an archive placed in QSPI: proves execute-in-place and that an
// exception thrown there unwinds into internal-flash code.
int throwFromQspi(int value) {
	if (value > 0) throw std::runtime_error("thrown from QSPI");

	return value;
}
