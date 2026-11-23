# M5Stack PaperS3

<p align="center">
  <a href="https://docs.m5stack.com/en/core/PaperS3"><img src="https://docs.m5stack.com/assets/m5logo.svg" alt="M5Stack" height="36"></a>
</p>

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/517/geastack-papers3-cover.png" alt="GeaStack on the M5Stack PaperS3: TypeScript and CSS compiled to native code" width="760">
</p>

Build apps for the M5Stack PaperS3 in TypeScript, JSX and CSS, the way web
apps are built. GeaStack compiles them into native C++ firmware, with no
JavaScript engine on the device.

Target ID: `esp32-s3-m5stack-papers3`.

## What works

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/517/C139-main-pic_01.jpg" alt="M5Stack PaperS3 board overview" width="480">
</p>
<p align="center"><sub>Board overview from <a href="https://docs.m5stack.com/en/core/PaperS3">M5Stack</a></sub></p>

- 4.7 inch, 960 × 540 e-paper with 16 gray levels, driven directly by the
  ESP32-S3. Apps see a 540 × 960 portrait screen.
- Fast partial updates by default, and a cleaner grayscale mode for text
  (see [E-paper tips](#e-paper-tips)).
- GT911 capacitive touch with two fingers.
- BMI270 accelerometer and gyroscope through the shared sensor binding.
- Battery level from the 1800 mAh cell.
- microSD card storage.
- Wi-Fi and Bluetooth.

The board has no button the ESP32 can read, so apps go back to the launcher
through touch. Sound and the BM8563 clock are not wired up yet.

## Tutorial: your first PaperS3 app

### 1. Install the basics

You need Git, Node.js 22 or newer, npm and Python 3. On macOS, also install
the Apple command line tools:

```sh
xcode-select --install
```

Then install the GeaStack CLI:

```sh
npm install --global @geastack/cli
```

You don't need to clone any GeaStack repository.

### 2. Create an app

```sh
gea create my-paper-app --starter counter
cd my-paper-app
```

This creates the Component Counter, a small touchscreen counter, and installs
its dependencies. Without `--starter counter`, the wizard lets you pick a
blank app or another example from the gallery.

### 3. Install the ESP-IDF toolchain

```sh
gea setup --esp-idf
```

This installs ESP-IDF, or checks the one you already have. `gea doctor` tells
you if anything is still missing.

### 4. Register your PaperS3

Connect the PaperS3 with a USB-C cable and switch it on. Then run:

```sh
gea boards discover
```

It lists the USB devices it finds, with the board's USB serial (on ESP32
boards this is the Wi-Fi MAC address, like `AA:BB:CC:DD:EE:FF`).

The PaperS3 isn't in the `gea setup` board list yet, so add it to your app's
`.gea/boards.json` by hand. Put your board's serial in place of the example:

```json
{
  "papers3": {
    "target": "esp32-s3-m5stack-papers3",
    "adapter": "esp32-idf",
    "transports": {
      "usbSerial": { "serial": "AA:BB:CC:DD:EE:FF" }
    }
  }
}
```

`papers3` is the alias you'll use in every command. Check it with:

```sh
gea boards list
```

### 5. Build and flash

```sh
gea flash --board papers3 --monitor
```

The first build takes a few minutes. Later builds are much faster. When it
finishes, the counter appears on the screen and the serial log opens in your
terminal.

To compile without flashing, for example without a board connected:

```sh
gea build --target esp32-s3-m5stack-papers3
```

### 6. Make it yours

Open `src/App.tsx` and `src/styles.css`, change the text, layout or styles,
and flash again with the same command.

## E-paper tips

E-paper keeps its image without power, but every refresh takes time, so it
suits screens that change on a tap rather than every frame.

You can set how the panel refreshes from your app's entry file,
`src/index.tsx`:

```ts
import { Display, mount } from '@geastack/core'
import { App } from './App'

Display.setEpaperRefreshConfig({
  grayscale: true,              // cleaner text and grays, slower refresh
  fullRefreshEveryPartials: 10, // clear ghosting every 10 updates
})

mount(App)
```

- By default the panel uses the fast mode. Set `grayscale: true` for
  reading apps.
- Small updates can leave faint ghosts of the previous image. A full refresh
  every few updates clears them, and `Display.epaperFullRefresh()` forces one.
- High-contrast designs look best: dark text on white, clear borders, few
  mid-gray areas.
- The panel driver protects the display. If an app refreshes nonstop, it
  pauses the panel for a short rest.

## Hardware references

- [M5Stack PaperS3 documentation](https://docs.m5stack.com/en/core/PaperS3)
- [PaperBoy](https://gitlab.com/zephray/paperboy), the open source firmware
  the pin map was verified against
- [GeaStack on GitHub](https://github.com/geastack)
