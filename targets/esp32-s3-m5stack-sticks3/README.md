# M5Stack StickS3

<p align="center">
  <a href="https://docs.m5stack.com/en/core/StickS3"><img src="https://docs.m5stack.com/assets/m5logo.svg" alt="M5Stack" height="36"></a>
</p>

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/geastack-sticks3-cover.png" alt="GeaStack on the M5Stack StickS3: TypeScript and CSS compiled to native code" width="760">
</p>

Build apps for the M5Stack StickS3 in TypeScript, JSX and CSS, the way web
apps are built. GeaStack compiles them into native C++ firmware, with no
JavaScript engine on the device.

Target ID: `esp32-s3-m5stack-sticks3`.

## What works

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/K150-stickS3_main-products_01.webp" alt="M5Stack StickS3 board overview" width="480">
</p>
<p align="center"><sub>Board overview from <a href="https://docs.m5stack.com/en/core/StickS3">M5Stack</a></sub></p>

- ESP32-S3-PICO-1-N8R8 with 8 MB flash and 8 MB PSRAM.
- 1.14 inch, 135 × 240 ST7789V2 IPS screen. The default CSS device pixel
  ratio is 1.5, so apps lay out on a 90 × 160 CSS-pixel screen.
- The two front buttons, KEY1 and KEY2, arrive as `ArrowUp` and `ArrowDown`
  key events. Holding a button repeats it.
- BMI270 accelerometer.
- Wi-Fi and Bluetooth.

The StickS3 has no touchscreen and no microSD slot. The speaker and the
battery level are not wired up yet, because both go through the M5PM1 power
chip.

## Tutorial: your first StickS3 app

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
gea create my-stick-app --starter counter
cd my-stick-app
```

This creates the Component Counter, a small counter app, and installs its
dependencies. Without `--starter counter`, the wizard lets you pick a blank
app or another example from the gallery.

### 3. Drive it with the buttons

The counter is built for touch, so connect it to the StickS3's buttons. In
`src/App.tsx`, add a `keydown` method and an `onKeyDown` handler on the
outer `div`:

```tsx
  keydown(keyCode: number) {
    if (keyCode === 38) this.increment()      // KEY1 = ArrowUp
    else if (keyCode === 40) this.decrement() // KEY2 = ArrowDown
  }

  template() {
    return (
      <div class="screen" onKeyDown={event => this.keydown(event.keyCode)}>
        ...
      </div>
    )
  }
```

### 4. Install the ESP-IDF toolchain

```sh
gea setup --esp-idf
```

This installs ESP-IDF, or checks the one you already have. `gea doctor` tells
you if anything is still missing.

### 5. Register your StickS3

Connect the StickS3 with a USB-C cable and switch it on. Then run:

```sh
gea boards discover
```

It lists the USB devices it finds, with the board's USB serial (on ESP32
boards this is the Wi-Fi MAC address, like `AA:BB:CC:DD:EE:FF`).

The StickS3 isn't in the `gea setup` board list yet, so add it to your app's
`.gea/boards.json` by hand. Put your board's serial in place of the example:

```json
{
  "sticks3": {
    "target": "esp32-s3-m5stack-sticks3",
    "adapter": "esp32-idf",
    "transports": {
      "usbSerial": { "serial": "AA:BB:CC:DD:EE:FF" }
    }
  }
}
```

`sticks3` is the alias you'll use in every command. Check it with:

```sh
gea boards list
```

### 6. Build and flash

```sh
gea flash --board sticks3 --monitor
```

The first build takes a few minutes. Later builds are much faster. When it
finishes, the counter appears on the screen, the buttons change the number,
and the serial log opens in your terminal.

To compile without flashing, for example without a board connected:

```sh
gea build --target esp32-s3-m5stack-sticks3
```

### 7. Make it yours

Open `src/App.tsx` and `src/styles.css`, change the text, layout or styles,
and flash again with the same command. The screen is small, so large text and
one idea per screen work best.

## Hardware references

- [M5Stack StickS3 documentation](https://docs.m5stack.com/en/core/StickS3)
- [GeaStack on GitHub](https://github.com/geastack)
