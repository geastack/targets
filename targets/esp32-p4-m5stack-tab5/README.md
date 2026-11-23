# M5Stack Tab5

<p align="center">
  <a href="https://docs.m5stack.com/en/core/Tab5"><img src="https://docs.m5stack.com/assets/m5logo.svg" alt="M5Stack" height="36"></a>
</p>

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1132/geastack-tab5-cover.png" alt="GeaStack on the M5Stack Tab5: TypeScript and CSS compiled to native code" width="760">
</p>

Build apps for the M5Stack Tab5 in TypeScript, JSX and CSS, the way web apps
are built. GeaStack compiles them into native C++ firmware, with no
JavaScript engine on the device.

Target ID: `esp32-p4-m5stack-tab5`.

## What works

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1132/C145_01.webp" alt="M5Stack Tab5 board overview" width="480">
</p>
<p align="center"><sub>Board overview from <a href="https://docs.m5stack.com/en/core/Tab5">M5Stack</a></sub></p>

- ESP32-P4 with 16 MB flash and 32 MB PSRAM.
- 5 inch, 1280 × 720 IPS touchscreen. The default CSS device pixel ratio
  is 2, so apps lay out on a 360 × 640 CSS-pixel portrait screen, and they
  can switch orientation.
- Capacitive touch (GT911 on older units, ST7123/ST7121 on current ones).
- Built-in camera, with photos and hardware Motion-JPEG recording.
- ES8388 speaker output and ES7210 microphones, with echo cancellation for
  apps that play and record at the same time.
- BMI270 accelerometer and gyroscope.
- Battery level.
- microSD card storage at `/sdcard`.
- Wi-Fi through the onboard ESP32-C6 co-processor.

## Tutorial: your first Tab5 app

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
gea create my-tab5-app --starter counter
cd my-tab5-app
```

This creates the Component Counter, a small touchscreen counter, and installs
its dependencies. Without `--starter counter`, the wizard lets you pick a
blank app or another example from the gallery.

### 3. Set up the Tab5

Connect the Tab5 with a USB-C cable and run the setup wizard:

```sh
gea setup
```

Choose:

```text
? What do you want to set up?
  Known supported board

? Which board do you have?
  M5Stack Tab5

? Board alias
  m5tab
```

Pick the Tab5 when the wizard lists the connected USB devices, and accept the
ESP-IDF installer if it offers one. The wizard saves the board in your app's
`.gea/boards.json` and prints the flash command when it finishes.

### 4. Build and flash

```sh
gea flash --board m5tab --monitor
```

The first build takes a few minutes. Later builds are much faster. When it
finishes, the counter appears on the screen and the serial log opens in your
terminal.

To compile without flashing, for example without a board connected:

```sh
gea build --target esp32-p4-m5stack-tab5
```

### 5. Make it yours

Open `src/App.tsx` and `src/styles.css`, change the text, layout or styles,
and flash again with the same command.

The example gallery has bigger apps that suit the Tab5's screen, camera and
speaker. Run `gea create` without `--starter` and choose **Example
application** to browse them.

## Hardware references

- [M5Stack Tab5 documentation](https://docs.m5stack.com/en/core/Tab5)
- [GeaStack on GitHub](https://github.com/geastack)
