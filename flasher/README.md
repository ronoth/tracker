# Ronoth Web Flasher

Browser-based firmware installer for the ESP32-S3 boards (Heltec V3/V4 tracker
and ground station), built on [ESP Web Tools](https://esphome.github.io/esp-web-tools/).
Requires a Web Serial capable browser — Chrome or Edge on desktop.

The Wio Tracker L1 e-ink ground station is nRF52-based and is not flashable
here; use the `.uf2` release asset (double-tap reset, drag onto the USB drive).

## How it deploys

CI (`.github/workflows/build.yml`) merges each ESP32 build into a
`<name>-factory.bin` (bootloader @ 0x0, partition table @ 0x8000, boot_app0 @
0xe000, app @ 0x10000) and attaches it to the GitHub release. The `pages` job
then copies this directory plus those factory images into a site and deploys
it to GitHub Pages:

- automatically on every `v*` tag (after the release is created), or
- manually via *Run workflow* (workflow_dispatch), which pulls the **latest**
  release.

GitHub release assets don't send CORS headers, so the browser can't fetch them
directly — that's why the firmware is copied onto the Pages site at deploy
time instead of referenced by URL in the manifests.

One-time setup: in the repo settings, enable **Pages → Source: GitHub Actions**.

## Layout

- `index.html` — the flasher page (device picker + install button)
- `manifests/*.json` — one ESP Web Tools manifest per target; each flashes a
  single full-flash factory image at offset 0, so it's safe on blank boards
  and with "Erase device" checked. CI stamps the release tag into the
  `version` field at deploy time.
- `firmware/` — not in git; populated at deploy time (or locally, see below)

## Testing locally

Web Serial works on `localhost` without HTTPS. Populate `firmware/` from a
release that has factory images, then serve this directory:

```sh
cd flasher
mkdir -p firmware
curl -sL -o firmware/heltec3-tracker-factory.bin \
  https://github.com/ronoth/tracker/releases/latest/download/heltec3-tracker-factory.bin
# ... repeat for the other targets you want to test
python3 -m http.server 8000
# open http://localhost:8000 in Chrome
```
