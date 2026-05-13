#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

DIST_DIR="$ROOT_DIR/dist"
BOOT_APP0="$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
TMP_DIR="$(mktemp -d)"
KEEP_DIST_VERSIONS=3
trap 'rm -rf "$TMP_DIR"' EXIT

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "error: required command not found: $1" >&2
    exit 1
  }
}

fail() {
  echo "error: $*" >&2
  exit 1
}

assert_file() {
  [[ -f "$1" ]] || fail "missing file: $1"
}

assert_contains() {
  local file="$1"
  local text="$2"
  grep -Fq "$text" "$file" || fail "$file does not contain: $text"
}

assert_not_contains() {
  local file="$1"
  local text="$2"
  ! grep -Fq "$text" "$file" || fail "$file must not contain: $text"
}

assert_matches() {
  local file="$1"
  local pattern="$2"
  grep -Eq "$pattern" "$file" || fail "$file does not match: $pattern"
}

validate_config_and_get_version() {
  local config_json
  config_json="$(pio project config --json-output)"
  CONFIG_JSON="$config_json" python3 - <<'PY'
import json
import os
import sys

sections = {name: dict(items) for name, items in json.loads(os.environ["CONFIG_JSON"])}


def die(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(1)


def section(name):
    try:
        return sections[name]
    except KeyError:
        die(f"missing platformio section [{name}]")


def flags(sec):
    value = sec.get("build_flags", [])
    if isinstance(value, list):
        return value
    return [line.strip() for line in str(value).splitlines() if line.strip()]


def expect(sec_name, key, expected):
    actual = section(sec_name).get(key)
    if actual != expected:
        die(f"[{sec_name}] {key} is {actual!r}, expected {expected!r}")


def expect_empty(sec_name, key):
    actual = section(sec_name).get(key, "")
    if actual not in ("", [], None):
        die(f"[{sec_name}] {key} is {actual!r}, expected empty")


def expect_define(sec_name, name, value=None):
    prefix = f"-D{name}"
    expected = prefix if value is None else f"{prefix}="
    for flag in flags(section(sec_name)):
        if value is None and flag == prefix:
            return
        if value is not None and flag.startswith(expected) and value in flag:
            return
    die(f"[{sec_name}] missing build flag {name}={value}")


def define_value(sec_name, name):
    prefix = f"-D{name}="
    matches = []
    for flag in flags(section(sec_name)):
        if flag.startswith(prefix):
            matches.append(flag.split("=", 1)[1])
    if not matches:
        die(f"[{sec_name}] missing build flag {name}")
    if len(matches) > 1:
        die(f"[{sec_name}] has duplicate build flag {name}")
    return matches[0]


def reject_define(sec_name, name):
    needle = f"-D{name}"
    for flag in flags(section(sec_name)):
        if flag.startswith(needle):
            die(f"[{sec_name}] must not define {name}")


expect("common", "board_build.partitions", "partitions.csv")

debug_log = define_value("common", "MYMOTA32_DEBUG_LOG")
core_debug = define_value("common", "CORE_DEBUG_LEVEL")
if debug_log not in ("0", "1"):
    die(f"[common] MYMOTA32_DEBUG_LOG is {debug_log!r}, expected 0 or 1")
if debug_log == "1" and core_debug == "0":
    die("[common] MYMOTA32_DEBUG_LOG=1 requires CORE_DEBUG_LEVEL above 0")
if debug_log == "0" and core_debug != "0":
    die("[common] MYMOTA32_DEBUG_LOG=0 should use CORE_DEBUG_LEVEL=0")
print(f"debug build: MYMOTA32_DEBUG_LOG={debug_log} CORE_DEBUG_LEVEL={core_debug}", file=sys.stderr)

expect("env:mymota32-esp32-d0wd-v3-4m", "board", "esp32dev")
expect("env:mymota32-esp32-d0wd-v3-4m", "board_build.f_flash", "40000000L")
expect("env:mymota32-esp32-d0wd-v3-4m", "board_build.flash_mode", "dio")
expect_define("env:mymota32-esp32-d0wd-v3-4m", "MYMOTA32_TARGET", "esp32-d0wd-v3-4m")
reject_define("env:mymota32-esp32-d0wd-v3-4m", "MYMOTA32_ESP32_U4WDH")
reject_define("env:mymota32-esp32-d0wd-v3-4m", "CORE32SOLO1")
reject_define("env:mymota32-esp32-d0wd-v3-4m", "FRAMEWORK_ARDUINO_SOLO1")

expect("env:mymota32-esp32-u4wdh-d-4m", "board", "esp32dev")
expect("env:mymota32-esp32-u4wdh-d-4m", "board_build.f_flash", "40000000L")
expect("env:mymota32-esp32-u4wdh-d-4m", "board_build.flash_mode", "dio")
expect_define("env:mymota32-esp32-u4wdh-d-4m", "MYMOTA32_TARGET", "esp32-u4wdh-d-4m")
expect_define("env:mymota32-esp32-u4wdh-d-4m", "MYMOTA32_ESP32_U4WDH", "1")
reject_define("env:mymota32-esp32-u4wdh-d-4m", "CORE32SOLO1")
reject_define("env:mymota32-esp32-u4wdh-d-4m", "FRAMEWORK_ARDUINO_SOLO1")

expect("env:mymota32-esp32-u4wdh-s-4m", "board", "esp32-solo1")
expect_empty("env:mymota32-esp32-u4wdh-s-4m", "board_espidf.custom_sdkconfig")
expect("env:mymota32-esp32-u4wdh-s-4m", "board_build.f_flash", "40000000L")
expect("env:mymota32-esp32-u4wdh-s-4m", "board_build.flash_mode", "dio")
expect_define("env:mymota32-esp32-u4wdh-s-4m", "MYMOTA32_TARGET", "esp32-u4wdh-s-4m")
expect_define("env:mymota32-esp32-u4wdh-s-4m", "MYMOTA32_ESP32_U4WDH", "1")
expect_define("env:mymota32-esp32-u4wdh-s-4m", "FRAMEWORK_ARDUINO_SOLO1")

expect("env:mymota32-esp32-c3-4m", "board", "esp32-c3-devkitm-1")
expect("env:mymota32-esp32-c3-4m", "board_build.f_flash", "80000000L")
expect("env:mymota32-esp32-c3-4m", "board_build.flash_mode", "dio")
expect_define("env:mymota32-esp32-c3-4m", "MYMOTA32_TARGET", "esp32-c3-4m")

version = define_value("common", "MYMOTA32_VERSION").replace('\\"', '"').strip('"')
print(version)
PY
}

esptool() {
  pio pkg exec -p tool-esptoolpy -- esptool.py "$@"
}

verify_app_image() {
  local image="$1"
  local expected_type="$2"
  local expected_freq="$3"
  local info_file="$4"

  esptool image_info "$image" >"$info_file"
  assert_contains "$info_file" "Detected image type: $expected_type"
  assert_contains "$info_file" "Flash size: 4MB"
  assert_contains "$info_file" "Flash freq: $expected_freq"
  assert_contains "$info_file" "Flash mode: DIO"
  assert_matches "$info_file" '^Checksum: .*\(valid\)$'
  assert_matches "$info_file" '^Validation hash: .*\(valid\)$'
}

verify_first_bytes() {
  local image="$1"
  local expected_hex="$2"
  local got
  got="$(od -An -tx1 -N"$(( ${#expected_hex} / 2 ))" "$image" | tr -d ' \n')"
  [[ "$got" == "$expected_hex" ]] || fail "$image starts with $got, expected $expected_hex"
}

verify_merged_layout() {
  local image="$1"
  shift
  python3 - "$image" "$@" <<'PY'
from pathlib import Path
import sys

image = Path(sys.argv[1])
pairs = sys.argv[2:]
if len(pairs) % 2:
    print("error: verify_merged_layout needs offset/path pairs", file=sys.stderr)
    sys.exit(1)

data = image.read_bytes()
for offset_text, path_text in zip(pairs[0::2], pairs[1::2]):
    offset = int(offset_text, 0)
    path = Path(path_text)
    blob = path.read_bytes()
    chunk = data[offset:offset + len(blob)]
    if chunk != blob:
        print(f"error: {image} does not contain {path} at {offset_text}", file=sys.stderr)
        sys.exit(1)
PY
}

build_target() {
  local env_name="$1"
  local target_name="$2"
  local chip="$3"
  local image_type="$4"
  local flash_freq="$5"
  local bootloader_offset="$6"
  local display_name="$7"

  local build_dir="$ROOT_DIR/.pio/build/$env_name"
  local raw_bin="$DIST_DIR/mymota32-$VERSION-$target_name.bin"
  local factory_bin="$DIST_DIR/mymota32-$VERSION-$target_name-factory.bin"
  local image_info_file="$TMP_DIR/mymota32-$VERSION-$target_name.image_info.txt"
  local boot_app0="$BOOT_APP0"
  local safeboot_offset="0x10000"
  local safeboot_size="$((0xD0000))"
  local ota_slot_size="$((0x1D0000))"
  local ota_slot_margin="$((10 * 1024))"
  local ota_slot_limit="$((ota_slot_size - ota_slot_margin))"
  local app0_offset="0xe0000"

  echo "==> Building $display_name ($env_name)"
  pio run -e "$env_name" -t clean >/dev/null
  pio run -e "$env_name"

  assert_file "$build_dir/firmware.bin"
  assert_file "$build_dir/bootloader.bin"
  assert_file "$build_dir/partitions.bin"
  if [[ "$target_name" == "esp32-u4wdh-s-4m" ]]; then
    boot_app0="$HOME/.platformio/packages/framework-arduino-solo1/tools/partitions/boot_app0.bin"
  fi
  assert_file "$boot_app0"

  cp "$build_dir/firmware.bin" "$raw_bin"
  cmp -s "$build_dir/firmware.bin" "$raw_bin" || fail "copy failed for $raw_bin"
  local raw_size
  raw_size="$(stat -c '%s' "$raw_bin")"
  (( raw_size <= ota_slot_limit )) || fail "$raw_bin is $raw_size bytes; must stay at least 10240 bytes below the 0x1D0000 dual-OTA slot"

  verify_app_image "$raw_bin" "$image_type" "$flash_freq" "$image_info_file"
  verify_first_bytes "$raw_bin" "e9"

  local factory_status
  if (( raw_size <= safeboot_size )); then
    esptool --chip "$chip" merge_bin \
      -o "$factory_bin" \
      --flash_mode dio \
      --flash_freq "$flash_freq" \
      --flash_size 4MB \
      "$bootloader_offset" "$build_dir/bootloader.bin" \
      0x8000 "$build_dir/partitions.bin" \
      0xe000 "$boot_app0" \
      "$safeboot_offset" "$raw_bin" \
      "$app0_offset" "$raw_bin"

    verify_merged_layout "$factory_bin" \
      "$bootloader_offset" "$build_dir/bootloader.bin" \
      0x8000 "$build_dir/partitions.bin" \
      0xe000 "$boot_app0" \
      "$safeboot_offset" "$raw_bin" \
      "$app0_offset" "$raw_bin"
    factory_status="${factory_bin#$ROOT_DIR/}"
  else
    rm -f "$factory_bin"
    factory_status="skipped; raw image exceeds 0xD0000 safeboot slot"
  fi

  if [[ "$target_name" == "esp32-d0wd-v3-4m" || "$target_name" == "esp32-u4wdh-d-4m" ]]; then
    local esp32_sdkconfig="$HOME/.platformio/packages/framework-arduinoespressif32/tools/esp32-arduino-libs/esp32/sdkconfig"
    assert_file "$esp32_sdkconfig"
    assert_contains "$esp32_sdkconfig" "# CONFIG_FREERTOS_UNICORE is not set"
    assert_not_contains "$esp32_sdkconfig" "CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE=y"
  fi

  if [[ "$target_name" == "esp32-u4wdh-s-4m" ]]; then
    local solo_sdkconfig="$HOME/.platformio/packages/framework-arduino-solo1/tools/esp32-arduino-libs/esp32/sdkconfig"
    assert_file "$solo_sdkconfig"
    assert_contains "$solo_sdkconfig" "CONFIG_FREERTOS_UNICORE=y"
    assert_contains "$solo_sdkconfig" "CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE=y"
    assert_file "$build_dir/firmware.map"
    assert_contains "$build_dir/firmware.map" "framework-arduino-solo1/tools/esp32-arduino-libs"
    assert_not_contains "$build_dir/firmware.map" "framework-arduinoespressif32/tools/esp32-arduino-libs"
  fi

  echo "    target:  $display_name"
  echo "    OTA:     ${raw_bin#$ROOT_DIR/}"
  echo "    factory: $factory_status"
  echo "    margin:  $((ota_slot_size - raw_size)) bytes below 0x1D0000 OTA slot"
  if (( raw_size <= safeboot_size )); then
    echo "    safeboot: $((safeboot_size - raw_size)) bytes below 0xD0000"
  else
    echo "    safeboot: exceeds 0xD0000 by $((raw_size - safeboot_size)) bytes"
  fi
}

purge_gzip_artifacts() {
  find "$DIST_DIR" -maxdepth 1 -type f -name '*.gz' -delete
}

prune_old_dist_versions() {
  DIST_DIR="$DIST_DIR" KEEP_DIST_VERSIONS="$KEEP_DIST_VERSIONS" python3 - <<'PY'
from pathlib import Path
import os
import re

dist = Path(os.environ["DIST_DIR"])
keep_count = int(os.environ["KEEP_DIST_VERSIONS"])
pattern = re.compile(r"^mymota32-(\d+)\.(\d+)\.(\d+)-.+\.bin$")

versions = set()
for path in dist.glob("mymota32-*.bin"):
    match = pattern.match(path.name)
    if match:
        versions.add(tuple(int(part) for part in match.groups()))

keep = set(sorted(versions, reverse=True)[:keep_count])
removed = 0
for path in dist.glob("mymota32-*.bin"):
    match = pattern.match(path.name)
    if not match:
        continue
    version = tuple(int(part) for part in match.groups())
    if version not in keep:
        path.unlink()
        removed += 1

kept = ", ".join(".".join(str(part) for part in version) for version in sorted(keep, reverse=True))
if kept:
    print(f"==> Keeping dist versions: {kept}")
if removed:
    print(f"==> Removed {removed} old dist binary artifact(s)")
PY
}

refresh_checksums() {
  find dist -maxdepth 1 -type f -name '*.bin' -print | sort | xargs md5sum >dist/MD5SUMS
  find dist -maxdepth 1 -type f -name '*.bin' -print | sort | xargs sha256sum >dist/SHA256SUMS
}

need_cmd pio
need_cmd python3
need_cmd cmp
need_cmd od
need_cmd stat

assert_file platformio.ini
assert_file partitions.csv
assert_file "$BOOT_APP0"
mkdir -p "$DIST_DIR"

VERSION="$(validate_config_and_get_version)"
echo "==> Version $VERSION"

echo "==> Purging gzip artifacts from dist"
purge_gzip_artifacts

echo "==> Clearing PlatformIO build cache"
rm -rf "$ROOT_DIR/.cache"

build_target "mymota32-esp32-d0wd-v3-4m" "esp32-d0wd-v3-4m" "esp32" "ESP32" "40m" "0x1000" "ESP32-D0WD-V3"
build_target "mymota32-esp32-u4wdh-d-4m" "esp32-u4wdh-d-4m" "esp32" "ESP32" "40m" "0x1000" "ESP32-U4WDH-D"
build_target "mymota32-esp32-u4wdh-s-4m" "esp32-u4wdh-s-4m" "esp32" "ESP32" "40m" "0x1000" "ESP32-U4WDH-S"
build_target "mymota32-esp32-c3-4m" "esp32-c3-4m" "esp32c3" "ESP32-C3" "80m" "0x0000" "ESP32-C3 4M"

prune_old_dist_versions
refresh_checksums

echo "==> Refreshed dist/MD5SUMS and dist/SHA256SUMS"
echo "==> Use the raw .bin for Tasmota/web OTA; use -factory.bin only for serial write_flash at 0x0"
echo "==> Done"
