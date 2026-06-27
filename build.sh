#!/bin/bash
# build.sh — M5PaperDashboard 固件编译 & 烧录(M5Stack M5Paper)
#
# 用法:
#   ./build.sh                    # 只编译
#   ./build.sh -f                 # 编译 + 烧录
#   ./build.sh -w                 # 只烧录已有 .bin(不编译)
#   ./build.sh -w --bin <file>    # 烧录指定 .bin(不编译)
#
# 烧录目标设备写死为下面的 M5PAPER_DEV(by-id 路径,稳定不随枚举变),不接受端口
# 参数,从根上避免误刷到 ttyACM0/ttyUSB* 上的其它板子(本机 ttyACM0 是另一台
# AtomS3R)。多带的端口参数会被忽略。要刷到别的 M5Paper,用环境变量覆盖:
#   M5PAPER_DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2104..._01XXXX-if00-port0 ./build.sh -f
#
# Flash-only(-w)不重编译——把已编译好的 .bin 直接写进设备。默认 bin 为
# ./dashboard.bin(仅 app 分区 @0x10000,用于在分区方案不变时重刷同一分区)。
# 若所选 .bin 同目录下还有整套镜像(bootloader.bin + partitions.bin +
# boot_app0.bin),-w 会改写整套——完整恢复。
#
# 板型说明:
#   M5Paper = 经典 ESP32(非 S3):ESP32-D0WDQ6-V3 / 16MB Flash(DIO)/ 8MB PSRAM,
#   USB 经 CP2104 转串口(/dev/ttyUSB*)。FQBN 用 m5stack:esp32:m5stack_paper。
#   ★ 经典 ESP32 的 bootloader 烧录偏移是 0x1000(不是 S3 的 0x0),整套镜像
#     的 4 个偏移见下方 WRITE_ARGS。

set -e

# 工具链路径可用环境变量覆盖(默认值适配标准 Arduino 1.8.19 安装)
ARDUINO="${ARDUINO:-$HOME/Downloads/arduino-1.8.19/arduino}"
ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/m5stack/tools/esptool_py/4.5.1/esptool.py}"

# 仓库内路径相对脚本所在目录,便于他人 clone 后直接用
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$SCRIPT_DIR/dashboard.ino"

FQBN="m5stack:esp32:m5stack_paper:\
PSRAM=enabled,\
FlashMode=dio,\
FlashFreq=80,\
FlashSize=16M,\
PartitionScheme=default,\
CPUFreq=240,\
UploadSpeed=921600,\
DebugLevel=none,\
EraseFlash=none"

BUILD_DIR="/tmp/dashboard_build"
OUT_DIR="$SCRIPT_DIR"
BIN_NAME="dashboard.bin"

FLASH_PORT=""
DO_BUILD=1        # -w 关掉它:跳过编译,直接刷已有 .bin
BIN_OVERRIDE=""   # --bin 指定要刷的 .bin(默认 $OUT_DIR/$BIN_NAME)

# M5Paper 稳定设备 ID(CP2104 桥,序列号固定 → by-id 路径不随枚举顺序变)。
# 烧录目标写死为它:-f/-w 都只刷这一台,避免误刷到 ttyACM0(AtomS3R)等其它设备。
M5PAPER_DEV="${M5PAPER_DEV:-/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_01FF7F5F-if00-port0}"

# ── 解析参数 ────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f)
            FLASH_PORT="$M5PAPER_DEV"; shift 1     # 目标写死,不取端口参数
            ;;
        -w)
            DO_BUILD=0                              # flash-only:跳过编译,直接刷已有 .bin
            FLASH_PORT="$M5PAPER_DEV"; shift 1     # 目标写死,不取端口参数
            ;;
        --bin)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "ERROR: --bin needs a file path"; exit 1
            fi
            BIN_OVERRIDE="$2"; shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [-f] | [-w [--bin <file>]]"
            echo "  (no args)        只编译"
            echo "  -f               编译后烧录(整套镜像)到写死的 M5Paper"
            echo "  -w               只刷已有 ./dashboard.bin(不编译,app @0x10000)"
            echo "  -w --bin <file>  刷指定 .bin(不编译);若其同目录有整套镜像则完整恢复"
            echo "  注:烧录目标设备已写死,不接受端口参数(可用 M5PAPER_DEV 环境变量覆盖)"
            exit 0
            ;;
        *)
            # 设备 ID 写死后端口参数已无意义:误带的端口(不以 - 开头)直接忽略;
            # 但拼错的选项(以 - 开头)仍报错,免得静默刷错 bin。
            if [[ "$1" == -* ]]; then
                echo "Unknown option: $1"
                echo "Run '$0 --help' for usage."
                exit 1
            fi
            echo "Note: 忽略多余参数 '$1'(烧录设备已写死,无需端口)"
            shift 1
            ;;
    esac
done

mkdir -p "$BUILD_DIR"

# ── 编译(-w 时跳过)─────────────────────────────────────────────────────────
if [[ $DO_BUILD -eq 1 ]]; then
    echo "=== Building M5PaperDashboard firmware ==="
    echo "    FQBN: $FQBN"
    echo "    Sketch: $SKETCH"

    "$ARDUINO" \
        --board "$FQBN" \
        --pref "build.path=$BUILD_DIR" \
        --verify \
        "$SKETCH"

    # ── 拷出产物 ──────────────────────────────────────────────────────────────
    BIN_SRC="$BUILD_DIR/dashboard.ino.bin"
    if [[ ! -f "$BIN_SRC" ]]; then
        echo "ERROR: expected binary not found: $BIN_SRC"
        exit 1
    fi

    cp "$BIN_SRC" "$OUT_DIR/$BIN_NAME"
    SIZE=$(stat -c%s "$OUT_DIR/$BIN_NAME")
    echo "=== Build done: $OUT_DIR/$BIN_NAME  (${SIZE} bytes) ==="
fi

# ── 烧录(可选)────────────────────────────────────────────────────────────
if [[ -n "$FLASH_PORT" ]]; then
    if [[ ! -e "$FLASH_PORT" ]]; then
        echo "ERROR: 目标设备不存在: $FLASH_PORT"
        echo "       (M5Paper 没插好?或换了 USB 口换了序列号?用 ls /dev/serial/by-id/ 查看)"
        exit 1
    fi

    # 要刷的 app .bin:--bin 优先,否则仓库根目录的 dashboard.bin
    APP_BIN="${BIN_OVERRIDE:-$OUT_DIR/$BIN_NAME}"
    if [[ ! -f "$APP_BIN" ]]; then
        echo "ERROR: app binary not found: $APP_BIN"
        exit 1
    fi

    # 找配套的 bootloader/分区/boot_app0:
    #   -f(刚编译):用 /tmp 构建产物
    #   -w(不编译):找 app .bin 同目录的整套镜像
    BOOT_APP="$HOME/.arduino15/packages/m5stack/hardware/esp32/2.1.4/tools/partitions/boot_app0.bin"
    BOOTLOADER=""; PARTITIONS=""
    if [[ $DO_BUILD -eq 1 ]]; then
        BOOTLOADER="$BUILD_DIR/dashboard.ino.bootloader.bin"
        PARTITIONS="$BUILD_DIR/dashboard.ino.partitions.bin"
    else
        BIN_DIR="$(cd "$(dirname "$APP_BIN")" && pwd)"
        if [[ -f "$BIN_DIR/bootloader.bin" && -f "$BIN_DIR/partitions.bin" ]]; then
            BOOTLOADER="$BIN_DIR/bootloader.bin"
            PARTITIONS="$BIN_DIR/partitions.bin"
            [[ -f "$BIN_DIR/boot_app0.bin" ]] && BOOT_APP="$BIN_DIR/boot_app0.bin"
        fi
    fi

    # 有整套镜像就完整刷;否则只覆盖 app 分区(分区方案不变时够用)
    # ★ 经典 ESP32:bootloader @ 0x1000(S3 是 0x0)
    if [[ -n "$BOOTLOADER" && -f "$BOOTLOADER" && -n "$PARTITIONS" && -f "$PARTITIONS" && -f "$BOOT_APP" ]]; then
        echo ""
        echo "=== Flashing FULL image to $FLASH_PORT ==="
        echo "    app: $APP_BIN"
        WRITE_ARGS=(
            0x1000  "$BOOTLOADER"
            0x8000  "$PARTITIONS"
            0xe000  "$BOOT_APP"
            0x10000 "$APP_BIN"
        )
    else
        echo ""
        echo "=== Flashing APP ONLY to $FLASH_PORT ==="
        echo "    app: $APP_BIN  (@0x10000,假定分区方案未变)"
        WRITE_ARGS=( 0x10000 "$APP_BIN" )
    fi

    python3 "$ESPTOOL" \
        --chip    esp32 \
        --port    "$FLASH_PORT" \
        --baud    921600 \
        --before  default_reset \
        --after   hard_reset \
        write_flash \
        -z \
        --flash_mode  dio \
        --flash_freq  80m \
        --flash_size  16MB \
        "${WRITE_ARGS[@]}"

    echo "=== Flash done ==="
fi
