#!/usr/bin/env bash
# Installs the ch341 (CH340/CH341 USB-serial) kernel module via DKMS.
#
# Why this is needed: NVIDIA's L4T/tegra kernel ships with
# CONFIG_USB_SERIAL_CH341 compiled out, and there is no
# linux-modules-extra-$(uname -r) package for tegra kernels (that mechanism
# only exists for stock Ubuntu kernel flavors like generic/aws/azure/gcp).
# So instead of installing a prebuilt module, this builds the mainline
# driver from source against the kernel headers already on the system,
# matched to the running kernel's version, and registers it with DKMS so
# it survives kernel updates.
#
# Safe to re-run: skips work that's already done.
set -euo pipefail

KVER="$(uname -r)"
KMAJMIN="$(echo "$KVER" | grep -oP '^\d+\.\d+')"

if lsmod | grep -q '^ch341'; then
  echo "ch341 already loaded."
else
  if [ ! -e "/lib/modules/$KVER/build" ]; then
    echo "No matching kernel headers for $KVER (expected /lib/modules/$KVER/build to exist)." >&2
    echo "Install the headers package for this exact kernel first." >&2
    exit 1
  fi

  # dkms itself: the DKMS *tool* (dkms.conf/dkms.dbdir logic), an apt package
  # (~150-250KB, arch=all). Installs the `dkms` CLI to /usr/sbin/dkms and its
  # tracking DB to /var/lib/dkms - not the ch341 module itself, just the
  # machinery that will build/track it below.
  if ! dpkg -s dkms >/dev/null 2>&1; then
    sudo apt-get install -y dkms
  fi

  if dkms status 2>/dev/null | grep -q '^ch341/1\.0'; then
    echo "ch341/1.0 already registered with dkms."
  else
    # Staging dir: just the driver source + build recipe, deleted once
    # `dkms add` below has copied what it needs out of it.
    BUILD_DIR="$(mktemp -d /tmp/ch341-dkms.XXXXXX)"
    trap 'rm -rf "$BUILD_DIR"' EXIT

    # The mainline (upstream Linux) ch341 driver source, fetched from
    # kernel.org and matched to this machine's running kernel version -
    # this is the actual driver code; nothing else on this system provides
    # it (see the header comment: CONFIG_USB_SERIAL_CH341 is compiled out).
    curl -fsSL "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial/ch341.c?h=v${KMAJMIN}" \
      -o "$BUILD_DIR/ch341.c"

    cat > "$BUILD_DIR/Makefile" <<'EOF'
obj-m += ch341.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
EOF

    cat > "$BUILD_DIR/dkms.conf" <<'EOF'
PACKAGE_NAME="ch341"
PACKAGE_VERSION="1.0"
BUILT_MODULE_NAME[0]="ch341"
DEST_MODULE_LOCATION[0]="/kernel/drivers/usb/serial"
AUTOINSTALL="yes"
EOF

    # `dkms add` copies $BUILD_DIR (ch341.c + Makefile + dkms.conf) into
    # /usr/src/ch341-1.0/ - DKMS's permanent copy of the driver source,
    # kept there so it can rebuild against future kernel updates.
    sudo dkms add "$BUILD_DIR"
  fi

  if dkms status 2>/dev/null | grep "^ch341/1\.0" | grep -q "$KVER"; then
    echo "ch341 already built+installed for $KVER."
  else
    # `dkms build` compiles /usr/src/ch341-1.0/ against this kernel's
    # headers, output kept under /var/lib/dkms/ch341/1.0/$KVER/.
    sudo dkms build ch341/1.0
    # `dkms install` copies the built ch341.ko into the kernel's own
    # module tree at /lib/modules/$KVER/updates/dkms/ch341.ko and runs
    # depmod, so it's alongside the system's other modules and will
    # auto-load via udev whenever a CH340/CH341 device is plugged in -
    # not just this one time.
    sudo dkms install ch341/1.0
  fi

  # Loads the now-installed ch341.ko into the running kernel immediately,
  # so it doesn't need a replug/reboot to take effect on this run.
  sudo modprobe ch341
fi

echo "---ttyUSB devices---"
ls -la /dev/ttyUSB* 2>/dev/null || echo "(none yet - plug/replug the USB-serial adapter)"

# Without this, the device node udev creates is root:dialout mode 660,
# and every fresh plug/reboot recreates it - meaning the invoking user
# would need `dialout` membership (which only takes effect on their next
# login, not retroactively) or a manual `chmod` every single time. This
# udev rule makes CH340/CH341 adapters world-rw permanently: written once
# to /etc/udev/rules.d/, it's reapplied by udev on every future plug/
# reboot with no relogin, replug-then-fix, or manual chmod ever needed
# again. Matches on the CH340's USB vendor:product id (1a86:7523, from
# `lsusb`), not a specific /dev/ttyUSB* number, so it survives enumeration
# order changing too. Also adds a stable /dev/esp32maker symlink to the
# same device (alongside, not instead of, the usual /dev/ttyUSB*), so
# scripts/platformio.ini can target a fixed name instead of whichever
# ttyUSBN number this happens to enumerate as.
UDEV_RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="esp32maker"'
UDEV_RULE_FILE=/etc/udev/rules.d/99-ch340-usbserial.rules
if [ "$(cat "$UDEV_RULE_FILE" 2>/dev/null)" != "$UDEV_RULE" ]; then
  echo "$UDEV_RULE" | sudo tee "$UDEV_RULE_FILE" >/dev/null
  sudo udevadm control --reload-rules
fi
# Re-applies rules to any adapter that's already plugged in right now,
# so this run's device doesn't need a physical replug to pick up the fix.
# (--attr-match only checks the tty device's own attributes, not the
# parent USB device's idVendor/idProduct the way a rule's ATTRS{} does,
# so it silently matches nothing here - a plain trigger is what actually
# re-evaluates all rules against already-connected devices.)
sudo udevadm trigger

# Belt-and-suspenders: dialout membership is still worth having (covers
# any other serial gear that isn't covered by the rule above). Doesn't
# help this session (needs a relogin), but costs nothing to set now.
if ! id -nG "$USER" | grep -qx dialout; then
  echo "Adding $USER to the dialout group (takes effect on next login)."
  sudo usermod -aG dialout "$USER"
fi

echo "---ttyUSB devices after udev fix---"
ls -la /dev/ttyUSB* 2>/dev/null || echo "(none yet - plug/replug the USB-serial adapter)"
