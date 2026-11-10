#!/bin/sh
# Idempotently installs the hidraw access rule for the Universal RC Joystick.
#   sudo joystick/install-udev.sh
set -eu

rule_src="$(cd "$(dirname "$0")" && pwd)/99-gaz-hidraw-joystick.rules"
rule_dst=/etc/udev/rules.d/99-gaz-hidraw-joystick.rules

if [ "$(id -u)" -ne 0 ]; then
    echo "install-udev.sh: needs root, run it with sudo" >&2
    exit 1
fi

if cmp -s "$rule_src" "$rule_dst"; then
    echo "rule already installed: $rule_dst"
else
    install -m 0644 "$rule_src" "$rule_dst"
    echo "installed $rule_dst"
fi

udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=hidraw
echo "hidraw permissions refreshed; replug the joystick if it is still root-only"
