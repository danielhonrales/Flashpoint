#!/usr/bin/env bash
# Installs pi-receiver.service on this Pi and (re)starts it. Run on the Pi after pulling:
#   bash install-receiver.sh Player1        (Player2 on the other Pi)
# The name is what the Quests' Select Pi menu shows for this Pi. The installed service runs as
# the user running this script, from this folder; the repo's pi-receiver.service is left as it
# is, so pulls don't conflict. Run it again after pulling changes, or to rename the Pi.
set -euo pipefail

name="${1:-}"
if [[ ! "$name" =~ ^[A-Za-z0-9_-]{1,24}$ ]]; then
    echo "Usage: bash install-receiver.sh <name>   e.g. Player1 or Player2" >&2
    echo "The name is 1 to 24 letters, digits, '-' or '_'." >&2
    exit 1
fi

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
user="${SUDO_USER:-$(id -un)}"
unit=/etc/systemd/system/pi-receiver.service

# This Pi's user, folder and name; any other ExecStart options (e.g. --device) are kept.
sed -e "s|^User=.*|User=$user|" \
    -e "s|^WorkingDirectory=.*|WorkingDirectory=$here|" \
    -e "/^ExecStart=/{s/ --name [^ ]*//; s/\$/ --name $name/}" \
    "$here/pi-receiver.service" | sudo tee "$unit" > /dev/null
echo "Installed $unit: user $user, folder $here, name $name"

since="$(date '+%Y-%m-%d %H:%M:%S')"
sudo systemctl daemon-reload
sudo systemctl enable --quiet pi-receiver
sudo systemctl restart pi-receiver

# Loading the stimuli takes a few seconds before it starts announcing.
for _ in $(seq 20); do
    sleep 1
    log="$(sudo journalctl -u pi-receiver --since "$since" -o cat --no-pager)"
    if grep -q "announcing \"THERMAL-PI-1:$name\"" <<< "$log"; then
        echo "Running, and announcing itself to the Quests as $name."
        echo "Its log: journalctl -u pi-receiver -f"
        exit 0
    fi
    if ! systemctl is-active --quiet pi-receiver; then break; fi
done

echo "pi-receiver hasn't started announcing. Its log since the restart:" >&2
sudo journalctl -u pi-receiver --since "$since" -o cat --no-pager | tail -n 20 >&2
exit 1
