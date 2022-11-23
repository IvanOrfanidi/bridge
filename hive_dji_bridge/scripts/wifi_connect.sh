#!/bin/bash

# If value is 0, then check is disabled.
WIFI_SIGNAL_LEVEL_FOR_CONNECTION="70"
WIFI_SIGNAL_LEVEL_FOR_DISCONNECTION="30"

. /opt/ros/$(ls /opt/ros)/setup.bash

. /home/dji/.drone_setup

echo "Wi-Fi signal level for connection: $WIFI_SIGNAL_LEVEL_FOR_CONNECTION"
echo "Wi-Fi signal level for disconnection: $WIFI_SIGNAL_LEVEL_FOR_DISCONNECTION"

route_old=""
log_old=""

while true; do
  nmcli device wifi rescan

  connected=$(nmcli -t -f active,ssid dev wifi | egrep '^yes' | cut -d ':' -f2)
  success=false
  log_new=""

  wifi_list=$(cat $(rospack find drone)/config/wifi.cfg)
  for ssid in $wifi_list; do
    id=$(echo "$ssid"|cut -d ':' -f 1)
    pass=$(echo "$ssid"|cut -d ':' -f 2)
    signal=$(nmcli -t -f active,ssid,signal dev wifi | egrep "$id" | cut -d ':' -f3)
    if [ "$signal" == "" ]; then
      continue
    fi
    log_new+="    Check ssid $id.\n"

    if [ "$id" = "$connected" ]; then
      if [ "$WIFI_SIGNAL_LEVEL_FOR_DISCONNECTION" -ne 0 ] && (("$signal" < "$WIFI_SIGNAL_LEVEL_FOR_DISCONNECTION")); then
        nmcli c down $id
      else
        log_new+="    Network $id already connected. Signal level $signal%.\n"
        success=true
        break
      fi
    fi

    if nmcli d wifi list|grep "$id"; then
      log_new+="    Found network $id with high priority whan current connected. Try reconnect.\n";
      for i in $(nmcli c|grep "$id"|grep -Eo '[0-9a-f\-]{36}'); do nmcli c delete $i; done
      if [ "$WIFI_SIGNAL_LEVEL_FOR_CONNECTION" -eq 0 ] || (("$signal" > "$WIFI_SIGNAL_LEVEL_FOR_CONNECTION")); then
        nmcli dev wifi connect "$id" password "$pass"
        connected=$(nmcli -t -f active,ssid dev wifi | egrep '^yes' | cut -d ':' -f2)
        if [ "$id" = "$connected" ]; then
          log_new+="    Successfully connected $id. Signal level $signal%.\n"
          success=true
          break
        else
          log_new+="    Failed connect $id. Try another network.\n"
        fi
      else
        log_new+="    Weak wifi signal. Signal level $signal%.\n"
      fi
    fi
  done

  if [ $success == "false" ]; then
    log_new+="    Can't connect any network.\n"
  fi

  if [ "$log_old" != "$log_new" ]; then
    log_old="${log_new}"
    echo -ne $(date "+%Y.%m.%d_%H:%M:%S: \n")$log_new
  fi

  route_new=$(route -n)
  if [ "$route_old" != "$route_new" ]; then
    route_old="${route_new}"
    route -n
  fi

  sleep 1
done
