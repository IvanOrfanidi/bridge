#!/bin/bash

. /home/dji/.drone_setup

DRONE_SERVER="${DRONE_SERVER:-drone-test.aniklab.com}"
SSH_SERVER="${SSH_SERVER:-${DRONE_SERVER}}"

while true; do
  SSH_REMOTE_PORT=$(cat $HOME/.drone/ssh_tunnel.conf | tr -dc '0-9')
  if [ "$SSH_REMOTE_PORT" = "" ]; then
    echo "failed to get remote port out of the config file"
    echo "using default port 1121"
    SSH_REMOTE_PORT=1121
    break
  else
    echo "got remote port $SSH_REMOTE_PORT"
    break
  fi
  sleep 2
done

. /opt/ros/$(ls /opt/ros)/setup.bash

ssh -N -o StrictHostKeyChecking=no -i $(rospack find drone_payload)/config/id_rsa root@${SSH_SERVER} \
  -o ServerAliveInterval=5 -o ServerAliveCountMax=1 -o TCPKeepAlive=yes -o GlobalKnownHostsFile=/dev/null -o UserKnownHostsFile=/dev/null \
  -p 2222 -R ${SSH_REMOTE_PORT}:localhost:22
