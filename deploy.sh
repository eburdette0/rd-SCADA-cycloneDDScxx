#!/bin/bash
DEST_IP="$1"
DEBUG_PORT="$2"
BINARY="$3"
BINDIR="$4"
RIGCONFIG="$5"
RIGCONFIGDEST="$6"
DEST_DIR="/usr/bin"

# kill gdbserver on tx8m and delete old binary
ssh root@${DEST_IP} "sh -c '/usr/bin/killall -q gdbserver; rm -rf ${DEST_DIR}/${BINARY}  exit 0'"

# send binary to target
scp ${BINDIR}/${BINARY} root@${DEST_IP}:${DEST_DIR}/${BINARY}
scp ${RIGCONFIG} root@${DEST_IP}:${RIGCONFIGDEST}/RigConfig.json

# start gdbserver on target
ssh -t root@${DEST_IP} "sh -c 'cd ${DEST_DIR}; gdbserver localhost:${DEBUG_PORT} ${BINARY} ${RIGCONFIGDEST}/RigConfig.json'"
