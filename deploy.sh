#!/bin/bash

PWD="3mdebTUX"
USER="tuxedo14passwd123"
IP="192.168.4.126"
TEST_DIR="/home/tuxedo14passwd123/Desktop/"
FLASHROM_CMD="time ./flashrom -p tuxec:romsize=128K -r ec-read.rom -V | tee ./tuxec-read.log"

sshpass -p "$PWD" scp ./flashrom $USER@$IP:$TEST_DIR
sshpass -p "$PWD" ssh $USER@$IP "cd $TEST_DIR && echo $PWD | sudo -S $FLASHROM_CMD"
