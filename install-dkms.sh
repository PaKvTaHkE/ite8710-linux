#!/bin/sh
# Устанавливает драйвер через DKMS: он будет загружаться сам и пересобираться при обновлении ядра.
set -e
V=1.0
SRC=/usr/src/ite-cir-8305-$V
sudo apt install -y dkms "linux-headers-$(uname -r)"
sudo dkms remove -m ite-cir-8305 -v $V --all 2>/dev/null || true
sudo rm -rf "$SRC"
sudo mkdir -p "$SRC"
sudo cp ite-cir-8305.c ite-cir.h Kbuild Makefile dkms.conf "$SRC"/
sudo dkms add -m ite-cir-8305 -v $V
sudo dkms build -m ite-cir-8305 -v $V
sudo dkms install -m ite-cir-8305 -v $V
if [ -n "$POLL" ]; then
  echo "options ite_cir_8305 poll=1" | sudo tee /etc/modprobe.d/ite-cir-8305.conf
fi
sudo modprobe ite_cir_8305
echo "Готово. Проверка: sudo ir-keytable -t"
