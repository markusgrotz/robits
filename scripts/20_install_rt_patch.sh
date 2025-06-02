#!/bin/bash -eux

echo """
This script installs a Linux kernel with the PREEMPT-RT patch.

References:

 - https://chenna.me/blog/2020/02/23/how-to-setup-preempt-rt-on-ubuntu-18-04/
 - https://wiki.archlinux.org/title/Realtime_kernel_patchset
 - https://wiki.linuxfoundation.org/realtime/preempt_rt_versions

PREEMPT_RT was merged in Linux 6.12 https://www.phoronix.com/news/Linux-6.12-Does-Real-Time
"""

# kernel version without -generic flag
KERNEL_VERSION=`uname -r | sed 's/-generic//g'`

#if [ "${KERNEL_VERSION}" != "5.15.0-41" ]; then
#    echo "Script was written for a different kernel version"
#    exit 1
#fi

TEMP_DIR=$(mktemp -d -p $HOME rt_patch_XXXXXXXXXX)
cd $TEMP_DIR


# install dependencies
sudo apt install build-essential git libssl-dev libelf-dev flex bison 
sudo apt install bc curl ca-certificates gnupg2 libssl-dev lsb-release bison flex dwarves zstd libncurses-dev
sudo apt install cpufrequtils rt-tests

# testing
sudo cyclictest --smp -p98 -m -l 10000 -q | tee `date -I`_test_`uname -r`.log

# download source files
curl -SLO https://www.kernel.org/pub/linux/kernel/v6.x/linux-6.1.134.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v6.x/linux-6.1.134.tar.sign
curl -SLO https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.1/patch-6.1.134-rt51.patch.sign
curl -SLO https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.1/patch-6.1.134-rt51.patch.xz

xz -d *.xz

gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6092693E
gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 2872E4CC

gpg2 --verify linux-*.tar.sign
gpg2 --verify patch-*.patch.sign


tar xf linux-*.tar
cd linux-*/
patch -p1 < ../patch-*.patch

#sed -i -e 's/CONFIG_SYSTEM_TRUSTED_KEYS="debian/canonical-certs.pem"/CONFIG_SYSTEM_TRUSTED_KEYS=""/g' .config

# configure to enable PREEMPT_RT
scripts/config --enable CONFIG_PREEMPT_RT
scripts/config --enable CONFIG_PREEMPT
scripts/config --disable CONFIG_PREEMPT_NONE
scripts/config --disable CONFIG_PREEMPT_VOLUNTARY
scripts/config --set-str CONFIG_SYSTEM_TRUSTED_KEYS ""

yes "" | make oldconfig

# we don't need this step but this will show a TUI to the user
make menuconfig

make -j12

# https://stackoverflow.com/questions/56149191/linux-latest-stable-compilation-cannot-represent-change-to-vmlinux-gdb-py
rm vmlinux-gdb.py

make deb-pkg -j12

dpkg -i  ../linux-*.deb

# configure system
if [ ! -f /etc/security/limits.d/99-realtime.conf ]; then

sudo groupadd realtime
sudo usermod -aG realtime $USER

sudo tee /etc/security/limits.d/99-realtime.conf <<EOF
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF

# disable frequency scaling
sudo systemctl disable ondemand
sudo systemctl enable cpufrequtils
echo "GOVERNOR=performance"  | sudo tee /etc/default/cpufrequtils
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils

else
  echo "File /etc/security/limits.d/99-realtime.conf already exists. Skipping real-time configuration"
fi



echo """
Installation complete. Please reboot your system.

Please run
cyclictest --smp -p98 -m | tee \$(date -I)_test_\$(uname -r).log
"""