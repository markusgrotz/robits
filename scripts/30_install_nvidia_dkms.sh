#!/bin/bash -eux

export NVIDIA_DRIVER_VERSION=525

sudo apt install linux-headers-$(uname -r)
sudo apt install nvidia-dkms-${NVIDIA_DRIVER_VERSION}${SERVER}

NVIDIA_DKMS_VERSION=$(dpkg-query -W -f='${Version}' nvidia-dkms-${NVIDIA_DRIVER_VERSION})
NVIDIA_SRC_DIR="/usr/src/nvidia-${NVIDIA_DKMS_VERSION}"

if [ ! -f "${NVIDIA_SRC_DIR}/conftest.sh" ]; then
    echo "Unable to find DKMS src at ${NVIDIA_SRC_DIR}"
    exit 1
fi

cat <<EOF | sudo patch ${NVIDIA_SRC_DIR}/conftest.sh
--- a/conftest.sh
+++ b/conftest.sh
@@ -716,8 +716,9 @@
 if [ -n "\$IGNORE_PREEMPT_RT_PRESENCE" ]; then
-    exit 0
+    echo Skipping preempt RT check
 fi
EOF


dkms status

sudo dkms install -m nvidia/${NVIDIA_DKMS_VERSION} -j 12
sudo depmod -a
sudo modprobe nvidia