export ROOT=`pwd`
export KOBJ=`pwd`
export CROSS_COMPILE=arm-eabi-
export ARCH=arm
declare SW_DIR="drivers/net/wireless/compat-wireless_R5.SP7.01/"
cd drivers/net/wireless/compat-wireless_R5.SP7.01/
make KLIB=${KOBJ} KLIB_BUILD=${KOBJ} clean -j10
make KLIB=${KOBJ} KLIB_BUILD=${KOBJ} -j10
cd ${ROOT}
mkdir -p $ROOT/MODULES_SP7
cp $ROOT/arch/arm/mach-msm/msm-buspm-dev.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/net/kineto_gan.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/scsi/scsi_wait_scan.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/spi/spidev.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/staging/ti-st/fm_drv.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/hid/hid-magicmouse.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/usb/class/cdc-acm.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/usb/serial/option.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/usb/serial/usb_wwan.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/bluetooth/btwilink.ko $ROOT/MODULES_SP7/.
cp $ROOT/drivers/bluetooth/hci_uart.ko $ROOT/MODULES_SP7/.
cp $ROOT/net/bluetooth/rfcomm/rfcomm.ko $ROOT/MODULES_SP7/.
cp $ROOT/net/bluetooth/bluetooth.ko $ROOT/MODULES_SP7/.
cp $ROOT/net/bluetooth/bnep/bnep.ko $ROOT/MODULES_SP7/.
cp $ROOT/net/bluetooth/hidp/hidp.ko $ROOT/MODULES_SP7/.
cp $SW_DIR/compat/compat.ko $ROOT/MODULES_SP7/.
cp $SW_DIR/drivers/net/wireless/wl12xx/wl12xx.ko $ROOT/MODULES_SP7/.
cp $SW_DIR/drivers/net/wireless/wl12xx/wl12xx_sdio.ko $ROOT/MODULES_SP7/.
cp $SW_DIR/net/mac80211/mac80211.ko $ROOT/MODULES_SP7/.
cp $SW_DIR/net/wireless/cfg80211.ko $ROOT/MODULES_SP7/.

#
# since i've implanted more updated and working compat-wireless BT drivers 
# to the kernel itself no need for the compat-wireless source ones
#

#cp $SW_DIR/drivers/bluetooth/btwilink.ko $ROOT/MODULES_SP7/.
#cp $SW_DIR/drivers/bluetooth/hci_uart.ko $ROOT/MODULES_SP7/.
#cp $SW_DIR/net/bluetooth/rfcomm/rfcomm.ko $ROOT/MODULES_SP7/.
#cp $SW_DIR/net/bluetooth/bluetooth.ko $ROOT/MODULES_SP7/.
#cp $SW_DIR/net/bluetooth/bnep/bnep.ko $ROOT/MODULES_SP7/.
#cp $SW_DIR/net/bluetooth/hidp/hidp.ko $ROOT/MODULES_SP7/.



