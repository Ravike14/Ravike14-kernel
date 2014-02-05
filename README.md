defconfig is "ravike14_defconfig" 

google aosp 4.6 toochain 
                   
for tiwlan compat-wirelesss source compilation

make -C drivers/net/wireless/compat-wireless_R5.SP5.01 KLIB=`pwd` KLIB_BUILD=`pwd` clean -j10

make -C drivers/net/wireless/compat-wireless_R5.SP5.01 KLIB=`pwd` KLIB_BUILD=`pwd` -j10

