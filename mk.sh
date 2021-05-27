export CROSS_COMPILE=aarch64-himix100-linux-

make clean PLAT=hi3559av100 DEBUG=1
make distclean PLAT=hi3559av100 DEBUG=1
make PLAT=hi3559av100 SPD=none BL33=../../kernel/linux-4.9.y_multi-core/arch/arm64/boot/uImage CCI_UP=0 DEBUG=1 BL33_SEC=0 HISILICON=1 fip 
