
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/include/$(CONFIG_GT615_FIRMWARE)/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/include/config1/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/include/$(CONFIG_GT615_CONFIG)/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/include/firmware1/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  gt9xx_driver.o
obj-y	+=  gt9xx_extents.o
obj-y	+=  gt9xx_update.o
obj-y	+=  goodix_tool.o

