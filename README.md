# BlakGhost Kernel
This BlackGHost custom kernel (3.18.140) source made to DOOGEE X5 (MT6580).

## Known information:
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `hct_hx8394f_dsi_vdo_hd_cmi` | Yes | Yes |
| LCM driver #2 | `hct_rm68200_dsi_vdo_hd_cpt` | Yes | Yes |
| LCM driver #3 | `hct_otm1282_dsi_vdo_hd_auo` | Yes | Yes |
| LCM driver #4 | `hct_ili9881_dsi_vdo_hd_cpt` | Yes | Yes |
| LCM driver #5 | `hct_otm1285a_dsi_vdo_hd_boe` | Yes | Yes |
| Touch panel | `FT5X05 (i2c 1-0038)` | Yes | Yes |
| GPU | `Mali-400 MP2` | Yes | Yes |
| Camera #1 | `s5k5e2y_mipi_raw` | Yes | Yes |
| Camera #2 | `gc2355_mipi_raw` | Yes | Yes |
| Accelerometer | `MXC400X (i2c 2-0015)` | Yes | Yes |
| ALS/PS | `EPL2182 (i2c 2-0049)` | Yes | Yes |
| Flash | `Samsung FNX2MB` | - | Yes |
| Lens | `DW9714AF` | Yes | Yes |
| RAM | `1 GB LPDDR3_1066` | - | Yes |
| Sound | `mtsndcard` | Yes | Yes |
| Accdet | `mt6580-accdet` | - | Yes |
| Other | `kd_camera_hw (i2c 0-0036)` | Yes | Yes |

## Current kernel features:
* Overclock CPU To 1500MHz (Disabled).
* Underclock CPU To 260MHz.
* Added 17+ CPU Governors. (smartmax,dancedance,pegasusq,ondmand_x,ondemandPlus,Blu_active,elementalX,nightmare,zzmoove,intelliactive,impulse,HYPER,bioshock,darkness,thunderX,Lionheart,bioshock),bioshock As Default.
* Added 6+ i/0 Scheduler. (bfq,fifo,fiops,sio,sioplus,zen), zen As Default.
* Added 13+ TCP Scheduler. (reno,bic,westwood,hispeed,hybla,htcp,vegas,veno,scalable,lp,yeah,illinols,cubic), westwood As Default.
* Optimized SLUB.
* Added KSM.
* Added Frandom.
* Added crc check control. 
* Added DT2W Gesture.
* Added THUNDERQUAKE ENGINE.
* Added Fast Charging Support. (Upto 1.5AM)
* Tweaked Mali For More Faster Rendering.
* MTK Dynamic Boost.
* Touch Boost.
* Added Color control.
* Removed io blockplugs for better performance. 
* Added Alucard Hotplug. 
* Added AuroSmp Hotplug.
* Added Thunder Charge Control.
* Added Dynamic Fsync 1.5.
* Added Power Suspend V1.8.1.
* Added Wireguard Support.
* Compiled with linaro 6. 
* Many Performance Changes. (crc and other checks are disabled) 
* Network tweaks, Responsive settings. 
* Lock wakelock for some services. 

## Build process:
* Clone that repo, by running:
`git clone https://github.com/svoboda18/android_blackghost_kernel kernel`
* Start the build:
`cd kernel ; bash Build-BlackGhostKernel`
* Then type `b` to **build** the kernel
  - Output kernel will be in zip named with date of the build.

## Acknowledgements:
* [andrey167 (4pda.ru)](https://4pda.ru/forum/index.php?showuser=6516960) [(@andrey167)](https://github.com/andrey167)
* [SnowCatPDA (4pda.ru)](https://4pda.ru/forum/index.php?showuser=334271) [(@SnowCat6)](https://github.com/SnowCat6)
* [nik-kst (4pda.ru)](https://4pda.ru/forum/index.php?showuser=4052130) [(@nik124seleznev)](https://github.com/nik124seleznev)

