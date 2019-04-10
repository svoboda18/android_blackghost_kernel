# BlakGhost Kernel
This BlackGHost custom kernel (3.18.138) source made to DOOGEE X5 (MT6580).

## Known information:
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `hct_hx8394f_dsi_vdo_hd_cmi` | Yes | Yes |
| LCM driver2 | `hct_rm68200_dsi_vdo_hd_cpt` | Yes | Yes |
| Touch panel | `FT5X05 (i2c 1-0038)` | Yes | Yes |
| GPU | `Mali-400 MP` | Yes | Yes |
| Camera #1 | `s5k5e2y_mipi_raw` | Yes | Yes |
| Camera #2 | `gc2355_mipi_raw` | Yes | Yes |
| Accelerometer | `MXC400X (i2c 2-0015)` | Yes | Yes |
| ALS/PS | `EPL2182 (i2c 2-0049)` | Yes | Yes |
| Flash | `Samsung FNX2MB` | Yes | Yes |
| Lens | `DW9714AF` | Yes | Yes |
| RAM | `1 GB LPDDR3_1066` | - | Yes |
| Sound | `mtsndcard` | - | Yes |
| Accdet | `mt6580-accdet` | - | Yes |
| Other | `kd_camera_hw (i2c 0-0036)` | - | Yes |

## Current kernel features:
* Add d2w gestuge
* Overclock CPU To 1630MHz (Disabled)
* Underclock CPU To 260MHz.
* Added 20+ CPU Governors. (smartmax,smartassV2,dancedance,pegasusq,ondmand_x,ondemandPlus,Blu_active,elementalX,nightmare,zzmoove,intelliactive,impulse,HYPER,barry_alien,darkness,alucard,thunderX,Lionheart,bioshock),
* Added 6+ i/0 Scheduler. (bfq,fifo,fiops,sio,sioplus,zen), zen As Default.
* Added Fast Charging Support (For USB 2.0-3.0)
* Tweaked Mali For More Faster Rendering.
* MTK Dynamic Boost.
* Touch Boost. 
* Fixed Kernel Behavior After Sleep (onedemand, conservative). 
* Removed IO BlockPlugs for Better Performance. 
* Added Alucard Hotplug. 
* Added Thunder Charge Control.
* Added Dynamic Fsync 1.5. 
* Added Power Suspend V1.8.1.
* Added WireGuard Support.
* Compiled with Linaro 6. 
* Many Performance Changes. (crc and other checks are disabled) 
* Network Tweaks, Responsive Settings. 
* Lock Wakelock for Some Services. 

## Build process:
* Clone thos repo, by running:
`git clone https://github.com/svoboda18/android_blackghost_kernel kernel`
* Start the build:
`cd kernel ; bash Build-BlackGhostKernel`
* Then type `b` to **build** the kerbel
  - Output kernel will be in zip named with date of the build.

## Acknowledgements:
* [snowcat (4pda.ru)](https://4pda.ru/forum/index.php?showuser=188334) [(@SnowCat6)](https://github.com/SnowCat6)
* [nik-kst (4pda.ru)](https://4pda.ru/forum/index.php?showuser=4052130) [(@nik124seleznev)](https://github.com/nik124seleznev)

