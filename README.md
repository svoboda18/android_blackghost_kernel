# BlakGhost Kernel
This BlackGHost custom kernel (4.9.x) source made to DOOGEE X5 (MT6580).

## Known information:
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `hct_hx8394f_dsi_vdo_hd_cmi` | Yes | Yes |
| LCM driver #2 | `hct_rm68200_dsi_vdo_hd_cpt` | Yes | Yes |
| LCM driver #3 | `hct_otm1282_dsi_vdo_hd_auo` | Yes | - |
| LCM driver #4 | `hct_ili9881_dsi_vdo_hd_cpt` | Yes | No |
| LCM driver #5 | `hct_otm1285a_dsi_vdo_hd_boe` | Yes | - |
| LCM driver #6 | `hct_hx8394d_dsi_vdo_hd_cmi` | Yes | - |
| LCM driver #7 | `hct_nt35521s_dsi_vdo_hd_boe_50_xld` | Yes | - |
| Touch panel | `fts_ts (FT5X05) (i2c 1-0038)` | Yes | Yes |
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
* Upstream to Linux v4.9.120
* Underclock to 260MHz
* Schedutil as default governor
* Rebased with new `fts_ts` Touch Driver

## Build process:
* Clone that repo, by running:
`git clone https://github.com/svoboda18/android_blackghost_kernel -b pie kernel`
* Start the build:
`cd kernel ; bash Build-BlackGhostKernel`
* Then type `b` to **build** the kernel
  - Output kernel will be in zip named with date of the build.

## Acknowledgements:
* [SnowCatPDA (4pda.ru)](https://4pda.ru/forum/index.php?showuser=334271) [(@SnowCat6)](https://github.com/SnowCat6)
