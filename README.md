# BlakGhost Kernel
This BlackGHost (YET) custom kernel (4.9.x) source made for DOOGEE X5 (MT6580)

*Note:* This is the best kernel ever made for this device as **almost** everything
is reversed from stock kernel.

## Known information:
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `hct_hx8394f_dsi_vdo_hd_cmi` | Yes | Yes |
| LCM driver #2 | `hct_rm68200_dsi_vdo_hd_cpt` | - | - |
| LCM driver #3 | `hct_otm1282_dsi_vdo_hd_auo` | - | - |
| LCM driver #4 | `hct_ili9881_dsi_vdo_hd_cpt` | - | - |
| LCM driver #5 | `hct_otm1285a_dsi_vdo_hd_boe` | - | - |
| LCM driver #6 | `hct_hx8394d_dsi_vdo_hd_cmi` | - | - |
| LCM driver #7 | `hct_nt35521s_dsi_vdo_hd_boe_50_xld` | - | - |
| Touch panel | `fts_ts (FT5X05) (i2c 1-0038)`[^1] | - | - |
| GPU | `Mali-400 MP2` | Yes | Yes |
| Camera #1 | `s5k5e2y_mipi_raw` | Yes | Yes |
| Camera #2 | `gc2355_mipi_raw` | Yes | Yes |
| Flashlight | `dummy_gpio` | Yes | Yes - Camera only |
| Accelerometer | `MXC400X (i2c 2-0015)` | Yes | Yes |
| ALS/PS | `EPL2182 (i2c 2-0049)` | Yes | Yes |
| Flash | `Samsung FNX2MB` | - | Yes |
| Lens | `DW9714AF` | Yes | Yes |
| RAM | `1 GB LPDDR3_1066` | - | Yes |
| Sound | `mtsndcard` | Yes | Yes |
| Accdet | `mt6580-accdet` | - | Yes |
| Other | `kd_camera_hw (i2c 0-0036)` | Yes | Yes |

[^1]: driver rebased to `svbfts_ts`, which is a modified version with *working* gestures of `fts_ts`.
## Current kernel features:
* Underclock to 260MHz.

## Build process:
* Clone that repo, by running:
`git clone https://github.com/svoboda18/android_blackghost_kernel -b quack-new kernel`
* Grab `Build-BlackGhostKernel` from `pie` branch
* Adapte defconfig to `X5_defconfig` in the build script.
* Start the build:
`cd kernel ; bash Build-BlackGhostKernel`
* Then type `b` to **build** the kernel
  - Output kernel will be in zip named with date of the build.

## Acknowledgements:
* [SnowCatPDA (4pda.ru)](https://4pda.ru/forum/index.php?showuser=334271) [(@SnowCat6)](https://github.com/SnowCat6)
