# BlakGhost Kernel
This is 3.18.x MT6580 costum blackghost kernel source made to DOOGEE X5.

## Known information
| Subsystem | Driver name | Availability | Working |
|-----------|-------------|--------------|---------|
| LCM driver | `hct_hx8394f_dsi_vdo_hd_cmi` | Yes | Yes |
| Touch panel | `FT5X05 (i2c 1-0038)` | Yes | Yes |
| GPU | `Mali-400 MP` | Yes | Yes |
| Camera #1 | `s5k5e2y_mipi_raw` | Yes | Yes |
| Camera #2 | `gc2355_mipi_raw` | Yes | Yes |
| Accelerometer | `MXC400X (i2c 2-0015)` | Yes | Yes |
| ALS/PS | `EPL2182 (i2c 2-0049)` | Yes | Yes |
| Flash | `Samsung FNX2MB` | Yes | Yes |
| Lens #1 | `DW9714AF` | Yes | No |
| RAM | `1 GB LPDDR3_1066` | - | Yes |
| Sound | `mtsndcard` | - | Yes |
| Accdet | `mt6580-accdet` | - | Yes |
| Other | `kd_camera_hw (i2c 0-0036)` | - | Yes |

## Build process
* Clone thos repo, by running:
`git clone https://github.com/svoboda18/android_blackghost_kernel kernel`
* Start the build:
`cd kernel ; bash Build-BlackGhostKernel`

## Acknowledgements
* [snowcat (4pda.ru)](https://4pda.ru/forum/index.php?showuser=188334) [(@SnowCat6)](https://github.com/SnowCat6)
* [nik-kst (4pda.ru)](https://4pda.ru/forum/index.php?showuser=4052130) [(@nik124seleznev)](https://github.com/nik124seleznev)

