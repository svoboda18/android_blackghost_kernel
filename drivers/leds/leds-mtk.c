#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <kd_flashlight.h>
#include <mt_gpio.h>

#define __devexit_p(x) x
#define __devexit __section(.devexit.text) __exitused __cold notrace

/* Varaibles */
unsigned int flashlight_status = 0;

/* Functions */
static void set_flashlight(struct led_classdev *led_cdev,
        enum led_brightness value)
{
    flashlight_status = value;

    if (1 == value || 255 == value) // enable flashlight
    {
	mdelay(12);
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_HIGH);
    }
    else // disable flashlight
    {
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_LOW);
    }
}

static struct led_classdev mtk_flashlight_led = {
	.name			= "flashlight",
	.brightness_set		= set_flashlight,
	.brightness		= LED_OFF,
};

static int mtk_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &mtk_flashlight_led);
        if (rc) {
                dev_err(&pdev->dev, "unable to register led class driver\n");
                return rc;
        }
	return rc;
}
static int __devexit mtk_led_remove(struct platform_device *pdev) {
	led_classdev_unregister(&mtk_flashlight_led);
	return 0;
}

static struct platform_driver mtk_led_driver = {
	.probe		= mtk_led_probe,
	.remove		= __devexit_p(mtk_led_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
	.name		= "gpio-leds",
	.owner		= THIS_MODULE,
	},
};

static int __init mtk_led_init(void)
{
	return platform_driver_register(&mtk_led_driver);
}
module_init(mtk_led_init);

static void __exit mtk_led_exit(void)
{
	platform_driver_unregister(&mtk_led_driver);
}
module_exit(mtk_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("svoboda18");
MODULE_DESCRIPTION("MTK Flashlight Filesystem Driver");
