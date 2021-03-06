#include <linux/module.h> /*this header file is for making modules*/
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/io.h>

MODULE_AUTHOR("Kensuke Saito");
MODULE_DESCRIPTION("driver for controlling LEDs");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static dev_t dev;
static struct cdev cdv;
static struct class *cls = NULL;
static volatile u32 *gpio_base = NULL;

static ssize_t led_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	char c;
	if (copy_from_user(&c, buf, sizeof(char)))
	{
		return -EFAULT;
	}
	if (c == '0')
	{
		gpio_base[10] = 1 << 17;
	}
	else if (c == '1')
	{
		gpio_base[7] = 1 << 17;
	}
	printk(KERN_INFO "gpio_base: %d\n", gpio_base[10]);
	//printk(KERN_INFO "recieved %c\n", c);
	return 1; // <- returns data output (if the return value is 0, it will continue to return data resulting in an infinite loop)
}

#if 0
static ssize_t sushi_read(struct file* filp, char* buf, size_t count, loff_t* pos)
{
    int size = 0;
     char sushi[] = {0xF0,0x9F,0x8D,0xA3,0x0A}; //寿司の絵文字のバイナリ
     if(copy_to_user(buf+size,(const char *)sushi, sizeof(sushi))){
        printk( KERN_INFO "sushi : copy_to_user failed\n" );
     return -EFAULT;
     }
     size += sizeof(sushi);
    return size;
}
#endif

static struct file_operations led_fops = {
	.owner = THIS_MODULE,
	.write = led_write
	//.read = sushi_read
};

/* initialize kernel module */
static int __init init_mod(void)
{
	int retval; //return value
	gpio_base = ioremap_nocache(0x3f200000, 0xA0);

	const u32 led = 17;
	const u32 index = led/10;
	const u32 shift = (led%10)*3;
	const u32 mask = ~(0x7 << shift);

	gpio_base[index] = (gpio_base[index] & mask) | (0x1 << shift);

	retval = alloc_chrdev_region(&dev, 0, 1, "myled17");
	if (retval < 0)
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval;
	}
	printk(KERN_INFO "%s is loaded. major: %d\n", __FILE__, MAJOR(dev));
	cdev_init(&cdv, &led_fops);
	retval = cdev_add(&cdv, dev, 1);
	if (retval < 0)
	{
		printk(KERN_ERR "cdev_add failed. major: %d, minor: %d\n", MAJOR(dev), MINOR(dev));
		return retval;
	}
	cls = class_create(THIS_MODULE, "myled17");
	if (IS_ERR(cls))
	{
		printk(KERN_ERR "class_create failed.");
		return PTR_ERR(cls);
	}
	device_create(cls, NULL, dev, NULL, "myled17");
	return 0;
}

/* cleanup */
static void __exit cleanup_mod(void)
{
	cdev_del(&cdv);
	device_destroy(cls, dev);
	class_destroy(cls);
	unregister_chrdev_region(dev, 1);
	printk(KERN_INFO "%s is unloaded. major: %d\n", __FILE__, MAJOR(dev));
}

module_init(init_mod); //register function in macro
module_exit(cleanup_mod); //same as above
