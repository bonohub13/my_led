#include <linux/module.h> /*this header file is for making modules*/
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/io.h>

MODULE_AUTHOR("Kensuke Saito");
MODULE_DESCRIPTION("driver for controlling LEDs");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static struct class *cls17 = NULL,
					*cls23 = NULL,
					*cls25 = NULL;
static dev_t dev17, dev23, dev25;
static struct cdev cdv17, cdv23, cdv25;
static volatile u32 *gpio17_base = NULL, *gpio23_base = NULL, *gpio25_base = NULL;

static ssize_t led17_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	char c;

	if (copy_from_user(&c, buf, sizeof(char)))
		return -EFAULT;
	
	if (c == '0')
		gpio17_base[10] = 1 << 17;
	else if (c == '1')
		gpio17_base[7] = 1 << 17;

	return 1;
}

static ssize_t led23_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	char c;

	if (copy_from_user(&c, buf, sizeof(char)))
		return -EFAULT;
	
	if (c == '0')
		gpio23_base[10] = 1 << 23;
	else if (c == '1')
		gpio23_base[7] = 1 << 23;

	return 1;
}

static ssize_t led25_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	char c;

	if (copy_from_user(&c, buf, sizeof(char)))
		return -EFAULT;
	
	if (c == '0')
		gpio25_base[10] = 1 << 25;
	else if (c == '1')
		gpio25_base[7] = 1 << 25;

	return 1;
}

static struct file_operations led17_fops = {
	.owner = THIS_MODULE, 
	.write = led17_write
};

static struct file_operations led23_fops = {
	.owner = THIS_MODULE, 
	.write = led23_write
};

static struct file_operations led25_fops = {
	.owner = THIS_MODULE, 
	.write = led25_write
};

static int __init init_mod(void)
{
	int retval17, retval23, retval25;

	gpio17_base = ioremap_nocache(0x3f200000, 0xA0);
	gpio23_base = ioremap_nocache(0x3f200000, 0xA0);
	gpio25_base = ioremap_nocache(0x3f200000, 0xA0);

	const u32 led17 = 17,
		  	  led23 = 23,
			  led25 = 25;
	const u32 index17 = led17/10,
		  	  index23 = led23/10,
			  index25 = led25/10;
	const u32 shift17 = (led17%10)*3,
		  	  shift23 = (led23%10)*3,
			  shift25 = (led25%10)*3;
	const u32 mask17 = ~(0x7 << shift17),
		  	  mask23 = ~(0x7 << shift23),
			  mask25 = ~(0x7 << shift25);

	gpio17_base[index17] = (gpio17_base[index17] & mask17) | (0x1 << shift17);
	gpio23_base[index23] = (gpio23_base[index23] & mask23) | (0x1 << shift23);
	gpio25_base[index25] = (gpio25_base[index25] & mask25) | (0x1 << shift25);
	
	retval17 = alloc_chrdev_region(&dev17, 0, 1, "myled17");
	retval23 = alloc_chrdev_region(&dev23, 0, 1, "myled23");
	retval25 = alloc_chrdev_region(&dev25, 0, 1, "myled25");

	if (retval17 < 0)
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval17;
	}
	printk(KERN_INFO "%s is loaded. major: %d\n", __FILE__, MAJOR(dev17));
	
	if (retval23 < 0)
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval23;
	}
	printk(KERN_INFO "%s is loaded. major: %d\n", __FILE__, MAJOR(dev23));
	
	if (retval25 < 0)
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n");
		return retval25;
	}
	printk(KERN_INFO "%s is loaded. major: %d\n", __FILE__, MAJOR(dev25));
	
	cdev_init(&cdv17, &led17_fops);
	retval17 = cdev_add(&cdv17, dev17, 1);
	cdev_init(&cdv23, &led23_fops);
	retval23 = cdev_add(&cdv23, dev23, 1);
	cdev_init(&cdv23, &led25_fops);
	retval23 = cdev_add(&cdv23, dev23, 1);
	
	if(retval17 < 0)
	{
		printk(KERN_ERR "cdev_add failed. major: %d, minor: %d", MAJOR(dev17), MINOR(dev17));
		return retval17;
	}
	
	if(retval23 < 0)
	{
		printk(KERN_ERR "cdev_add failed. major: %d, minor: %d", MAJOR(dev23), MINOR(dev23));
		return retval23;
	}
	
	if(retval25 < 0)
	{
		printk(KERN_ERR "cdev_add failed. major: %d, minor: %d", MAJOR(dev25), MINOR(dev25));
		return retval25;
	}
	
	cls17 = class_create(THIS_MODULE, "myled17");
	if (IS_ERR(cls17))
	{
		printk(KERN_ERR "class_create failed.");
		return PTR_ERR(cls17);
	}
	device_create(cls17, NULL, dev17, NULL, "myled17");
	
	cls23 = class_create(THIS_MODULE, "myled23");
	if (IS_ERR(cls23))
	{
		printk(KERN_ERR "class_create failed.");
		return PTR_ERR(cls23);
	}
	device_create(cls23, NULL, dev23, NULL, "myled23");

	cls25 = class_create(THIS_MODULE, "myled25");
	if (IS_ERR(cls25))
	{
		printk(KERN_ERR "class_create failed.");
		return PTR_ERR(cls25);
	}
	device_create(cls25, NULL, dev25, NULL, "myled25");

	return 0;
}

static void __exit cleanup_mod(void)
{
	cdev_del(&cdv17);
	cdev_del(&cdv23);
	cdev_del(&cdv25);
	device_destroy(cls17, dev17);
	device_destroy(cls23, dev23);
	device_destroy(cls25, dev25);
	class_destroy(cls17);
	class_destroy(cls23);
	class_destroy(cls25);
	unregister_chrdev_region(dev17, 1);
	unregister_chrdev_region(dev23, 1);
	unregister_chrdev_region(dev25, 1);
	printk(KERN_INFO "%s is unloaded. major: %d\n", __FILE__, MAJOR(dev17));
}

module_init(init_mod);
module_exit(cleanup_mod);
