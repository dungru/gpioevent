#include <linux/module.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>

#include <asm/io.h>
#include <asm/atomic.h>

#define DRV_NAME    "gpio-event-listener"

enum {
    GPIOEVENT_MAJOR = 233,
    GPIOEVENT_MINOR_BASE = 0,
};

#define SYSCFG_CHIPSIG_OFFSET 0x174
#define SYSCFG_CHIPSIG_CLR_OFFSET 0x178
#define AINTC_SICR_OFFSET 0x24

#define GPIOEVENT_WAIT _IO('T', 0x10)

static const dev_t dev = MKDEV(GPIOEVENT_MAJOR, GPIOEVENT_MINOR_BASE);
static struct cdev gpioevent_cdev;
static struct class *gpioevent_class;

u16 gpio_irq;
u16 irq_pin = EVENT_PIN;

static atomic_t user_flag = ATOMIC_INIT(1);

static struct task_struct *wait_task = NULL;

static int setup_pinmux(void){
// taken from
// https://github.com/bootc/linux/blob/rpi-i2cspi/drivers/spi/spi-bcm2708.c

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= \
        (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

   u32* gpio = ioremap(0x3F000000 + 0x200000, SZ_16K);

   if (NULL == gpio)
      return -EBUSY;

   // irq pin, GPIO 23 (pin P1/16)
   SET_GPIO_ALT((int)irq_pin, 0);
   INP_GPIO((int)irq_pin);

   iounmap(gpio);

#undef INP_GPIO
#undef OUT_GPIO
#undef SET_GPIO_ALT

   return 0;
}

static long
gpioevent_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if (cmd != GPIOEVENT_WAIT)
        return -EFAULT;

    /* yield as possible */
    while (!atomic_dec_and_test(&user_flag))
        schedule();

    wait_task = current;
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule();
    __set_current_state(TASK_RUNNING);
    local_irq_enable();
    atomic_set(&user_flag, 1);

    return 0;
}

static irqreturn_t gpioevent_handler(int irq, void *dev_info)
{
    if (wait_task != NULL) {
        local_irq_disable();
        wake_up_process(wait_task);
    }

    return IRQ_HANDLED;
}

struct file_operations gpioevent_fops = {
    .unlocked_ioctl = gpioevent_ioctl,
};

static int __init gpioevent_init(void)
{
    int ret;

    ret = register_chrdev_region(dev, 1, "gpioevent");
    if (ret)
        goto error;

    cdev_init(&gpioevent_cdev, &gpioevent_fops);
    ret = cdev_add(&gpioevent_cdev, dev, 1);
    if (ret)
        goto error_out_cdev;

    gpioevent_class = class_create(THIS_MODULE, "gpioevent");
    if (IS_ERR(gpioevent_class)) {
        printk(KERN_ERR "Error creating gpioevent class.\n");
        ret = PTR_ERR(gpioevent_class);
        goto error_out_class;
    }

    device_create(gpioevent_class, NULL, MKDEV(GPIOEVENT_MAJOR, 0), NULL, "gpioevent");

    ret = setup_pinmux();
    if (ret < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to apply pinmux settings.\n");
      goto error;
    }

    ret = gpio_request_one(irq_pin, GPIOF_IN, DRV_NAME " irq");
    if (ret < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to request IRQ pin %d.\n", irq_pin);
      goto err_free_gpio_return;
    }

    ret = gpio_to_irq(irq_pin);
    if(ret < 0){
      printk(KERN_ALERT DRV_NAME " : failed to get IRQ for pin %d.\n", irq_pin);
      goto err_free_irq_return;
    }else{
      gpio_irq = (u16)ret;
      ret = 0;
    }

    ret = request_any_context_irq(gpio_irq, gpioevent_handler,
                                    IRQF_TRIGGER_FALLING | IRQF_NO_THREAD,
                                    DRV_NAME, &gpioevent_cdev);

    if (ret < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to enable IRQ %d for pin %d.\n", gpio_irq, irq_pin);
      goto err_free_irq_return;
    }

    printk(KERN_INFO DRV_NAME " : GPIO Event Listener inserted. ioctl=%d\n", GPIOEVENT_WAIT);

    return 0;

err_free_irq_return:
    free_irq(gpio_irq, &gpioevent_cdev);
err_free_gpio_return:
    gpio_free(irq_pin);
error_out_class:
    cdev_del(&gpioevent_cdev);
error_out_cdev:
    unregister_chrdev_region(dev, 1);
error:
    return ret;
}

static void __exit gpioevent_exit(void)
{
    free_irq(gpio_irq, &gpioevent_cdev);
    gpio_free(irq_pin);

    device_destroy(gpioevent_class, MKDEV(GPIOEVENT_MAJOR, 0));
    class_destroy(gpioevent_class);
    cdev_del(&gpioevent_cdev);
    unregister_chrdev_region(MKDEV(GPIOEVENT_MAJOR, 0), 1);
}

MODULE_LICENSE("GPL");
module_init(gpioevent_init);
module_exit(gpioevent_exit);
