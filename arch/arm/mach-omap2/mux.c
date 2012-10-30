/*
 * linux/arch/arm/mach-omap2/mux.c
 *
 * OMAP2, OMAP3 and OMAP4 pin multiplexing configurations
 *
 * Copyright (C) 2004 - 2010 Texas Instruments Inc.
 * Copyright (C) 2003 - 2008 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <asm/system.h>

#include <plat/omap_hwmod.h>

#include "control.h"
#include "mux.h"

#define OMAP_MUX_BASE_OFFSET		0x30	/* Offset from CTRL_BASE */
#define OMAP_MUX_BASE_SZ		0x5ca

// omap mux的链表
struct omap_mux_entry {
	struct omap_mux		mux; // mux结构体  omap_mux结构体
	struct list_head	node;// 链表
};

static LIST_HEAD(mux_partitions);  // 初始化链表mux_partitions
static DEFINE_MUTEX(muxmode_mutex);

struct omap_mux_partition *omap_mux_get(const char *name)
{
	struct omap_mux_partition *partition;

	// 循环包含head类型链表mux_partitions的结构体partition
	// 其中mux_partitions类型的head在partition中的成员名称是
	// node，这个是一个for循环
	// 注意查看结构体类型omap_mux_partition *partition中有一
	// 个成员struct list_head node
	// 链表是 mux_partitions
	list_for_each_entry(partition, &mux_partitions, node) {
		// 比较名字是否一样，如果一样则返回结构体omap_mux_partition
		if (!strcmp(name, partition->name))
			return partition;
	}
	// 否则返回null
	return NULL;
}

u16 omap_mux_read(struct omap_mux_partition *partition, u16 reg)
{
	// 根据位数  根据寄存器的地址读取不同的地址
	if (partition->flags & OMAP_MUX_REG_8BIT)
		return __raw_readb(partition->base + reg);
	else
		return __raw_readw(partition->base + reg);
}

void omap_mux_write(struct omap_mux_partition *partition, u16 val,
			   u16 reg)
{
	if (partition->flags & OMAP_MUX_REG_8BIT)
		__raw_writeb(val, partition->base + reg);
	else
		__raw_writew(val, partition->base + reg);
}

void omap_mux_write_array(struct omap_mux_partition *partition,
				 struct omap_board_mux *board_mux)
{
	while (board_mux->reg_offset != OMAP_MUX_TERMINATOR) {
		omap_mux_write(partition, board_mux->value,
			       board_mux->reg_offset);
		board_mux++;
	}
}

#ifdef CONFIG_OMAP_MUX

static char *omap_mux_options;

// mux初始化gpio
// 输入参数omap_mux_partition   mux的结构体
// 输入参数gpio  是gpio号
// 输入参数val  是要设定的值
static int __init _omap_mux_init_gpio(struct omap_mux_partition *partition,
				      int gpio, int val)
{
	struct omap_mux_entry *e;
	struct omap_mux *gpio_mux = NULL;  // 用于获取要设定io口的mux结构体
	u16 old_mode;
	u16 mux_mode;
	int found = 0;
	// mux mode  根据partition->muxmodes这个链表
	struct list_head *muxmodes = &partition->muxmodes;

	// 如果gpio==0 则返回
	if (!gpio)
		return -EINVAL;

	// 循环 根据链表muxmodes来获取包含它的结构体omap_mux_entry
	list_for_each_entry(e, muxmodes, node) {
		struct omap_mux *m = &e->mux;  // 获取omap_mux结构体
		if (gpio == m->gpio) {  // 比较io口是否匹配
			gpio_mux = m;
			found++;  // 找到一个，自增
		}
	}

	// 如果没有找到，则报错
	if (found == 0) {
		pr_err("%s: Could not set gpio%i\n", __func__, gpio);
		return -ENODEV;
	}

	// 找到多个，也报错
	if (found > 1) {
		pr_info("%s: Multiple gpio paths (%d) for gpio%i\n", __func__,
			found, gpio);
		return -EINVAL;
	}

	// 到这里表示已经找到了这个gpio对应的mux，存在指向omap_mux指针gpio_mux中
	// 注意这里返回的是整个IO的寄存器，8bit或者16bit，因此不会是两个IO的寄存器
	old_mode = omap_mux_read(partition, gpio_mux->reg_offset);
	mux_mode = val & ~(OMAP_MUX_NR_MODES - 1);// 设定模式，最多8种模式 0-7
	if (partition->flags & OMAP_MUX_GPIO_IN_MODE3)
		mux_mode |= OMAP_MUX_MODE3;
	else
		mux_mode |= OMAP_MUX_MODE4;// 一般是mode4表示gpio口的

	// 打印输出gpio的设定  
	// gpio_mux->muxnames[0]表示第一功能
	// gpio表示gpio号
	// old_mode表示旧的模式  mux_mode 新的模式
	pr_debug("%s: Setting signal %s.gpio%i 0x%04x -> 0x%04x\n", __func__,
		 gpio_mux->muxnames[0], gpio, old_mode, mux_mode);
	// 写入寄存器
	omap_mux_write(partition, mux_mode, gpio_mux->reg_offset);

	// 如果配置成功则返回零
	return 0;
}

// 配置为gpio功能，这里的val不应该在在bit2 bit1 bit0有数
// 因此val应该是高位的标志而已，因为这个函数固定就是设定引脚
// mux为gpio功能的

// 注意  一个板子就一个 omap_mux_partition
// 这个omap_mux_partition中包含了所有io的mux功能和设置方法
// 因此需要轮询omap_mux_partition中的muxmodes链表
int __init omap_mux_init_gpio(int gpio, int val)
{
	struct omap_mux_partition *partition;
	int ret;

	// 循环包含链表mux_partitions的结构体omap_mux_partition
	list_for_each_entry(partition, &mux_partitions, node) {
		// 如果初始化失败则返回非零
		ret = _omap_mux_init_gpio(partition, gpio, val);
		if (!ret)
			return ret;
	}

	return -ENODEV;
}

static int __init _omap_mux_get_by_name(struct omap_mux_partition *partition,
					const char *muxname,   // mux名称
					struct omap_mux **found_mux)
{
	struct omap_mux *mux = NULL;
	struct omap_mux_entry *e;
	const char *mode_name;
	int found = 0, found_mode, mode0_len = 0;
	struct list_head *muxmodes = &partition->muxmodes;

	mode_name = strchr(muxname, '.');  // 返回muxname中第一次出现.的问题
	if (mode_name) {   // 如果有"."这个字符
		mode0_len = strlen(muxname) - strlen(mode_name);// mode0的长度
		mode_name++;// 去除"."之前的字符
	} else {
		mode_name = muxname;  
	}
	// 到这里，就是留下mode的功能，去除mode0的字符串

	// 轮询 partition中的muxmodes链表
	list_for_each_entry(e, muxmodes, node) {
		char *m0_entry;
		int i;

		mux = &e->mux;
		m0_entry = mux->muxnames[0];  // 获取第一功能

		/* First check for full name in mode0.muxmode format */
		// 如果是按照方式 mode0.mode3来做的话，需要确认mode0是否一致
		// 如果不一致则continue
		if (mode0_len && strncmp(muxname, m0_entry, mode0_len))   
			continue;

		/* Then check for muxmode only */
		// 检查8个mode功能
		for (i = 0; i < OMAP_MUX_NR_MODES; i++) {
			char *mode_cur = mux->muxnames[i];

			if (!mode_cur)  // null
				continue;

			if (!strcmp(mode_name, mode_cur)) {
				*found_mux = mux;
				found++;
				found_mode = i;   // 找到了，获取mode的数
			}
		}
	}

	if (found == 1) {  // 返回找到的mode号码
		return found_mode;
	}

	if (found > 1) {   // 多个则返回错误，因此有些功能需要写mode0.mode2这种方式
		pr_err("%s: Multiple signal paths (%i) for %s\n", __func__,
		       found, muxname);
		return -EINVAL;
	}
	// 返回错误
	pr_err("%s: Could not find signal %s\n", __func__, muxname);

	return -ENODEV;
}

// 根据名字获取mux
static int __init
omap_mux_get_by_name(const char *muxname,
			struct omap_mux_partition **found_partition,
			struct omap_mux **found_mux)
{
	struct omap_mux_partition *partition;

	// 轮询包含muxmodes 的partition结构体
	// mux_partitions是一个muxmodes的链表
	// 这里是获取包含muxmodes的结构体，这个结构体是partition，而且muxmodes
	// 在partition中的成员名是node
	// 根据实际测试应该就是一个
	list_for_each_entry(partition, &mux_partitions, node) {
		struct omap_mux *mux = NULL;
		// 返回mode的号码
		int mux_mode = _omap_mux_get_by_name(partition, muxname, &mux);
		if (mux_mode < 0)   // 继续找
			continue;

		*found_partition = partition;  // 找到mode对应的partition
		*found_mux = mux;  // 获取mux

		return mux_mode;
	}

	return -ENODEV;
}

// mux初始化信号
// 这里的val可以是0-7 用于设定模式，也可以或上需要的上拉下拉
int __init omap_mux_init_signal(const char *muxname, int val)
{
	struct omap_mux_partition *partition = NULL;
	struct omap_mux *mux = NULL;
	u16 old_mode;
	int mux_mode;

	// 获取mode的号码到mux_mode中，并且获取mux结构体
	mux_mode = omap_mux_get_by_name(muxname, &partition, &mux);
	if (mux_mode < 0)  
		return mux_mode;

	// 获取old的值
	old_mode = omap_mux_read(partition, mux->reg_offset);
	// 这里的val可以是0-7
	mux_mode |= val;
	pr_debug("%s: Setting signal %s 0x%04x -> 0x%04x\n",
			 __func__, muxname, old_mode, mux_mode);
	// 写新的值
	omap_mux_write(partition, mux_mode, mux->reg_offset);

	return 0;
}

// hwmod  mux 的初始化
// 注意参数bpads很关键，是外设指定功能引脚配置的数组
// 参数nr_pads表示bpads的个数
// 注意该函数中没有设定具体的io配置，仅仅是初始化
struct omap_hwmod_mux_info * __init
omap_hwmod_mux_init(struct omap_device_pad *bpads, int nr_pads)
{
	struct omap_hwmod_mux_info *hmux;
	int i;

	if (!bpads || nr_pads < 1)
		return NULL;

	// 分配omap_hwmod_mux_info类型  omap_hwmod_mux_info中包含了好几个omap_device_pad
	// 到hmux
	hmux = kzalloc(sizeof(struct omap_hwmod_mux_info), GFP_KERNEL);
	if (!hmux)
		goto err1;

	hmux->nr_pads = nr_pads;

	// 分配omap_device_pad类型  设备制定pad的配置
	// 到hmux中的pads数组中
	hmux->pads = kzalloc(sizeof(struct omap_device_pad) *
				nr_pads, GFP_KERNEL);
	if (!hmux->pads)
		goto err2;

	for (i = 0; i < hmux->nr_pads; i++) {
		struct omap_mux_partition *partition;
		struct omap_device_pad *bpad = &bpads[i], *pad = &hmux->pads[i];
		struct omap_mux *mux;
		int mux_mode;

		// 根据字符型名称获取mux的mode，以及mux结构体
		mux_mode = omap_mux_get_by_name(bpad->name, &partition, &mux);
		if (mux_mode < 0)
			goto err3;
		if (!pad->partition)
			pad->partition = partition;
		if (!pad->mux)
			pad->mux = mux;

		// 分配pad
		pad->name = kzalloc(strlen(bpad->name) + 1, GFP_KERNEL);
		if (!pad->name) {
			int j;

			for (j = i - 1; j >= 0; j--)
				kfree(hmux->pads[j].name);
			goto err3;
		}
		strcpy(pad->name, bpad->name);

		pad->flags = bpad->flags;  // 从bpad中复制过来
		pad->enable = bpad->enable;
		pad->idle = bpad->idle;
		pad->off = bpad->off;
		pr_debug("%s: Initialized %s\n", __func__, pad->name);
	}

	return hmux;

err3:
	kfree(hmux->pads);
err2:
	kfree(hmux);
err1:
	pr_err("%s: Could not allocate device mux entry\n", __func__);

	return NULL;
}

/* Assumes the calling function takes care of locking */
// hwmod mux 设定
void omap_hwmod_mux(struct omap_hwmod_mux_info *hmux, u8 state)
{
	int i;

	// 轮询omap_hwmod_mux_info中的pads
	for (i = 0; i < hmux->nr_pads; i++) {
		struct omap_device_pad *pad = &hmux->pads[i];
		int flags, val = -EINVAL;

		flags = pad->flags;

		switch (state) {
		case _HWMOD_STATE_ENABLED:
			if (flags & OMAP_DEVICE_PAD_ENABLED)
				break;
			flags |= OMAP_DEVICE_PAD_ENABLED;
			val = pad->enable;  // 设定引脚为enable的时候的功能
			pr_debug("%s: Enabling %s %x\n", __func__,
					pad->name, val);
			break;
		case _HWMOD_STATE_IDLE:
			if (!(flags & OMAP_DEVICE_PAD_REMUX))
				break;
			flags &= ~OMAP_DEVICE_PAD_ENABLED;
			val = pad->idle; // 设定引脚为idle的时候的功能
			pr_debug("%s: Idling %s %x\n", __func__,
					pad->name, val);
			break;
		case _HWMOD_STATE_DISABLED:
		default:
			/* Use safe mode unless OMAP_DEVICE_PAD_REMUX */
			if (flags & OMAP_DEVICE_PAD_REMUX)
				val = pad->off;   // 设定引脚为disable的时候的功能
			else
				val = OMAP_MUX_MODE7;
			flags &= ~OMAP_DEVICE_PAD_ENABLED;
			pr_debug("%s: Disabling %s %x\n", __func__,
					pad->name, val);
		};

		// 写入引脚功能设定
		if (val >= 0) {
			omap_mux_write(pad->partition, val,
					pad->mux->reg_offset);
			pad->flags = flags;
		}
	}
}

#ifdef CONFIG_DEBUG_FS

#define OMAP_MUX_MAX_NR_FLAGS	10
#define OMAP_MUX_TEST_FLAG(val, mask)				\
	if (((val) & (mask)) == (mask)) {			\
		i++;						\
		flags[i] =  #mask;				\
	}

/* REVISIT: Add checking for non-optimal mux settings */
static inline void omap_mux_decode(struct seq_file *s, u16 val)
{
	char *flags[OMAP_MUX_MAX_NR_FLAGS];
	char mode[sizeof("OMAP_MUX_MODE") + 1];
	int i = -1;

	sprintf(mode, "OMAP_MUX_MODE%d", val & 0x7);
	i++;
	flags[i] = mode;

	OMAP_MUX_TEST_FLAG(val, OMAP_PIN_OFF_WAKEUPENABLE);
	if (val & OMAP_OFF_EN) {
		if (!(val & OMAP_OFFOUT_EN)) {
			if (!(val & OMAP_OFF_PULL_UP)) {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_OFF_INPUT_PULLDOWN);
			} else {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_OFF_INPUT_PULLUP);
			}
		} else {
			if (!(val & OMAP_OFFOUT_VAL)) {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_OFF_OUTPUT_LOW);
			} else {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_OFF_OUTPUT_HIGH);
			}
		}
	}

	if (val & OMAP_INPUT_EN) {
		if (val & OMAP_PULL_ENA) {
			if (!(val & OMAP_PULL_UP)) {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_INPUT_PULLDOWN);
			} else {
				OMAP_MUX_TEST_FLAG(val, OMAP_PIN_INPUT_PULLUP);
			}
		} else {
			OMAP_MUX_TEST_FLAG(val, OMAP_PIN_INPUT);
		}
	} else {
		i++;
		flags[i] = "OMAP_PIN_OUTPUT";
	}

	do {
		seq_printf(s, "%s", flags[i]);
		if (i > 0)
			seq_printf(s, " | ");
	} while (i-- > 0);
}

#define OMAP_MUX_DEFNAME_LEN	32

static int omap_mux_dbg_board_show(struct seq_file *s, void *unused)
{
	struct omap_mux_partition *partition = s->private;
	struct omap_mux_entry *e;
	u8 omap_gen = omap_rev() >> 28;

	list_for_each_entry(e, &partition->muxmodes, node) {
		struct omap_mux *m = &e->mux;
		char m0_def[OMAP_MUX_DEFNAME_LEN];
		char *m0_name = m->muxnames[0];
		u16 val;
		int i, mode;

		if (!m0_name)
			continue;

		/* REVISIT: Needs to be updated if mode0 names get longer */
		for (i = 0; i < OMAP_MUX_DEFNAME_LEN; i++) {
			if (m0_name[i] == '\0') {
				m0_def[i] = m0_name[i];
				break;
			}
			m0_def[i] = toupper(m0_name[i]);
		}
		val = omap_mux_read(partition, m->reg_offset);
		mode = val & OMAP_MUX_MODE7;
		if (mode != 0)
			seq_printf(s, "/* %s */\n", m->muxnames[mode]);

		/*
		 * XXX: Might be revisited to support differences accross
		 * same OMAP generation.
		 */
		seq_printf(s, "OMAP%d_MUX(%s, ", omap_gen, m0_def);
		omap_mux_decode(s, val);
		seq_printf(s, "),\n");
	}

	return 0;
}

static int omap_mux_dbg_board_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_mux_dbg_board_show, inode->i_private);
}

static const struct file_operations omap_mux_dbg_board_fops = {
	.open		= omap_mux_dbg_board_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct omap_mux_partition *omap_mux_get_partition(struct omap_mux *mux)
{
	struct omap_mux_partition *partition;

	list_for_each_entry(partition, &mux_partitions, node) {
		struct list_head *muxmodes = &partition->muxmodes;
		struct omap_mux_entry *e;

		list_for_each_entry(e, muxmodes, node) {
			struct omap_mux *m = &e->mux;

			if (m == mux)
				return partition;
		}
	}

	return NULL;
}

static int omap_mux_dbg_signal_show(struct seq_file *s, void *unused)
{
	struct omap_mux *m = s->private;
	struct omap_mux_partition *partition;
	const char *none = "NA";
	u16 val;
	int mode;

	partition = omap_mux_get_partition(m);
	if (!partition)
		return 0;

	val = omap_mux_read(partition, m->reg_offset);
	mode = val & OMAP_MUX_MODE7;

	seq_printf(s, "name: %s.%s (0x%08x/0x%03x = 0x%04x), b %s, t %s\n",
			m->muxnames[0], m->muxnames[mode],
			partition->phys + m->reg_offset, m->reg_offset, val,
			m->balls[0] ? m->balls[0] : none,
			m->balls[1] ? m->balls[1] : none);
	seq_printf(s, "mode: ");
	omap_mux_decode(s, val);
	seq_printf(s, "\n");
	seq_printf(s, "signals: %s | %s | %s | %s | %s | %s | %s | %s\n",
			m->muxnames[0] ? m->muxnames[0] : none,
			m->muxnames[1] ? m->muxnames[1] : none,
			m->muxnames[2] ? m->muxnames[2] : none,
			m->muxnames[3] ? m->muxnames[3] : none,
			m->muxnames[4] ? m->muxnames[4] : none,
			m->muxnames[5] ? m->muxnames[5] : none,
			m->muxnames[6] ? m->muxnames[6] : none,
			m->muxnames[7] ? m->muxnames[7] : none);

	return 0;
}

#define OMAP_MUX_MAX_ARG_CHAR  7

static ssize_t omap_mux_dbg_signal_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	char buf[OMAP_MUX_MAX_ARG_CHAR];
	struct seq_file *seqf;
	struct omap_mux *m;
	unsigned long val;
	int buf_size, ret;
	struct omap_mux_partition *partition;

	if (count > OMAP_MUX_MAX_ARG_CHAR)
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	ret = strict_strtoul(buf, 0x10, &val);
	if (ret < 0)
		return ret;

	if (val > 0xffff)
		return -EINVAL;

	seqf = file->private_data;
	m = seqf->private;

	partition = omap_mux_get_partition(m);
	if (!partition)
		return -ENODEV;

	omap_mux_write(partition, (u16)val, m->reg_offset);
	*ppos += count;

	return count;
}

static int omap_mux_dbg_signal_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_mux_dbg_signal_show, inode->i_private);
}

static const struct file_operations omap_mux_dbg_signal_fops = {
	.open		= omap_mux_dbg_signal_open,
	.read		= seq_read,
	.write		= omap_mux_dbg_signal_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *mux_dbg_dir;

static void __init omap_mux_dbg_create_entry(
				struct omap_mux_partition *partition,
				struct dentry *mux_dbg_dir)
{
	struct omap_mux_entry *e;

	list_for_each_entry(e, &partition->muxmodes, node) {
		struct omap_mux *m = &e->mux;

		(void)debugfs_create_file(m->muxnames[0], S_IWUGO, mux_dbg_dir,
					  m, &omap_mux_dbg_signal_fops);
	}
}

static void __init omap_mux_dbg_init(void)
{
	struct omap_mux_partition *partition;
	static struct dentry *mux_dbg_board_dir;

	mux_dbg_dir = debugfs_create_dir("omap_mux", NULL);
	if (!mux_dbg_dir)
		return;

	mux_dbg_board_dir = debugfs_create_dir("board", mux_dbg_dir);
	if (!mux_dbg_board_dir)
		return;

	list_for_each_entry(partition, &mux_partitions, node) {
		omap_mux_dbg_create_entry(partition, mux_dbg_dir);
		(void)debugfs_create_file(partition->name, S_IRUGO,
					  mux_dbg_board_dir, partition,
					  &omap_mux_dbg_board_fops);
	}
}

#else
static inline void omap_mux_dbg_init(void)
{
}
#endif	/* CONFIG_DEBUG_FS */

static void __init omap_mux_free_names(struct omap_mux *m)
{
	int i;

	for (i = 0; i < OMAP_MUX_NR_MODES; i++)
		kfree(m->muxnames[i]);

#ifdef CONFIG_DEBUG_FS
	for (i = 0; i < OMAP_MUX_NR_SIDES; i++)
		kfree(m->balls[i]);
#endif

}

/* Free all data except for GPIO pins unless CONFIG_DEBUG_FS is set */
static int __init omap_mux_late_init(void)
{
	struct omap_mux_partition *partition;

	list_for_each_entry(partition, &mux_partitions, node) {
		struct omap_mux_entry *e, *tmp;
		list_for_each_entry_safe(e, tmp, &partition->muxmodes, node) {
			struct omap_mux *m = &e->mux;
			u16 mode = omap_mux_read(partition, m->reg_offset);

			if (OMAP_MODE_GPIO(mode))
				continue;

#ifndef CONFIG_DEBUG_FS
			mutex_lock(&muxmode_mutex);
			list_del(&e->node);
			mutex_unlock(&muxmode_mutex);
			omap_mux_free_names(m);
			kfree(m);
#endif
		}
	}

	omap_mux_dbg_init();

	return 0;
}
late_initcall(omap_mux_late_init);

// 不同的封装和superset是有区别的，需要fixup
static void __init omap_mux_package_fixup(struct omap_mux *p,
					struct omap_mux *superset)
{
	while (p->reg_offset !=  OMAP_MUX_TERMINATOR) {  // 直到最后  循环封装区别中的项目
		struct omap_mux *s = superset;
		int found = 0;
 
		while (s->reg_offset != OMAP_MUX_TERMINATOR) {  // 循环superset中的项目
			if (s->reg_offset == p->reg_offset) {  // 查找寄存器偏移是否一致，如果一致则break
				*s = *p;    // 如果找到了，则替换
				found++;
				break;
			}
			s++;
		}
		if (!found)   // 如果没有找到
			pr_err("%s: Unknown entry offset 0x%x\n", __func__,
			       p->reg_offset);
		p++;
	}
}

#ifdef CONFIG_DEBUG_FS   // 根据beagle的配置 CONFIG_DEBUG_FS=y

// 初始化封装的ball
static void __init omap_mux_package_init_balls(struct omap_ball *b,
				struct omap_mux *superset)
{
	while (b->reg_offset != OMAP_MUX_TERMINATOR) {  // 不同封装的中的ball。这里开始循环
		struct omap_mux *s = superset;  // 获取superset
		int found = 0;

		while (s->reg_offset != OMAP_MUX_TERMINATOR) {  // 轮询 superset
			if (s->reg_offset == b->reg_offset) {
				s->balls[0] = b->balls[0];  // 如果偏移地址一样的话，就复制到superset中的ball成员中
				s->balls[1] = b->balls[1];
				found++;
				break;
			}
			s++;
		}
		// 没有找到
		if (!found)
			pr_err("%s: Unknown ball offset 0x%x\n", __func__,
			       b->reg_offset);
		b++;
	}
}

#else	/* CONFIG_DEBUG_FS */

static inline void omap_mux_package_init_balls(struct omap_ball *b,
					struct omap_mux *superset)
{
}

#endif	/* CONFIG_DEBUG_FS */

static int __init omap_mux_setup(char *options)
{
	if (!options)
		return 0;

	omap_mux_options = options;

	return 1;
}
__setup("omap_mux=", omap_mux_setup);

/*
 * Note that the omap_mux=some.signal1=0x1234,some.signal2=0x1234
 * cmdline options only override the bootloader values.
 * During development, please enable CONFIG_DEBUG_FS, and use the
 * signal specific entries under debugfs.
 */
 // 根据cmdline中输入的值，来设定io的复用功能
 // 比如omap_mux=some.signal1=0x1234,some.signal2=0x1234
 // 根据这个来设定引脚的mux功能
static void __init omap_mux_set_cmdline_signals(void)
{
	char *options, *next_opt, *token;

	if (!omap_mux_options)
		return;

	options = kmalloc(strlen(omap_mux_options) + 1, GFP_KERNEL);
	if (!options)
		return;

	strcpy(options, omap_mux_options);
	next_opt = options;

	while ((token = strsep(&next_opt, ",")) != NULL) {
		char *keyval, *name;
		unsigned long val;

		keyval = token;
		name = strsep(&keyval, "=");
		if (name) {
			int res;

			res = strict_strtoul(keyval, 0x10, &val);
			if (res < 0)
				continue;

			omap_mux_init_signal(name, (u16)val);
		}
	}

	kfree(options);
}

// 复制mux的8个名称
static int __init omap_mux_copy_names(struct omap_mux *src,
				      struct omap_mux *dst)
{
	int i;
    // 循环8次 复制mux的8个name
	for (i = 0; i < OMAP_MUX_NR_MODES; i++) {
		if (src->muxnames[i]) {   // 如果有字符
			dst->muxnames[i] =
				kmalloc(strlen(src->muxnames[i]) + 1,
					GFP_KERNEL);   // 分配  src中的名字的字符长度
			if (!dst->muxnames[i])
				goto free;
			strcpy(dst->muxnames[i], src->muxnames[i]);  // 从src复制到dst
		}
	}

#ifdef CONFIG_DEBUG_FS
	for (i = 0; i < OMAP_MUX_NR_SIDES; i++) {
		if (src->balls[i]) {
			dst->balls[i] =
				kmalloc(strlen(src->balls[i]) + 1,
					GFP_KERNEL);
			if (!dst->balls[i])
				goto free;
			strcpy(dst->balls[i], src->balls[i]);
		}
	}
#endif

	return 0;

free:
	omap_mux_free_names(dst);
	return -ENOMEM;

}

#endif	/* CONFIG_OMAP_MUX */

// 根据gpio号码获取这个引脚的mux
static struct omap_mux *omap_mux_get_by_gpio(
				struct omap_mux_partition *partition,
				int gpio)
{
	struct omap_mux_entry *e;
	struct omap_mux *ret = NULL;
	// 轮询
	list_for_each_entry(e, &partition->muxmodes, node) {
		struct omap_mux *m = &e->mux;
		if (m->gpio == gpio) {  // 如果gpio号码对应，则返回
			ret = m;
			break;
		}
	}

	return ret;
}

/* Needed for dynamic muxing of GPIO pins for off-idle */
// 读取io引脚的mux模式mode的值
u16 omap_mux_get_gpio(int gpio)
{
	struct omap_mux_partition *partition;
	struct omap_mux *m;
	// 轮询
	list_for_each_entry(partition, &mux_partitions, node) {
		// 根据gpio口的号码获取相应的mux
		m = omap_mux_get_by_gpio(partition, gpio);
		// 读取mode的号码
		if (m)
			return omap_mux_read(partition, m->reg_offset);
	}

	// 操作失败，报错
	if (!m || m->reg_offset == OMAP_MUX_TERMINATOR)
		pr_err("%s: Could not get gpio%i\n", __func__, gpio);

	return OMAP_MUX_TERMINATOR;
}

/* Needed for dynamic muxing of GPIO pins for off-idle */
// 需要动态的设定io为引脚功能，这里根据val的值设定
void omap_mux_set_gpio(u16 val, int gpio)
{
	struct omap_mux_partition *partition;
	struct omap_mux *m = NULL;

	// 轮询包含mux_partitions链表的结构体，即partition
	list_for_each_entry(partition, &mux_partitions, node) {
		// 根据输入的gpio号 设定该引脚为gpio功能
		m = omap_mux_get_by_gpio(partition, gpio);  // 获取该gpio的mux
		if (m) {
			omap_mux_write(partition, val, m->reg_offset); // 设定为gpio功能
			return;
		}
	}

	// 操作失败，报错
	if (!m || m->reg_offset == OMAP_MUX_TERMINATOR)
		pr_err("%s: Could not set gpio%i\n", __func__, gpio);
}

static struct omap_mux * __init omap_mux_list_add(
					struct omap_mux_partition *partition,
					struct omap_mux *src)
{
	struct omap_mux_entry *entry;
	struct omap_mux *m;

	// 分配omap_mux_entry结构体
	entry = kzalloc(sizeof(struct omap_mux_entry), GFP_KERNEL);
	if (!entry)
		return NULL;

	m = &entry->mux;
	// 从src复制到m中
	memcpy(m, src, sizeof(struct omap_mux_entry));

#ifdef CONFIG_OMAP_MUX  // beagle这个配置是yes
     // 复制8个mux的name到m中
	if (omap_mux_copy_names(src, m)) {
		kfree(entry);
		return NULL;
	}
#endif

	mutex_lock(&muxmode_mutex);// muxmode的互斥量
	list_add_tail(&entry->node, &partition->muxmodes);  // 增加&entry->node到&partition->muxmodes之前
	mutex_unlock(&muxmode_mutex);

	return m;
}

/*
 * Note if CONFIG_OMAP_MUX is not selected, we will only initialize
 * the GPIO to mux offset mapping that is needed for dynamic muxing
 * of GPIO pins for off-idle.
 */
 
static void __init omap_mux_init_list(struct omap_mux_partition *partition,
				      struct omap_mux *superset)
{
	while (superset->reg_offset !=  OMAP_MUX_TERMINATOR) {   // 轮询superset
		struct omap_mux *entry;

#ifdef CONFIG_OMAP_MUX  // 经过测试 ，beagle中该配置是yes
		if (!superset->muxnames || !superset->muxnames[0]) { // 如果引脚的第一功能为空，则跳过
			superset++;
			continue;
		}
#else   // 在beagle的配置中，这里应该不执行的
		/* Skip pins that are not muxed as GPIO by bootloader */
		// 
		if (!OMAP_MODE_GPIO(omap_mux_read(partition,
				    superset->reg_offset))) {
			superset++;
			continue;
		}
#endif

		entry = omap_mux_list_add(partition, superset);
		if (!entry) {
			pr_err("%s: Could not add entry\n", __func__);
			return;
		}
		superset++;
	}
}

// 经过实际验证 beagle的配置中 CONFIG_OMAP_MUX=y
#ifdef CONFIG_OMAP_MUX
// 初始化mux
static void omap_mux_init_package(struct omap_mux *superset,
				  struct omap_mux *package_subset,
				  struct omap_ball *package_balls)
{
	// 如果和superset有区别，则开始fixup
	if (package_subset)    // 需要修改superset，根据package_subset中的区别
		omap_mux_package_fixup(package_subset, superset);
	// 因为superset中有ball的两个字符串，因此需要复制package_balls中的ball参数
	// 到superset中
	if (package_balls)   
		omap_mux_package_init_balls(package_balls, superset);
}

static void omap_mux_init_signals(struct omap_mux_partition *partition,
				  struct omap_board_mux *board_mux)
{
	omap_mux_set_cmdline_signals();
	omap_mux_write_array(partition, board_mux);
}

#else   // beagle配置中 以下是没有的

static void omap_mux_init_package(struct omap_mux *superset,
				  struct omap_mux *package_subset,
				  struct omap_ball *package_balls)
{
}

static void omap_mux_init_signals(struct omap_mux_partition *partition,
				  struct omap_board_mux *board_mux)
{
}

#endif

static u32 mux_partitions_cnt;

// 初始化mux
int __init omap_mux_init(const char *name, u32 flags,
			 u32 mux_pbase, u32 mux_size,
			 struct omap_mux *superset,   // 这里用的是 omap3_muxmodes
			 struct omap_mux *package_subset,
			 struct omap_board_mux *board_mux,
			 struct omap_ball *package_balls)
{
	struct omap_mux_partition *partition;

	// 分配一个omap_mux_partition类型的结构体
	partition = kzalloc(sizeof(struct omap_mux_partition), GFP_KERNEL);
	if (!partition)
		return -ENOMEM;

	// partition的名字
	partition->name = name;
	partition->flags = flags;  // 标志 这里可以是0
	partition->size = mux_size;  // mux的大小，即到哪里为止
	partition->phys = mux_pbase;  // mux的基地址
	partition->base = ioremap(mux_pbase, mux_size); // 获取虚拟地址
	if (!partition->base) {  // 如果无法获取虚拟地址 则报错
		pr_err("%s: Could not ioremap mux partition at 0x%08x\n",
			__func__, partition->phys);
		return -ENODEV;
	}

	// 初始化链表muxmodes 这个用于保存所有io的mux功能
	// 这里用于存放omap_mux
	INIT_LIST_HEAD(&partition->muxmodes);

	// node用于指向mux_partitions
	// 增加到链表mux_partitions中
	list_add_tail(&partition->node, &mux_partitions);
	mux_partitions_cnt++;
	// 增加一个partition  打印输出
	pr_info("%s: Add partition: #%d: %s, flags: %x\n", __func__,
		mux_partitions_cnt, partition->name, partition->flags);

	// 将package_subset中的区别覆盖到superset中
	// 将package_balls的信息复制到superset中，因为这里的superset中的ball为空
	omap_mux_init_package(superset, package_subset, package_balls);
	// 初始化list，将superset中的omap_mux链接到partition的成员mux_modes
	omap_mux_init_list(partition, superset);

	// 初始化信号，这里才是真正的设定io的模式为0-7，真正的复用功能设定
	omap_mux_init_signals(partition, board_mux);

	return 0;
}

