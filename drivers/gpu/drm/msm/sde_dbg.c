/* Copyright (c) 2009-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/slab.h>
#include <linux/list_sort.h>

#include "sde_dbg.h"
#include "sde/sde_hw_catalog.h"
#if defined(CONFIG_DISPLAY_SAMSUNG)
#include "ss_dsi_panel_common.h"
#endif

#define SDE_DBG_BASE_MAX		10

#define DEFAULT_PANIC		1
#define DEFAULT_REGDUMP		SDE_DBG_DUMP_IN_MEM
#define DEFAULT_DBGBUS_SDE	SDE_DBG_DUMP_IN_MEM
#define DEFAULT_DBGBUS_VBIFRT	SDE_DBG_DUMP_IN_MEM
#define DEFAULT_BASE_REG_CNT	0x100
#define GROUP_BYTES		4
#define ROW_BYTES		16
#define RANGE_NAME_LEN		40
#define REG_BASE_NAME_LEN	80

#define DBGBUS_FLAGS_DSPP	BIT(0)
#define DBGBUS_DSPP_STATUS	0x34C

#define DBGBUS_NAME_SDE		"sde"
#define DBGBUS_NAME_VBIF_RT	"vbif_rt"

/* offsets from sde top address for the debug buses */
#define DBGBUS_SSPP0	0x188
#define DBGBUS_AXI_INTF	0x194
#define DBGBUS_SSPP1	0x298
#define DBGBUS_DSPP	0x348
#define DBGBUS_PERIPH	0x418

#define TEST_MASK(id, tp)	((id << 4) | (tp << 1) | BIT(0))

/* following offsets are with respect to MDP VBIF base for DBG BUS access */
#define MMSS_VBIF_CLKON			0x4
#define MMSS_VBIF_TEST_BUS_OUT_CTRL	0x210
#define MMSS_VBIF_TEST_BUS_OUT		0x230

/* Vbif error info */
#define MMSS_VBIF_PND_ERR		0x190
#define MMSS_VBIF_SRC_ERR		0x194
#define MMSS_VBIF_XIN_HALT_CTRL1	0x204
#define MMSS_VBIF_ERR_INFO		0X1a0
#define MMSS_VBIF_ERR_INFO_1		0x1a4
#define MMSS_VBIF_CLIENT_NUM		14

/* print debug ranges in groups of 4 u32s */
#define REG_DUMP_ALIGN		16

#define RSC_DEBUG_MUX_SEL_SDM845 9

#define DBG_CTRL_STOP_FTRACE	BIT(0)
#define DBG_CTRL_PANIC_UNDERRUN	BIT(1)
#define DBG_CTRL_RESET_HW_PANIC	BIT(2)
#define DBG_CTRL_MAX			BIT(3)

#define DUMP_BUF_SIZE			(4096 * 512)
#define DUMP_CLMN_COUNT			4
#define DUMP_LINE_SIZE			256
#define DUMP_MAX_LINES_PER_BLK		512

#ifdef CONFIG_DISPLAY_SAMSUNG
/**
 * To print in kernel log
 */
#undef DEFAULT_REGDUMP
#undef DEFAULT_DBGBUS_SDE
#undef DEFAULT_DBGBUS_VBIFRT

#define DEFAULT_REGDUMP		SDE_DBG_DUMP_IN_LOG
#define DEFAULT_DBGBUS_SDE	SDE_DBG_DUMP_IN_LOG
#define DEFAULT_DBGBUS_VBIFRT	SDE_DBG_DUMP_IN_LOG
#endif

/**
 * struct sde_dbg_reg_offset - tracking for start and end of region
 * @start: start offset
 * @start: end offset
 */
struct sde_dbg_reg_offset {
	u32 start;
	u32 end;
};

/**
 * struct sde_dbg_reg_range - register dumping named sub-range
 * @head: head of this node
 * @reg_dump: address for the mem dump
 * @range_name: name of this range
 * @offset: offsets for range to dump
 * @xin_id: client xin id
 */
struct sde_dbg_reg_range {
	struct list_head head;
	u32 *reg_dump;
	char range_name[RANGE_NAME_LEN];
	struct sde_dbg_reg_offset offset;
	uint32_t xin_id;
};

/**
 * struct sde_dbg_reg_base - register region base.
 *	may sub-ranges: sub-ranges are used for dumping
 *	or may not have sub-ranges: dumping is base -> max_offset
 * @reg_base_head: head of this node
 * @sub_range_list: head to the list with dump ranges
 * @name: register base name
 * @base: base pointer
 * @off: cached offset of region for manual register dumping
 * @cnt: cached range of region for manual register dumping
 * @max_offset: length of region
 * @buf: buffer used for manual register dumping
 * @buf_len:  buffer length used for manual register dumping
 * @reg_dump: address for the mem dump if no ranges used
 * @cb: callback for external dump function, null if not defined
 * @cb_ptr: private pointer to callback function
 */
struct sde_dbg_reg_base {
	struct list_head reg_base_head;
	struct list_head sub_range_list;
	char name[REG_BASE_NAME_LEN];
	void __iomem *base;
	size_t off;
	size_t cnt;
	size_t max_offset;
	char *buf;
	size_t buf_len;
	u32 *reg_dump;
	void (*cb)(void *ptr);
	void *cb_ptr;
};

struct sde_debug_bus_entry {
	u32 wr_addr;
	u32 block_id;
	u32 test_id;
	void (*analyzer)(void __iomem *mem_base,
				struct sde_debug_bus_entry *entry, u32 val);
};

struct vbif_debug_bus_entry {
	u32 disable_bus_addr;
	u32 block_bus_addr;
	u32 bit_offset;
	u32 block_cnt;
	u32 test_pnt_start;
	u32 test_pnt_cnt;
};

struct sde_dbg_debug_bus_common {
	char *name;
	u32 enable_mask;
	bool include_in_deferred_work;
	u32 flags;
	u32 entries_size;
	u32 *dumped_content;
};

struct sde_dbg_sde_debug_bus {
	struct sde_dbg_debug_bus_common cmn;
	struct sde_debug_bus_entry *entries;
	u32 top_blk_off;
};

struct sde_dbg_vbif_debug_bus {
	struct sde_dbg_debug_bus_common cmn;
	struct vbif_debug_bus_entry *entries;
};

struct sde_dbg_dsi_debug_bus {
	u32 *entries;
	u32 size;
};

/**
 * struct sde_dbg_regbuf - wraps buffer and tracking params for register dumps
 * @buf: pointer to allocated memory for storing register dumps in hw recovery
 * @buf_size: size of the memory allocated
 * @len: size of the dump data valid in the buffer
 * @rpos: cursor points to the buffer position read by client
 * @dump_done: to indicate if dumping to user memory is complete
 * @cur_blk: points to the current sde_dbg_reg_base block
 */
struct sde_dbg_regbuf {
	char *buf;
	int buf_size;
	int len;
	int rpos;
	int dump_done;
	struct sde_dbg_reg_base *cur_blk;
};

/**
 * struct sde_dbg_base - global sde debug base structure
 * @evtlog: event log instance
 * @reg_base_list: list of register dumping regions
 * @dev: device pointer
 * @mutex: mutex to serialize access to serialze dumps, debugfs access
 * @power_ctrl: callback structure for enabling power for reading hw registers
 * @req_dump_blks: list of blocks requested for dumping
 * @panic_on_err: whether to kernel panic after triggering dump via debugfs
 * @dump_work: work struct for deferring register dump work to separate thread
 * @work_panic: panic after dump if internal user passed "panic" special region
 * @enable_reg_dump: whether to dump registers into memory, kernel log, or both
 * @dbgbus_sde: debug bus structure for the sde
 * @dbgbus_vbif_rt: debug bus structure for the realtime vbif
 * @dump_all: dump all entries in register dump
 * @dsi_dbg_bus: dump dsi debug bus register
 * @regbuf: buffer data to track the register dumping in hw recovery
 * @cur_evt_index: index used for tracking event logs dump in hw recovery
 * @dbgbus_dump_idx: index used for tracking dbg-bus dump in hw recovery
 * @vbif_dbgbus_dump_idx: index for tracking vbif dumps in hw recovery
 */
static struct sde_dbg_base {
	struct sde_dbg_evtlog *evtlog;
	struct list_head reg_base_list;
	struct device *dev;
	struct mutex mutex;
	struct sde_dbg_power_ctrl power_ctrl;

	struct sde_dbg_reg_base *req_dump_blks[SDE_DBG_BASE_MAX];

	u32 panic_on_err;
	struct work_struct dump_work;
	bool work_panic;
	u32 enable_reg_dump;

	struct sde_dbg_sde_debug_bus dbgbus_sde;
	struct sde_dbg_vbif_debug_bus dbgbus_vbif_rt;
	struct sde_dbg_dsi_debug_bus dbgbus_dsi;
	bool dump_all;
	bool dsi_dbg_bus;
	u32 debugfs_ctrl;

	struct sde_dbg_regbuf regbuf;
	u32 cur_evt_index;
	u32 dbgbus_dump_idx;
	u32 vbif_dbgbus_dump_idx;
} sde_dbg_base;

/* sde_dbg_base_evtlog - global pointer to main sde event log for macro use */
struct sde_dbg_evtlog *sde_dbg_base_evtlog;

/**
 * _sde_dbg_enable_power - use callback to turn power on for hw register access
 * @enable: whether to turn power on or off
 * Return: zero if success; error code otherwise
 */
static inline int _sde_dbg_enable_power(int enable)
{
	if (!sde_dbg_base.power_ctrl.enable_fn)
		return -EINVAL;
	return sde_dbg_base.power_ctrl.enable_fn(
			sde_dbg_base.power_ctrl.handle,
			sde_dbg_base.power_ctrl.client,
			enable);
}

/**
 * _sde_dump_reg - helper function for dumping rotator register set content
 * @dump_name: register set name
 * @reg_dump_flag: dumping flag controlling in-log/memory dump location
 * @base_addr: starting address of io region for calculating offsets to print
 * @addr: starting address offset for dumping
 * @len_bytes: range of the register set
 * @dump_mem: output buffer for memory dump location option
 * @from_isr: whether being called from isr context
 */
static void _sde_dump_reg(const char *dump_name, u32 reg_dump_flag,
		char *base_addr, char *addr, size_t len_bytes, u32 **dump_mem,
		bool from_isr)
{
	u32 in_log, in_mem, len_align, len_padded;
	u32 *dump_addr = NULL;
	char *end_addr;
	int i;
	int rc;

	if (!len_bytes)
		return;

	in_log = (reg_dump_flag & SDE_DBG_DUMP_IN_LOG);
	in_mem = (reg_dump_flag & SDE_DBG_DUMP_IN_MEM);

	pr_debug("%s: reg_dump_flag=%d in_log=%d in_mem=%d\n",
		dump_name, reg_dump_flag, in_log, in_mem);

	if (!in_log && !in_mem)
		return;

	if (in_log)
		dev_info(sde_dbg_base.dev, "%s: start_offset 0x%lx len 0x%zx\n",
				dump_name, (unsigned long)(addr - base_addr),
					len_bytes);

	len_align = (len_bytes + REG_DUMP_ALIGN - 1) / REG_DUMP_ALIGN;
	len_padded = len_align * REG_DUMP_ALIGN;
	end_addr = addr + len_bytes;

	if (in_mem) {
		if (dump_mem && !(*dump_mem)) {
			phys_addr_t phys = 0;
			*dump_mem = dma_alloc_coherent(sde_dbg_base.dev,
					len_padded, &phys, GFP_KERNEL);
		}

		if (dump_mem && *dump_mem) {
			dump_addr = *dump_mem;
			dev_info(sde_dbg_base.dev,
				"%s: start_addr:0x%pK len:0x%x reg_offset=0x%lx\n",
				dump_name, dump_addr, len_padded,
				(unsigned long)(addr - base_addr));
		} else {
			in_mem = 0;
			pr_debug("dump_mem: kzalloc fails!\n");
		}
	}

	if (!from_isr) {
		rc = _sde_dbg_enable_power(true);
		if (rc) {
			pr_debug("failed to enable power %d\n", rc);
			return;
		}
	}

	for (i = 0; i < len_align; i++) {
		u32 x0, x4, x8, xc;

		x0 = (addr < end_addr) ? readl_relaxed(addr + 0x0) : 0;
		x4 = (addr + 0x4 < end_addr) ? readl_relaxed(addr + 0x4) : 0;
		x8 = (addr + 0x8 < end_addr) ? readl_relaxed(addr + 0x8) : 0;
		xc = (addr + 0xc < end_addr) ? readl_relaxed(addr + 0xc) : 0;

		if (in_log)
			dev_info(sde_dbg_base.dev,
					"0x%lx : %08x %08x %08x %08x\n",
					(unsigned long)(addr - base_addr),
					x0, x4, x8, xc);

		if (dump_addr) {
			dump_addr[i * 4] = x0;
			dump_addr[i * 4 + 1] = x4;
			dump_addr[i * 4 + 2] = x8;
			dump_addr[i * 4 + 3] = xc;
		}

		addr += REG_DUMP_ALIGN;
	}

	if (!from_isr)
		_sde_dbg_enable_power(false);
}

/**
 * _sde_dbg_get_dump_range - helper to retrieve dump length for a range node
 * @range_node: range node to dump
 * @max_offset: max offset of the register base
 * @Return: length
 */
static u32 _sde_dbg_get_dump_range(struct sde_dbg_reg_offset *range_node,
		size_t max_offset)
{
	u32 length = 0;

	if ((range_node->start > range_node->end) ||
		(range_node->end > max_offset) || (range_node->start == 0
		&& range_node->end == 0)) {
		length = max_offset;
	} else {
		length = range_node->end - range_node->start;
	}

	return length;
}

static int _sde_dump_reg_range_cmp(void *priv, struct list_head *a,
		struct list_head *b)
{
	struct sde_dbg_reg_range *ar, *br;

	if (!a || !b)
		return 0;

	ar = container_of(a, struct sde_dbg_reg_range, head);
	br = container_of(b, struct sde_dbg_reg_range, head);

	return ar->offset.start - br->offset.start;
}

/**
 * _sde_dump_reg_by_ranges - dump ranges or full range of the register blk base
 * @dbg: register blk base structure
 * @reg_dump_flag: dump target, memory, kernel log, or both
 */
static void _sde_dump_reg_by_ranges(struct sde_dbg_reg_base *dbg,
	u32 reg_dump_flag)
{
	char *addr;
	size_t len;
	struct sde_dbg_reg_range *range_node;

	if (!dbg || !(dbg->base || dbg->cb)) {
		pr_debug("dbg base is null!\n");
		return;
	}

	dev_info(sde_dbg_base.dev, "%s:=========%s DUMP=========\n", __func__,
			dbg->name);
	if (dbg->cb) {
		dbg->cb(dbg->cb_ptr);
	/* If there is a list to dump the registers by ranges, use the ranges */
	} else if (!list_empty(&dbg->sub_range_list)) {
		/* sort the list by start address first */
		list_sort(NULL, &dbg->sub_range_list, _sde_dump_reg_range_cmp);
		list_for_each_entry(range_node, &dbg->sub_range_list, head) {
			len = _sde_dbg_get_dump_range(&range_node->offset,
				dbg->max_offset);
			addr = dbg->base + range_node->offset.start;
			pr_debug("%s: range_base=0x%pK start=0x%x end=0x%x\n",
				range_node->range_name,
				addr, range_node->offset.start,
				range_node->offset.end);

			_sde_dump_reg(range_node->range_name, reg_dump_flag,
					dbg->base, addr, len,
					&range_node->reg_dump, false);
		}
	} else {
		/* If there is no list to dump ranges, dump all registers */
		dev_info(sde_dbg_base.dev,
				"Ranges not found, will dump full registers\n");
		dev_info(sde_dbg_base.dev, "base:0x%pK len:0x%zx\n", dbg->base,
				dbg->max_offset);
		addr = dbg->base;
		len = dbg->max_offset;
		_sde_dump_reg(dbg->name, reg_dump_flag, dbg->base, addr, len,
				&dbg->reg_dump, false);
	}
}

/**
 * _sde_dump_reg_by_blk - dump a named register base region
 * @blk_name: register blk name
 */
static void _sde_dump_reg_by_blk(const char *blk_name)
{
	struct sde_dbg_base *dbg_base = &sde_dbg_base;
	struct sde_dbg_reg_base *blk_base;

	if (!dbg_base)
		return;

	list_for_each_entry(blk_base, &dbg_base->reg_base_list, reg_base_head) {
		if (strlen(blk_base->name) &&
			!strcmp(blk_base->name, blk_name)) {
			_sde_dump_reg_by_ranges(blk_base,
				dbg_base->enable_reg_dump);
			break;
		}
	}
}

/**
 * _sde_dump_reg_all - dump all register regions
 */
static void _sde_dump_reg_all(void)
{
	struct sde_dbg_base *dbg_base = &sde_dbg_base;
	struct sde_dbg_reg_base *blk_base;

	if (!dbg_base)
		return;

	list_for_each_entry(blk_base, &dbg_base->reg_base_list, reg_base_head)
		if (strlen(blk_base->name))
			_sde_dump_reg_by_blk(blk_base->name);
}

static void _sde_dbg_dump_sde_dbg_bus(struct sde_dbg_sde_debug_bus *bus)
{
	bool in_log, in_mem;
	u32 **dump_mem = NULL;
	u32 *dump_addr = NULL;
	u32 status = 0;
	struct sde_debug_bus_entry *head;
	phys_addr_t phys = 0;
	int list_size;
	int i;
	u32 offset;
	void __iomem *mem_base = NULL;
	struct sde_dbg_reg_base *reg_base;
	int rc;

	if (!bus || !bus->cmn.entries_size)
		return;

	list_for_each_entry(reg_base, &sde_dbg_base.reg_base_list,
			reg_base_head)
		if (strlen(reg_base->name) &&
			!strcmp(reg_base->name, bus->cmn.name))
			mem_base = reg_base->base + bus->top_blk_off;

	if (!mem_base) {
		pr_debug("unable to find mem_base for %s\n", bus->cmn.name);
		return;
	}

	dump_mem = &bus->cmn.dumped_content;

	/* will keep in memory 4 entries of 4 bytes each */
	list_size = (bus->cmn.entries_size * 4 * 4);

	in_log = (bus->cmn.enable_mask & SDE_DBG_DUMP_IN_LOG);
	in_mem = (bus->cmn.enable_mask & SDE_DBG_DUMP_IN_MEM);

	if (!in_log && !in_mem)
		return;

	dev_info(sde_dbg_base.dev, "======== start %s dump =========\n",
			bus->cmn.name);

	if (in_mem) {
		if (!(*dump_mem))
			*dump_mem = dma_alloc_coherent(sde_dbg_base.dev,
				list_size, &phys, GFP_KERNEL);

		if (*dump_mem) {
			dump_addr = *dump_mem;
			dev_info(sde_dbg_base.dev,
				"%s: start_addr:0x%pK len:0x%x\n",
				__func__, dump_addr, list_size);
		} else {
			in_mem = false;
			pr_debug("dump_mem: allocation fails\n");
		}
	}

	rc = _sde_dbg_enable_power(true);
	if (rc) {
		pr_debug("failed to enable power %d\n", rc);
		return;
	}

	for (i = 0; i < bus->cmn.entries_size; i++) {
		head = bus->entries + i;
		writel_relaxed(TEST_MASK(head->block_id, head->test_id),
				mem_base + head->wr_addr);
		wmb(); /* make sure test bits were written */

		if (bus->cmn.flags & DBGBUS_FLAGS_DSPP) {
			offset = DBGBUS_DSPP_STATUS;
			/* keep DSPP test point enabled */
			if (head->wr_addr != DBGBUS_DSPP)
				writel_relaxed(0x7001, mem_base + DBGBUS_DSPP);
		} else {
			offset = head->wr_addr + 0x4;
		}

		status = readl_relaxed(mem_base + offset);

		if (in_log)
			dev_info(sde_dbg_base.dev,
					"waddr=0x%x blk=%d tst=%d val=0x%x\n",
					head->wr_addr, head->block_id,
					head->test_id, status);

		if (dump_addr && in_mem) {
			dump_addr[i*4]     = head->wr_addr;
			dump_addr[i*4 + 1] = head->block_id;
			dump_addr[i*4 + 2] = head->test_id;
			dump_addr[i*4 + 3] = status;
		}

		if (head->analyzer)
			head->analyzer(mem_base, head, status);

		/* Disable debug bus once we are done */
		writel_relaxed(0, mem_base + head->wr_addr);
		if (bus->cmn.flags & DBGBUS_FLAGS_DSPP &&
						head->wr_addr != DBGBUS_DSPP)
			writel_relaxed(0x0, mem_base + DBGBUS_DSPP);
	}
	_sde_dbg_enable_power(false);

	dev_info(sde_dbg_base.dev, "======== end %s dump =========\n",
			bus->cmn.name);
}

static void _sde_dbg_dump_vbif_debug_bus_entry(
		struct vbif_debug_bus_entry *head, void __iomem *mem_base,
		u32 *dump_addr, bool in_log)
{
	int i, j;
	u32 val;

	if (!dump_addr && !in_log)
		return;

	for (i = 0; i < head->block_cnt; i++) {
		writel_relaxed(1 << (i + head->bit_offset),
				mem_base + head->block_bus_addr);
		/* make sure that current bus blcok enable */
		wmb();
		for (j = head->test_pnt_start; j < head->test_pnt_cnt; j++) {
			writel_relaxed(j, mem_base + head->block_bus_addr + 4);
			/* make sure that test point is enabled */
			wmb();
			val = readl_relaxed(mem_base + MMSS_VBIF_TEST_BUS_OUT);
			if (dump_addr) {
				*dump_addr++ = head->block_bus_addr;
				*dump_addr++ = i;
				*dump_addr++ = j;
				*dump_addr++ = val;
			}
			if (in_log)
				dev_info(sde_dbg_base.dev,
					"testpoint:%x arb/xin id=%d index=%d val=0x%x\n",
					head->block_bus_addr, i, j, val);
		}
	}
}

static void _sde_dbg_dump_vbif_dbg_bus(struct sde_dbg_vbif_debug_bus *bus)
{
	bool in_log, in_mem;
	u32 **dump_mem = NULL;
	u32 *dump_addr = NULL;
	u32 value, d0, d1;
	unsigned long reg, reg1, reg2;
	struct vbif_debug_bus_entry *head;
	phys_addr_t phys = 0;
	int i, list_size = 0;
	void __iomem *mem_base = NULL;
	struct vbif_debug_bus_entry *dbg_bus;
	u32 bus_size;
	struct sde_dbg_reg_base *reg_base;
	int rc;

	if (!bus || !bus->cmn.entries_size)
		return;

	list_for_each_entry(reg_base, &sde_dbg_base.reg_base_list,
			reg_base_head)
		if (strlen(reg_base->name) &&
			!strcmp(reg_base->name, bus->cmn.name))
			mem_base = reg_base->base;

	if (!mem_base) {
		pr_debug("unable to find mem_base for %s\n", bus->cmn.name);
		return;
	}

	dbg_bus = bus->entries;
	bus_size = bus->cmn.entries_size;
	list_size = bus->cmn.entries_size;
	dump_mem = &bus->cmn.dumped_content;

	dev_info(sde_dbg_base.dev, "======== start %s dump =========\n",
			bus->cmn.name);

	if (!dump_mem || !dbg_bus || !bus_size || !list_size)
		return;

	/* allocate memory for each test point */
	for (i = 0; i < bus_size; i++) {
		head = dbg_bus + i;
		list_size += (head->block_cnt * head->test_pnt_cnt);
	}

	/* 4 bytes * 4 entries for each test point*/
	list_size *= 16;

	in_log = (bus->cmn.enable_mask & SDE_DBG_DUMP_IN_LOG);
	in_mem = (bus->cmn.enable_mask & SDE_DBG_DUMP_IN_MEM);

	if (!in_log && !in_mem)
		return;

	if (in_mem) {
		if (!(*dump_mem))
			*dump_mem = dma_alloc_coherent(sde_dbg_base.dev,
				list_size, &phys, GFP_KERNEL);

		if (*dump_mem) {
			dump_addr = *dump_mem;
			dev_info(sde_dbg_base.dev,
				"%s: start_addr:0x%pK len:0x%x\n",
				__func__, dump_addr, list_size);
		} else {
			in_mem = false;
			pr_debug("dump_mem: allocation fails\n");
		}
	}

	rc = _sde_dbg_enable_power(true);
	if (rc) {
		pr_debug("failed to enable power %d\n", rc);
		return;
	}

	value = readl_relaxed(mem_base + MMSS_VBIF_CLKON);
	writel_relaxed(value | BIT(1), mem_base + MMSS_VBIF_CLKON);

	/* make sure that vbif core is on */
	wmb();

	/**
	 * Extract VBIF error info based on XIN halt and error status.
	 * If the XIN client is not in HALT state, or an error is detected,
	 * then retrieve the VBIF error info for it.
	 */
	reg = readl_relaxed(mem_base + MMSS_VBIF_XIN_HALT_CTRL1);
	reg1 = readl_relaxed(mem_base + MMSS_VBIF_PND_ERR);
	reg2 = readl_relaxed(mem_base + MMSS_VBIF_SRC_ERR);
	dev_err(sde_dbg_base.dev,
			"XIN HALT:0x%lX, PND ERR:0x%lX, SRC ERR:0x%lX\n",
			reg, reg1, reg2);
	reg >>= 16;
	reg &= ~(reg1 | reg2);
	for (i = 0; i < MMSS_VBIF_CLIENT_NUM; i++) {
		if (!test_bit(0, &reg)) {
			writel_relaxed(i, mem_base + MMSS_VBIF_ERR_INFO);
			/* make sure reg write goes through */
			wmb();

			d0 = readl_relaxed(mem_base + MMSS_VBIF_ERR_INFO);
			d1 = readl_relaxed(mem_base + MMSS_VBIF_ERR_INFO_1);

			dev_err(sde_dbg_base.dev,
					"Client:%d, errinfo=0x%X, errinfo1=0x%X\n",
					i, d0, d1);
		}
		reg >>= 1;
	}

	for (i = 0; i < bus_size; i++) {
		head = dbg_bus + i;

		writel_relaxed(0, mem_base + head->disable_bus_addr);
		writel_relaxed(BIT(0), mem_base + MMSS_VBIF_TEST_BUS_OUT_CTRL);
		/* make sure that other bus is off */
		wmb();

		_sde_dbg_dump_vbif_debug_bus_entry(head, mem_base, dump_addr,
				in_log);
		if (dump_addr)
			dump_addr += (head->block_cnt * head->test_pnt_cnt * 4);
	}

	_sde_dbg_enable_power(false);

	dev_info(sde_dbg_base.dev, "======== end %s dump =========\n",
			bus->cmn.name);
}

/**
 * _sde_dump_array - dump array of register bases
 * @blk_arr: array of register base pointers
 * @len: length of blk_arr
 * @do_panic: whether to trigger a panic after dumping
 * @name: string indicating origin of dump
 * @dump_dbgbus_sde: whether to dump the sde debug bus
 * @dump_dbgbus_vbif_rt: whether to dump the vbif rt debug bus
 */
static void _sde_dump_array(struct sde_dbg_reg_base *blk_arr[],
	u32 len, bool do_panic, const char *name, bool dump_dbgbus_sde,
	bool dump_dbgbus_vbif_rt, bool dump_all)
{
	int i;

	mutex_lock(&sde_dbg_base.mutex);

	if (dump_all)
		sde_evtlog_dump_all(sde_dbg_base.evtlog);

	if (dump_all || !blk_arr || !len) {
		_sde_dump_reg_all();
	} else {
		for (i = 0; i < len; i++) {
			if (blk_arr[i] != NULL)
				_sde_dump_reg_by_ranges(blk_arr[i],
					sde_dbg_base.enable_reg_dump);
		}
	}

	if (dump_dbgbus_sde)
		_sde_dbg_dump_sde_dbg_bus(&sde_dbg_base.dbgbus_sde);

	if (dump_dbgbus_vbif_rt)
		_sde_dbg_dump_vbif_dbg_bus(&sde_dbg_base.dbgbus_vbif_rt);

	if (sde_dbg_base.dsi_dbg_bus || dump_all)
		dsi_ctrl_debug_dump(sde_dbg_base.dbgbus_dsi.entries,
				    sde_dbg_base.dbgbus_dsi.size);

#if defined(CONFIG_DISPLAY_SAMSUNG)
	if (do_panic && sde_dbg_base.panic_on_err)
		ss_store_xlog_panic_dbg();
#endif

	if (do_panic && sde_dbg_base.panic_on_err)
		panic(name);

	mutex_unlock(&sde_dbg_base.mutex);
}

/*
 * sde_dbg_debugfs_open - debugfs open handler for evtlog dump
 * @inode: debugfs inode
 * @file: file handle
 */
static int sde_dbg_debugfs_open(struct inode *inode, struct file *file)
{
	if (!inode || !file)
		return -EINVAL;

	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	file->private_data = inode->i_private;
	return 0;
}

/**
 * sde_evtlog_dump_read - debugfs read handler for evtlog dump
 * @file: file handler
 * @buff: user buffer content for debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_evtlog_dump_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char evtlog_buf[SDE_EVTLOG_BUF_MAX];

	if (!buff || !ppos)
		return -EINVAL;

	len = sde_evtlog_dump_to_buffer(sde_dbg_base.evtlog, evtlog_buf,
			SDE_EVTLOG_BUF_MAX, true);
	if (len < 0 || len > count) {
		pr_debug("len is more than user buffer size");
		return 0;
	}

	if (copy_to_user(buff, evtlog_buf, len))
		return -EFAULT;
	*ppos += len;

	return len;
}

/**
 * sde_evtlog_dump_write - debugfs write handler for evtlog dump
 * @file: file handler
 * @user_buf: user buffer content from debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_evtlog_dump_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	_sde_dump_array(NULL, 0, sde_dbg_base.panic_on_err, "dump_debugfs",
		true, true, true);

	return count;
}

static const struct file_operations sde_evtlog_fops = {
	.open = sde_dbg_debugfs_open,
	.read = sde_evtlog_dump_read,
	.write = sde_evtlog_dump_write,
};

/**
 * sde_dbg_ctrl_read - debugfs read handler for debug ctrl read
 * @file: file handler
 * @buff: user buffer content for debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_dbg_ctrl_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char buf[24] = {'\0'};

	if (!buff || !ppos)
		return -EINVAL;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(buf, sizeof(buf), "0x%x\n", sde_dbg_base.debugfs_ctrl);
	pr_debug("%s: ctrl:0x%x len:0x%zx\n",
		__func__, sde_dbg_base.debugfs_ctrl, len);

	if ((count < sizeof(buf)) || copy_to_user(buff, buf, len)) {
		pr_debug("error copying the buffer! count:0x%zx\n", count);
		return -EFAULT;
	}

	*ppos += len;	/* increase offset */
	return len;
}

/**
 * sde_dbg_ctrl_write - debugfs read handler for debug ctrl write
 * @file: file handler
 * @user_buf: user buffer content from debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_dbg_ctrl_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	u32 dbg_ctrl = 0;
	char buf[24];

	if (!file) {
		pr_debug("DbgDbg: %s: error no file --\n", __func__);
		return -EINVAL;
	}

	if (count >= sizeof(buf))
		return -EFAULT;


	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0; /* end of string */

	if (kstrtouint(buf, 0, &dbg_ctrl)) {
		pr_debug("%s: error in the number of bytes\n", __func__);
		return -EFAULT;
	}

	pr_debug("dbg_ctrl_read:0x%x\n", dbg_ctrl);
	sde_dbg_base.debugfs_ctrl = dbg_ctrl;

	return count;
}

static const struct file_operations sde_dbg_ctrl_fops = {
	.open = sde_dbg_debugfs_open,
	.read = sde_dbg_ctrl_read,
	.write = sde_dbg_ctrl_write,
};

/*
 * sde_evtlog_filter_show - read callback for evtlog filter
 * @s: pointer to seq_file object
 * @data: pointer to private data
 */
static int sde_evtlog_filter_show(struct seq_file *s, void *data)
{
	struct sde_dbg_evtlog *evtlog;
	char buffer[64];
	int i;

	if (!s || !s->private)
		return -EINVAL;

	evtlog = s->private;

	for (i = 0; !sde_evtlog_get_filter(
				evtlog, i, buffer, ARRAY_SIZE(buffer)); ++i)
		seq_printf(s, "*%s*\n", buffer);
	return 0;
}

/*
 * sde_evtlog_filter_open - debugfs open handler for evtlog filter
 * @inode: debugfs inode
 * @file: file handle
 * Returns: zero on success
 */
static int sde_evtlog_filter_open(struct inode *inode, struct file *file)
{
	if (!inode || !file)
		return -EINVAL;

	return single_open(file, sde_evtlog_filter_show, inode->i_private);
}

/*
 * sde_evtlog_filter_write - write callback for evtlog filter
 * @file: pointer to file structure
 * @user_buf: pointer to incoming user data
 * @count: size of incoming user buffer
 * @ppos: pointer to file offset
 */
static ssize_t sde_evtlog_filter_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	char *tmp_filter = NULL;
	ssize_t rc = 0;

	if (!user_buf)
		return -EINVAL;

	if (count > 0) {
		/* copy user provided string and null terminate it */
		tmp_filter = kzalloc(count + 1, GFP_KERNEL);
		if (!tmp_filter)
			rc = -ENOMEM;
		else if (copy_from_user(tmp_filter, user_buf, count))
			rc = -EFAULT;
	}

	/* update actual filter configuration on success */
	if (!rc) {
		sde_evtlog_set_filter(sde_dbg_base.evtlog, tmp_filter);
		rc = count;
	}
	kfree(tmp_filter);

	return rc;
}

static const struct file_operations sde_evtlog_filter_fops = {
	.open =		sde_evtlog_filter_open,
	.write =	sde_evtlog_filter_write,
	.read =		seq_read,
	.llseek =	seq_lseek,
	.release =	seq_release
};

/**
 * sde_dbg_reg_base_release - release allocated reg dump file private data
 * @inode: debugfs inode
 * @file: file handle
 * @Return: 0 on success
 */
static int sde_dbg_reg_base_release(struct inode *inode, struct file *file)
{
	struct sde_dbg_reg_base *dbg;

	if (!file)
		return -EINVAL;

	dbg = file->private_data;
	if (!dbg)
		return -ENODEV;

	mutex_lock(&sde_dbg_base.mutex);
	if (dbg && dbg->buf) {
		kfree(dbg->buf);
		dbg->buf_len = 0;
		dbg->buf = NULL;
	}
	mutex_unlock(&sde_dbg_base.mutex);

	return 0;
}

/**
 * sde_dbg_reg_base_is_valid_range - verify if requested memory range is valid
 * @off: address offset in bytes
 * @cnt: memory size in bytes
 * Return: true if valid; false otherwise
 */
static bool sde_dbg_reg_base_is_valid_range(u32 off, u32 cnt)
{
	static struct sde_dbg_base *dbg_base = &sde_dbg_base;
	struct sde_dbg_reg_range *node;
	struct sde_dbg_reg_base *base;

	pr_debug("check offset=0x%x cnt=0x%x\n", off, cnt);

	list_for_each_entry(base, &dbg_base->reg_base_list, reg_base_head) {
		list_for_each_entry(node, &base->sub_range_list, head) {
			pr_debug("%s: start=0x%x end=0x%x\n", node->range_name,
					node->offset.start, node->offset.end);

			if (node->offset.start <= off
					&& off <= node->offset.end
					&& off + cnt <= node->offset.end) {
				pr_debug("valid range requested\n");
				return true;
			}
		}
	}

	pr_debug("invalid range requested\n");
	return false;
}

/**
 * sde_dbg_reg_base_offset_write - set new offset and len to debugfs reg base
 * @file: file handler
 * @user_buf: user buffer content from debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_dbg_reg_base_offset_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sde_dbg_reg_base *dbg;
	u32 off = 0;
	u32 cnt = DEFAULT_BASE_REG_CNT;
	char buf[24];

	if (!file)
		return -EINVAL;

	dbg = file->private_data;
	if (!dbg)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end of string */

	if (sscanf(buf, "%5x %x", &off, &cnt) != 2)
		return -EFAULT;

	if (off > dbg->max_offset)
		return -EINVAL;

	if (off % sizeof(u32))
		return -EINVAL;

	if (cnt > (dbg->max_offset - off))
		cnt = dbg->max_offset - off;

	if (cnt == 0)
		return -EINVAL;

	if (!sde_dbg_reg_base_is_valid_range(off, cnt))
		return -EINVAL;

	mutex_lock(&sde_dbg_base.mutex);
	dbg->off = off;
	dbg->cnt = cnt;
	mutex_unlock(&sde_dbg_base.mutex);

	pr_debug("offset=%x cnt=%x\n", off, cnt);

	return count;
}

/**
 * sde_dbg_reg_base_offset_read - read current offset and len of register base
 * @file: file handler
 * @user_buf: user buffer content from debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_dbg_reg_base_offset_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	struct sde_dbg_reg_base *dbg;
	int len = 0;
	char buf[24] = {'\0'};

	if (!file)
		return -EINVAL;

	dbg = file->private_data;
	if (!dbg)
		return -ENODEV;

	if (!ppos)
		return -EINVAL;

	if (*ppos)
		return 0;	/* the end */

	mutex_lock(&sde_dbg_base.mutex);
	if (dbg->off % sizeof(u32)) {
		mutex_unlock(&sde_dbg_base.mutex);
		return -EFAULT;
	}

	len = snprintf(buf, sizeof(buf), "0x%08zx %zx\n", dbg->off, dbg->cnt);
	if (len < 0 || len >= sizeof(buf)) {
		mutex_unlock(&sde_dbg_base.mutex);
		return 0;
	}

	if ((count < sizeof(buf)) || copy_to_user(buff, buf, len)) {
		mutex_unlock(&sde_dbg_base.mutex);
		return -EFAULT;
	}

	*ppos += len;	/* increase offset */
	mutex_unlock(&sde_dbg_base.mutex);

	return len;
}

/**
 * sde_dbg_reg_base_reg_write - write to reg base hw at offset a given value
 * @file: file handler
 * @user_buf: user buffer content from debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_dbg_reg_base_reg_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sde_dbg_reg_base *dbg;
	size_t off;
	u32 data, cnt;
	char buf[24];
	int rc;

	if (!file)
		return -EINVAL;

	dbg = file->private_data;
	if (!dbg)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end of string */

	cnt = sscanf(buf, "%zx %x", &off, &data);

	if (cnt < 2)
		return -EFAULT;

	if (off % sizeof(u32))
		return -EFAULT;

	mutex_lock(&sde_dbg_base.mutex);
	if (off >= dbg->max_offset) {
		mutex_unlock(&sde_dbg_base.mutex);
		return -EFAULT;
	}

	rc = _sde_dbg_enable_power(true);
	if (rc) {
		mutex_unlock(&sde_dbg_base.mutex);
		pr_debug("failed to enable power %d\n", rc);
		return rc;
	}

	writel_relaxed(data, dbg->base + off);

	_sde_dbg_enable_power(false);

	mutex_unlock(&sde_dbg_base.mutex);

	pr_debug("addr=%zx data=%x\n", off, data);

	return count;
}

/**
 * sde_dbg_reg_base_reg_read - read len from reg base hw at current offset
 * @file: file handler
 * @user_buf: user buffer content from debugfs
 * @count: size of user buffer
 * @ppos: position offset of user buffer
 */
static ssize_t sde_dbg_reg_base_reg_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sde_dbg_reg_base *dbg;
	size_t len;
	int rc;

	if (!file)
		return -EINVAL;

	dbg = file->private_data;
	if (!dbg) {
		pr_debug("invalid handle\n");
		return -ENODEV;
	}

	if (!ppos)
		return -EINVAL;

	mutex_lock(&sde_dbg_base.mutex);
	if (!dbg->buf) {
		char dump_buf[64];
		char *ptr;
		int cnt, tot;

		dbg->buf_len = sizeof(dump_buf) *
			DIV_ROUND_UP(dbg->cnt, ROW_BYTES);
		dbg->buf = kzalloc(dbg->buf_len, GFP_KERNEL);

		if (!dbg->buf) {
			mutex_unlock(&sde_dbg_base.mutex);
			return -ENOMEM;
		}

		if (dbg->off % sizeof(u32)) {
			mutex_unlock(&sde_dbg_base.mutex);
			return -EFAULT;
		}

		ptr = dbg->base + dbg->off;
		tot = 0;

		rc = _sde_dbg_enable_power(true);
		if (rc) {
			mutex_unlock(&sde_dbg_base.mutex);
			pr_debug("failed to enable power %d\n", rc);
			return rc;
		}

		for (cnt = dbg->cnt; cnt > 0; cnt -= ROW_BYTES) {
			hex_dump_to_buffer(ptr, min(cnt, ROW_BYTES),
					   ROW_BYTES, GROUP_BYTES, dump_buf,
					   sizeof(dump_buf), false);
			len = scnprintf(dbg->buf + tot, dbg->buf_len - tot,
					"0x%08x: %s\n",
					((int) (unsigned long) ptr) -
					((int) (unsigned long) dbg->base),
					dump_buf);

			ptr += ROW_BYTES;
			tot += len;
			if (tot >= dbg->buf_len)
				break;
		}

		_sde_dbg_enable_power(false);

		dbg->buf_len = tot;
	}

	if (*ppos >= dbg->buf_len) {
		mutex_unlock(&sde_dbg_base.mutex);
		return 0; /* done reading */
	}

	len = min(count, dbg->buf_len - (size_t) *ppos);
	if (copy_to_user(user_buf, dbg->buf + *ppos, len)) {
		mutex_unlock(&sde_dbg_base.mutex);
		pr_debug("failed to copy to user\n");
		return -EFAULT;
	}

	*ppos += len; /* increase offset */
	mutex_unlock(&sde_dbg_base.mutex);

	return len;
}

static const struct file_operations sde_off_fops = {
	.open = sde_dbg_debugfs_open,
	.release = sde_dbg_reg_base_release,
	.read = sde_dbg_reg_base_offset_read,
	.write = sde_dbg_reg_base_offset_write,
};

static const struct file_operations sde_reg_fops = {
	.open = sde_dbg_debugfs_open,
	.release = sde_dbg_reg_base_release,
	.read = sde_dbg_reg_base_reg_read,
	.write = sde_dbg_reg_base_reg_write,
};
