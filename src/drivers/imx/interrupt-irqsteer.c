// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright 2019 NXP
//
// Author: Daniel Baluta <daniel.baluta@nxp.com>
// Author: Paul Olaru <paul.olaru@nxp.com>

#include <sof/common.h>
#include <sof/drivers/interrupt.h>
#include <sof/lib/cpu.h>
#include <sof/lib/io.h>
#include <sof/lib/memory.h>
#include <sof/lib/uuid.h>
#include <sof/list.h>
#include <sof/spinlock.h>
#include <sof/string.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* fa00558c-d653-4851-a03a-b21f125a9524 */
DECLARE_SOF_UUID("irq-imx", irq_imx_uuid, 0xfa00558c, 0xd653, 0x4851,
		 0xa0, 0x3a, 0xb2, 0x1f, 0x12, 0x5a, 0x95, 0x24);

DECLARE_TR_CTX(irq_i_tr, SOF_UUID(irq_imx_uuid), LOG_LEVEL_INFO);

/*
 * The IRQ_STEER module takes 512 shared interrupts and delivers them
 * via 8 interrupt lines to any other component. It supports 5 channels,
 * one of them being for the DSP itself (channel 0).
 *
 *                        +-----------+
 * IRQ 0-63 ------/64---> |           | ---/8----> Channel 0 (DSP)
 * IRQ 64-127 ----/64---> |           |
 * IRQ 128-191 ---/64---> |           | ---/8----> Channel 1 (M4)
 * IRQ 192-255 ---/64---> | IRQ_STEER |
 * IRQ 256-319 ---/64---> |           | ---/8----> Channel 2 (SCU2)
 * IRQ 320-383 ---/64---> |           | ---/8----> Channel 3 (SCU1)
 * IRQ 384-447 ---/64---> |           |
 * IRQ 448-511 ---/64---> |           | ---/1----> Channel 4 (CTI)
 *                        +-----------+
 *
 * IRQ steer channel block diagram (all 5 channels are identical)
 *
 * +---------------------------------------------------------+
 * |                  +---+          +----+           +---+  |
 * ---> IRQ 0-63 ---> |   |          |    |           |   |  |
 * |  [MASK 0-63] --> | & | --/64--> | OR | ---/1---> | & | ----> OUT[0]
 * |                  |   | [STATUS] |    | [MD0] --> |   |  |
 * |                  +---+          +----+           +---+  |
 * |                                                         |
 * | ... (same for the other IRQ lines and outputs to OUT[7] |
 * |                                                         |
 * +---------------------------------------------------------+
 *
 * In the schematic above:
 *    IRQ 0-511: Input IRQ lines (shared IRQs). IRQs 0-31 are reserved.
 *    MASK 0-511: Configurable mask for interrupts.
 *    MD0-MD7: Master disable register, block an entire output interrupt
 *    line.
 *    STATUS: Read only register which shows what interrupts are active.
 *    OUT: The 8 interrupt lines that lead to the DSP, leading to arch
 *    IRQs IRQ_NUM_IRQSTR_DSP0 through 7.
 *
 * Usage of the hardware: We turn on the hardware itself, then we
 * configure the mask (all mask bits default to 0), enable our arch
 * interrupts and wait for an interrupt on an output line.
 *
 * Upon receiving an arch interrupt, the driver must check the STATUS
 * registers corresponding to the arch interrupt in order to figure out
 * what the actual, input shared interrupt was, and then call any
 * registered callback to handle the condition leading to the interrupt.
 *
 * The hardware also supports forcing an interrupt from the software; I
 * have omitted this from the schematic since it is not relevant to the
 * usage in this driver.
 */

/* The MASK, SET (unused) and STATUS registers are 512-bit registers
 * split into 16 32-bit registers that we can directly access.
 *
 * To get the proper register for the shared interrupt irq, we do
 * IRQSTR_CH_MASK(IRQSTR_INT_REG(irq)) (MASK can be replaced by SET or
 * STATUS).
 *
 * The interrupt mapping to registers is defined in
 * platform/drivers/interrupt.h for each platform.
 *
 * The IRQSTR_CH_* macros perform the second part of this calculation
 * (offset) automatically.
 */

#define IRQSTR_CHANCTL          0x00
#define IRQSTR_CH_MASK_BASE	0x04
#define IRQSTR_INT_REG(irq)		((irq) / 32)
#define IRQSTR_INT_BIT(irq)		((irq) % 32)
#define IRQSTR_INT_MASK(irq)		(1 << IRQSTR_INT_BIT(irq))

static struct irqstr_desc {
	uint32_t base_addr; /* IRQ steer base memory address */
	uint32_t min_virq;  /* min_virq and max_virq defines the range of virtual IRQs */
	uint32_t max_virq;
	uint32_t irqs_nr;   /* IRQ number handled by IRQ steer */
	uint32_t irqs_rs;   /* number of reserved IRQs */
	uint32_t irqs_pl;   /* number of IRQs per line */
	char *names[8];     /* IQR steer line names */
} irqstr_descriptors[] = {
#ifdef CONFIG_IMX8
	{
		.base_addr = 0x510A0000,
		.min_virq = IRQ_NUM_IRQSTR_DSP0,
		.max_virq = IRQ_NUM_IRQSTR_DSP7,
		.irqs_nr = 512,
		.irqs_rs = 32,
		.irqs_pl = 64,
		.names = { "dblog0", "dblog1", "dblog2", "dblog3",
			   "dblog4", "dblog5", "dblog6", "dblog7", },
	},
	{
		.base_addr = 0x410C0000, /* Local VPU SS address */
		.min_virq = IRQ_NUM_IRQSTR_LCL0,
		.max_virq = IRQ_NUM_IRQSTR_LCL1,
		.irqs_nr = 128,
		.irqs_rs = 0,
		.irqs_pl = 64,
		.names = { "vpu0", "vpu1", "vpu2", "vpu3",
			   "vpu4", "vpu5", "vpu6", "vpu7", },
	},
#elif defined CONFIG_IMX8X
	{
		.base_addr = 0x51080000,
		.min_virq = IRQ_NUM_IRQSTR_DSP0,
		.max_virq = IRQ_NUM_IRQSTR_DSP7,
		.irqs_nr = 512,
		.irqs_rs = 32,
		.irqs_pl = 64,
		.names = { "dblog0", "dblog1", "dblog2", "dblog3",
			   "dblog4", "dblog5", "dblog6", "dblog7", },
	},
#endif
};

static struct irqstr_desc *get_irqstr_desc(int virq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(irqstr_descriptors); i++)
		if (virq >= irqstr_descriptors[i].min_virq &&
		    virq <= irqstr_descriptors[i].max_virq)
			return &irqstr_descriptors[i];

	tr_err(&irq_i_tr, "get_irqstr_desc(): missing steer for IRQ %u", virq);

	return 0;
}

/* HW register access helper methods */

static inline void irqstr_write(struct irqstr_desc *sdesc, uint32_t reg, uint32_t value)
{
	io_reg_write(sdesc->base_addr + reg, value);
	tr_err(&irq_i_tr, "irqstr_write(): 0x%08x = 0x%08x",
		sdesc->base_addr + reg, value);
}

static inline uint32_t irqstr_read(struct irqstr_desc *sdesc, uint32_t reg)
{
	return io_reg_read(sdesc->base_addr + reg);
}

static inline void irqstr_update_bits(struct irqstr_desc *sdesc, uint32_t reg, uint32_t mask,
				      uint32_t value)
{
	io_reg_update_bits(sdesc->base_addr + reg, mask, value);
}

/* IRQ_STEER helper methods
 * These methods are usable in any IRQ_STEER driver, not specific to SOF
 */

static void irqstr_enable_hw(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(irqstr_descriptors); i++)
		irqstr_write(&irqstr_descriptors[i], IRQSTR_CHANCTL, 1);
}

static void irqstr_disable_hw(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(irqstr_descriptors); i++)
		irqstr_write(&irqstr_descriptors[i], IRQSTR_CHANCTL, 0);
}

/* irqstr_get_status_word() - Get an interrupt status word
 * @index The index of the status word
 *
 * Get the status of interrupts 32*index .. 32*(index+1)-1 in a word.
 * This status is in one hardware register.
 * Return: Status register word for the corresponding interrupts
 */
static uint32_t irqstr_get_status_word(struct irqstr_desc *sdesc, uint32_t index)
{
	uint32_t regs = sdesc->irqs_nr >> 5;
	uint32_t stat_base = IRQSTR_CH_MASK_BASE + regs * 4 * 2;
	uint32_t ch_status = stat_base + 0x04 * (regs - 1 - index);

	return irqstr_read(sdesc, ch_status);
}

static uint32_t irqstr_fixup_irq(uint32_t irq)
{
#ifdef CONFIG_IMX8M
	return irq - 32;
#else
	return irq;
#endif
}

/* Mask, that is, disable interrupts */
static void irqstr_mask_int(struct irqstr_desc *sdesc, uint32_t irq)
{
	uint32_t mask;
	uint32_t regs = sdesc->irqs_nr >> 5;
	uint32_t mask_reg = IRQSTR_CH_MASK_BASE + 4 * (regs - 1 - IRQSTR_INT_REG(irq));

	if (irq < sdesc->irqs_rs || irq >= sdesc->irqs_nr)
		return; // Unusable interrupts

	irq = irqstr_fixup_irq(irq);

	mask = IRQSTR_INT_MASK(irq);
	irqstr_update_bits(sdesc, mask_reg, mask, 0);
}

/* Unmask, that is, enable interrupts */
static void irqstr_unmask_int(struct irqstr_desc *sdesc, uint32_t irq)
{
	uint32_t mask;
	uint32_t regs = sdesc->irqs_nr >> 5;
	uint32_t mask_reg = IRQSTR_CH_MASK_BASE + 4 * (regs - 1 - IRQSTR_INT_REG(irq));
	uint32_t reg, val;

	if (irq < sdesc->irqs_rs || irq >= sdesc->irqs_nr)
		return; // Unusable interrupts

	irq = irqstr_fixup_irq(irq);

	mask = IRQSTR_INT_MASK(irq);

	tr_err(&irq_i_tr, "irqstr_unmask_int(): irq %u, regs: %u, mask_reg: 0x%08x, mask: 0x%08x",
		irq, regs, mask_reg, mask);

	reg = sdesc->base_addr + mask_reg;

	tr_err(&irq_i_tr, "irqstr_unmask_int(): read mask_reg: 0x%08x", reg);

	val = io_reg_read(reg);
		tr_err(&irq_i_tr, "irqstr_unmask_int(): read mask_reg: 0x%08x, val: 0x%08x",
			reg, val);

	irqstr_update_bits(sdesc, mask_reg, mask, mask);

	tr_err(&irq_i_tr, "irqstr_unmask_int(YES!): irq %u, mask_reg: 0x%08x, mask: 0x%08x",
		irq, mask_reg, mask);
}

#ifdef CONFIG_IMX8M

/* Quirk of the driver in SOF (Quirk is specific to 8MP):
 * -> IRQSTR has 5 input channels each with 32 interrupts
 * -> IRQSTR has 3 output channels each with 64 interrupts
 * -> IRQ in[31:0]    => IRQ out[63:32]   (output channel #0)
 * -> IRQ in[63:32]   => IRQ out[95:64]   (output channel #1, low half)
 * -> IRQ in[95:64]   => IRQ out[127:96]  (output channel #1, high half)
 * -> IRQ in[127:96]  => IRQ out[159:128] (output channel #2, low half)
 * -> IRQ in[159:128] => IRQ out[191:160] (output channel #2, high half)
 * Thus in SOF irqsteer we shift everything with 32 and we get:
 * -> Interrupts 0-31 are not usable
 * -> Interrupts 32-63 map to hw irqs 0-31 (irqsteer0)
 * -> Interrupts 64-127 map to hw irqs 32-95 (irqsteer1)
 * -> Interrupts 128-191 map to hw irqs 96-159 (irqsteer2)
 */

const char * const irq_name_irqsteer[] = {
	"irqsteer0",
	"irqsteer1",
	"irqsteer2"
};
#else
/* SOF specific part of the driver */

/* Quirk of the driver in SOF (Quirk is specific to 8QXP/8QM):
 * -> Interrupts 0-31 are hardware
 * -> Interrupts 32-63 are unusable, as they are reserved in irqstr. We
 *  will never get an event on these shared interrupt lines.
 * -> Interrupts 64-543 are usable, mapping to 32-512 in IRQSTR itself
 * The above functions expect the 32-512 interrupts valid, not the
 * shifted SOF ones.
 */
#endif

#define IRQ_MAX_TRIES	1000

/* Extract the 64 status bits corresponding to output interrupt line
 * index (64 input interrupts)
 */
#ifdef CONFIG_IMX8M
static uint64_t get_irqsteer_interrupts(uint32_t index)
{
	uint64_t result = 0;

	result = irqstr_get_status_word(2 * index);
	result <<= 32;

	/* line 0 is special only maps interrupts [63..32],
	 * interval [31..0] is not used
	 */
	if (index == 0)
		return result;

	result |= irqstr_get_status_word(2 * index - 1);
	return result;
}
#else
static uint64_t get_irqsteer_interrupts(struct irqstr_desc *sdesc, uint32_t index)
{
	uint64_t result = irqstr_get_status_word(sdesc, 2 * index + 1);

	result <<= 32;
	result |= irqstr_get_status_word(sdesc, 2 * index);
	return result;
}
#endif

/**
 * get_first_irq() Get the first IRQ bit set in this group.
 * @ints The 64 input interrupts
 *
 * Get the first pending IRQ in the group. For example, get_first_irq(0x40)
 * will return 6 (as 1 << 6 is 0x40), while get_first_irq(0) will return -1.
 *
 * Return: -1 if all interrupts are clear, or a shift value if at least
 * one interrupt is set.
 */
static int get_first_irq(uint64_t ints)
{
	return ffsll(ints) - 1;
}

static inline void handle_irq_batch(struct irq_cascade_desc *cascade,
				    uint32_t line_index, uint64_t status)
{
	int core = cpu_get_id();
	struct list_item *clist;
	struct irq_desc *child = NULL;
	struct irqstr_desc *sdesc;
	int bit;
	bool handled;

	while (status) {
		bit = get_first_irq(status);
		handled = false;
		status &= ~(1ull << bit); /* Release interrupt */

		spin_lock(&cascade->lock);

		/* Get child if any and run handler */
		list_for_item(clist, &cascade->child[bit].list) {
			child = container_of(clist, struct irq_desc, irq_list);

			if (child->handler && (child->cpu_mask & 1 << core)) {
				/* run handler in non atomic context */
				spin_unlock(&cascade->lock);
				child->handler(child->handler_arg);
				spin_lock(&cascade->lock);

				handled = true;
			}
		}

		spin_unlock(&cascade->lock);

		if (!handled) {
			tr_err(&irq_i_tr, "irq_handler(): nobody cared, bit %d",
			       bit);
			sdesc = get_irqstr_desc(cascade->desc.irq);
			/* Mask this interrupt so it won't happen again */
			if (sdesc)
				irqstr_mask_int(sdesc, line_index * sdesc->irqs_nr / 32 + bit);
		}
	}
}

static inline void irq_handler(void *data, uint32_t virq)
{
	struct irq_desc *parent = data;
	struct irq_cascade_desc *cascade =
		container_of(parent, struct irq_cascade_desc, desc);
	uint64_t status;
	uint32_t tries = IRQ_MAX_TRIES;
	struct irqstr_desc *sdesc;
	uint32_t line_index;

	sdesc = get_irqstr_desc(virq);
	line_index = virq - sdesc->min_virq;
	status = get_irqsteer_interrupts(sdesc, line_index);

	while (status) {
		/* Handle current interrupts */
		handle_irq_batch(cascade, line_index, status);

		/* Any interrupts happened while we were handling the
		 * current ones?
		 */
		status = get_irqsteer_interrupts(sdesc, line_index);
		if (!status)
			break;

		/* Any device keeping interrupting while we're handling
		 * or can't clear?
		 */

		if (!--tries) {
			tries = IRQ_MAX_TRIES;
			tr_err(&irq_i_tr, "irq_handler(): IRQ storm, status 0x%08x%08x",
			       (uint32_t)(status >> 32), (uint32_t)status);
		}
	}
}

#define DEFINE_IRQ_HANDLER(n) \
	static inline void irqstr_irqhandler_##n(void *arg) \
	{ \
		irq_handler(arg, n); \
	}

DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP0)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP1)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP2)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP3)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP4)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP5)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP6)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_DSP7)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_LCL0)
DEFINE_IRQ_HANDLER(IRQ_NUM_IRQSTR_LCL1)

static void irq_mask(struct irq_desc *desc, uint32_t irq, unsigned int core)
{
	struct irqstr_desc *sdesc = get_irqstr_desc(desc->irq);
	uint32_t irq_base;

	if (!sdesc)
		return;

	irq_base = desc->irq - sdesc->min_virq;

	/* Compute the actual IRQ_STEER IRQ number */
	irq_base *= sdesc->irqs_pl;
	irq += irq_base;

	irqstr_mask_int(sdesc, irq);
}

static void irq_unmask(struct irq_desc *desc, uint32_t irq, unsigned int core)
{
	struct irqstr_desc *sdesc = get_irqstr_desc(desc->irq);
	uint32_t irq_base;

	if (!sdesc) {
		tr_err(&irq_i_tr, "sdesc not found for: irq %u", irq);
		return;
	}

	tr_err(&irq_i_tr, "irq %u desc->irq %u, sdesc->min_virq %u",
		irq, desc->irq, sdesc->min_virq);

	irq_base = desc->irq - sdesc->min_virq;

	/* Compute the actual IRQ_STEER IRQ number */
	irq_base *= sdesc->irqs_pl;
	irq += irq_base;

	tr_err(&irq_i_tr, "irq_unmask(): irq %u", irq);

	irqstr_unmask_int(sdesc, irq);
}

static const struct irq_cascade_ops irq_ops = {
	.mask = irq_mask,
	.unmask = irq_unmask,
};

/* IRQ_STEER interrupts */
#define IRQSTR_CASCADE_TMPL_DECL(iname, n) \
	{ \
		.name = iname, \
		.irq = n, \
		.handler = irqstr_irqhandler_##n, \
		.ops = &irq_ops, \
		.global_mask = false, \
	},

static const struct irq_cascade_tmpl dsp_irq[] = {
	IRQSTR_CASCADE_TMPL_DECL("dblog0", IRQ_NUM_IRQSTR_DSP0)
	IRQSTR_CASCADE_TMPL_DECL("dblog1", IRQ_NUM_IRQSTR_DSP1)
	IRQSTR_CASCADE_TMPL_DECL("dblog2", IRQ_NUM_IRQSTR_DSP2)
	IRQSTR_CASCADE_TMPL_DECL("dblog3", IRQ_NUM_IRQSTR_DSP3)
	IRQSTR_CASCADE_TMPL_DECL("dblog4", IRQ_NUM_IRQSTR_DSP4)
	IRQSTR_CASCADE_TMPL_DECL("dblog5", IRQ_NUM_IRQSTR_DSP5)
	IRQSTR_CASCADE_TMPL_DECL("dblog6", IRQ_NUM_IRQSTR_DSP6)
	IRQSTR_CASCADE_TMPL_DECL("dblog7", IRQ_NUM_IRQSTR_DSP7)
	IRQSTR_CASCADE_TMPL_DECL("vpu0", IRQ_NUM_IRQSTR_LCL0)
	IRQSTR_CASCADE_TMPL_DECL("vpu1", IRQ_NUM_IRQSTR_LCL1)
};

int irqstr_get_sof_int(int irqstr_int, bool local)
{
	int line, irq;
	struct irqstr_desc *desc;

#ifdef CONFIG_IMX8
	desc = local ? &irqstr_descriptors[1] : &irqstr_descriptors[0];
#elif defined CONFIG_IMX8X
	desc = local ? 0 : &irqstr_descriptors[0];
#endif
	/* Is it a valid interrupt? */
	if (!desc || irqstr_int < 0 || irqstr_int >= desc->irqs_nr)
		return -EINVAL;

	line = irqstr_int / desc->irqs_pl;
	irq  = irqstr_int % desc->irqs_pl;

	return interrupt_get_irq(irq, desc->names[line]);
}

void platform_interrupt_init(void)
{
	int i, j;
	uint32_t regs, ch_mask;
	struct irqstr_desc *sdesc;

	/* Turn off the hardware so we don't have stray interrupts while
	 * initializing
	 */
	irqstr_disable_hw();
	/* Mask every external IRQ first */
	for (i = 0; i < ARRAY_SIZE(irqstr_descriptors); i++) {
		sdesc = &irqstr_descriptors[i];
		regs = sdesc->irqs_nr >> 5;
		tr_err(&irq_i_tr, "platform_interrupt_init(): base = 0x%08x, regs = %u",
			sdesc->base_addr, regs);

		for (j = 0; j < regs; j++) {
			ch_mask = IRQSTR_CH_MASK_BASE + 4 * j;
			tr_err(&irq_i_tr, "platform_interrupt_init(): base = 0x%08x, ch_mask = 0x%08x",
				sdesc->base_addr, ch_mask);
			irqstr_write(sdesc, ch_mask, 0xFFFFFFFF);
		}
	}
	/* Turn on the IRQ_STEER hardware */
	irqstr_enable_hw();

	for (i = 0; i < ARRAY_SIZE(dsp_irq); i++)
		interrupt_cascade_register(dsp_irq + i);
}

void platform_interrupt_set(uint32_t irq)
{
	if (interrupt_is_dsp_direct(irq))
		arch_interrupt_set(irq);
}

void platform_interrupt_clear(uint32_t irq, uint32_t mask)
{
	if (interrupt_is_dsp_direct(irq))
		arch_interrupt_clear(irq);
}

uint32_t platform_interrupt_get_enabled(void)
{
	return 0;
}

void interrupt_mask(uint32_t irq, unsigned int cpu)
{
	struct irq_cascade_desc *cascade = interrupt_get_parent(irq);

	if (cascade && cascade->ops->mask)
		cascade->ops->mask(&cascade->desc, irq - cascade->irq_base,
				   cpu);

}

void interrupt_unmask(uint32_t irq, unsigned int cpu)
{
	struct irq_cascade_desc *cascade = interrupt_get_parent(irq);

	if (cascade && cascade->ops->unmask)
		cascade->ops->unmask(&cascade->desc, irq - cascade->irq_base,
				     cpu);

}
