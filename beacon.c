// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT7601U AP beacon support
 * Ported from mt76x02_beacon.c / mt76x02_usb_core.c
 */

#include "mt7601u.h"
#include "mac.h"

#define PRE_TBTT_USEC	8000
#define N_BCN_SLOTS	5
#define BCN_SLOT_SIZE	((8192 / N_BCN_SLOTS) & ~63)

/* Missing register definitions for beacon support */
#define MT_TBTT_TIMER		0x1124
#define MT_TBTT_TIMER_VAL	GENMASK(16, 0)
#define MT_TSF_TIMER_DW0	0x111c
#define MT_TSF_TIMER_DW1	0x1120

static void mt7601u_set_beacon_offsets(struct mt7601u_dev *dev)
{
	u32 regs[4] = {};
	u16 val;
	int i;

	for (i = 0; i < N_BCN_SLOTS; i++) {
		val = i * BCN_SLOT_SIZE;
		regs[i / 4] |= (val / 64) << (8 * (i % 4));
	}

	for (i = 0; i < 4; i++)
		mt76_wr(dev, MT_BCN_OFFSET(i), regs[i]);
}

static struct mt76_txwi *
mt7601u_beacon_txwi(struct mt7601u_dev *dev, struct sk_buff *skb)
{
	struct mt76_txwi *txwi;

	txwi = (struct mt76_txwi *)skb_push(skb, sizeof(*txwi));
	memset(txwi, 0, sizeof(*txwi));

	/* Beacon rate: 1Mbps CCK */
	txwi->rate_ctl = cpu_to_le16(FIELD_PREP(MT_RXWI_RATE_MCS, 0) |
				     FIELD_PREP(MT_RXWI_RATE_PHY, MT_PHY_TYPE_CCK));
	txwi->len_ctl = cpu_to_le16(skb->len - sizeof(*txwi));

	return txwi;
}

static int
mt7601u_write_beacon(struct mt7601u_dev *dev, int offset, struct sk_buff *skb)
{
	if (WARN_ON_ONCE(BCN_SLOT_SIZE < skb->len + sizeof(struct mt76_txwi)))
		return -ENOSPC;

	mt7601u_beacon_txwi(dev, skb);
	mt7601u_wr_copy(dev, offset, skb->data, skb->len);
	dev_kfree_skb(skb);
	return 0;
}

static void mt7601u_pre_tbtt_work(struct work_struct *work)
{
	struct mt7601u_dev *dev = container_of(work, struct mt7601u_dev,
					       pre_tbtt_work);
	struct sk_buff *skb;
	int bcn_addr;

	if (!dev->beacon_mask)
		return;

	dev->tbtt_count++;

	/* Resync beacon timer to account for drift (1us per tick) */
	if (dev->tbtt_count >= 63) {
		u32 timer_val = dev->beacon_int << 4;

		if (dev->tbtt_count == 63)
			timer_val -= 1;

		mt76_rmw_field(dev, MT_BEACON_TIME_CFG,
			       MT_BEACON_TIME_CFG_INTVAL, timer_val);

		if (dev->tbtt_count >= 64)
			dev->tbtt_count = 0;
	}

	/* Prevent corrupt transmissions during update */
	mt76_set(dev, MT_BCN_BYPASS_MASK, 0xffff);
	dev->beacon_data_count = 0;

	skb = ieee80211_beacon_get(dev->hw, dev->beacon_vif, 0);
	if (!skb)
		goto out;

	bcn_addr = MT_BEACON_BASE + (BCN_SLOT_SIZE * dev->beacon_data_count);
	if (!mt7601u_write_beacon(dev, bcn_addr, skb))
		dev->beacon_data_count++;

out:
	mt76_wr(dev, MT_BCN_BYPASS_MASK,
		0xff00 | ~(0xff00 >> dev->beacon_data_count));

	/* Restart timer for next TBTT */
	mt7601u_restart_pre_tbtt_timer(dev);
}

static enum hrtimer_restart mt7601u_pre_tbtt_interrupt(struct hrtimer *timer)
{
	struct mt7601u_dev *dev = container_of(timer, struct mt7601u_dev,
					       pre_tbtt_timer);

	queue_work(system_highpri_wq, &dev->pre_tbtt_work);
	return HRTIMER_NORESTART;
}

static void mt7601u_start_pre_tbtt_timer(struct mt7601u_dev *dev)
{
	u64 time;
	u32 tbtt;

	tbtt = mt7601u_rr(dev, MT_TBTT_TIMER);
	tbtt = FIELD_GET(MT_TBTT_TIMER_VAL, tbtt);
	tbtt *= 32; /* convert to usec */

	if (tbtt <= PRE_TBTT_USEC) {
		queue_work(system_highpri_wq, &dev->pre_tbtt_work);
		return;
	}

	time = (tbtt - PRE_TBTT_USEC) * 1000ull;
	hrtimer_start(&dev->pre_tbtt_timer, time, HRTIMER_MODE_REL);
}

void mt7601u_restart_pre_tbtt_timer(struct mt7601u_dev *dev)
{
	u32 tbtt;
	u64 time;

	tbtt = mt7601u_rr(dev, MT_TBTT_TIMER);
	tbtt = FIELD_GET(MT_TBTT_TIMER_VAL, tbtt);
	tbtt *= 32;

	/* Convert beacon interval (TU = 1024 usec) to nsec */
	time = ((1000000000ull * dev->beacon_int) >> 10);

	/* Adjust to fire 8ms before TBTT */
	if (tbtt < PRE_TBTT_USEC)
		time -= (PRE_TBTT_USEC - tbtt) * 1000ull;
	else
		time += (tbtt - PRE_TBTT_USEC) * 1000ull;

	hrtimer_start(&dev->pre_tbtt_timer, time, HRTIMER_MODE_REL);
}

void mt7601u_beacon_set_timer(struct mt7601u_dev *dev, int beacon_int)
{
	dev->beacon_int = beacon_int;

	if (beacon_int) {
		u32 timer_val = beacon_int << 4;

		mt76_rmw_field(dev, MT_BEACON_TIME_CFG,
			       MT_BEACON_TIME_CFG_INTVAL, timer_val);
	}
}

void mt7601u_mac_set_beacon_enable(struct mt7601u_dev *dev,
				   struct ieee80211_vif *vif, bool enable)
{
	if (enable) {
		dev->beacon_mask = 1;
		dev->beacon_vif = vif;
		dev->tbtt_count = 0;

		mt76_set(dev, MT_BEACON_TIME_CFG,
			 MT_BEACON_TIME_CFG_BEACON_TX |
			 MT_BEACON_TIME_CFG_TBTT_EN |
			 MT_BEACON_TIME_CFG_TIMER_EN);

		if (dev->beacon_int)
			mt7601u_start_pre_tbtt_timer(dev);
	} else {
		dev->beacon_mask = 0;
		dev->beacon_vif = NULL;

		mt76_clear(dev, MT_BEACON_TIME_CFG,
			   MT_BEACON_TIME_CFG_BEACON_TX |
			   MT_BEACON_TIME_CFG_TBTT_EN |
			   MT_BEACON_TIME_CFG_TIMER_EN);

		hrtimer_cancel(&dev->pre_tbtt_timer);
		cancel_work_sync(&dev->pre_tbtt_work);
	}
}

void mt7601u_init_beacon_config(struct mt7601u_dev *dev)
{
	mt76_clear(dev, MT_BEACON_TIME_CFG,
		   MT_BEACON_TIME_CFG_TIMER_EN |
		   MT_BEACON_TIME_CFG_TBTT_EN |
		   MT_BEACON_TIME_CFG_BEACON_TX);
	mt76_set(dev, MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_SYNC_MODE);
	mt76_wr(dev, MT_BCN_BYPASS_MASK, 0xffff);
	mt7601u_set_beacon_offsets(dev);

	hrtimer_init(&dev->pre_tbtt_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->pre_tbtt_timer.function = mt7601u_pre_tbtt_interrupt;
	INIT_WORK(&dev->pre_tbtt_work, mt7601u_pre_tbtt_work);
}
