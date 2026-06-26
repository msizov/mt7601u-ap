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
	txwi->rate_ctl = cpu_to_le16(FIELD_PREP(MT_TXWI_RATE_MCS, 0) |
				     FIELD_PREP(MT_TXWI_RATE_PHY_MODE,
						MT_PHY_TYPE_CCK));
	txwi->len_ctl = cpu_to_le16(skb->len - sizeof(*txwi));

	return txwi;
}

static int
mt7601u_write_beacon(struct mt7601u_dev *dev, int offset, struct sk_buff *skb)
{
	int ret = 0;

	if (WARN_ON_ONCE(BCN_SLOT_SIZE < skb->len + sizeof(struct mt76_txwi))) {
		ret = -ENOSPC;
		goto out;
	}

	/* Make sure there is enough headroom for the txwi we are about to
	 * push; normally mac80211 reserves it via hw->extra_tx_headroom.
	 */
	if (skb_cow(skb, sizeof(struct mt76_txwi))) {
		ret = -ENOMEM;
		goto out;
	}

	mt7601u_beacon_txwi(dev, skb);
	mt7601u_wr_copy(dev, offset, skb->data, skb->len);

out:
	dev_kfree_skb_any(skb);
	return ret;
}

static void
mt7601u_update_beacon_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt7601u_dev *dev = priv;
	struct sk_buff *skb;

	if (!(dev->beacon_mask & BIT(0)))
		return;

	skb = ieee80211_beacon_get(dev->hw, vif, 0);
	if (skb) {
		int bcn_addr = MT_BEACON_BASE +
			       (BCN_SLOT_SIZE * dev->beacon_data_count);

		if (!mt7601u_write_beacon(dev, bcn_addr, skb))
			dev->beacon_data_count++;
	}
}

static void mt7601u_resync_beacon_timer(struct mt7601u_dev *dev)
{
	u32 timer_val = dev->beacon_int << 4;

	dev->tbtt_count++;

	/* Beacon timer drifts by 1us every tick; the timer is configured
	 * in 1/16 TU (64us) units.
	 */
	if (dev->tbtt_count < 63)
		return;

	/* The updated beacon interval takes effect after two TBTT, because
	 * at this point the original interval has already been loaded into
	 * the next TBTT_TIMER value.
	 */
	if (dev->tbtt_count == 63)
		timer_val -= 1;

	mt76_rmw_field(dev, MT_BEACON_TIME_CFG,
		       MT_BEACON_TIME_CFG_INTVAL, timer_val);

	if (dev->tbtt_count >= 64)
		dev->tbtt_count = 0;
}

static void mt7601u_pre_tbtt_work(struct work_struct *work)
{
	struct mt7601u_dev *dev = container_of(work, struct mt7601u_dev,
					       pre_tbtt_work);

	if (!dev->beacon_mask || test_bit(MT7601U_STATE_SCANNING, &dev->state))
		return;

	mt7601u_resync_beacon_timer(dev);

	/* Prevent corrupt transmissions during update */
	mt76_set(dev, MT_BCN_BYPASS_MASK, 0xffff);
	dev->beacon_data_count = 0;

	ieee80211_iterate_active_interfaces_atomic(dev->hw,
		IEEE80211_IFACE_ITER_RESUME_ALL,
		mt7601u_update_beacon_iter, dev);

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

static void mt7601u_stop_pre_tbtt_timer(struct mt7601u_dev *dev)
{
	do {
		hrtimer_cancel(&dev->pre_tbtt_timer);
		cancel_work_sync(&dev->pre_tbtt_work);
		/* Timer can be rearmed by the work. */
	} while (hrtimer_active(&dev->pre_tbtt_timer));
}

static void mt7601u_pre_tbtt_enable(struct mt7601u_dev *dev, bool en)
{
	if (en && dev->beacon_mask && dev->beacon_int &&
	    !hrtimer_active(&dev->pre_tbtt_timer))
		mt7601u_start_pre_tbtt_timer(dev);
	if (!en)
		mt7601u_stop_pre_tbtt_timer(dev);
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
	u8 old_mask = dev->beacon_mask;

	/* Stop pre-TBTT timer/work before mutating beacon state. */
	mt7601u_pre_tbtt_enable(dev, false);

	if (!dev->beacon_mask)
		dev->tbtt_count = 0;

	if (enable)
		dev->beacon_mask = 1;
	else
		dev->beacon_mask = 0;

	if (!!old_mask == !!dev->beacon_mask)
		goto out;

	if (dev->beacon_mask)
		mt76_set(dev, MT_BEACON_TIME_CFG,
			 MT_BEACON_TIME_CFG_BEACON_TX |
			 MT_BEACON_TIME_CFG_TBTT_EN |
			 MT_BEACON_TIME_CFG_TIMER_EN);
	else
		mt76_clear(dev, MT_BEACON_TIME_CFG,
			   MT_BEACON_TIME_CFG_BEACON_TX |
			   MT_BEACON_TIME_CFG_TBTT_EN |
			   MT_BEACON_TIME_CFG_TIMER_EN);

out:
	mt7601u_pre_tbtt_enable(dev, true);
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

void mt7601u_exit_beacon_config(struct mt7601u_dev *dev)
{
	if (!test_bit(MT7601U_STATE_INITIALIZED, &dev->state))
		return;

	dev->beacon_mask = 0;

	mt76_clear(dev, MT_BEACON_TIME_CFG,
		   MT_BEACON_TIME_CFG_TIMER_EN |
		   MT_BEACON_TIME_CFG_SYNC_MODE |
		   MT_BEACON_TIME_CFG_TBTT_EN |
		   MT_BEACON_TIME_CFG_BEACON_TX);

	mt7601u_stop_pre_tbtt_timer(dev);
}
