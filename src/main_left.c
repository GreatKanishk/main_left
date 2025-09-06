/*
 * Copyright (c) 2024 Buzzy System
 * Modified from Nordic Semiconductor ASA Peripheral HR sample
 * main_left.c - Left Buzzy (Peripheral only)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/pwm.h>

/* Custom UUIDs */
#define BT_UUID_BUZZY_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_BUZZY_CHAR_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 buzzy_service_uuid = BT_UUID_INIT_128(BT_UUID_BUZZY_SERVICE_VAL);
static struct bt_uuid_128 buzzy_char_uuid = BT_UUID_INIT_128(BT_UUID_BUZZY_CHAR_VAL);

/* Data structure */
struct buzzy_cmd {
	int32_t run_flag;   /* 0 = stop, 1 = run */
	int32_t pwm;        /* >0 = motor ON, 0 = motor OFF */
	int32_t delay_ms;   /* delay time in milliseconds */
};

/* Current command state */
static struct buzzy_cmd current_cmd = {0, 0, 0};

/* Advertising data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BUZZY_SERVICE_VAL),
#if defined(CONFIG_BT_EXT_ADV)
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
#endif /* CONFIG_BT_EXT_ADV */
};

#if !defined(CONFIG_BT_EXT_ADV)
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#endif /* !CONFIG_BT_EXT_ADV */

/* Use atomic variable, 2 bits for connection and disconnection state */
static ATOMIC_DEFINE(state, 2U);

#define STATE_CONNECTED    1U
#define STATE_DISCONNECTED 2U

/* GPIO definitions */
#if defined(CONFIG_GPIO)
/* The devicetree node identifiers for motor and LED */
// #define MOTOR_NODE DT_ALIAS(led0)
#define LED_NODE DT_ALIAS(led1)
#define MOTOR_NODE DT_ALIAS(servo)

// DT_NODE_HAS_STATUS_OKAY(MOTOR_NODE) && 


#if DT_NODE_HAS_STATUS_OKAY(LED_NODE)
#include <zephyr/drivers/gpio.h>
#define HAS_MOTOR_LED 1
static const struct pwm_dt_spec motor = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool led_is_on;


//BLE Radio Power Setting Function
static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_lvl;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_write_tx_power_level *)
			  rsp->data)->status : 0;
		printk("Set Tx power err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	printk("Actual Tx Power: %d\n", rp->selected_tx_power);

	net_buf_unref(rsp);
}


/* Timer for motor control */
static struct k_timer motor_timer;

static int set_motor_duty(int duty_percent)
{
	if (!pwm_is_ready_dt(&motor)) {
		printk("PWM device not ready\n");
		return -ENODEV;
	}

	if (duty_percent < 0) duty_percent = 0;
	if (duty_percent > 100) duty_percent = 100;

	uint32_t period = motor.period; // fixed from devicetree
	uint32_t pulse = (period * duty_percent) / 100;

	int ret = pwm_set_pulse_dt(&motor, pulse);
	if (ret < 0) {
		printk("Failed to set PWM duty %d%% (err %d)\n", duty_percent, ret);
	}
	return ret;
}



static void motor_timer_expiry(struct k_timer *timer)
{
	/* Turn off motor after 1000ms */
	// gpio_pin_set(motor.port, motor.pin, 0);

  set_motor_duty(0);
}





static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static int gpio_setup(void)
{
	int err;

	printk("Checking Motor device...");
	// if (!gpio_is_ready_dt(&motor)) {
	// 	printk("failed.\n");
	// 	return -EIO;
	// }

  if (!pwm_is_ready_dt(&motor)) {
      printk("Error: PWM device is not ready\n");
      return -EIO;
  }
	printk("done.\n");

	printk("Checking LED device...");
	if (!gpio_is_ready_dt(&led)) {
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	// printk("Configuring Motor GPIO pin...");
	// err = gpio_pin_configure_dt(&motor, GPIO_OUTPUT_INACTIVE);
	// if (err) {
	// 	printk("failed.\n");
	// 	return -EIO;
	// }
	// printk("done.\n");

	printk("Configuring LED GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);
	k_timer_init(&motor_timer, motor_timer_expiry, NULL);

	return 0;
}

static void blink_start(void)
{
	printk("Start blinking LED...\n");
	led_is_on = false;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static void blink_stop(void)
{
	struct k_work_sync work_sync;

	printk("Stop blinking LED.\n");
	k_work_cancel_delayable_sync(&blink_work, &work_sync);

	/* Keep LED on */
	led_is_on = true;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
}
#endif /* MOTOR_NODE && LED_NODE */
#endif /* CONFIG_GPIO */

/* GATT Service callbacks */
static ssize_t buzzy_char_write(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len,
				uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(struct buzzy_cmd)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	
	if (len != sizeof(struct buzzy_cmd)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	memcpy(&current_cmd, buf, len);
	
	printk("Received command: run=%d, pwm=%d, delay=%d\n",
		current_cmd.run_flag, current_cmd.pwm, current_cmd.delay_ms);
	
#if defined(HAS_MOTOR_LED)
	if (current_cmd.run_flag == 0 || current_cmd.pwm == 0) {
		/* Stop motor immediately */
		k_timer_stop(&motor_timer);
		set_motor_duty(0);
	} else {
		/* Start motor for 1000ms */
    int bpm = current_cmd.delay_ms;   // app sends BPM directly
		int beat_ms = 60000 / bpm;
		int buzz_ms = 30000 / bpm;
		if (buzz_ms > 1000) buzz_ms = 1000;
		if (buzz_ms > beat_ms) buzz_ms = beat_ms; // safety
		int rest_ms = beat_ms - buzz_ms;

		set_motor_duty(current_cmd.pwm);
		k_timer_start(&motor_timer, K_MSEC(buzz_ms), K_NO_WAIT);
	}
#endif
	
	return len;
}

static ssize_t buzzy_char_read(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &current_cmd, sizeof(current_cmd));
}

/* GATT Service Definition */
BT_GATT_SERVICE_DEFINE(buzzy_service,
	BT_GATT_PRIMARY_SERVICE(&buzzy_service_uuid),
	BT_GATT_CHARACTERISTIC(&buzzy_char_uuid.uuid,
			      BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			      buzzy_char_read, buzzy_char_write, NULL),
);

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
    set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, 0 ,8);
		printk("Connected\n");

		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	(void)atomic_set_bit(state, STATE_DISCONNECTED);
	
#if defined(HAS_MOTOR_LED)
	/* Stop motor if running */
	k_timer_stop(&motor_timer);
	set_motor_duty(0); // Motor OFF
#endif
	
	/* Reset command */
	memset(&current_cmd, 0, sizeof(current_cmd));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* Auth callbacks for pairing */
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	bt_conn_auth_cb_register(&auth_cb_display);

#if !defined(CONFIG_BT_EXT_ADV)

  set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, 4);

	printk("Starting Legacy Advertising (connectable and scannable)\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

#else /* CONFIG_BT_EXT_ADV */
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U,
		.secondary_max_skip = 0U,
		.options = (BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_CODED),
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};
	struct bt_le_ext_adv *adv;

	printk("Creating a Coded PHY connectable non-scannable advertising set\n");
	err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create Coded PHY extended advertising set (err %d)\n", err);

		printk("Creating a non-Coded PHY connectable non-scannable advertising set\n");
		adv_param.options &= ~BT_LE_ADV_OPT_CODED;
		err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
		if (err) {
			printk("Failed to create extended advertising set (err %d)\n", err);
			return 0;
		}
	}

	printk("Setting extended advertising data\n");
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set extended advertising data (err %d)\n", err);
		return 0;
	}

	printk("Starting Extended Advertising (connectable non-scannable)\n");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising set (err %d)\n", err);
		return 0;
	}
#endif /* CONFIG_BT_EXT_ADV */

	printk("Advertising successfully started\n");

#if defined(HAS_MOTOR_LED)
	err = gpio_setup();
	if (err) {
		return 0;
	}

	blink_start();
#endif /* HAS_MOTOR_LED */

	/* Main loop */
	while (1) {
		k_sleep(K_SECONDS(1));

		if (atomic_test_and_clear_bit(state, STATE_CONNECTED)) {
			/* Connected callback executed */

#if defined(HAS_MOTOR_LED)
			blink_stop();
#endif /* HAS_MOTOR_LED */
		} else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED)) {
#if !defined(CONFIG_BT_EXT_ADV)
      set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, 8);
			printk("Starting Legacy Advertising (connectable and scannable)\n");
			err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
					      ARRAY_SIZE(sd));
			if (err) {
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}

#else /* CONFIG_BT_EXT_ADV */
			printk("Starting Extended Advertising (connectable and non-scannable)\n");
			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				printk("Failed to start extended advertising set (err %d)\n", err);
				return 0;
			}
#endif /* CONFIG_BT_EXT_ADV */

#if defined(HAS_MOTOR_LED)
			blink_start();
#endif /* HAS_MOTOR_LED */
		}
	}

	return 0;
}