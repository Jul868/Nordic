#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(ble_test);

#define BT_UUID_MY_CUSTOM_SERV_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678)
#define BT_UUID_MY_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_MY_CUSTOM_SERV_VAL)
#define BT_UUID_MY_ANALOGIC_CHAR_VAL BT_UUID_128_ENCODE(0x98679657, 0x1234, 0x5678, 0x1234, 0x567812345678)
#define BT_UUID_ANALOGIC_CHAR BT_UUID_DECLARE_128(BT_UUID_MY_ANALOGIC_CHAR_VAL)

volatile bool ble_ready = false;
static char adc_value_str[8];
int adc_value_int = 0;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MY_CUSTOM_SERV_VAL)
};

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_5
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_CHANNEL 0
#define ADC_ACQUISITION ADC_ACQ_TIME_DEFAULT

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static struct adc_channel_cfg adc_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION,
    .channel_id = ADC_CHANNEL,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0
};
int16_t sample_buffer[1];
struct adc_sequence sequence = {
    .channels = BIT(ADC_CHANNEL),
    .buffer = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION,
};

ssize_t read_adc_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);

static void adc_value_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", notifications_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(custom_srv,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MY_CUSTOM_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_ANALOGIC_CHAR, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_adc_value, NULL, adc_value_str),
    BT_GATT_CCC(adc_value_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

ssize_t read_adc_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    const char *val = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, val, strlen(val));
}

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

void init_ble(void) {
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    bt_conn_cb_register(&conn_callbacks);
    LOG_INF("Bluetooth initialized");
    ble_ready = true;
}

int map_adc_value_to_range(int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void adc_read_and_notify(void) {
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device is not ready");
        return;
    }

    if (adc_read(adc_dev, &sequence) == 0) {
        int16_t adc_raw_value = sample_buffer[0];
        int16_t adc_mapped_value = map_adc_value_to_range(adc_raw_value, 0, (1 << ADC_RESOLUTION) - 1, 0, 100);
        snprintf(adc_value_str, sizeof(adc_value_str), "%d", adc_mapped_value);

        const struct bt_gatt_attr *adc_attr = &custom_srv.attrs[1];
        int err = bt_gatt_notify(NULL, adc_attr, adc_value_str, strlen(adc_value_str));
        if (err) {
            LOG_ERR("Failed to send notification (err %d)", err);
        } else {
            LOG_INF("ADC Value: %d (Raw: %d)", adc_mapped_value, adc_raw_value);
        }
    } else {
        LOG_ERR("Failed to read ADC value");
    }
}

int main(void) {
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device is not ready");
        return -1;
    }

    int err = adc_channel_setup(adc_dev, &adc_cfg);
    if (err) {
        LOG_ERR("ADC channel setup failed (err %d)", err);
        return -1;
    }

    init_ble();
    while (!ble_ready) {
        k_sleep(K_MSEC(1000));
    }

    LOG_INF("Bluetooth is ready, start advertising");
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return -1;
    }

    while (true) {
        adc_read_and_notify();
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
