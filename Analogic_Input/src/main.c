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

LOG_MODULE_REGISTER(ble_test);

#define SLEEP_TIME_MS   100  // Define el tiempo de espera en milisegundos entre lecturas de ADC.
#define ADC_NODE        DT_NODELABEL(adc)  // Obtiene el dispositivo ADC del Device Tree.
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define ADC_RESOLUTION  10  // Define la resolución del ADC a 10 bits.
#define ADC_GAIN        ADC_GAIN_1_5  // Configura la ganancia del ADC.
#define ADC_REFERENCE   ADC_REF_INTERNAL  // Establece la referencia interna del ADC.
#define ADC_CHANNEL     0  // Define el número de canal de ADC a usar.
#define ADC_PORT        SAADC_CH_PSELP_PSELP_AnalogInput0  // Define el puerto analógico a usar.
#define ADC_ACQUISITION ADC_ACQ_TIME_DEFAULT  // Configura el tiempo de adquisición por defecto para el ADC.

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);  // Obtiene el descriptor del dispositivo ADC.

struct adc_channel_cfg chl0_cfg = {  // Configuración del canal del ADC.
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION,
    .channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = ADC_PORT
#endif
};

int16_t sample_buffer[1];  // Buffer para almacenar la muestra de ADC.
struct adc_sequence sequence = {  // Secuencia de configuración para la lectura del ADC.
    .channels = BIT(ADC_CHANNEL),
    .buffer = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION
};

void main(void)
{
    int ret;

    if (!device_is_ready(adc_dev)) {
        printk("ADC device is not ready\n");
        return;
    }

    if (!device_is_ready(led.port)) {
        printk("LED device is not ready\n");
        return;
    }

    ret = adc_channel_setup(adc_dev, &chl0_cfg);
    if (ret) {
        printk("ADC setup failed with error %d\n", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return;
    }

    while (1) {
        ret = adc_read(adc_dev, &sequence);
        if (ret) {
            printk("ADC reading failed with error %d\n", ret);
            return;
        }

        int32_t mv_value = sample_buffer[0];
        int32_t adc_vref = adc_ref_internal(adc_dev);
        adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

        uint32_t delay_ms = mv_value;

        gpio_pin_set_dt(&led,1);
        k_msleep(delay_ms);
        gpio_pin_set_dt(&led,0);
        k_msleep(delay_ms);

        printk("ADC Value in mV: %d, Delay: %d ms\n", mv_value, delay_ms);
    }
}