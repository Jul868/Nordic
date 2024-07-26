// Incluye las librerías necesarias para el kernel de Zephyr, logging, Bluetooth y los servicios Bluetooth.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>

// Registra un módulo de log para este archivo con el nombre 'ble_test'.
LOG_MODULE_REGISTER(ble_test);

// Variable global para indicar si Bluetooth está listo para usar.
volatile bool ble_ready = false;
// Variable global para almacenar el nivel de batería inicial.
uint8_t battery_level = 100;

// Define los datos de publicidad Bluetooth que incluyen banderas y el UUID del servicio de acceso genérico.
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0a, 0x18), /* UUID del servicio GAP */
};

// Callback de conexión Bluetooth
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected");
    }
}

// Callback de desconexión Bluetooth
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
}

// Estructura para definir callbacks de conexión y desconexión.

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

// Función callback que se llama cuando Bluetooth está listo.
void bt_ready(int err)
{
    if (err) {
        // Registra un error si la inicialización de Bluetooth falla.
        LOG_ERR("Bluetooth init failed (err %d)", err);
    }

    // Registra que Bluetooth ha sido inicializado correctamente.
    LOG_INF("Bluetooth initialized\n");
    // Establece la variable ble_ready a true indicando que Bluetooth está listo.
    ble_ready = true;

    // Registra los callbacks de conexión y desconexión.
    bt_conn_cb_register(&conn_callbacks);
}


// Función para inicializar Bluetooth.
int init_ble(void)
{
    LOG_INF("Initializing Bluetooth...");
    int err;

    // Habilita Bluetooth y asigna la función bt_ready como callback.
    err = bt_enable(bt_ready);
    if (err) {
        // Registra un error si no se puede habilitar Bluetooth.
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    return 0;
}

// Función principal del programa.
void main(void)
{
    // Llama a la función para inicializar Bluetooth.
    init_ble();
    // Bucle que espera hasta que Bluetooth esté listo.
    while (!ble_ready) {
        LOG_INF("Waiting for Bluetooth to be ready");
        k_sleep(K_MSEC(1000));
    }
    LOG_INF("Bluetooth is ready");

    int err;
    // Inicia la publicidad Bluetooth con los datos definidos.
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        // Imprime un error si la publicidad no puede iniciarse.
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    // Bucle infinito para simular el uso de batería y actualizar su nivel.
    while (true){
        k_sleep(K_MSEC(1000));

        // Restablece el nivel de batería si cae por debajo del 25%.
        if(battery_level < 25){
            battery_level = 100;
        }else{
            battery_level--;
        }
        // Actualiza el nivel de batería a través del servicio Bluetooth Battery Service (BAS).
        bt_bas_set_battery_level(battery_level);
    }
    // Imprime un mensaje de saludo al final (nunca se alcanza debido al bucle infinito).
    printk("Hello World! %s\n", CONFIG_BOARD);
}
