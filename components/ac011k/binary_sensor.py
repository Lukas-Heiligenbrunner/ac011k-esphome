import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import AC011KComponent, CONF_AC011K_ID

BINARY_SENSORS = {
    "plugged": (
        "set_plugged_binary_sensor",
        binary_sensor.binary_sensor_schema(device_class="connectivity"),
    ),
    "charging": (
        "set_charging_binary_sensor",
        binary_sensor.binary_sensor_schema(device_class="battery_charging"),
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AC011K_ID): cv.use_id(AC011KComponent),
        **{cv.Optional(k): v[1] for k, v in BINARY_SENSORS.items()},
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AC011K_ID])
    for key, (setter, _) in BINARY_SENSORS.items():
        if key in config:
            bs = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(hub, setter)(bs))
