import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import AC011KComponent, CONF_AC011K_ID

TEXT_SENSORS = {
    "evse_state": "set_evse_state_sensor",
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AC011K_ID): cv.use_id(AC011KComponent),
        **{cv.Optional(k): text_sensor.text_sensor_schema() for k in TEXT_SENSORS},
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AC011K_ID])
    for key, setter in TEXT_SENSORS.items():
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(hub, setter)(sens))
