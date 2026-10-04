import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import somfy_ns
from .cover import SomfyTimeBasedCover

DEPENDENCIES = ["somfy"]

CONF_COVER_ID = "cover_id"

SomfyInvertSwitch = somfy_ns.class_("SomfyInvertSwitch", switch.Switch, cg.Component)

# Runtime toggle for a cover's invert_direction. The stored choice wins over
# the cover's YAML value once the switch has been used.
CONFIG_SCHEMA = (
    switch.switch_schema(
        SomfyInvertSwitch,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:swap-vertical",
        default_restore_mode="RESTORE_DEFAULT_OFF",
    )
    .extend({cv.Required(CONF_COVER_ID): cv.use_id(SomfyTimeBasedCover)})
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    cover = await cg.get_variable(config[CONF_COVER_ID])
    cg.add(var.set_cover(cover))
