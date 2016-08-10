"""
Home-Assistant Custom component for Laundry Notifications by Nolan Gilley.
"""
import logging
import time
from homeassistant.components import device_tracker, scene, notify
from homeassistant.helpers.event import track_state_change, track_time_change
from homeassistant.const import STATE_HOME

DOMAIN = "laundry"

DEPENDENCIES = ['sensor', 'device_tracker']

# Attribute to tell how much time to wait in "Complete" state before notifying.
CONF_WAIT_TIME = "wait_time"

COMPLETE = "Complete"
RUNNING = "Running"
EMPTY = "Empty"

LAURENS_DEVICE = 'device_tracker.lauren_s6'
NOLANS_DEVICE = 'device_tracker.nolan_phone'



def setup(hass, config):
    """ Sets up the simple alarms. """
    logger = logging.getLogger(__name__)
    logger.info("Starting laundry automation.")
    sensors = ["sensor.washer_status", "sensor.dryer_status"]
    wait_time = config[DOMAIN].get(CONF_WAIT_TIME, 240)
    fluxing = True # keep track of fluxing locally in this component. The input boolean still needs to be 'on'

    def track_complete_status(entity_id, old_state, new_state):
        """ Called when appliance goes from running to complete. """
        state = hass.states.get(entity_id).state
        actually_complete = True
        for i in range(1,wait_time):
            state = hass.states.get(entity_id).state
            if state == COMPLETE:
                time.sleep(1)
            else:
                actually_complete = False
                logger.info("LAUNDRY NOT ACTUALLY COMPLETE!!")
                break
        if actually_complete:
            fluxing = False
            if 'dryer' in entity_id:
                hass.services.call('scene', 'turn_on', {"entity_id":"scene.red"})
                message = "The dryer is complete, please empty it!"
            elif 'washer' in entity_id:
                hass.services.call('scene', 'turn_on', {"entity_id":"scene.blue"})
                message = "The washing machine is complete, please empty it!"
            logger.info("LAUNDRY ACTUALLY COMPLETE!!")
            if hass.states.get(LAURENS_DEVICE).state == STATE_HOME:
                hass.services.call('notify', 'join_lauren', {"message":message})
            if hass.states.get(NOLANS_DEVICE).state == STATE_HOME:
                hass.services.call('notify', 'join', {"message":message})

    def appliance_emptied(entity_id, old_state, new_state):
        """ Called when appliance goes from running to complete. """
        hass.services.call('scene', 'turn_on', {"entity_id":"scene.normal"})
        fluxing = True

    def flux_update(service):
        """ Called when appliance goes from running to complete. """
        if fluxing and hass.states.get('input_boolean.flux_automation').state == 'on':
            hass.services.call('switch', 'flux_update')  

    track_state_change(
            hass, sensors,
            track_complete_status, RUNNING, COMPLETE)
    track_state_change(
            hass, sensors,
            appliance_emptied, COMPLETE, EMPTY)
    track_time_change(hass, flux_update, second=[0,30])
    return True
