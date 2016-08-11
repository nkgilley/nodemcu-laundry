"""
Home-Assistant Custom component for Laundry Notifications by Nolan Gilley.

This component will monitor the laundry mqtt sensors sensor.washer_status and
sensor.dryer_status.  When the washer or dryer has been complete for the wait_time
specified in the config it will run the washer or dryer complete scene and notify
whoever is home that the laundry is complete.

This component will also handle the flux automation.  For the flux automation to work
you need to add an input boolean called flux_automation and make sure it is enabled.
You also need to have the flux component configured.

Example Config:

laundry:
  wait_time: 300

switch:
  - platform: flux
    lights:
      - light.desk
      - light.lamp

input_boolean:
  flux_automation:
    name: Flux automation
    initial: on
    icon: mdi:lightbulb

scene:
  - name: Normal
    entities:
      light.lamp:
        state: on
        profile: reading
        transition: 2
      light.desk:
        state: on
        profile: relax
        transition: 2
  - name: Red
    entities:
      light.lamp:
        state: on
        rgb_color: [255, 0, 0]
        brightness: 200
      light.desk:
        state: on
        rgb_color: [255, 0, 0]
        brightness: 200
  - name: Blue
    entities:
      light.lamp:
        state: on
        rgb_color: [0, 0, 255]
        brightness: 200
      light.desk:
        state: on
        rgb_color: [0, 0, 255]
        brightness: 200
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

fluxing = True # keep track of fluxing locally in this component. The input_boolean.flux_automation still needs to be 'on'


def setup(hass, config):
    """ Sets up the simple alarms. """
    logger = logging.getLogger(__name__)
    logger.info("Starting laundry automation.")
    sensors = ["sensor.washer_status", "sensor.dryer_status"]
    wait_time = config[DOMAIN].get(CONF_WAIT_TIME, 240)

    def track_complete_status(entity_id, old_state, new_state):
        """ Called when appliance goes from running to complete. """
        actually_complete = True
        for i in range(1, wait_time):
            state = hass.states.get(entity_id).state
            if state == COMPLETE:
                time.sleep(1)
            else:
                actually_complete = False
                logger.info("LAUNDRY NOT ACTUALLY COMPLETE!!")
                break
        if actually_complete:
            global fluxing
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
        """ Called when appliance goes from complete to empty. """
        hass.services.call('scene', 'turn_on', {"entity_id":"scene.normal"})
        global fluxing
        fluxing = True

    def flux_update(service):
        """ Called every 30 seconds to flux the lights. """
        global fluxing
        ib_flux_state = hass.states.get('input_boolean.flux_automation')
        if fluxing and ib_flux_state:
            if ib_flux_state.state == 'on':
                hass.services.call('switch', 'flux_update')  

    track_state_change(
            hass, sensors,
            track_complete_status, RUNNING, COMPLETE)
    track_state_change(
            hass, sensors,
            appliance_emptied, COMPLETE, EMPTY)
    track_time_change(hass, flux_update, second=[0,30])
    return True
