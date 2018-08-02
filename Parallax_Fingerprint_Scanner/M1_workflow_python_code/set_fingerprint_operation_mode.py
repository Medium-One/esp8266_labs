'''
This workflow sets the operation mode: Add/Delete/Verify mode for the fingerprint reader

Last Updated: July 20, 2018

Author: Medium One
'''

import MQTT
import Store

adduser = IONode.get_input('in1')['event_data']['value']
deluser = IONode.get_input('in2')['event_data']['value']
vfyuser = IONode.get_input('in3')['event_data']['value']


mode = Store.get("fingerprint_mode")          # Get the stored fingerprint scan mode.
if mode is None:
    Store.set_data("fingerprint_mode", "idle", -1)    
    mode = "idle"
    
if (mode == "idle"):
    if (adduser == 'on'):  
        new_value = "add"
        msg = "Add mode: Please place your finger on reader."
        MQTT.publish_event_to_client('esp32', 'C1') 
    elif (deluser == 'on'):
        new_value = "delete"
        msg = "Delete mode: Delete now."
        MQTT.publish_event_to_client('esp32', 'C2') 
    elif (vfyuser == 'on'):                  
        new_value = "verify"  
        msg = "Verify mode: Please place your finger on reader."
        MQTT.publish_event_to_client('esp32', 'C3') 
elif (mode == "add"): 
    if (adduser == 'off'):  
        new_value = "idle"  
        msg = "Back to Idle Mode."
elif (mode == "delete"): 
    if (deluser == 'off'):  
        new_value = "idle"  
        msg = "Back to Idle Mode."
elif (mode == "verify"): 
    if (deluser == 'off'):  
        new_value = "idle"  
        msg = "Back to Idle Mode."
        
Store.set_data("fingerprint_mode", new_value, -1)

# push (bool) to send this notification to push (iOS or Android)
IONode.set_output('out1', {"message": msg, "sms":False,"email":False,"push":True})

# for debug purpose
log('new value =' )
log(new_value)
log('new message =' )
log(msg)

