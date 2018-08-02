'''
This workflow sends the status of the completed fingerprint operation to the mobile IoT app.

Last Updated: July 20, 2018

Author: Medium One
'''

import MQTT
import Store

status = IONode.get_input('in1')['event_data']['value']

if (status == "pass"):
    msg = "The operation is successful."
else:    
    msg = "The operation is not successful."

# push (bool) to send this notification to push (iOS or Android)
IONode.set_output('out1', {"message": msg, "sms":False,"email":False,"push":True})


