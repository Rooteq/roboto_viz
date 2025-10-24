# roboto_viz
Visualization gui app for roboto_diffbot project

IMPORTANT TODO: 
- add checking for found dock location - abort if varies much;

Status CAN messages:
  - CAN ID 0x201: Turn on Green LED (robot OK)
  - CAN ID 0x202: Turn on Orange LED (robot warning)
  - CAN ID 0x203: Turn on Red LED (robot error)
  0x204 Turn on Buzzer
  0x205 Turn off Buzzer

Change the OK can message to be sent only once

Great! Actually, don't use the parameter get at the start to load default zone. Just let it be somewhere in the code (tell me where so I can change that). Make sure that if the robot starts at some zone, it loads its polygon points.