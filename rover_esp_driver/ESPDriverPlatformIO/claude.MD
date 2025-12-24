Create a new command in json_cmd.h, with the same schema. the command number if 400.
it can take the value 0, 1, or 2. by default it is 0
- when it is 0, the data streamed should not be in JSON format. Instead, it must be just like MotionCal was expected it on the serial bus(a code snipped is below). The data should in