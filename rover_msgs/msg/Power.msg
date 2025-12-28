# Power.msg
# Power telemetry from INA219 sensor
std_msgs/Header header
float32 voltage      # Battery voltage (V)
float32 current      # Current draw (mA)
float32 power        # Power consumption (mW)
