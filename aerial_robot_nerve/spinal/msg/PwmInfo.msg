float32 min_pwm
float32 max_pwm
float32 neutral_pwm
float32 min_thrust # signed lower thrust limit used for yaw saturation
float32 idle_thrust # minimum thrust while armed; sets min_pwm on one-way ESCs
float32 force_landing_thrust
uint8 pwm_conversion_mode
spinal/MotorInfo[] motor_info
spinal/MotorInfo[] reverse_motor_info
