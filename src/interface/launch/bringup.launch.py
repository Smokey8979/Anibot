import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('interface')
    urdf_file = os.path.join(pkg, 'urdf', 'chassis_fixed.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='interface',
            executable='new_odom',  #odometery
            name='serial_odom_bridge',
            output='screen',
            parameters=[{
                'serial_port':        '/dev/ttyACM0',
                'baudrate':           115200,

                # ── Mechanical constants ────────────────────────────────────
                'encoder_ppr':        28,       # encoder pulses per motor shaft revolution
                'gear_ratio':         19.2,     # gearbox ratio
                'wheel_radius':       0.041,    # metres
                'wheel_base':         0.355,    # metres (CALIBRATED — 360° spin test ±1.1°)

                # ── Drive limits ────────────────────────────────────────────
                'publish_rate':       20.0,     # Hz — odom + TF publish rate
                'max_linear_speed':   0.20,     # m/s — maps to max_pwm
                'max_pwm':            150,

                # ── Safety ──────────────────────────────────────────────────
                'cmd_timeout_ms':     500,      # stop motors if no cmd_vel for 500 ms

                # ── Encoder polarity ────────────────────────────────────────
                # Set invert_left / invert_right ONLY if that encoder physically
                # counts DOWNWARD when the wheel rolls forward.
                # These flags must NOT be used to fix motor direction — fix that
                # at the Arduino level or by swapping motor wire polarity.
                'invert_left':        False,
                'invert_right':       False,

                # ── Wheel imbalance compensation ────────────────────────────
                # left_scale < 1.0 gives left motor less PWM to stop rightward drift.
                # This is a PWM trim, NOT an encoder correction — odom uses raw ticks.
                'left_scale':         0.82,
                'right_scale':        1.0,

                # ── Motor channel assignment ────────────────────────────────
                # swap_motors=True  : Arduino M1 = physical RIGHT, M2 = physical LEFT
                # swap_motors=False : Arduino M1 = physical LEFT,  M2 = physical RIGHT
                #
                # On this robot: M1 connector is wired to the RIGHT motor,
                #                M2 connector is wired to the LEFT  motor.
                # Therefore swap_motors MUST BE True.
                #
                # With swap_motors=False the left_scale=0.82 penalty goes to the
                # RIGHT motor instead of the left, causing the robot to always
                # veer right, AND the encoder assignment is mirrored so the odom
                # reports turns in the OPPOSITE direction to reality → AMCL breaks.
                'swap_motors':        False,     # ← CRITICAL: must be True for this robot

                # ── Soft velocity ramping ───────────────────────────────────
                'ramp_step':          10,       # PWM ramp up:   0 → 150 in 0.75 s
                'ramp_step_stop':     20,       # PWM ramp down: 150 → 0  in 0.40 s
            }]
        ),
    ])
