from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['./fake_blimp.sh BurnCream'], name='fake_blimp_1', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh SillyAh'], name='fake_blimp_2', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Turbo'], name='fake_blimp_3', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh GameChamber'], name='fake_blimp_4', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh FiveGuys'], name='fake_blimp_5', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh SuperBeef'], name='fake_blimp_6', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Yoshi'], name='fake_blimp_6', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Luigi'], name='fake_blimp_6', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Geoph'], name='fake_blimp_6', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh ThisGuy'], name='fake_blimp_6', output='screen', shell=True)
    ])
