import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription,LaunchService
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    world_name = LaunchConfiguration('world_name', default='world_name').perform(context)
    nav = LaunchConfiguration('nav', default='false').perform(context)
    machine_type = LaunchConfiguration('machine_type', default='ROSOrin_Mecanum').perform(context)
    gui = LaunchConfiguration('gui', default='true').perform(context)

    nav_arg = DeclareLaunchArgument('nav',default_value=nav)
    gui_arg = DeclareLaunchArgument('gui', default_value=gui)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value=use_sim_time)
    world_name_arg = DeclareLaunchArgument('world_name',default_value=world_name)
    machine_type_arg = DeclareLaunchArgument('machine_type', default_value=machine_type)


    robot_gazebo_path = get_package_share_directory('robot_gazebo')


    world = os.path.join(robot_gazebo_path,"worlds", "robocup_home.sdf")
    gz_args = f'-r "{world}"' if gui == 'true' else f'-r -s "{world}"'
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')]),
                launch_arguments=[('gz_args', gz_args)])
    

    ros_gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_path, 'launch/ros_gz_bridge.launch.py')
            ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    spwan_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_path, 'launch/spwan_model.launch.py')
            ),
        launch_arguments={
            'world_name': world_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # spawn_objects                                                                
    spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_path, 'launch/spawn_objects.launch.py')
            ),
        launch_arguments={
            'world_name': world_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    return ([
        use_sim_time_arg,
        world_name_arg,
        machine_type_arg,
        nav_arg,
        gui_arg,
        SetEnvironmentVariable('MACHINE_TYPE', machine_type),
        gz_sim,
        spawn_objects_launch,
        spwan_model_launch,
        ros_gz_bridge_launch,
    ])
    


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])



if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
