from launch import LaunchDescription
from launch_ros.actions import Node
#封装终端指令相关类--------------
#from launch.actions import ExecuteProcess
#from launch.substitutions import Node
#参数声明与获取-----------------
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration
#文件包含相关-------------------
#from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#分组相关-----------------------
#from launch_ros.actions import PushRosNamespace
#from launch.actions import GroupAction
#事件相关----------------------
#from launch.event_handlers import OnProcessStart,OnProcessExit
#from launch.actions import ExecuteProcess,RegisterEventHandler,LogInfo
#获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os

"""
需求：演示node的使用
    def __init__(
        self, *,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
    参数说明
    :param: package 被执行的程序所属的功能包；
    :param: executable 可执行程序；
    :param: name 节点名称；
    :param: namespace 节点命名空间；
    :param: exec_name 设置程序标签；
    :param: parameters 设置参数；
    :param: remappings 话题重映射
    :param: ros_arguments 为节点传参；
    传参格式：--ros-args xx yy zz
    :param: arguments 为节点传参；
    传参格式：xx yy zz --ros-args 
"""
def generate_launch_description():
    turtle1 = Node(
        package= "turtlesim", 
        executable="turtlesim_node",
        exec_name= "my_label",
        ros_arguments=["--remap","__ns:=/t2"])
    turtle2 = Node(
        package= "turtlesim", 
        executable="turtlesim_node",
        #respawn = True,自动重启
        name= "haha",
        #方式一
        # parameters=[{"background_r":255,"background_g":0,"background_b":0}]
        #方式二（更常用），读取yaml文件(通过yaml文件的绝对路径来读取)
        #parameters=["/home/jzh/learn_ros2/ws02_tools/install/cpp01_launch/share/cpp01_launch/config/haha.yaml"]
        #优化：动态获取文件路径
        parameters=[os.path.join(get_package_share_directory("cpp01_launch"),"config","haha.yaml")]
        )
    return LaunchDescription([turtle2])