from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_description_py'

setup(
    # 包的名称
    name=package_name,
    # 版本号
    version='0.0.0',
    # 参数 exclude=['test'] 意味着在查找包时会排除名为 test 的子包
    packages=find_packages(exclude=['test']),
     # 资源文件配置
    data_files=[
        # 安装ament index文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安装package.xml文件
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件夹下的所有.launch.py文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装urdf文件夹下的所有文件和子文件夹
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
    ],
    # 安装依赖项
    install_requires=['setuptools'],
    # 是否启用zip安装模式
    zip_safe=True,
    # 包的维护者
    maintainer='shakima',
    # 维护者的电子邮件
    maintainer_email='zbohao7@gmail.com',
    description='仿真的学习功能包',
    license='TODO: License declaration',
    # 测试所需的依赖项
    tests_require=['pytest'],
    # 入口点配置，用于设置可执行文件
    entry_points={
        'console_scripts': [
        ],
    },
)
