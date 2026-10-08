"""launch ファイルの構成チェック(ROS 不要).

ROS 2 Humble の IncludeLaunchDescription は launch 引数のスコープを分けないため、
include 先(gazebo_ros の gzserver.launch.py など)で宣言された引数が呼び出し元に漏れる。
gzserver.launch.py は params_file(既定 '')を宣言するので、同じ名前の引数を使うと
Nav2 に空のパスが渡って launch 全体が停止する(実際に起きた不具合の再発防止)。
"""

import ast
import os

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NAV_LAUNCH = os.path.join(SRC, 'kumawatch_navigation', 'launch', 'navigation.launch.py')
PATROL_LAUNCH = os.path.join(SRC, 'kumawatch_mission', 'launch', 'kumawatch_patrol.launch.py')

# gazebo_ros(Humble)の gzserver/gzclient.launch.py が宣言する引数のうち、衝突しやすいもの
GAZEBO_ARGS = {'params_file', 'world', 'verbose', 'pause', 'physics', 'extra_gazebo_args'}


def declared_args(path):
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    names = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'
                and node.args and isinstance(node.args[0], ast.Constant)):
            names.add(node.args[0].value)
    return names


def test_navigation_args_do_not_collide_with_gazebo():
    assert declared_args(NAV_LAUNCH) & GAZEBO_ARGS == set()


def test_patrol_launch_scopes_includes():
    with open(PATROL_LAUNCH, encoding='utf-8') as f:
        text = f.read()
    assert 'GroupAction(scoped=True' in text
