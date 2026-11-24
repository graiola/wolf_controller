"""Expand the real ROS 2 launch graph without starting robots or processes.

The installed planner launch is deliberately included, so a stale filename or
argument contract fails here. Only the external robot-description/world launch
and xacro output are fixtures. LaunchService runs real deferred timer events with
zero delays; ExecuteProcess is recorded after launch_ros resolves namespaces and
parameter sources.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
import pytest
import yaml


CONTROLLER = Path(__file__).resolve().parents[1]


def _parameters(command):
    """Read actual generated parameter arguments, preserving their precedence."""
    result = {}

    def merge(tree):
        if not isinstance(tree, dict):
            return
        if "ros__parameters" in tree:
            result.update(tree["ros__parameters"])
        else:
            for child in tree.values():
                merge(child)

    for index, argument in enumerate(command[:-1]):
        if argument == "--params-file":
            merge(yaml.safe_load(Path(command[index + 1]).read_text()))
        elif argument == "-p":
            name, value = command[index + 1].split(":=", 1)
            result[name] = yaml.safe_load(value)
    return result


@pytest.fixture
def expand(monkeypatch, tmp_path):
    nodes = {}
    package_lookups = []
    processes = []
    description = tmp_path / "wolf_description_utils"
    (description / "launch").mkdir(parents=True)
    (description / "launch" / "world.launch.xml").write_text("<launch/>")
    timer_init = TimerAction.__init__

    def immediate_timer(action, **kwargs):
        # Keep native deferred event/scoping semantics; only remove wall-clock waits.
        kwargs["period"] = 0.0
        timer_init(action, **kwargs)

    def package_share(_substitution, package):
        package_lookups.append(package)
        if package == "wolf_controller":
            return str(CONTROLLER)
        if package in ("wolf_planner", "wolf_controller_utils"):
            # The real installed launch and configuration are the integration boundary.
            return get_package_share_directory(package)
        if package == "wolf_description_utils":
            return str(description)
        if package == "wolf_gazebo_resources" or package.endswith("_description"):
            return str(tmp_path / package)
        raise AssertionError(f"Unexpected package lookup: {package}")

    def xacro_output(substitution, context):
        command = perform_substitutions(context, substitution.command)
        assert command.startswith("xacro ")
        return '<robot name="launch_test"/>'

    def capture_process(action, context):
        if not isinstance(action, Node):
            processes.append([perform_substitutions(context, item) for item in action.cmd])
            return None
        executable = action.cmd[0][0]
        assert isinstance(executable, ExecutableInPackage)
        package = perform_substitutions(context, executable.package)
        name = perform_substitutions(context, executable.executable)
        command = [name, *[perform_substitutions(context, item) for item in action.cmd[1:]]]
        nodes[action.node_name] = {
            "package": package,
            "executable": name,
            "namespace": action.expanded_node_namespace,
            "command": command,
            "parameters": _parameters(command),
            "remappings": action.expanded_remapping_rules or [],
            # Catch stale or uninstalled Wolf executable names without launching them.
            "executable_path": executable.perform(context)
            if package in ("wolf_controller", "wolf_planner") else None,
        }
        return None

    monkeypatch.setattr(FindPackageShare, "find", package_share)
    monkeypatch.setattr(Command, "perform", xacro_output)
    monkeypatch.setattr(ExecuteProcess, "execute", capture_process)
    monkeypatch.setattr(TimerAction, "__init__", immediate_timer)

    def run(**arguments):
        service = LaunchService(noninteractive=True)
        context = service.context
        context.launch_configurations.update(arguments)
        launch = AnyLaunchDescriptionSource(
            str(CONTROLLER / "launch" / "wolf_controller_bringup.launch.xml")
        ).get_launch_description(context)

        service.include_launch_description(launch)
        assert service.run() == 0
        return nodes, package_lookups, processes

    return run


@pytest.mark.parametrize("robot_name", ["", "spot1", "fleet/spot1"])
@pytest.mark.parametrize("use_sim_time", ["false", "true"])
def test_combined_bringup_namespace_and_clock(expand, robot_name, use_sim_time):
    nodes, _, _ = expand(
        robot_name=robot_name, launch_planner="true", use_sim_time=use_sim_time,
        planner_type="adaptive", planner_autostart="true", plot_node_gui="true",
    )
    namespace = "/" + robot_name
    prefix = namespace.rstrip("/")
    planner = nodes[prefix + "/wolf_planner"]
    assert planner["package"] == "wolf_planner"
    assert planner["executable"] == "wolf_planner_node"
    assert planner["parameters"]["planner_type"] == "adaptive"
    assert planner["parameters"]["autostart"] is True
    assert not any(name.endswith("/wolf_planner_rviz") for name in nodes)

    for node in nodes.values():
        assert node["namespace"] == namespace
        assert node["parameters"]["use_sim_time"] is (use_sim_time == "true")

    state_publisher = nodes[prefix + "/robot_state_publisher"]
    assert state_publisher["parameters"]["frame_prefix"] == (robot_name + "/" if robot_name else "")
    assert state_publisher["remappings"] == []  # Relative joint_states already uses the namespace.
    for name in ("joint_state_broadcaster_spawner", "wolf_controller_spawner"):
        command = nodes[prefix + "/" + name]["command"]
        assert command[command.index("-c") + 1] == prefix + "/controller_manager"
    assert nodes[prefix + "/keyboard_node"]["executable_path"].endswith(
        "/wolf_controller/keyboard_node")
    assert nodes[prefix + "/wolf_plot_node"]["executable_path"].endswith(
        "/wolf_controller/plot_node")


def test_planner_defaults_and_config_overrides(expand, tmp_path):
    extra_params = tmp_path / "planner.yaml"
    extra_params.write_text("/**:\n  ros__parameters:\n    startup_timeout: 42.0\n")
    files = {
        name: str(tmp_path / name)
        for name in ("urdf_file", "task_file", "reference_file", "gait_file")
    }
    nodes, _, _ = expand(
        launch_planner="true", robot_model="go1", planner_params_file=str(extra_params),
        **{"planner_" + name: value for name, value in files.items()},
    )
    planner = nodes["/wolf_planner"]
    assert planner["parameters"]["planner_type"] == "default"
    assert planner["parameters"]["autostart"] is False
    assert planner["parameters"]["startup_timeout"] == 42.0
    for name, value in files.items():
        assert planner["parameters"][name] == value
    parameter_paths = [
        planner["command"][i + 1] for i, arg in enumerate(planner["command"])
        if arg == "--params-file"
    ]
    assert parameter_paths[-1] == str(extra_params)


def test_planner_bundled_model_files(expand):
    nodes, _, _ = expand(launch_planner="true", robot_model="go1")
    config = Path(get_package_share_directory("wolf_planner")) / "config" / "robots" / "go1"
    params = nodes["/wolf_planner"]["parameters"]
    for name, filename in (
        ("urdf_file", "go1.urdf"), ("task_file", "task.info"),
        ("reference_file", "reference.info"), ("gait_file", "gait.info"),
    ):
        assert params[name] == str(config / filename)
        assert Path(params[name]).is_file()


@pytest.mark.parametrize("robot_name", ["", "spot1"])
def test_controller_only_does_not_resolve_planner(expand, robot_name):
    nodes, package_lookups, processes = expand(robot_name=robot_name, gazebo="false", spawn_robot="false")
    prefix = "/" + robot_name if robot_name else ""
    assert "wolf_planner" not in package_lookups
    assert all(node["package"] != "wolf_planner" for node in nodes.values())
    assert prefix + "/wolf_controller_spawner" in nodes
    assert prefix + "/keyboard_node" in nodes
    assert prefix + "/spawn_robot" not in nodes
    assert len(processes) == 1
    assert "replace_yaml_placeholders.py" in " ".join(processes[0])
