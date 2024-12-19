import docker, os, subprocess

def open_dashboard_wifi(wifi_dashboard_shell=None, wifi_dashboard_docker=docker.from_env(), config_file="cdonoso_config.yaml", layout_file="cdonoso_layout.yaml"):
    subprocess.run(["xhost", "+local:root"], check=True)

    try:
        wifi_dashboard_docker.containers.get('wifi_dashboard').stop()
        wifi_dashboard_docker.containers.get('wifi_dashboard').remove()
    except:
        pass

    package_env = os.getenv("PACKAGES_DIR")

    wifi_dashboard_shell = wifi_dashboard_docker.containers.run(
        "dashboard_ros2:latest",
        name="wifi_dashboard",
        command="bash",
        volumes={
            "/dev/shm": {"bind": "/dev/shm", "mode": "rw"},
            "/tmp/.X11-unix": {"bind": "/tmp/.X11-unix", "mode": "rw"},
            package_env: {"bind": "/ros_ws/src", "mode": "rw", "consistency": "delegated"},
        },
        environment={
            "QT_X11_NO_MITSHM": "1",
            "DISPLAY": os.getenv("DISPLAY")
        },
        network_mode="host",
        privileged=True,
        detach=True,
        tty=True
    )

    wifi_dashboard_shell.exec_run(
        f'bash -c "source /opt/ros/humble/setup.bash && cd /ros_ws/src/robot_dashboard && python3 dashboard_wifi_monitor.py configs/{config_file} layouts/{layout_file}"',
        detach=True
    )

    return wifi_dashboard_shell
