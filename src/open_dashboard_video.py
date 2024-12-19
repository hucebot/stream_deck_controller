import docker, os, subprocess


def open_dashboard_video(video_dashboard_shell=None, video_dashboard_docker=docker.from_env(), config_file="cdonoso_config.yaml", layout_file="cdonoso_layout.yaml"):
    subprocess.run(["xhost", "+local:root"], check=True)
    
    try:
        existing_container = video_dashboard_docker.containers.get('video_dashboard')
        if existing_container:
            existing_container.stop()
            existing_container.remove()
    except docker.errors.NotFound:
        pass

    package_env = os.getenv("PACKAGES_DIR")

    video_dashboard_shell = video_dashboard_docker.containers.run(
        "dashboard_ros2:latest",
        name="video_dashboard",
        command="bash",
        volumes={
            "/dev/shm": {"bind": "/dev/shm", "mode": "rw"},
            "/tmp/.X11-unix": {"bind": "/tmp/.X11-unix", "mode": "rw"},
            package_env: {"bind": "/ros_ws/src", "mode": "rw", "consistency": "delegated"},
        },
        environment={
            "QT_X11_NO_MITSHM": "1",
            "DISPLAY": os.getenv("DISPLAY"),
            "ROS_MASTER_URI": os.getenv("ROS_MASTER_URI"),
            "ROS_IP": os.getenv("ROS_IP"),
        },
        network_mode="host",
        privileged=True,
        detach=True,
        tty=True
    )


    video_dashboard_shell.exec_run(
        f'bash -c "source /opt/ros/humble/setup.bash && cd /ros_ws/src/robot_dashboard && python3 video_window.py configs/{config_file} layouts/{layout_file}"',
        detach=True,
    )

    return video_dashboard_shell