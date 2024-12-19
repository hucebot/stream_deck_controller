import docker
import os
import subprocess

def open_dxl_input(dxl_input_shell=None, dxl_input_docker=docker.from_env()):
    try:
        existing_container = dxl_input_docker.containers.get('dxl_6d')
        if existing_container:
            existing_container.stop()
            existing_container.remove()
    except docker.errors.NotFound:
        pass

    try:
        dxl_input_shell = dxl_input_docker.containers.run(
            "dxl_6d:latest",
            name="dxl_6d",
            command="bash",
            volumes={
                "/tmp/.X11-unix": {"bind": "/tmp/.X11-unix", "mode": "rw"},
                "/dev": {"bind": "/dev", "mode": "rw"},
            },
            environment={
                "QT_X11_NO_MITSHM": "0",
            },
            network_mode="host",
            privileged=True,
            detach=True,
            tty=True
        )

        output = dxl_input_shell.exec_run(
            f'bash -c "source /catkin_ws/devel/setup.bash  && roslaunch dxl_6d_input dxl_6d_input.launch"',
            detach=False
        )

        print(output.output.decode("utf-8"))

    except docker.errors.APIError as e:
        print(f"Error: {e}")
        return None

    return dxl_input_shell