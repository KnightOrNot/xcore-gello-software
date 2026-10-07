import os
import subprocess

current_file_path = os.path.abspath(__file__)


def run_docker_container():
    user = os.getenv("USER")
    container_name = f"xcore_gello_software_{user}"
    gello_path = os.path.abspath(os.path.join(current_file_path, "../../"))
    volume_mapping = f"{gello_path}:/xcore-gello-software"

    cmd = [
        "docker",
        "run",
        "--runtime=nvidia",
        "--rm",
        "--name",
        container_name,
        "--privileged",
        "--volume",
        volume_mapping,
        "--volume",
        f"{os.path.expanduser('~')}:/homefolder",
        "--net=host",
        "--volume",
        "/dev/serial/by-id/:/dev/serial/by-id/",
        "-it",
        "xcore-gello-software:latest",
        "bash",
        "-c",
        "pip install -e . dynamixel-sdk && exec bash",
    ]

    subprocess.run(cmd)


if __name__ == "__main__":
    run_docker_container()
