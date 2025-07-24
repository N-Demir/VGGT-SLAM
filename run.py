import os
from pathlib import Path
import socket
import subprocess
import threading
import time

import modal

MODAL_SECRETS = [modal.Secret.from_name("wandb-secret"), modal.Secret.from_name("github-token")]
data_volume = modal.Volume.from_name("data", create_if_missing=True) 
output_volume = modal.Volume.from_name("output", create_if_missing=True)
MODAL_VOLUMES = {
    "/root/data": data_volume,
    "/root/output": output_volume,
}

def dummy_function():
    # Testing whether this could get models downloaded and cuda things prebuilt
    # but needs to be placed into a python function unfortunately so that modal can properly
    # run it with `run_function` and attach a volume
    print("Running dummy function")
    subprocess.run("python main.py --image_folder=~/data/does/not/exist/images", shell=True, cwd=".")


app = modal.App("vggt-slam", image=modal.Image.from_dockerfile(Path(__file__).parent / "Dockerfile")
    # GCloud
    .add_local_file(Path.home() / "gcs-tour-project-service-account-key.json", "/root/gcs-tour-project-service-account-key.json", copy=True)
    .run_commands(
        "gcloud auth activate-service-account --key-file=/root/gcs-tour-project-service-account-key.json",
        "gcloud config set project tour-project-442218",
        "gcloud storage ls"
    )
    .env({"GOOGLE_APPLICATION_CREDENTIALS": "/root/gcs-tour-project-service-account-key.json"})
    .run_commands("gcloud storage ls")
    # SSH server
    .apt_install("openssh-server")
    .run_commands(
        "mkdir -p /run/sshd" #, "echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config", "echo 'root: ' | chpasswd" #TODO: uncomment this if the key approach doesn't work
    )
    .add_local_file(Path.home() / ".ssh/id_rsa.pub", "/root/.ssh/authorized_keys", copy=True)
    # Add Conda (for some reason necessary for ssh-based code running)
    .run_commands("conda init bash && echo 'conda activate base' >> ~/.bashrc")
    # Fix Git
    .run_commands("git config --global pull.rebase true")
    .run_commands("git config --global user.name 'Nikita Demir'")
    .run_commands("git config --global user.email 'nikitde1@gmail.com'")
    # Set CUDA Architecture (depends on the GPU)
    .env({"TORCH_CUDA_ARCH_LIST": "7.5;8.0;8.9"})
    .workdir("/root/workspace")
    # Clone EDGS repository into current directory
    .run_commands("git clone https://github.com/N-Demir/vggt-slam.git . --recursive")
    # Install base requirements
    .run_commands("pip3 install -r requirements.txt")
    # Clone and install modified GTSAM with SL(4) factors
    .run_commands("git clone --depth 1 https://github.com/MIT-SPARK/gtsam_with_sl4.git gtsam")
    .run_commands("cd gtsam && mkdir -p build && cd build")
    .run_commands("cd gtsam/build && cmake .. -DGTSAM_BUILD_PYTHON=ON -DGTSAM_FORCE_STATIC_LIB=ON -DCMAKE_INSTALL_PREFIX=$(pwd)/../install -DCMAKE_POSITION_INDEPENDENT_CODE=ON")
    .run_commands("cd gtsam/build && make -j$(nproc)")
    .run_commands("cd gtsam/build && pip install -e python/")
    # Clone and install Salad
    .run_commands("git clone https://github.com/Dominic101/salad.git")
    .run_commands("pip install -e ./salad")
    # Clone and install VGGT
    .run_commands("git clone https://github.com/N-Demir/vggt.git")
    .run_commands("pip install -e ./vggt")
    # Install current repo in editable mode
    .run_commands("pip install -e .")
    # # Post install, try actually running a demo example to prebuild/download things
    .run_commands("git pull")
    .run_function(dummy_function, secrets=MODAL_SECRETS, volumes=MODAL_VOLUMES, gpu="A100-80GB")
    # Get the latest code
    .run_commands("git pull", force_build=True)
)


LOCAL_PORT = 9090


def wait_for_port(host, port, q):
    start_time = time.monotonic()
    while True:
        try:
            with socket.create_connection(("localhost", 22), timeout=30.0):
                break
        except OSError as exc:
            time.sleep(0.01)
            if time.monotonic() - start_time >= 30.0:
                raise TimeoutError("Waited too long for port 22 to accept connections") from exc
        q.put((host, port))


@app.function(
    timeout=3600 * 24,
    gpu="T4",
    secrets=MODAL_SECRETS,
    volumes=MODAL_VOLUMES
)
def run_server(q):
    with modal.forward(22, unencrypted=True) as tunnel:
        host, port = tunnel.tcp_socket
        threading.Thread(target=wait_for_port, args=(host, port, q)).start()

        # Added these commands to get the env variables that docker loads in through ENV to show up in my ssh
        import os
        import shlex
        from pathlib import Path

        output_file = Path.home() / "env_variables.sh"

        with open(output_file, "w") as f:
            for key, value in os.environ.items():
                escaped_value = shlex.quote(value)
                f.write(f'export {key}={escaped_value}\n')
        subprocess.run("echo 'source ~/env_variables.sh' >> ~/.bashrc", shell=True)

        subprocess.run(["/usr/sbin/sshd", "-D"])  # TODO: I don't know why I need to start this here


@app.function(
    timeout=3600 * 24,
    gpu="T4",
    secrets=MODAL_SECRETS,
    volumes=MODAL_VOLUMES
)
def run_shell_script(shell_file_path: str):
    """Run a shell script on the remote Modal instance."""
    # Run the shell script
    print(f"Running shell script: {shell_file_path}")
    subprocess.run("bash " + shell_file_path, 
                  shell=True, 
                  cwd=".")


@app.function(
    timeout=3600,
    gpu="T4",
    secrets=MODAL_SECRETS,
    volumes=MODAL_VOLUMES,
)
def run(capture_name: str):
    data_volume.reload()
    print(f"Running vggt-slam on {capture_name}")
    subprocess.run(f"python main.py --image_folder=/root/data/{capture_name}/images --log_results --colmap_output=/root/data/{capture_name}/sparse_vggt_slam/0", shell=True)
    data_volume.commit()


@app.local_entrypoint()
def main(server: bool = False, shell_file: str | None = None):   
    if server:
        import sshtunnel

        with modal.Queue.ephemeral() as q:
            run_server.spawn(q)
            host, port = q.get()
            print(f"SSH server running at {host}:{port}")

            ssh_tunnel = sshtunnel.SSHTunnelForwarder(
                (host, port),
                ssh_username="root",
                ssh_password=" ",
                remote_bind_address=("127.0.0.1", 22),
                local_bind_address=("127.0.0.1", LOCAL_PORT),
                allow_agent=False,
            )

            try:
                ssh_tunnel.start()
                print(f"SSH tunnel forwarded to localhost:{ssh_tunnel.local_bind_port}")
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nShutting down SSH tunnel...")
            finally:
                ssh_tunnel.stop()

    if shell_file:
        # Run the shell script on the remote instance
        print(f"Running shell script: {shell_file}")
        run_shell_script.remote(shell_file)