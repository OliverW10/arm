from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient
import argparse

files = ["util.py", "camera_apriltags.py"]

parser = argparse.ArgumentParser("Upload code to the Pi. IP defaults to 10.1.1.53")
parser.add_argument("-ip", "--ip", help="Specify a custom ip")
args = parser.parse_args()

server_ip = "10.1.1.53" if args.ip is None else args.ip
username = "ubuntu"
password = "Raspberry0406"

ssh = SSHClient()
ssh.set_missing_host_key_policy(AutoAddPolicy())
print(f"Connecting to the pi at {server_ip} ... ", end="")
ssh.connect(server_ip, username=username, password=password)
print("Done")

# print("Making file system writable ... ", end="")
# stdout, stdin, stderr = ssh.exec_command(
#     "sudo mount -o remount,rw / ; sudo mount -o remount,rw /boot"
# )
# for line in stderr:
#     print(line)
# exit_status = stdout.channel.recv_exit_status()
# if exit_status != 0:
#     print(f"Something's gone wrong! Error exit status: {exit_status}")
#     quit()
# else:
#     print("Done")

print("Uploading files ... ", end="")
scp = SCPClient(ssh.get_transport())
scp.put(files, recursive=True)
print("Done")

# print("Making file system read-only ... ", end="")
# ssh.exec_command("sudo mount -o remount,ro / ; sudo mount -o remount,ro /boot")
# print("Done")

scp.close()
