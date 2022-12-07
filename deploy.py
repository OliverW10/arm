#!/usr/bin/python3
import subprocess
import os
import sys

pi_ip = "10.1.1.31"

def get_git_files():
    unstaged = subprocess.check_output(["git", "diff", "--name-only"], text=True)
    staged = subprocess.check_output(["git", "diff", "--name-only", "--staged"], text=True)
    return (staged+unstaged).splitlines()

def transfer_files(files):
    for file in files:
        print("transfering:", file)
        command = f'sshpass -f "./passwordfile" scp {file} pi@{pi_ip}:~/arm/{file}'
        # print("with command:", command)
        os.system(command)

if __name__ == "__main__":
    if len(sys.argv) == 1:
        transfer_files(get_git_files())
    else:
        files = []
        for name in sys.argv[1:]:
            if os.path.isfile(name):
                files.append(name)
            elif os.path.isfile("src/"+name):
                files.append("src/"+name)
            else:
                print(f"couldnt find file: {name}")
        transfer_files(files)