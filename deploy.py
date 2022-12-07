#!/usr/bin/python3
import subprocess
import os
import sys
import hashlib
import json

pi_ip = "10.1.1.31"

def get_git_files():
    unstaged = subprocess.check_output(["git", "diff", "--name-only"], text=True)
    staged = subprocess.check_output(["git", "diff", "--name-only", "--staged"], text=True)
    return (staged+unstaged).splitlines()

hashes_file = "hashesfile.json"
tracked_dirs = ["src", "tests"]
tracked_files = ["Makefile", "test_runner.sh"]

def find_changed_files():
    all_files = tracked_files
    for d in tracked_dirs:
        for f in os.listdir(d):
            all_files.append(d+"/"+f)
    
    with open(hashes_file, "r+") as f:
        hashes: dict = json.load(f)
    
    changed_files = []
    for file in all_files:
        h = hashlib.sha1(open(file,'rb').read()).hexdigest()
        if not hashes.get(file) == h:
            changed_files.append(file)
            hashes[file] = h

    with open(hashes_file, "w") as f:
        json.dump(hashes, f)
    return changed_files

def transfer_files(files):
    if len(files) == 0:
        print("no files to transfer")
        return

    for file in files:
        print("transfering:", file)
        command = f'sshpass -f "./passwordfile" scp {file} pi@{pi_ip}:~/arm/{file}'
        # print("with command:", command)
        os.system(command)


if __name__ == "__main__":
    if len(sys.argv) == 1:
        transfer_files(find_changed_files())
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