#!/bin/bash
sshpass -f "./passwordfile" scp -r src/ pi@10.1.1.53:~/arm/
echo "Deployed to 10.1.1.53"