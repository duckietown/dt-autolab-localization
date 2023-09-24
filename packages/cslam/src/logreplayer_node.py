#!/usr/bin/env python3
import subprocess

if __name__ == "__main__":
    # Define the command to run
    command = "lcm-logplayer /data/log --lcm-url=udpm://239.255.4.196:7667?ttl=1"

    while True:
        try:
            # Run the command, and wait for it to finish before proceeding
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running the command: {e}")
