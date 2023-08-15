"""
invoke tool configuration

Install invoke: `pip install invoke`

More info: https://www.pyinvoke.org/
"""

import os

from invoke import Collection, Config, Exit, task

PROJECT_NAME = "pico-dev"

@task
def pico_dev_build(ctx):
    """Build image"""
    ctx.run(f"docker build -f .devcontainer/pico-dev.Dockerfile -t {PROJECT_NAME}:latest .")

@task
def pico_dev_run(ctx):
    """Run container"""
    ctx.run(f"docker run --rm -it --privileged  --hostname {PROJECT_NAME} --platform linux/amd64 {PROJECT_NAME}:latest /bin/bash")

# Add all tasks to the namespace
ns = Collection(pico_dev_build, pico_dev_run)

# Configure every task to act as a shell command
#   (will print colors, allow interactive CLI)
# Add our extra configuration file for the project
config = Config(defaults={"run": {"pty": True}})
ns.configure(config)
