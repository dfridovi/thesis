"""
Launch file to start all processes for thesis demo.
"""

from fabric.api import env, run

env.hosts = ["squirrel@10.9.160.238"]
env.password = "asdf"

def test():
    run("printenv", warn_only=True)
