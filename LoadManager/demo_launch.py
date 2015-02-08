"""
Launch file to start all process for thesis demo.
"""

import spur

shell = spur.LocalShell()
result = shell.run(["echo", "-n", "hello"])
print result.output
