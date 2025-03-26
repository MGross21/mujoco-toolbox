import os
import subprocess
from dotenv import load_dotenv

load_dotenv()

# Print debug info
print("Current environment:")
print(f"GH_TOKEN exists: {'GH_TOKEN' in os.environ}")
print(f"PYPI_TOKEN exists: {'PYPI_TOKEN' in os.environ}")

# Use Windows-compatible command construction
cmd = [
    "poetry", "run", "semantic-release", 
    # "--patch", # Force patch version increment
    # "-vv",    # Maximum verbosity
    "--strict",  # Strict mode
    "publish"  # Publish command
]

subprocess.run(cmd, check=True)

