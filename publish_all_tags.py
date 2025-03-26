import os
import subprocess
import tempfile
import re
from packaging import version
from dotenv import load_dotenv

load_dotenv()

# Print debug info
print("Current environment:")
print(f"GH_TOKEN exists: {'GH_TOKEN' in os.environ}")
print(f"PYPI_TOKEN exists: {'PYPI_TOKEN' in os.environ}")

# Get all available tags from git
result = subprocess.run(["git", "tag", "-l"], capture_output=True, text=True, check=True)
all_tags = result.stdout.strip().split('\n')

# Filter for version tags (starting with 'v')
version_tags = [tag for tag in all_tags if tag.startswith('v')]

# Sort tags by semantic version (oldest first)
sorted_tags = sorted(version_tags, key=lambda x: version.parse(x[1:]))

print(f"Found {len(sorted_tags)} version tags to process:")
for tag in sorted_tags:
    print(f" - {tag}")

# Confirm with user
proceed = input("\nDo you want to publish all these versions to PyPI? (yes/no): ")
if proceed.lower() not in ["yes", "y"]:
    print("Operation cancelled.")
    exit(0)

for tag in sorted_tags:
    print(f"\n=== Processing tag {tag} ===")
    
    # Create a temporary directory
    with tempfile.TemporaryDirectory() as temp_dir:
        # Extract the code at this tag - FIXED VERSION
        archive_result = subprocess.run(
            ["git", "archive", tag, "--format=tar"], 
            capture_output=True, 
            check=True
        )
        
        # Use the output from previous command as input to tar
        subprocess.run(
            ["tar", "-x", "-C", temp_dir], 
            input=archive_result.stdout,
            check=True
        )
        
        # Navigate to the temp directory
        original_dir = os.getcwd()
        os.chdir(temp_dir)
        
        try:
            # Build the package
            print(f"Building package for {tag}...")
            subprocess.run(["poetry", "build"], check=True)
            
            # Publish to PyPI
            print(f"Publishing {tag} to PyPI...")
            subprocess.run(["poetry", "publish", "--username", "__token__", 
                          "--password", os.environ.get("PYPI_TOKEN")], check=True)
            
            print(f"Successfully published {tag}")
        except subprocess.CalledProcessError as e:
            print(f"Error processing {tag}: {e}")
        finally:
            # Return to original directory
            os.chdir(original_dir)

print("\nBack-publishing complete!")

