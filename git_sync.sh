#!/bin/bash

# Define the branch to sync (default: main)
BRANCH=${1:-main}

echo "Synchronizing repository with remote..."

# Step 1: Fetch all updates from the remote (including branches and tags)
git fetch --all --prune

# Step 2: Pull the latest changes for the given branch
echo "Pulling latest changes from $BRANCH..."
git checkout $BRANCH
git pull origin $BRANCH

# Step 3: Delete local branches that have been merged and are no longer on the remote
echo "Pruning local branches no longer on the remote..."
for branch in $(git branch --merged $BRANCH); do
  # Skip the current branch (to avoid deleting it)
  if [[ "$branch" != "$BRANCH" && "$branch" != "* $BRANCH" ]]; then
    # Delete the local branch if it's not on remote
    git branch -d $branch 2>/dev/null
    git branch -D $branch 2>/dev/null
    echo "Deleted local branch: $branch"
  fi
done

# Step 4: Delete local tags that are not on the remote
echo "Pruning local tags that are not on the remote..."
for tag in $(git tag); do
  if ! git ls-remote --tags origin | grep -q "refs/tags/$tag"; then
    git tag -d "$tag"
    echo "Deleted local tag: $tag"
  fi
done

# Step 5: Clean up deprecated remote branches (branches that are merged and deleted on remote)
echo "Cleaning up remote tracking branches..."
git remote prune origin

echo "Git sync completed successfully!"

# Usage:
# chmod +x git_sync.sh
# git_sync.sh <branch or none for main>