#!/bin/bash

# Exit on error
set -e

echo "Git Repository Repair Tool"
echo "------------------------"

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "Error: Not in a git repository root directory"
    echo "Please run this script from the root of your git repository"
    exit 1
fi

# Store the problematic commit hash
BAD_COMMIT="52196a83247ce76c925d15ac6bb2fe45312e68b3"

echo "1. Creating backup..."
BACKUP_DIR="../git_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
cp -r .git "$BACKUP_DIR/"
echo "Backup created at: $BACKUP_DIR"

echo "2. Checking repository status..."
git fsck --full 2>git_fsck_errors.log || true
echo "FSCheck errors saved to git_fsck_errors.log"

echo "3. Attempting to fix objects..."
# Try to recover objects from remote
echo "Fetching objects from remote..."
git fetch origin --prune || true

# Try to fix packed refs
echo "Fixing packed refs..."
mv .git/packed-refs .git/packed-refs.bak || true
git gc --prune=now || true

# Try to recover the specific bad object
echo "Attempting to recover bad object: $BAD_COMMIT"
git update-ref -d refs/original/refs/heads/master || true
git fetch origin refs/heads/*:refs/heads/* --force || true

echo "4. Verifying repository..."
git fsck --full

echo "5. Final cleanup..."
git gc --aggressive --prune=now

echo "Done!"
echo ""
echo "If problems persist, try these commands:"
echo "1. Clone fresh and copy changes:"
echo "   git clone <repository-url> ../fresh_repo"
echo "   cp -r .git/refs/heads/* ../fresh_repo/.git/refs/heads/"
echo ""
echo "2. Or try to skip the bad commit:"
echo "   git reset --hard HEAD~1"
echo "   git pull --force"
echo ""
echo "3. Or create new branch from last good commit:"
echo "   git checkout -b recovery <last-good-commit>"
echo "   git cherry-pick <commits-to-recover>" 