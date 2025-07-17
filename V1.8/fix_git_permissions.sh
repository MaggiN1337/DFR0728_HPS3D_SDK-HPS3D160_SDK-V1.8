#!/bin/bash

# Exit on error
set -e

echo "Fixing Git repository permissions..."

# Get current user
CURRENT_USER=$(whoami)

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "Error: Not in a git repository root directory"
    echo "Please run this script from the root of your git repository"
    exit 1
fi

echo "Current user: $CURRENT_USER"
echo "Current directory: $(pwd)"

# Fix ownership
echo "Setting ownership..."
sudo chown -R $CURRENT_USER:$CURRENT_USER .git/

# Fix permissions
echo "Setting directory permissions..."
find .git/ -type d -exec chmod 755 {} \;

echo "Setting file permissions..."
find .git/ -type f -exec chmod 644 {} \;

# Special handling for hooks and other executables
echo "Setting executable permissions..."
find .git/hooks -type f -exec chmod 755 {} \;
[ -f .git/hooks/pre-commit ] && chmod 755 .git/hooks/pre-commit
[ -f .git/hooks/post-commit ] && chmod 755 .git/hooks/post-commit
[ -f .git/hooks/pre-push ] && chmod 755 .git/hooks/pre-push

# Fix pack files if they exist
if [ -d ".git/objects/pack" ]; then
    echo "Setting pack file permissions..."
    chmod 644 .git/objects/pack/*
fi

# Fix refs
echo "Setting ref permissions..."
chmod 644 .git/packed-refs 2>/dev/null || true
find .git/refs -type f -exec chmod 644 {} \;

# Fix config
echo "Setting config permissions..."
chmod 644 .git/config

# Fix HEAD
echo "Setting HEAD permissions..."
chmod 644 .git/HEAD

# Fix index
echo "Setting index permissions..."
chmod 644 .git/index 2>/dev/null || true

echo "Done!"
echo ""
echo "You can now try your git command again"
echo "If you still have problems, try:"
echo "1. git config --local core.sharedRepository group"
echo "2. git config --global core.sharedRepository group" 