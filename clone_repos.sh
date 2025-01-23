#!/bin/bash

YAML_FILE="/config-data/config_input.yaml"

# Install APT dependencies
echo "Installing APT dependencies..."
yq '.dependencies[] | select(.apt) | .apt' "$YAML_FILE" | while read -r package; do
    echo "Installing $package..."
    sudo apt-get install -y "$package"
done

# Install pip dependencies
echo "Installing pip dependencies..."
yq '.dependencies[] | select(.pip3) | .pip3' "$YAML_FILE" | while read -r package; do
    echo "Installing $package..."
    pip install "$package"
done

# Clone ROS repositories
echo "Cloning ROS repositories..."
yq '.ros_repositories[] | .owner + "/" + .repo + " " + (.branch_or_hash // "main")' "$YAML_FILE" | while read -r OWNER_REPO BRANCH_HASH; do
    REPO_URL="https://github.com/$OWNER_REPO.git"
    REPO_NAME=$(basename "$OWNER_REPO")

    if [ -d "$REPO_NAME" ]; then
        echo "Repository $REPO_NAME already exists, skipping..."
    else
        echo "Cloning $REPO_URL..."
        git clone "$REPO_URL"
        cd "$REPO_NAME" || exit
        git checkout "$BRANCH_HASH"
        cd ..
    fi
done

echo "Setup complete!"
