export GZ_SIM_RESOURCE_PATH="$(pwd)/models:$(pwd)/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/bin:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

bind '"\C-b": "gz sim -v 4 -r worlds/minimal.world"'

parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/[\1]/'
}

export PS1="gz \[\033[32m\]\u@\h\[\033[00m\]:\[\033[34m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\]\$ "

echo "Gazebo Harmonic environment ready"
