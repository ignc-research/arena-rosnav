## One line zsh-install & setup
We recommend using zsh. You can install it by just copy pasting this one command:
```
sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
     -p git \
     -p ssh-agent \
     -p https://github.com/zsh-users/zsh-autosuggestions \
     -p https://github.com/zsh-users/zsh-completions
```
Afterwards

```
source $HOME/.zshrc
```
and follow the setup wizard
