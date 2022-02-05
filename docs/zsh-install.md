## Fast zsh-install & setup
We recommend using zsh. You can install it by just copy pasting this one command:
```
sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
     -p git \
     -p ssh-agent \
     -p https://github.com/zsh-users/zsh-autosuggestions \
     -p https://github.com/zsh-users/zsh-completions
```
Afterwards

     git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
     git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
     vim $HOME/.zshrc
     
Set ZSH_THEME="powerlevel10k/powerlevel10k" in ~/.zshrc

Set plugins=( 
    # other plugins...
    zsh-autosuggestions
)

Finally,
```
source $HOME/.zshrc
```
and follow the setup wizard
