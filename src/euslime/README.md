euslime
=======

Slime for Euslisp


## Quick Start

1. Setup
    ```bash
    # Clone code
    mkdir ~/euslime_ws/src -p
    cd euslime_ws/src
    git clone https://github.com/jsk-ros-pkg/euslime.git
    # Update submodules
    cd euslime
    git submodule init
    git submodule update
    # Install dependencies
    rosdep install -yr --from-paths . --ignore-src
    ```

1. Build
    ```bash
    cd ~/euslime_ws
    catkin config --install
    catkin build

1. Configure your emacs init file

    ```lisp
    ;; ~/.emacs.el
    (add-to-list 'load-path "~/euslime_ws/install/share/euslime")
    (require 'euslime-config)
    (setq inferior-euslisp-program "roseus")
    (slime-setup '(slime-fancy slime-banner slime-repl-ansi-color))
    ```

1. Run

    Source the package

    ```bash
    source ~/euslime_ws/install/setup.bash
    ```

    Then open emacs and type the command:

    ```bash
    M-x euslime
    ```

## Cheat sheet

| On slime buffer | |
| --- | --- |
| [TAB] | completion |
| C-c C-d d |  describe/ help |
| C-c C-d a |  apropos |
| C-c C-d p |  apropos package |
| M-.  |  look for definition |
| C-c [RET] |  macroexpansion |
| ,quit  |  quit session |
| ,restart-inferior-lisp  |  restart session |

| On editing buffers | |
| --- | --- |
| C-c C-c | load expression |
| C-c C-l | load-file |

| On other slime buffers | |
| --- | --- |
| q | quit buffer |
| [RET] | select option |