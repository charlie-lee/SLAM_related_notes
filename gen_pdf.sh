#!/bin/sh
filename=SLAM_related_notes
ext=pdf
# {pygments, tango, espresso, zenburn, kate, monochrome, breezedark, haddock}
hl_style=tango
options=$@ # input arguments
str_echo="input options: gen, preview, clone"

if [ -z "$options" ]; then
    echo $str_echo
else
    for option in $options; do
        if [ "$option" = "gen" ]; then
            pandoc -s -f markdown+tex_math_single_backslash -t latex \
                   main.md chapters/*.md  --pdf-engine=xelatex \
                   --template=/home/charlieli/.pandoc/default.latex \
                   --highlight-style=$hl_style --filter pandoc-citeproc \
                   -o $filename.$ext
        elif [ "$option" = "preview" ]; then
            cp $filename.$ext ~/sf_D_DRIVE
        elif [ "$option" = "clone" ]; then
            # remove dst dir and clone the current one
            rm -rf ~/Workspace/cygwinWorkspace/markdown/$filename
            git clone . ~/Workspace/cygwinWorkspace/markdown/$filename
        else
            echo $str_echo
            break
        fi
    done
fi

## sync to windows (use git instead)
#rsync -avh ../SLAM_related_notes ~/Workspace/cygwinWorkspace/markdown
