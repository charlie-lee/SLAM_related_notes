#!/bin/sh

filename=SLAM_related_notes
ext=pdf
hl_style=tango # {pygments, tango, espresso, zenburn, kate, monochrome, breezedark, haddock}

pandoc -s -f markdown+tex_math_single_backslash -t latex main.md chapters/*.md --pdf-engine=xelatex --highlight-style=$hl_style --filter pandoc-citeproc -o $filename.$ext

#cp $filename.$ext ~/sf_D_DRIVE

## sync to windows (use git instead)
#rsync -avh ../SLAM_related_notes ~/Workspace/cygwinWorkspace/markdown
git clone . ~/Workspace/cygwinWorkspace/markdown/$filename
