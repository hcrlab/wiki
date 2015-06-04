# Make your development environment pretty!

## Summary
- Care about the colors your eyes are looking at all day.
- For example, look at solarized.

## Sublime (Linux/OSX/Windows)
- What is it? A modern text editor, extensible with Python plugins.
- [Install](http://www.sublimetext.com/3), [install package manager](https://sublime.wbond.net/installation).
- Package & settings recommendations:
- [Curated list from Greg Smith](http://static.incompl.com/sublime/) (made for sublime2, all work for sublime3).
- [Curated list from Kenneth Reitz](http://www.kennethreitz.org/essays/sublime-text-2-love) (propaganda at top, scroll down for recommendations)
- [Curated list from scotch.io](http://scotch.io/bar-talk/best-of-sublime-text-3-features-plugins-and-settings)
- [Max's settings file](https://github.com/mbforbes/fourteen/blob/master/backup_settings/Preferences.sublime-settings).
- [Mike's settings file](https://drive.google.com/a/cs.washington.edu/file/d/0B69dr0PEJ95qc0RTbUpTQ196NFU/edit).

## Vim
To make vim treat roslaunch files (.launch) as XML, add this to your .vimrc:
```vim
au BufNewFile,BufRead *.launch set filetype=xml
```

See other [vim tips](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/vim.md).

## Git
To use colors in the git command line, add the following to your ~/.gitconfig
```
[color]
  diff = auto
  status = auto
  branch = auto
  interactive = auto
  ui = true
  pager = true
```

### icdiff
icdiff is a side-by-side diff tool for the terminal that highlights changes. Install it from the Github page: [icdiff](https://github.com/jeffkaufman/icdiff). You can use it with git using `git icdiff`.
