# Automatic code formatting

Assume you want to follow the [Google C++ Style Guide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html) for C++ and [PEP8](https://www.python.org/dev/peps/pep-0008/) for Python. Instead of manually formatting your code, there are already tools for doing this automatically. You can even make them format your code automatically every time you save.

## Tools
### clang-format
Formats code according to several different C++ style guides (Google by default). To install, download the latest version of Clang from the [LLVM Downloads Page](http://llvm.org/releases/download.html). Untar it to ~/local and then add the bin folder to your path in your .bashrc:

```bash
# Clang
export PATH=~/local/clang+llvm-3.6.0-x86_64-linux-gnu/bin:$PATH
```

### yapf
Formats code according to PEP8 or the Google Python style guide. See [yapf](https://github.com/google/yapf). To install, just run
```bash
sudo pip install yapf
```

## Vim setup
Install [Vundle](https://github.com/gmarik/Vundle.vim) if you don't already have it. This is useful for installing vim plugins.

Add these two lines to your .vimrc, between `vundle#begin()` and `vundle#end()`.

```vim
Plugin 'rhysd/vim-clang-format'
Plugin 'mindriot101/vim-yapf'
```

Then install the plugins by opening `vim` and typing `:PluginInstall`

To auto-format C++ code on save, add this to your .vimrc:
```vim
let g:clang_format#auto_format = 1
```

Otherwise, you can just type `:ClangFormat` while editing a C++ file.

To format Python code, type `:Yapf` while editing a file.
