let data_dir = has('nvim') ? stdpath('data') . '/site' : '~/.vim'
if empty(glob(data_dir . '/autoload/plug.vim'))
  silent execute '!curl -fLo '.data_dir.'/autoload/plug.vim --create-dirs  https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim'
  autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

call plug#begin('~/.vim/vim-plug')
Plug 'airblade/vim-gitgutter'
Plug 'bfrg/vim-cpp-modern'
Plug 'christoomey/vim-tmux-navigator'
Plug 'edkolev/tmuxline.vim'
Plug 'github/copilot.vim'
Plug 'godlygeek/tabular'
Plug 'gruvbox-community/gruvbox'
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
Plug 'junegunn/fzf.vim'
Plug 'LunarWatcher/auto-pairs'
Plug 'neoclide/coc.nvim', {'branch': 'release'}
Plug 'plasticboy/vim-markdown'
Plug 'preservim/nerdcommenter'
Plug 'preservim/nerdtree'
Plug 'preservim/tagbar'
Plug 'qpkorr/vim-bufkill'
Plug 'ryanoasis/vim-devicons'
Plug 'sainnhe/sonokai'
Plug 'tiagofumo/vim-nerdtree-syntax-highlight'
Plug 'tmhedberg/SimpylFold'
Plug 'tmux-plugins/vim-tmux'
Plug 'tpope/vim-fugitive'
Plug 'tpope/vim-surround'
Plug 'vim-airline/vim-airline'
Plug 'vim-airline/vim-airline-themes'
Plug 'Yggdroot/indentLine'
call plug#end()

" General
filetype plugin on
filetype indent on
set autoread
set so=10
set wildmode=longest:full,full
set wildmenu
set backspace=eol,start,indent
set whichwrap+=<,>,h,l
set ignorecase
set smartcase
set hlsearch
set incsearch
set inccommand=nosplit
set cindent
set showmatch
set mat=2
set noerrorbells
set novisualbell
set t_vb=
set tm=500
set nobackup
set nowb
set noswapfile
set encoding=utf-8
set noshowmode
set foldmethod=syntax
set nofoldenable
set conceallevel=0
set shortmess+=c   " Shut off completion messages
set belloff+=ctrlg " If Vim beeps during completion
set hidden

au CursorHold,CursorHoldI * checktime
set updatetime=100

let g:SimpylFold_docstring_preview=0
let g:SimpylFold_fold_docstring=0
let b:SimpylFold_fold_docstring=0
let g:SimpylFold_fold_import=0
let b:SimpylFold_fold_import=0
function! NeatFoldText()
    let suba = getline(v:foldstart)
    let foldmarkerpat = join(map(split(&l:foldmarker,','), "v:val.'\\d\\='"), '\|')
    let suba = substitute(suba, foldmarkerpat, '', 'g')
    let suba = substitute(suba, '\s*$', '', '')
    let lines = v:foldend - v:foldstart + 1
    let text = suba
    let fillchar = matchstr(&fillchars, 'fold:\zs.')
    if strlen(fillchar) == 0
        let fillchar = '-'
    endif
    let lines = repeat(fillchar, 4).' ' . lines . ' lines '.repeat(fillchar, 3)
    if has('float')
        let nuw = max([float2nr(log10(line('$')))+3, &numberwidth])
    else
        let nuw = &numberwidth
    endif
    let n = winwidth(winnr()) - &foldcolumn - nuw - strlen(lines)
    let text = text[:min([strlen(text), n])]
    if text[-1:] != ' '
        if strlen(text) < n
            let text .= ' '
        else
            let text = substitute(text, '\s*.$', '', '')
        endif
    endif
    let text .= repeat(fillchar, n - strlen(text))
    let text .= lines
    return text
endfunction
set foldtext=NeatFoldText()

" Interface
syntax on
set mouse=nv
let &t_SI.="\e[5 q" "SI = INSERT mode
let &t_SR.="\e[4 q" "SR = REPLACE mode
let &t_EI.="\e[1 q" "EI = NORMAL mode
if has("autocmd")
  au VimEnter,InsertLeave * silent execute '!echo -ne "\e[2 q"' |
  au InsertEnter,InsertChange *
    \ if v:insertmode == 'i' |
    \   silent execute '!echo -ne "\e[5 q"' |
    \ elseif v:insertmode == 'r' |
    \   silent execute '!echo -ne "\e[3 q"' |
    \ endif |
  au VimLeave * silent execute '!echo -ne "\e[ q"' |
endif
if has("clipboard")
  set clipboard=unnamed
  if has("unnamedplus")
    set clipboard+=unnamedplus
  endif
endif
set number relativenumber
augroup numbertoggle
  autocmd!
  autocmd BufEnter,FocusGained,InsertLeave,WinEnter * if &nu | set rnu   | endif
  autocmd BufLeave,FocusLost,InsertEnter,WinLeave   * if &nu | set nornu | endif
augroup END
set cursorline
set background=dark
set laststatus=2

if has('termguicolors')
  let &t_8f = "\<Esc>[38:2:%lu:%lu:%lum"
  let &t_8b = "\<Esc>[48:2:%lu:%lu:%lum"
  set termguicolors
  "set term=xterm-256color
endif

let g:python_highlight_all=1
let g:indentLine_char='|'
let g:tmuxline_powerline_separators=1
let g:vim_markdown_conceal=0
let g:vim_markdown_conceal_code_blocks=0
autocmd VimEnter * redraw!
autocmd BufNewFile,BufRead *.launch set syntax=xml

" Color Scheme and Theme
let g:sonokai_style = 'default'
let g:sonokai_enable_italic=1
let g:sonokai_disable_italic_comment=1
let g:airline_powerline_fonts=1
let g:airline#extensions#tabline#enabled=1
let g:airline#extensions#tabline#left_sep = ' '
let g:airline#extensions#tabline#left_alt_sep = '|'
let g:airline#extensions#tabline#formatter = 'unique_tail'
let g:airline#extensions#syntastic#enabled=1
let g:airline_theme='sonokai'
colorscheme sonokai

" Indentation
set tabstop=2
set softtabstop=2
set shiftwidth=2
set expandtab
set smarttab
set ai
set si
set wrap
set colorcolumn=120
set sidescroll=1
set sidescrolloff=50
set breakindent
set breakindentopt=sbr
let &showbreak = '↪>  '
setl cino+=(0 " for function arg alignment

" Navigation
nnoremap J <PageDown>
nnoremap K <PageUp>
vnoremap J :m '>+1<CR>gv=gv
vnoremap K :m '<-2<CR>gv=gv

" Gitgutter always show sign column
set signcolumn=yes

" Remove all trailing whitespace by pressing F5
nnoremap <F5> :let _s=@/<Bar>:%s/\s\+$//e<Bar>:let @/=_s<Bar><CR>

" Remove highlight after search with two enters
nnoremap <silent> <CR> :nohlsearch<CR><CR>

" Remove Ex mode via Shift-Q
nnoremap Q <Nop>

" Split Screen Navigation
set splitright
set splitbelow
nnoremap <C-J> <C-W><C-J>
nnoremap <C-K> <C-W><C-K>
nnoremap <C-L> <C-W><C-L>
nnoremap <C-H> <C-W><C-H>

" FZF disable :W
command! -nargs=* W w

" Tagbar
nmap <C-b> :TagbarToggle<CR>
let g:tagbar_sort=0
let g:tagbar_width=30

" Buffer Navigation
nnoremap <Tab> :bnext<CR>
nnoremap <S-Tab> :bprevious<CR>
nnoremap <leader>x :bp<CR>:bd #<CR>

" C++ Highlighting
let g:cpp_attributes_highlight=1
let g:cpp_member_highlight=1
let g:cpp_simple_highlight=1

" NERDTree Config
let g:NERDTreeWinSize=25
map <C-n> :NERDTreeToggle<CR>
autocmd StdinReadPre * let s:std_in=1
autocmd VimEnter * if argc() == 0 && !exists("s:std_in") | NERDTree | endif
autocmd StdinReadPre * let s:std_in=1
autocmd VimEnter * if argc() == 1 && isdirectory(argv()[0]) && !exists("s:std_in") | exe 'NERDTree' argv()[0] | wincmd p | ene | exe 'cd '.argv()[0] | endif
autocmd bufenter * if (winnr("$") == 1 && exists("b:NERDTree") && b:NERDTree.isTabTree()) | q | endif

" Coc.nvim
function! s:check_back_space() abort
  let col = col('.') - 1
  return !col || getline('.')[col - 1]  =~ '\s'
endfunction

inoremap <silent><expr> <TAB>
\ coc#pum#visible() ? coc#pum#next(1):
\ <SID>check_back_space() ? "\<Tab>" :
\ coc#refresh()
inoremap <expr> <S-TAB> coc#pum#visible() ? coc#pum#prev(1) : "\<C-h>"
inoremap <expr> <cr> coc#pum#visible() ? coc#_select_confirm() : "\<CR>"
