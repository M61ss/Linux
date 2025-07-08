local vim = vim
local Plug = vim.fn['plug#']

vim.call('plug#begin')

vim.call('plug#end')

home = os.getenv("HOME")
package.path = home .. "/.config/nvim/?.lua;" .. package.path

require "common"
