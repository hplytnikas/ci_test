local M = {
  "catppuccin/nvim",
  lazy = false, -- make sure we load this during startup if it is your main colorscheme
  priority = 1000, -- make sure to load this before all the other start plugins
  -- commit = "b8d7e08eed9a61eb2f49b9196b01f7f2932735ff",
}

function M.config()
  vim.cmd.colorscheme "catppuccin-mocha"
  vim.cmd "hi Normal guibg=NONE ctermbg=NONE" -- allow transparency
end

return M
