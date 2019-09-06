
# There's usually much more than this in my profile!

$SCRIPTPATH = "C:\Program Files (x86)\Vim"

$VIMPATH    = $SCRIPTPATH + "\vim80\vim.exe"

 

Set-Alias vi   $VIMPATH

Set-Alias vim  $VIMPATH

 

# for editing your PowerShell profile

Function Edit-Profile

{

    vim $profile

}

 

# for editing your Vim settings

Function Edit-Vimrc

{

    vim $home\_vimrc

}
