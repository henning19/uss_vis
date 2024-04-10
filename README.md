# README
This is the README of the visualisation team of the ultrasonicsensor implementation team

generate sshkey
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
copy key by: cat ~/.ssh/id_rsa.pub | xclip -sel clip
add to git hub, by creating new ssh key for this user

$ ssh -T git@github.com

add this prompt: The authenticity of host 'github.com (140.82.121.3)' can't be established.
ECDSA key fingerprint is SHA256:p2QAMXNIC1TJ
Are you sure you want to continue connecting (yes/no)?

choose yes, if user sshkey is the same 
output if sucesseful: > Hi USERNAME! You've successfully authenticated, but GitHub does not
> provide shell access.

now git remote.... should work
