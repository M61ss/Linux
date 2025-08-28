# Git setup

## git-email (with Gmail)

```shell
sudo apt install git-email
```

Edit `~/.gitconfig` adding these lines:

```conf
[sendemail] 
    smtpserver = smtp.googlemail.com 
    smtpencryption = tls 
    smtpserverport = 587 
    smtpuser = <your_email>@gmail.com
```

## git format-patch

To create a .patch file run:

```shell
git add <...>
git commit <...>
git format-patch --to=<sender_email>@gmail.com HEAD~..HEAD
```

> [!NOTE]
>
> The `HEAD~` option tells git to create the patch of the latest commit only. if you want to create a patch of your last two or more commits then simply change `HEAD~` to `HEAD~2` or `HEAD~n` where n is the number of commits you want to include.

## Send the patch

```shell
git send-email *.patch --to=<reciever_email>@gmail.com --cc=<carbon_copy>@gmail.com
```
