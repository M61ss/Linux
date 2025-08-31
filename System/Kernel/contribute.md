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

## Create the patch

To create a .patch file run:

```shell
git add <...>
git commit <...> --signoff
git format-patch -M HEAD~n
```

> [!NOTE]
>
> In `HEAD~n` n is the number of commits you want to include.

## Check the patch

You should always check the correctness of your patch:

```shell
./scripts/checkpatch.pl --strict /path/to/your/patch.patch
```

### Common issue: signature

If you forgot to sign your commit, you can simply do that:

```shell
# In case of single commit
git commit --amend --signoff
# In case of multiple commits
git rebase --signoff HEAD~N
```

## Send the patch

In order to know to who send your email, run:

```shell
./scripts/get_maintainer.pl --separator=, --no-rolestats path/to/patch/dir/*.patch
```

Then, you can send the email using git:

> [!DANGER]
>
> It is very important to be careful before send the patch, so remove from the following command `--dry-run` flag only when you are sure that all is correct.

```shell
git send-email --dry-run /path/to/patch/dir --to=<reciever1_email>@gmail.com,<reciever2_email>@gmail.com --cc="<carbon1_copy>@gmail.com,<carbon2_copy>@gmail.com"
```

> [!WARNING]
>
> Starting from 2022 Google disabled support for potentially dangerous app (for example old apps, like git send-email), so you need to open your account google, type in "search" label "password" and open "App password". Here, add a name for the app which require a password (for example, in this case "git-send-email" could be a good name). Then, copy the password that google prompts. Use that password to access from git send-email. Remember that this password is composed by 16 chars, so whitespaces have not to be included inserting password.

