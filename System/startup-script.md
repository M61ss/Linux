# Create a startup script 

Create a script in whatever location, then run:

```shell
crontab -e
```

It will open the selected editor. Edit that file adding a line like this:

```shell
@reboot /path/to/script
```
