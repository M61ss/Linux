# Kernel documentation

## Compile docs

Move to `Documentation` folder and run:

```shell
# for HTML docs
make htmldocs > docs.log 2>&1
# for PDF docs
make pdfdocs > docs.log 2>&1
```

It will print compile output and, first time, also necessary instructions to install deps in docs.log. Copy and paste in the terminal the suggested command to install deps, but pay attention to those start with `python3-`. Those are python packages which you maybe don't want to install as system wide, so you should create a venv, serch the respective package names used by pip and install them in the venv. So, activate this one before starting to compile docs. 
\
For example, probably in the suggested command to install deps there is `python3-yaml`. If you install this from apt, you will have this package installed as system wide. You don't want this, so, searching on internet, you will find that this package exists in pip and its name is `pyyaml`. So, in your venv install that, activate the venv and compile the documentation. If you have all requirements satisfied, it will work.