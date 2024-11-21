# Bash Scripting <!-- omit from toc -->

- [Basics](#basics)
  - [Sample script](#sample-script)
  - [Make script executable](#make-script-executable)
  - [Execute script](#execute-script)
  - [Add a script to PATH](#add-a-script-to-path)
  - [Special variables](#special-variables)
  - [`shift`](#shift)
  - [Comments](#comments)
  - [ShellCheck (static analysis)](#shellcheck-static-analysis)
- [Flow control statements](#flow-control-statements)
  - [`test`](#test)
  - [`test` (strings)](#test-strings)
  - [`test` (numbers)](#test-numbers)
  - [`test` (files)](#test-files)
  - [`test` (logic)](#test-logic)
  - [Operator `[ ]`](#operator--)
  - [`if`](#if)
  - [Pattern matching](#pattern-matching)
  - [Operator \[\[ \]\]](#operator---1)
  - [case](#case)
- [Loops](#loops)
  - [`for`](#for)
  - [`seq`](#seq)
  - [`while`](#while)
  - [Expand filenames](#expand-filenames)
- [Functions](#functions)
  - [Multi-file script](#multi-file-script)
  - [Arrays](#arrays)
  - [getopts](#getopts)
  - [Best practice](#best-practice)
  - [Bash vs Python for scripting](#bash-vs-python-for-scripting)

## Basics

### Sample script

```shell
$ vim script.sh

#!/bin/bash
echo Total number of inputs: $#
echo First input: "$1"
echo Second input: "$2"
exit 0
```

### Make script executable

```shell
$ chmod a+x script.sh 
# or
$ chmod 755 script.sh
```

### Execute script

```shell
$ ./script.sh AAPL GOOGL MSFT
Total number of inputs: 3
First input: AAPL
Second input: GOOGL
```

### Add a script to PATH

```shell
$ export PATH=$PATH:.
$ script.sh
```

### Special variables

- **$0**: Script name.
- **$1, $2,...., $n**: Script command line parameters.
- **$\***: All command line parameters.
- **$@**: All command line parameters.
- **$#**: Number of command line parameters.
- **$$**: PID of the shell which is executing the script.
- **$?**: return value of last executed command.

More at this [link](https://tiswww.case.edu/php/chet/bash/bashref.html#Special-Parameters).

### `shift`

```shell
$ vim test.sh

#!/bin/bash
echo [$#] $*
shift         # erases first parameter
echo [$#] $*
shift 3       # erases first three parameters
echo [$#] $*
```

```shell
./test.sh -a -b -c -d -e -f -g -h
[8] -a -b -c -d -e -f -g -h
[7] -b -c -d -e -f -g -h
[4] -e -f -g -h
```

### Comments

```shell
#!/bin/bash

# This is a comment

exit 0
```

### ShellCheck (static analysis)

[ShellCheck](https://github.com/koalaman/shellcheck).

## Flow control statements

### `test`

> [!IMPORTANT] Do not use test
>
> Consider to use the [operator [ ]](#operator--) instead of `test`.

```shell
$ test 5 -gt 3; echo $?               # 0
$ test 5 -gt 3; echo $?               # 1
$ test "string" == "string"; echo $?  # 0
$ test "string" == "other"; echo $?   # 1
$ test -f /etc/passwd; echo $?        # 0
$ test -d /etc/passwd; echo $?        # 1
```

### `test` (strings)

```
-n STRING           the length of STRING is nonzero
-z STRING           the length of STRING is zero
STRING1 = STRING2   the strings are equal
STRING1 != STRING2  the strings are not equal
```

### `test` (numbers)

```
INT1 -eq INT2  ->  INT1 is equal to INT2
INT1 -ge INT2  ->  INT1 is greater than or equal to INT2
INT1 -gt INT2  ->  INT1 is greater than INT2
INT1 -le INT2  ->  INT1 is less than or equal to INT2
INT1 -lt INT2  ->  INT1 is less than INT2
INT1 -ne INT2  ->  INT1 is not equal to INT2
```

### `test` (files)

```
-d FILE   FILE exists and is a directory
-e FILE   FILE exists
-f FILE   FILE exists and is a regular file
-h FILE   FILE exists and is a symbolic link (same as -L)
-O FILE   FILE exists and is owned by the effective user ID
-r FILE   FILE exists and the user has read access
-s FILE   FILE exists and has a size greater than zero
-w FILE   FILE exists and the user has write access
-x FILE   FILE exists and the user has execute (or search) access
```

### `test` (logic)

```shell
$ test -f /etc/passwd && test -r /etc/passwd; echo $?
0

$ test ! -d /etc/passwd || test ! -w /etc/passwd; echo $?
0
```

**DEPRECATED**, but still existing:

```
! EXPRESSION                  EXPRESSION is false
EXPRESSION1 -a EXPRESSION2    both EXPRESSION1 and EXPRESSION2 are true
EXPRESSION1 -o EXPRESSION2    either EXPRESSION1 or EXPRESSION2 is true
```

*Example*:

```shell
$ test -f /etc/passwd -a -r /etc/passwd; echo $?
0

$ test ! -d /etc/passwd -o ! -w /etc/passwd; echo $?
0
```

### Operator `[ ]`

It works as `test`, but it is more readable.

> [!WARNING] Spaces
>
> Spaces before and after the content of `[ ]` are **mandatory**.

```shell
$ [ -f /etc/passwd ] && [ -r /etc/passwd ]; echo $?
0

$ [ ! -d /etc/passwd ] || [ ! -w /etc/passwd ]; echo $?
0
```

### `if`

```shell
if test condition; then
  # ...
else
  # ...
fi
```

Using `[ ]`:

```shell
if [ condition ]; then
  # ...
else
  # ...
fi
```

`elif`:

```shell
if [ condition ]; then
  # ...
elif [ condition ]; then
  # ...
else
  # ...
fi
```

*Examples*:

```shell
if [ -f "$1" ] && [ -r "$1" ]; then
  echo "$1" is a readable file!
fi
```

```shell
if [ ! -d "$1" ] || [ ! -x "$1" ]; then
  echo "$1" is not an executable directory!
fi
```

```shell
if [ $# -ne 3 ]; then
  echo "params != 3"
else
  echo "params == 3"
fi
```

```shell
if [ $# -lt 2 ]; then
  echo "params < 2"
elif [ $# -lt 4 ]; then
  echo "2 <= params < 4"
else
  echo "params >= 4"
fi
```

It is possible to use a synthetic syntax to create small `if` (same idea of inline-if):

```shell
$ [ 1 –eq 0 ] && echo "pass"           #
$ [ 1 –eq 1 ] && echo "pass"           # pass
$ [ 1 –eq 0 ] || echo "fail"           # fail
$ [ 1 –eq 1 ] || echo "fail"           #
$ [ 1 –eq 1 ] && (echo "pass"; pwd)    # pass /home/nicola
```

`if` can be used also for arbitrary commands:

```shell
if grep some_user /etc/passwd; then
  echo "some_user exists.\n"
else
  echo "some_user does not exist.\n"
fi
```

### Pattern matching

```
String      Pattern  |  Match
---------------------|--------
ABCDEF      A*       |  Yes
                     |
ABCDEF      AB??EF   |  Yes
                     |
ABCDEF      ABCNN*   |  No
```

More info about bash pattern matching [here](https://www.gnu.org/software/bash/manual/html_node/Pattern-Matching.html).

> [!WARNING]
>
> `test` and `[ ]` do not support pattern matching.

### Operator [[ ]]

Basically, it works like `[ ]`, but it supports pattern matching. More info [here](https://mywiki.wooledge.org/BashFAQ/031)

*Example*:

```shell
if [[ "$1" == s?cc* ]]; then
  echo "success"
fi

if [[ "$1" != [0-9]* ]]; then
  echo "success"
fi
```

### case

Useful to avoid lots of `elif`:

```shell
case expression in
  PATTERN_1)
    # ...
    ;;

  PATTERN_2)
    # ...
    ;;

  PATTERN_N)
    # ...
    ;;
  *)
    # ...
    ;;
esac
```

*Examples*:

It is possible to easily check type of provided path:

```shell
case "$1" in
  /*)
    echo "Absolute path"
    ;;
  */*)
    echo "Relative path"
    ;;
  *)
    echo "Simple filename"
    ;;
esac
```

We can also check if a parameter is a number or not:

```shell
case "$1" in
  ''|*[!0-9]*)
    echo "Not a number"
    ;;
  *)
    echo "Number"
    ;;
esac
```

---

## Loops

### `for`

```shell
for arg in list; do
   # ...
done
```

*Examples*: 

```shell
for i in 1 2 3 4 5; do
  echo "5 * %i = $(expr 5 \* $i)"
done
```

```shell
for fname in "$HOME"/*; do
   echo "$fname"
done
```

### `seq`

To iterate fast on indexes, it is possible to use `seq` to create a valid index list:

```
$ seq 1 5
1
2
3
4
5
```

*Example*:

```shell
for i in $(seq 1 5); do
  echo "5 * $i = $(expr 5 \* $i)"
done

5 * 1 = 5
5 * 2 = 10
5 * 3 = 15
5 * 4 = 20
5 * 5 = 25
```

### `while`

```shell
while [ condition ]; do
  # ...
done
```

*Example*:

```shell
i=10
while [ "$i" -gt 0 ]; do
  echo $i
  i=$(( i - 1 ))
done
```

### Expand filenames

To avoid problems, you must use `""` for filenames because often it contains spaces and it could lead to an error.

Given a script like this:

```shell
 $ vim script.sh

 #!/bin/bash
 for fname in "$HOME"/*; do
   if [ -f "$fname" ]; then
     echo F "$fname"
   elif [ -d "$fname" ]; then
     echo D "$fname"
   fi
 done
```

It lead to an error in cases like this:

```shell
$ touch "$HOME"/"Some file"
$ ./script.sh
./script.sh: line 4: [: /home/user/Some: binary operator expected
```

## Functions

```shell
functionname() {
  # ...
}
```

- They access parameters like scripts using \$1, ..., \$n notation.
- They can return a value using `return` followed by the returned value.
- Their return value can be read using $?, like scripts.

*Example*:

```shell
#!/bin/bash

process() {
  [ -d "$1" ] && [ -x "$1" ] && return 0
  return 1
}

for f in "$@"; do
  echo -n "$1"
  if process "$f"; then
    echo " [pass]"
  else
    echo " [fail]"
  fi
done

exit 0
```

The function `process` returns 0 if the directory exists and it is executable, 1 otherwise.

### Multi-file script

```shell
$ vim lib.sh

#!/bin/bash
process() {
  [ -d "$1" ] && [ -x "$1" ] && return 0
  return 1
}
```

```shell
$ vim script.sh

 #!/bin/bash

 source lib.sh
 # or
 # . lib.sh

for f in "$@"; do
  echo -n "$1"
  if process "$f"; then
    echo " [pass]"
  else
    echo " [fail]"
  fi
done

exit 0
```

`source` works like Python `import`.

### Arrays

```shell
# void array
$ arr=()

# filled array
$ arr=(1 2 3)

# adding values
$ arr+=(4 5)

# overwirte a value
$ arr[0]=3

# print all values
$ echo ${arr[@]}

# print all available indexes 
$ echo ${!arr[@]}

# print array length
$ echo ${#arr[@]}

# prints n elements starting from s
$ echo ${arr[@]:s:n}
```

*Example*:

```shell
#!/bin/bash

files=(/var/log/kern.log /var/log/auth.log /var/log/syslog)
keyw=(lucas tom)

echo "* searching ${#files[@]} files with ${#keyw[@]} keywords"
echo "* press enter to continue..."
read

for f in ${files[@]}; do
  for k in ${keyw[@]}; do
    l=$(cat ${f} | grep ${k} | wc -l)
    echo "* ${f}: found ${l} occurrences of ${k}!"
  done
done
```

### getopts

built-in function to manage parameters from command line. It exists also in Java, C, Python, etc.

- `getopts` must be used combined with a `while` and a `case`.
- `"m:dh"` string represents parameters to be checked. Single characters, like `d` e `h`, represents parameters without arguments. Characters followed by `:`, like `m`, represents parameters with arguments.
- `getopts` scans command line and, at every iteration, it updates `o` variable so that it can be analyzed by `case`.
- `case` block usually assigns to some variables arguments' values (`OPTARG`).

```shell
while getopts "m:dh" o; do
  case "$o" in
    m)
      MESSAGE="$OPTARG"
      ;;
    d)
      DEBUG=TRUE
      ;;
    h)
      usage
      ;;
    *)
      usage
      ;;
  esac
done
shift $(expr $OPTIND - 1)
```

*Example*:

```shell
#!/bin/bash

# Default values
MESSAGE="Hello World!"
DEBUG=FALSE

# Usage function
usage() {
    echo "Usage: $0 [-h] [-m <string>] [-d] filename" 1>&2
    exit 1
}

# In case of optional [] parameters default values are overriden
while getopts "m:dh" o; do
    case "$0" in
        m)
          MESSAGE="$OPTARG"
          ;;
        d)
          DEBUG=TRUE
          ;;
        h)
          usage
          ;;
        *)
          usage
          ;;
    esac
done
# Shift parameters away. $1 becomes filename
shift $(expr $OPTIND - 1)

# Additional checks
# Check if filename exists
[ -e "$1" ] || usage

echo m = "$MESSAGE"
echo d = "$DEBUG"
echo filename = "$1"
exit 0
```

### Best practice

- Since it is an old language, indentation still optional. Anyway, it still be very important.
- Global variables must be uppercase (ex. `USAGE="$0 usage: ..."`).
- Parameters check is performed in negative way: this means that usually it has to be checked failure cases, then, if verified, script exits with an error code. This practice is necessary to avoid excessive indentation.
- Exits codes use different values to distinguish success (`exit 0`) from failure (`exit 1`). Different type of failures can be represented using different positive numbers.
- Since filenames can contain spaces, all variables must be expanded between double quotes (ex. echo "$filename").
- It would be nice to organize script following this order:
  1. Shabang.
  2. Global variables definition.
  3. Function definition.
  4. Parameters check.
  5. Body.
  6. Termination.

*Example*:

```shell
#!/bin/bash

# Global variables
USAGE="usage: $0 dirname"

# Definizione funzioni
usage() {
  echo "$USAGE"
  exit 1
}

# Parameters check
if [ $# -ne 1 ]; then
	usage
fi

if [ ! -d "$1" ] || [ ! -x "$1" ]; then
  usage
fi

# Body
F=0; D=0
for fname in "$1"/*; do
	if [ -f "$fname" ]; then
	  F=$(( F + 1 ))
	fi

  if [ -d "$fname" ]; then
	  D=$(( D + 1 ))
	fi
done

echo "#files=$F, #directories=$D"

# Termination
exit 0
```

### Bash vs Python for scripting

**Bash**

Pros:

- Notissima, installata ovunque
- Deep Unix integration (ex. piping and redirection).

Cons:

* No support for OOP, data structures and multi-threading.
* Weak debug tools.

**Python**

Pros:

- It supports OOP, data structures and multi-threading.
- It has debug tools.

Cons:

* Dependency managment.
* More verbose than bash.

Comparison:

```shell
#!/bin/bash

if [ $# -lt 1 ]; then
  echo "usage: $0 f1 .. fn"
  exit 1
fi

l=0
for fname in $*; do
   l=$(wc -l "$fname" | cut -d ' ' -f 1)
   echo "$fname": "$l"
done

exit 0
```

```python
#!/usr/bin/env python

import sys
import subprocess

if len(sys.argv) < 2:
   sys.stderr.write("usage: %s f1 .. fn\n" % (sys.argv[0]))
   sys.exit(1)

for fname in sys.argv[1:]:
  out  = subprocess.check_output(['wc', '-l', fname])
  print("%s: %s" % (fname, out.split(' ')[0]))

sys.exit(0)
```

They look similar.
