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
  - [Script multi-file](#script-multi-file)
  - [Arrays](#arrays)
  - [getopts](#getopts)
  - [Buone pratiche](#buone-pratiche)
  - [Bash vs Python per la scrittura di script](#bash-vs-python-per-la-scrittura-di-script)

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

Bash supporta la definizione di funzioni:

- Accedono a parametri di invocazione con sintassi \$1 . . . \$n (come gli script)
- Ritornano al chiamante con istruzione return (script usano exit)
- Valori di ritorno possono essere letti dal chiamante con sintassi $? (come gli script)
- Definite con sintassi:

```
nomefunzione() {
  .
  .
  .
}
```

L'esempio seguente definisce una funzione che ritorna 0 in caso il primo parametro sia una directory eseguibile, 1 viceversa.

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

### Script multi-file

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
 # oppure
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

### Arrays

```shell
# array definito vuoto
$ arr=()

# array definito con valori all’intero
$ arr=(1 2 3)

# aggiunge valori ad array esistente
$ arr+=(4 5)

# sovrascrive valore di indice 0
$ arr[0]=3

# mostra i valori nell’array
$ echo ${arr[@]}

# mostra gli indici validi dell’array
$ echo ${!arr[@]}

# mostra il numero di valori nell’array
$ echo ${#arr[@]}

# mostra n elementi partendo da indice s
$ echo ${arr[@]:s:n}
```

Ad esempio:

```shell
#!/bin/bash

files=(/var/log/kern.log /var/log/auth.log /var/log/syslog)
keyw=(nicola marzia)

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

Funzione standard (**builtin**) per gestire parametri a linea di comando. Esiste in Java, C, Python, etc.

- getopts va sempre utilizzata abbinata ad un while e un case
- La stringa "m:dh" rappresenta i parametri da controllare. Le lettere singole (e.g., d e h) rappresentano parametri senza argomenti. Le lettere seguite da **:** (e.g., m) rappresentano parametri con argomenti
- getopts scansiona la linea di comando e ad ogni ciclo aggiorna la variabile **o** affinchè sia analizzata dal blocco case
- Il blocco case, tipicamente, assegna a delle variabili il valore degli argomenti (**OPTARG**)

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

Ad esempio:

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

### Buone pratiche

- Trattandosi di un linguaggio antico, l'indentazione è ancora facoltativa (in Python, recente, è obbligatoria!). Indentazione è comunque di fondamentale importanza!
- Variabili globali sono MAIUSCOLE (ad es. USAGE="$0 usage: ...")
- Il controllo dei parametri avviene in **via negativa**. Si controllano le condizioni di fallimento e, se verificate, si termina lo script ritornando un codice errore (exit 1). Questa pratica evita indentazione eccessiva
- I valori di uscita (exit) utilizzano valori diversi per distinguere successo (exit 0) da fallimento (exit 1).Per differenziare fra diversi tipi di fallimento si possono utilizzare numeri positivi > 1 (ad es. exit 2)
- I filesystem moderni supportano la presenza di spazi. Per questo motivo, tutte le variabili fuori dal controllo del programmatore (ad es. nomi di file) vanno espanse fra doppie virgolette (ad es. echo "$filename")
- E' buona norma (best practice) aderire ad un canovaccio noto e consolidato:
  - Definizione interprete
  - Definizione variabili globali
  - Definizione funzioni
  - Controllo parametri
  - Corpo principale
  - Terminazione

```shell
#!/bin/bash

# Definizione variabili globali
USAGE="usage: $0 dirname"

# Definizione funzioni
usage() {
  echo "$USAGE"
  exit 1
}

# Controllo parametri
if [ $# -ne 1 ]; then
	usage
fi

if [ ! -d "$1" ] || [ ! -x "$1" ]; then
  usage
fi

# Corpo principale
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

# Terminazione
exit 0
```

### Bash vs Python per la scrittura di script

**Bash**

- Notissima, installata ovunque
- Integrazione profonda con Unix (piping, ridirezione)

* Manca supporto per OOP, strutture dati, multi-threading
* Tool di debug scarsi

**Python**

- Supporto per OOP, strutture dati, multi-threading
- Tool di debug

* Gestione delle dipendenze
* Codice più prolisso per cose semplici

L'esempio che segue mostra lo stesso programma scritto nei due linguaggi. Le somiglianze sono piuttosto evidenti.

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
