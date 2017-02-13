#!/bin/sh

aclocal --install -I m4 &&
    autoheader &&
    libtoolize -c --automake --force &&
    autoconf &&
    automake --add-missing --copy &&
    ./configure --with-opencv=/usr/ --enable-debug=full --prefix=/home/rmedina/FLL/target/x86_64/usr/local/fll/ "$@"
#    ./configure --enable-doc --with-opencv=/usr/ "$@"
