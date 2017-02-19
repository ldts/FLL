dnl
dnl Process this file with autoconf to produce a configure script.
dnl
AC_CONFIG_MACRO_DIR([m4])

AC_INIT([FLL],m4_normalize(m4_include([config/version])),raquel.medina.rodriguez@gmail.com)

FLL_VERSION_MAJOR=`expr $PACKAGE_VERSION : '\([[0-9]]*\)'`
FLL_VERSION_MINOR=`expr $PACKAGE_VERSION : '\[[0-9]]*\.\([[0-9]]*\)'`
FLL_REVISION=`expr $PACKAGE_VERSION : '\[[0-9]]*\.[[0-9]]*\.\([[0-9]]*\)'`
test "x$FLL_REVISION" = "x" && FLL_REVISION=0

AC_CONFIG_HEADERS(include/fll_config.h)
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_SRCDIR(facelockedloop/main.c)
AC_PREFIX_DEFAULT(/usr/local/fll)
AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_PROG_INSTALL
AM_PROG_LEX
AC_PROG_YACC

dnl
dnl Do no let autoconf set the default value of CFLAGS
dnl
if $FLL_EMPTY_CFLAGS; then
        CFLAGS=""
fi

AC_ARG_WITH(cc,
    AS_HELP_STRING([--with-cc=compiler],[use specific C compiler]),
    [
	case "$withval" in
	"" | y | ye | yes | n | no)
	    AC_MSG_ERROR([You must supply an argument to --with-cc.])
	  ;;
	esac
	CC="$withval"
    ])
AC_PROG_CC


if test "x$CFLAGS" = "x"; then
	FLL_EMPTY_CFLAGS=true
else
	FLL_EMPTY_CFLAGS=false
fi

AC_DEFINE_UNQUOTED(FLL_BUILD_STRING,"$build",[Build system alias])
FLL_BUILD_STRING="$build"
AC_DEFINE_UNQUOTED(FLL_HOST_STRING,"$host",[Host system alias])
FLL_HOST_STRING="$host"
AC_DEFINE_UNQUOTED(FLL_COMPILER,"`$CC -v 2>&1 | tail -n 1`",[Compiler])

AM_INIT_AUTOMAKE([1.14 foreign])
AM_MAINTAINER_MODE
AC_PROG_LIBTOOL

if test x$host_alias = x; then
  build_for=$host
else
  build_for=$host_alias
fi

AC_MSG_CHECKING([for target architecture])
case "$build_for" in
 arm*-*)
	target_arch=arm
	;;
 x86_64-*|amd64-*)
	target_arch=x86_64
	;;
 i*86*-*)
	target_arch=x86_32
	;;
 aarch64-unknown-*)
	target_arch=arm64
	;;
 *) echo ""
   echo "*******************************************"
   echo "Unsupported target: $build_for !!!!!"
   echo "*******************************************"
   echo ""
   exit 1;;
esac
AC_MSG_RESULT([$target_arch])

dnl
dnl Debug build (default: off)
dnl
debug_mode=
AC_MSG_CHECKING(whether to enable debug mode)
AC_ARG_ENABLE(debug,
	AS_HELP_STRING([--enable-debug[=full]], [Enable debug mode in programs]),
	[case "$enableval" in
	y | yes) debug_mode=partial ;;
	full)    debug_mode=full ;;
	*) unset debug_mode ;;
	esac])
AC_MSG_RESULT(${debug_mode:-no})
AM_CONDITIONAL(FLL_BUILD_DEBUG,[test \! x$debug_mode = x])

dnl
dnl Stats build (default: off)
dnl
stats_mode=
AC_MSG_CHECKING(whether to enable stats mode)
AC_ARG_ENABLE(stats,
	AS_HELP_STRING([--enable-stats[=full]], [Enable stats mode in programs]),
	[case "$enableval" in
	y | yes) stats_mode=partial ;;
	full)    stats_mode=full ;;
	*) unset stats_mode ;;
	esac])
AC_MSG_RESULT(${stats_mode:-no})
AM_CONDITIONAL(FLL_BUILD_STATS,[test \! x$stats_mode = x])

dnl
dnl Host mode build (default: off)
dnl
host_mode=
AC_MSG_CHECKING(whether to enable host mode)
AC_ARG_ENABLE(host,
	AS_HELP_STRING([--enable-host], [Enable host mode]),
	[case "$enableval" in
	y | yes) host_mode=yes ;;
	*) unset host_mode ;;
	esac])
AC_MSG_RESULT(${host_mode:-no})
AM_CONDITIONAL(FLL_BUILD_HOST,[test \! x$host_mode = x])

dnl
dnl Used with sparse
dnl
checkflags="-nostdinc -isystem \$(SYSROOT)/usr/include -Wbitwise -Wno-transparent-union -D_GNU_SOURCE -D_XOPEN_SOURCE=500 -D_REENTRANT \$(DEFS) \$(DEFAULT_INCLUDES) \$(INCLUDES) \$(AM_CPPFLAGS) \$(CPPFLAGS) -I\$(top_srcdir)/include -isystem \$(shell \$(CC) -print-file-name=include) \$(shell \$(CC) -dM -E -xc /dev/null|sed -e 's/^\\#define /-D/' -e \"s/ /=\'/\" -e \"s/\$\$/\'/\")"
AC_SUBST(CHECKFLAGS, $checkflags)

dnl
dnl Enable assertions (default: no)
dnl
use_assert=
if test -n "$debug_mode"; then
  use_assert=y
fi
AC_MSG_CHECKING(whether assertions should be enabled)
AC_ARG_ENABLE(assert,
	AS_HELP_STRING([--enable-assert], [Enable runtime assertions]),
	[case "$enableval" in
	y | yes) use_assert=y ;;
	n | no) unset use_assert ;;
	esac])
AC_MSG_RESULT(${use_assert:-no})

dnl
dnl OpenCV
dnl
AC_MSG_CHECKING(for OpenCV libraries)
AC_ARG_WITH(opencv,
	AS_HELP_STRING([--with-opencv], [specify where to find opencv]),
	[case "$withval" in
	      y | yes) opencv=y ;;
	      n | no) opencv=n ;;
	      *)
	      opencv=y
	      opencvinc="-I$withval/include/opencv2"
	      opencvlib="-L$withval/lib"
	      ;;
	      esac])
AC_MSG_RESULT(${opencv:-no})
AC_SUBST(opencvinc)
AC_SUBST(opencvlib)
OLD_CFLAGS=$CFLAGS
OLD_LIBS=$LIBS
LIBS=$opencvlib
CFLAGS=$opencvinc
opencv=n
opencv2=n
AC_MSG_CHECKING([for OpenCV older than 2.4.x])
AC_CHECK_LIB(opencv_imgproc, cvResize,
	     [
	     OPENCV_ADD_LDFLAG="-lopencv_core -lopencv_highgui -lopencv_objdetect -lopencv_features2d -lopencv_imgproc"
	     opencv=y
	     opencv2=y
	     ]
	     )
AC_SUBST(OPENCV_ADD_LDFLAG)

CFLAGS=$OLD_CFLAGS
LIBS=$OLD_LIBS
AM_CONDITIONAL([USEOPENCV], [test  x"$opencv" != xn])
AM_CONDITIONAL([HAVE_OPENCV2], [test x"$opencv2" != xn])
dnl
dnl Documentation package:
dnl checking for doxygen (html, docbook, latex, man documentation)
dnl Adapted from xenomai project: xenomai.org 
dnl
if test x"$docdir" = x; then
	docdir=$datadir/doc/$PACKAGE-$VERSION
fi
AC_SUBST(docdir)
unset FLL_MAYBE_DOCDIR
AC_SUBST(FLL_MAYBE_DOCDIR)

AC_MSG_CHECKING(for Doxygen doc generation)
AC_ARG_ENABLE(doc,
	AS_HELP_STRING([--enable-doc], [Build Doxygen documentation]),
	[case "$enableval" in
	y | yes) FLL_BUILD_DOC=y ;;
	*) unset FLL_BUILD_DOC ;;
	esac])

if test \! -d $srcdir/doc ; then
    if test x$FLL_BUILD_DOC = xy ;
    then
	AC_MSG_ERROR([doc folder is missing.])
    fi
    AC_MSG_RESULT([not present])
else
    AC_MSG_RESULT(${FLL_BUILD_DOC:-no})
fi

AC_CHECK_PROG(DOXYGEN, doxygen, doxygen)

if test x${FLL_BUILD_DOC} = xy -a x"$DOXYGEN" = x ; then
   AC_MSG_ERROR([Missing Doxygen.])
fi

AC_CHECK_PROG(DOXYGEN_HAVE_DOT, dot, YES, NO)
if test x"$DOXYGEN_HAVE_DOT" = xYES ; then
   DOXYGEN_SHOW_INCLUDE_FILES=NO
else
   DOXYGEN_SHOW_INCLUDE_FILES=YES
fi

AC_MSG_CHECKING(whether compiling Docbook XML documentation)
AC_ARG_ENABLE(dbx,
	AS_HELP_STRING([--enable-dbx], [Build DocBook XML documentation.]),
	[case "$enableval" in
	y | ye | yes) FLL_DBX_DOC=y ;;
	*) unset FLL_DBX_DOC ;;
	esac])

if test \! -f $srcdir/doc/catalog.in ; then
   	if test x$FLL_DBX_DOC = xy
	then
		AC_MSG_ERROR([DBX catalog is missing!!])
	fi
	AC_MSG_RESULT([not present])
else
	AC_MSG_RESULT(${FLL_DBX_DOC:-no})
fi
AM_CONDITIONAL(FLL_DBX_DOC,[test "$FLL_DBX_DOC"=y])
dnl DBX_DOC_ROOT = "doc/docbook"
dnl AC_SUBST(DBX_DOC_ROOT)
dnl DBX_GEN_DOC_ROOT = "doc/generated"
dnl AC_SUBST(DBX_GEN_DOC_ROOT)

dnl
dnl XSL over network:
dnl
AC_MSG_CHECKING(for docbook-xsl root dir)
AC_ARG_WITH(docbook-xsl-root,
	AS_HELP_STRING([--with-dbx-xsl-root],[specify the Docbook XML XSL
stylesheet root. Default is to use well-known locations (or network if
--enable-dbx-network was passed)]),
        [ DBX_XSL_ROOT="$withval" ])
if test x"$DBX_XSL_ROOT" = x; then
   	if test x"$enable_dbx_network" != x -a -n "$FLL_DBX_DOC" ; then
      	   AC_MSG_ERROR([The DocBook XML DTD was not found and cannot access via
      	   the net.])
      	fi
	DBX_XLS_ROOT="docbook.sourceforge.net/release/xsl/current/"
	AC_MSG_RESULT([not present, going to sourceforge.net])
else
	DBX_XSL_ROOT="file://$DBX_XSL_ROOT"
	AC_MSG_RESULT(${DBX_XSL_ROOT:?})
fi
AC_SUBST(DBX_XSL_ROOT)    

dnl
dnl LaTeX
dnl
AC_MSG_CHECKING(whether using LaTeX non-stop mode)
AC_ARG_ENABLE(verbose-latex,
	AS_HELP_STRING([--enable-verbose-latex], [Uses LaTeX non-stop mode]),
	[case "$enableval" in
	y | yes) FLL_DOC_LATEX_NONSTOP=y ;;
	*) unset FLL_DOC_LATEX_NONSTOP ;;
	esac])
AC_MSG_RESULT(${FLL_DOC_LATEX_NONSTOP:-no})

if test x${FLL_DOC_LATEX_NONSTOP} = xy; then
   LATEX_BATCHMODE=NO
   LATEX_MODE=nonstopmode
else
   LATEX_BATCHMODE=YES
   LATEX_MODE=batchmode
fi

AC_DEFINE_UNQUOTED(FLL_VERSION_MAJOR,$FLL_VERSION_MAJOR,[config])
AC_DEFINE_UNQUOTED(FLL_VERSION_MINOR,$FLL_VERSION_MINOR,[config])
AC_DEFINE_UNQUOTED(FLL_REVISION_LEVEL,$FLL_REVISION_LEVEL,[config])
AC_DEFINE_UNQUOTED(FLL_PREFIX,"$prefix",[config])

AH_VERBATIM(FLL___,[#ifdef __IN_FLL__])
AH_BOTTOM([#endif /* __IN_FLL__ */])

dnl Documentation.
AM_CONDITIONAL(FLL_BUILD_DOC,[test "$FLL_BUILD_DOC" = y])

dnl Common CFLAGS and LDFLAGS
FLL_CFLAGS="-D_GNU_SOURCE -D_REENTRANT -Wall -pipe -Winline $target_cflags"
FLL_LDFLAGS=-Wl,--export-dynamic

if test x$debug_mode = xpartial; then
   FLL_CFLAGS="-g -O2 $FLL_CFLAGS"
elif test x$debug_mode = xfull; then
   FLL_CFLAGS="-g -O0 $FLL_CFLAGS"
else
   FLL_CFLAGS="-O2 $FLL_CFLAGS"
fi
FLL_CFLAGS="$FLL_CFLAGS -D__IN_FLL__"

if test -n "$host_mode"; then
   AC_DEFINE(FLL_BUILD_HOST, 1, [config])
fi

if test -n "$debug_mode"; then
   AC_DEFINE(FLL_BUILD_DEBUG, 1, [config])
fi

if test -n "$stats_mode"; then
   AC_DEFINE(FLL_BUILD_STATS, 1, [config])
fi

if test x$use_assert = x; then
   FLL_CFLAGS="-DNDEBUG $FLL_CFLAGS"
fi

FLL_CFLAGS="$FLL_CFLAGS -Wextra -Wstrict-prototypes -Wmissing-prototypes -Wno-long-long -Wno-unused-parameter -Werror -fstrict-aliasing"
FLL_EXTRA_CFLAGS="-Wno-unused-function"
LIBS=

AC_MSG_CHECKING([for test source generation])
AC_RUN_IFELSE([AC_LANG_PROGRAM([[ ]], [[ ]])],
  [AC_MSG_RESULT(ok)], [AC_MSG_RESULT(failed)], [AC_MSG_RESULT(untestable)])
dnl
dnl Build the Makefiles
dnl

AC_SUBST(DOXYGEN_SHOW_INCLUDE_FILES)
AC_SUBST(DOXYGEN_HAVE_DOT)
AC_SUBST(DOXYGEN)
AC_SUBST(LATEX_BATCHMODE)
AC_SUBST(LATEX_MODE)

AC_SUBST(FLL_BUILD_STRING)
AC_SUBST(FLL_HOST_STRING)
AC_SUBST(FLL_CFLAGS)
AC_SUBST(FLL_EXTRA_CFLAGS)
AC_SUBST(FLL_LDFLAGS)

AM_EXTRA_RECURSIVE_TARGETS([servolib facelockedloop])
AC_CONFIG_FILES([ \
	Makefile \
	include/Makefile \
	servolib/Makefile \
	facelockedloop/Makefile \
	])

AC_OUTPUT()
