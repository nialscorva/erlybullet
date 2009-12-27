#!/usr/bin/perl

use File::Basename;

$files=`find src -iname \\*.erl -exec basename {} \\;`;
$mods = $files;
$mods =~ s/\.erl//gm;
$mods = "\'".join("\',\'",split(/\n/,$mods))."\'";

while(<>)
{ 
  s/\@\@MODULES\@\@/$mods/g;
  print $_;
}

