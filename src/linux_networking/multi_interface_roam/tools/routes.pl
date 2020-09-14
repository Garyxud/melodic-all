#!/usr/bin/perl

use strict;

open RULES, "ip rule|" or die "Can't get ip rules";

while(<RULES>) {
   print;
   if( m/lookup\s+(\S+)/ ) {
      open TABLE, "ip route show table $1|";
      while(<TABLE>) {
         print "\t$_";
      }
   }
}
