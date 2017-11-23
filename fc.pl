#!/usr/bin/perl -w

use strict;
use HTML::Tree;
use LWP::Simple;
use Data::Dumper;
use Text::CSV;

my $funky = "http://firstchoicebyandymark.com/everything";

my $content = get($funky);

my $tree = HTML::Tree->new();

$tree->parse($content);

my $csv = Text::CSV->new ( { eol => "\n", binary => 1 } )  # should set binary attribute.
                  or die "Cannot use CSV: ".Text::CSV->error_diag ();


for my $item ( $tree->look_down('class', 'item-box') ) {

        my $img = $item->look_down('_tag', 'img');
        my $url = $img->{src};
        my $text = $img->{alt};

        $text =~ m/Picture of (.*\S) *\((.*)\)$/;
        my $name = $1;
        my $part = $2;

        my $price = $item->look_down('class', 'price actual-price')->as_text;
        $price =~ s/ *FIRST Credits *//i;

        my @row = ($part, $name, $url, $price);
        $csv->print (*STDOUT, \@row);
        print "";
}
