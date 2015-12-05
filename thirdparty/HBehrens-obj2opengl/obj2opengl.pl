#! /usr/bin/perl
=head1 NAME

 obj2opengl - converts obj files to arrays for glDrawArrays
 
=head1 SYNOPSIS

 obj2opengl [options] file

 use -help or -man for further information

=head1 DESCRIPTION

This script expects and OBJ file consisting of vertices,
texture coords and normals. Each face must contain
exactly 3 vertices. The texture coords are two dimonsional.

The resulting .H file offers three float arrays to be rendered
with glDrawArrays.

=head1 AUTHOR

Heiko Behrens (http://www.HeikoBehrens.net)

=head1 VERSION

14th August 2012

=head1 COPYRIGHT

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

=head1 ACKNOWLEDGEMENTS

This script is based on the work of Margaret Geroch.

=head1 REQUIRED ARGUMENTS

The first or the last argument has to be an OBJ file according 
to this () specification.

=head1 OPTIONS

=over

=item B<-help>

Print a brief help message and exits.

=item B<-man>

Prints the extended manual page and exits.

=item B<-noScale>    

Prevents automatic scaling. Otherwise the object will be scaled
such the the longest dimension is 1 unit.

=item B<-scale <float>>

Sets the scale factor explicitly. Please be aware that negative numbers
are not handled correctly regarding the orientation of the normals.

=item B<-noMove>

Prevents automatic scaling. Otherwise the object will be moved to the center of
its vertices.

=item B<-o>, B<-outputFilename>

Name of the output file name. If omitted, the output file the same as the
input filename but with the extension .h

=item B<-nameOfObject>

Specifies the name of the generated variables. If omitted, same as 
output filename without path and extension.

=item B<-noverbose>

Runs this script silently.
   
=cut

use Getopt::Long;
use File::Basename;
use Pod::Usage;

# -----------------------------------------------------------------
# Main Program
# -----------------------------------------------------------------
handleArguments();

# derive center coords and scale factor if neither provided nor disabled
unless(defined($scalefac) && defined($xcen)) {
	calcSizeAndCenter();
}

if($verbose) {
	printInputAndOptions();
}

# TODO check integrity: Does every referenced vertex, normal and coord exist?
loadData();
normalizeNormals();

if($verbose) {
	printStatistics();
}

writeOutput();

# -----------------------------------------------------------------
# Sub Routines
# -----------------------------------------------------------------

sub handleArguments() {
	my $help = 0;
	my $man = 0;
	my $noscale = 0;
	my $nomove = 0;
	$verbose = 1;
	$errorInOptions = !GetOptions (
		"help" => \$help,
		"man"  => \$man,
		"noScale" => \$noscale,
		"scale=f" => \$scalefac,
		"noMove" => \$nomove,
		"center=f{3}" => \@center,
		"outputFilename=s" => \$outFilename,
		"nameOfObject=s" => \$object,
		"verbose!" => \$verbose,
		);
	
	if($noscale) {
		$scalefac = 1;
	}
	
	if($nomove) {
		@center = (0, 0, 0);
	}
	
	if(defined(@center)) {
		$xcen = $center[0];
		$ycen = $center[1];
		$zcen = $center[2];
	}
	
	if($#ARGV == 0) {
		my ($file, $dir, $ext) = fileparse($ARGV[0], qr/\.[^.]*/);
		$inFilename = $dir . $file . $ext;
	} else {
		$errorInOptions = true;
	}

	# (optional) derive output filename from input filename
	unless($errorInOptions || defined($outFilename)) {
		my ($file, $dir, $ext) = fileparse($inFilename, qr/\.[^.]*/);
		$outFilename = $dir . $file . ".h";
	}
	
	# (optional) define object name from output filename
	unless($errorInOptions || defined($object)) {
		my ($file, $dir, $ext) = fileparse($outFilename, qr/\.[^.]*/);
	  	$object = $file;
	}
	
	($inFilename ne $outFilename) or
		die ("Input filename must not be the same as output filename")
		unless($errorInOptions);
		
	if($errorInOptions || $man || $help) {
		pod2usage(-verbose => 2) if $man;
		pod2usage(-verbose => 1) if $help;
		pod2usage(); 
	}
	
	# check wheter file exists
	open ( INFILE, "<$inFilename" ) 
	  || die "Can't find file '$inFilename' ...exiting \n";
	close(INFILE);
}

# Stores center of object in $xcen, $ycen, $zcen
# and calculates scaling factor $scalefac to limit max
#   side of object to 1.0 units
sub calcSizeAndCenter() {
	open ( INFILE, "<$inFilename" ) 
	  || die "Can't find file $inFilename...exiting \n";

	$numVerts = 0;
	
	my (
		$xsum, $ysum, $zsum, 
		$xmin, $ymin, $zmin,
		$xmax, $ymax, $zmax,
		);

	while ( $line = <INFILE> ) 
	{
	  chop $line;
	  
	  if ($line =~ /v\s+.*/)
	  {
	  
	    $numVerts++;
	    @tokens = split(' ', $line);
	    
	    $xsum += $tokens[1];
	    $ysum += $tokens[2];
	    $zsum += $tokens[3];
	    
	    if ( $numVerts == 1 )
	    {
	      $xmin = $tokens[1];
	      $xmax = $tokens[1];
	      $ymin = $tokens[2];
	      $ymax = $tokens[2];
	      $zmin = $tokens[3];
	      $zmax = $tokens[3];
	    }
	    else
	    {   
	        if ($tokens[1] < $xmin)
	      {
	        $xmin = $tokens[1];
	      }
	      elsif ($tokens[1] > $xmax)
	      {
	        $xmax = $tokens[1];
	      }
	    
	      if ($tokens[2] < $ymin) 
	      {
	        $ymin = $tokens[2];
	      }
	      elsif ($tokens[2] > $ymax) 
	      {
	        $ymax = $tokens[2];
	      }
	    
	      if ($tokens[3] < $zmin) 
	      {
	        $zmin = $tokens[3];
	      }
	      elsif ($tokens[3] > $zmax) 
	      {
	        $zmax = $tokens[3];
	      }
	    
	    }
	 
	  }
	 
	}
	close INFILE;
	
	#  Calculate the center
	unless(defined($xcen)) {
		$xcen = $xsum / $numVerts;
		$ycen = $ysum / $numVerts;
		$zcen = $zsum / $numVerts;
	}
	
	#  Calculate the scale factor
	unless(defined($scalefac)) {
		my $xdiff = ($xmax - $xmin);
		my $ydiff = ($ymax - $ymin);
		my $zdiff = ($zmax - $zmin);
		
		if ( ( $xdiff >= $ydiff ) && ( $xdiff >= $zdiff ) ) 
		{
		  $scalefac = $xdiff;
		}
		elsif ( ( $ydiff >= $xdiff ) && ( $ydiff >= $zdiff ) ) 
		{
		  $scalefac = $ydiff;
		}
		else 
		{
		  $scalefac = $zdiff;
		}
		$scalefac = 1.0 / $scalefac;
	}
}

sub printInputAndOptions() {
	print "Input file     : $inFilename\n";
	print "Output file    : $outFilename\n";
	print "Object name    : $object\n";
	print "Center         : <$xcen, $ycen, $zcen>\n";
	print "Scale by       : $scalefac\n";
}

sub printStatistics() {
	print "----------------\n";
	print "Vertices       : $numVerts\n";
	print "Faces          : $numFaces\n";
	print "Texture Coords : $numTexture\n";
	print "Normals        : $numNormals\n";
}

# reads vertices into $xcoords[], $ycoords[], $zcoords[]
#   where coordinates are moved and scaled according to
#   $xcen, $ycen, $zcen and $scalefac
# reads texture coords into $tx[], $ty[] 
#   where y coordinate is mirrowed
# reads normals into $nx[], $ny[], $nz[]
#   but does not normalize, see normalizeNormals()
# reads faces and establishes lookup data where
#   va_idx[], vb_idx[], vc_idx[] for vertices
#   ta_idx[], tb_idx[], tc_idx[] for texture coords
#   na_idx[], nb_idx[], nc_idx[] for normals
#   store indizes for the former arrays respectively
#   also, $face_line[] store actual face string
sub loadData {
	$numVerts = 0;
	$numFaces = 0;
	$numTexture = 0;
	$numNormals = 0;
	
	open ( INFILE, "<$inFilename" )
	  || die "Can't find file $inFilename...exiting \n";
	
	while ($line = <INFILE>) 
	{
	  chop $line;
	  
	  # vertices
	  if ($line =~ /v\s+.*/)
	  {
	    @tokens= split(' ', $line);
	    $x = ( $tokens[1] - $xcen ) * $scalefac;
	    $y = ( $tokens[2] - $ycen ) * $scalefac;
	    $z = ( $tokens[3] - $zcen ) * $scalefac;    
	    $xcoords[$numVerts] = $x; 
	    $ycoords[$numVerts] = $y;
	    $zcoords[$numVerts] = $z;
	
	    $numVerts++;
	  }
	  
	  # texture coords
	  if ($line =~ /vt\s+.*/)
	  {
	    @tokens= split(' ', $line);
	    $x = $tokens[1];
	    $y = 1 - $tokens[2];
	    $tx[$numTexture] = $x;
	    $ty[$numTexture] = $y;
	    
	    $numTexture++;
	  }
	  
	  #normals
	  if ($line =~ /vn\s+.*/)
	  {
	    @tokens= split(' ', $line);
	    $x = $tokens[1];
	    $y = $tokens[2];
	    $z = $tokens[3];
	    $nx[$numNormals] = $x; 
	    $ny[$numNormals] = $y;
	    $nz[$numNormals] = $z;
	
	    $numNormals++;
	  }
	  
	  # faces
	  if ($line =~ /f\s+([^ ]+)\s+([^ ]+)\s+([^ ]+)(\s+([^ ]+))?/) 
	  {
	  	@a = split('/', $1);
	  	@b = split('/', $2);
	  	@c = split('/', $3);
	  	$va_idx[$numFaces] = $a[0]-1;
	  	$ta_idx[$numFaces] = $a[1]-1;
	  	$na_idx[$numFaces] = $a[2]-1;
	
	  	$vb_idx[$numFaces] = $b[0]-1;
	  	$tb_idx[$numFaces] = $b[1]-1;
	  	$nb_idx[$numFaces] = $b[2]-1;
	
	  	$vc_idx[$numFaces] = $c[0]-1;
	  	$tc_idx[$numFaces] = $c[1]-1;
	  	$nc_idx[$numFaces] = $c[2]-1;
	  	
	  	$face_line[$numFaces] = $line;
	  	
		$numFaces++;
		
		# ractangle => second triangle
		if($5 != "")
		{
			@d = split('/', $5);
			$va_idx[$numFaces] = $a[0]-1;
			$ta_idx[$numFaces] = $a[1]-1;
			$na_idx[$numFaces] = $a[2]-1;

			$vb_idx[$numFaces] = $d[0]-1;
			$tb_idx[$numFaces] = $d[1]-1;
			$nb_idx[$numFaces] = $d[2]-1;

			$vc_idx[$numFaces] = $c[0]-1;
			$tc_idx[$numFaces] = $c[1]-1;
			$nc_idx[$numFaces] = $c[2]-1;

			$face_line[$numFaces] = $line;

			$numFaces++;
		}
		
	  }  
	}
	
	close INFILE;
}

sub normalizeNormals {
	for ( $j = 0; $j < $numNormals; ++$j) 
	{
	 $d = sqrt ( $nx[$j]*$nx[$j] + $ny[$j]*$ny[$j] + $nz[$j]*$nz[$j] );
	  
	  if ( $d == 0 )
	  {
	    $nx[$j] = 1;
	    $ny[$j] = 0;
	    $nz[$j] = 0;
	  }
	  else
	  {
	    $nx[$j] = $nx[$j] / $d;
	    $ny[$j] = $ny[$j] / $d;
	    $nz[$j] = $nz[$j] / $d;
	  }
	    
	}
}

sub fixedIndex {
    local $idx = $_[0];
    local $num = $_[1];
    if($idx >= 0)
    {
        $idx;
    } else {
        $num + $idx + 1;
    }
}

sub writeOutput {
	open ( OUTFILE, ">$outFilename" ) 
	  || die "Can't create file $outFilename ... exiting\n";
	
	print OUTFILE "/*\n";
	print OUTFILE "created with obj2opengl.pl\n\n";

	# some statistics
	print OUTFILE "source file    : $inFilename\n";
	print OUTFILE "vertices       : $numVerts\n";
	print OUTFILE "faces          : $numFaces\n";
	print OUTFILE "normals        : $numNormals\n";
	print OUTFILE "texture coords : $numTexture\n";
	print OUTFILE "\n\n";
	
	# example usage
	print OUTFILE "// include generated arrays\n";
	print OUTFILE "#import \"".$outFilename."\"\n";
	print OUTFILE "\n";
	print OUTFILE "// set input data to arrays\n";
	print OUTFILE "glVertexPointer(3, GL_FLOAT, 0, ".$object."Verts);\n";
	print OUTFILE "glNormalPointer(GL_FLOAT, 0, ".$object."Normals);\n"
		if $numNormals > 0;
	print OUTFILE "glTexCoordPointer(2, GL_FLOAT, 0, ".$object."TexCoords);\n"
		if $numTexture > 0;
	print OUTFILE "\n";
	print OUTFILE "// draw data\n";
	print OUTFILE "glDrawArrays(GL_TRIANGLES, 0, ".$object."NumVerts);\n";
	print OUTFILE "*/\n\n";
	
	# needed constant for glDrawArrays
	print OUTFILE "unsigned int ".$object."NumVerts = ".($numFaces * 3).";\n\n";
	
	# write verts
	print OUTFILE "float ".$object."Verts \[\] = {\n"; 
	for( $j = 0; $j < $numFaces; $j++)
	{
		$ia = fixedIndex($va_idx[$j], $numVerts);
		$ib = fixedIndex($vb_idx[$j], $numVerts);
		$ic = fixedIndex($vc_idx[$j], $numVerts);
		print OUTFILE "  // $face_line[$j]\n";
		print OUTFILE "  $xcoords[$ia], $ycoords[$ia], $zcoords[$ia],\n";
		print OUTFILE "  $xcoords[$ib], $ycoords[$ib], $zcoords[$ib],\n";
		print OUTFILE "  $xcoords[$ic], $ycoords[$ic], $zcoords[$ic],\n";
	}
	print OUTFILE "};\n\n";
	
	# write normals
	if($numNormals > 0) {
		print OUTFILE "float ".$object."Normals \[\] = {\n"; 
		for( $j = 0; $j < $numFaces; $j++) {
			$ia = fixedIndex($na_idx[$j], $numNormals);
			$ib = fixedIndex($nb_idx[$j], $numNormals);
			$ic = fixedIndex($nc_idx[$j], $numNormals);
			print OUTFILE "  // $face_line[$j]\n";
			print OUTFILE "  $nx[$ia], $ny[$ia], $nz[$ia],\n";
			print OUTFILE "  $nx[$ib], $ny[$ib], $nz[$ib],\n";
			print OUTFILE "  $nx[$ic], $ny[$ic], $nz[$ic],\n";
		}
		
		print OUTFILE "};\n\n";
	}
	
	# write texture coords
	if($numTexture) {
		print OUTFILE "float ".$object."TexCoords \[\] = {\n"; 
		for( $j = 0; $j < $numFaces; $j++) {
			$ia = fixedIndex($ta_idx[$j], $numTexture);
			$ib = fixedIndex($tb_idx[$j], $numTexture);
			$ic = fixedIndex($tc_idx[$j], $numTexture);
			print OUTFILE "  // $face_line[$j]\n";
			print OUTFILE "  $tx[$ia], $ty[$ia],\n";
			print OUTFILE "  $tx[$ib], $ty[$ib],\n";
			print OUTFILE "  $tx[$ic], $ty[$ic],\n";
		}
		
		print OUTFILE "};\n\n";
	}
	
	close OUTFILE;
}

