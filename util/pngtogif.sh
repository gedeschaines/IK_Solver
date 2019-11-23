#!/bin/bash

#FILE:  pngtogif.sh
#DATE:  05 AUG 2012
#AUTH:  G. E. Deschaines
#DESC:  Converts a sequence of PNG files to GIF files and merges
#       the GIF files into an animated GIF file.

#NOTE:  Requires the ImageMagick convert program.

# Get list of PNG files.

PNG_LIST=(`ls *.png`)
if [ "${#PNG_LIST[*]}" == "0" ]
then
  echo "error:  No PNG files in present working directory."
  exit -1
fi

# Convert each PNG file into a GIF file.

GIF_LIST=""
for FILE in ${PNG_LIST[@]}
do
  NAME=${FILE%.png}
  echo "converting:  $FILE"
  convert $FILE $NAME.gif
  GIF_LIST="$GIF_LIST $NAME.gif"
done

# Merge all GIF files into the animated gif file.

echo "Creating animated gif file:  img_anim.gif"
convert -dispose None -delay 8 $GIF_LIST -loop 0 img_anim.gif

exit 0

