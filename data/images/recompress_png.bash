#!/bin/bash
FILES=`find . -name "*.png"`
TEMPFILE=/tmp/recompressed.png
for f in $FILES; do
  # http://www.imagemagick.org/script/command-line-options.php
  # png:compression-level=value
  #~ valid values are 0 through 9, with 0 providing the least but fastest
  #~ compression and 9 usually providing the best and always the slowest.
  SIZE_BEFORE=`ls $f -l | awk '{print $5}'`
  convert $f  -define "png:compression-level=9" $TEMPFILE
  SIZE_AFTER=`ls $TEMPFILE -l | awk '{print $5}'`
  if (("$SIZE_BEFORE" <= "$SIZE_AFTER")); then
    #echo -e "$f:\tbefore:$SIZE_BEFORE < after:$SIZE_AFTER: skipping"
    :
  else
    RATIO=$((100 * $SIZE_AFTER / $SIZE_BEFORE))
    #~ echo -e "$f:\tbefore:$SIZE_BEFORE < after:$SIZE_AFTER: replacing"
    printf '%-100s compressed at %i %%\n' $f $RATIO;
    cp $TEMPFILE $f
  fi
done

#~ cleaning
rm $TEMPFILE
