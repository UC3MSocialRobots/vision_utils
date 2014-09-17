ROOT=../../../../../../../
POSDIR=${ROOT}/media/disk/Datasets/faces/lfwcrop_color/faces
NEGDIR=${ROOT}/media/disk/Datasets/faces/negatives
OUTXML=faces_benchmark.xml

echo "<?xml version=\"1.0\" ?>" > $OUTXML
echo "<images>" >> $OUTXML

echo "Adding positives..."
for POS in `ls -1 $POSDIR/*.ppm | head -n 2000`; do
  echo "  <image filename=\"$POS\">" >> $OUTXML
  echo "    <face visible=\"1\" frontal=\"1\"/>" >> $OUTXML
  echo "    <detectors haar=\"-1\" color=\"-1\" texture=\"-1\"/>" >> $OUTXML
  echo "  </image>" >> $OUTXML
done

echo "Adding negatives..."
for POS in `ls -1 $NEGDIR/*.jpg`; do
  echo "  <image filename=\"$POS\">" >> $OUTXML
  echo "    <face visible=\"0\" frontal=\"2\"/>" >> $OUTXML
  echo "    <detectors haar=\"-1\" color=\"-1\" texture=\"-1\"/>" >> $OUTXML
  echo "  </image>" >> $OUTXML
done

echo "</images>" >> $OUTXML
echo "Finished, generated $OUTXML"
less $OUTXML
