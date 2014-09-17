#!/bin/bash
echo "<?xml version=\"1.0\"?>"
echo "<images>"
for c in `seq --equal-width 63 `
do
    echo "   <image filename=\"pz$c.jpg\">"
    echo "   </image>"
    echo
done
echo "</images>"
