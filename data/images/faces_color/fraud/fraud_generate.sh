var=0
for j in 0 1
do
  for s in 0 1
  do
    for a in 0 1 2
    do
      for f in 0 1
      do
         echo "# F$f A$a S$s J$j"
         echo $var
         var=$(($var +1))
      done
    done
  done
  echo "#"
done

