for i in `ls`
do
  for j in `grep -L "^$i$" CMakeLists.txt`
  do 
    echo $i
  done
done
