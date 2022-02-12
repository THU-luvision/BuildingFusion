if [ $1  == '1' ]
then
   make -j -C ../build
   ../build/af 0 0 0.005 0
elif [ $1 == '2' ]
then
   make -j -C ../build
else
   ../build/af 0 0 0.005 0	
fi

