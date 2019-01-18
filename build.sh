echo "Uncompress vocabulary ..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz

cd ..
echo "Converting vocabulary to binary"
./tools/bin_vocabulary

