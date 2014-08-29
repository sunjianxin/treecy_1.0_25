./findcylinder 0 0.225 0.02
mkdir section_0-0.225
mv section*.pcd section_0-0.225
mv section*.txt section_0-0.225

./findcylinder 0.225 0.48 0.02 
mkdir section_0.225-0.48
mv section*.pcd section_0.225-0.48
mv section*.txt section_0.225-0.48

./findcylinder 0.48 0.795 0.02 
mkdir section_0.48 0.795
mv section*.pcd section_0.48 0.795
mv section*.txt section_0.48 0.795

./findcylinder 0.795 1.32 0.02
mkdir section_0.795 1.32
mv section*.pcd section_0.795 1.32
mv section*.txt section_0.795 1.32

./findcylinder 1.32 1.89 0.02
mkdir section_1.32 1.89
mv section*.pcd section_1.32 1.89
mv section*.txt section_1.32 1.89

./findcylinder 1.89 2.505 0.014
mkdir section_1.89 2.505
mv section*.pcd section_1.89 2.505
mv section*.txt section_1.89 2.505

./findcylinder 2.505 3.195 0.02 
mkdir section_2.505 3.195
mv section*.pcd section_2.505 3.195
mv section*.txt section_2.505 3.195                     
