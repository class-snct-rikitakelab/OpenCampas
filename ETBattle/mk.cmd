echo off 

del /Q *.rxe 
make clean all 

IF EXIST *.rxe sh rxeflash.sh