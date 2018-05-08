%only change the line bellow by correcting the txt file path
fileID = fopen('E:\Example\Example_folder\matrixdata.txt','r');
formatSpec = '%d';
sizeA = [ 50 50];
A = fscanf(fileID, formatSpec, sizeA);
imagesc(A)