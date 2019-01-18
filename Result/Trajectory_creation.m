clear all;
clc;

fidin1=fopen('CameraTrajectory_room_ORB.txt');
fidout=fopen('Trajectory1.txt','w');
while ~feof(fidin1)                                      % 判斷是否為文件末尾               
    tline=fgetl(fidin1);                                 % 從文件讀行   
    if double(tline(1))>=48&&double(tline(1))<=57       % 判斷首字符是否為數值
       fprintf(fidout,'%s\n\n',tline);                  % 如果是數字行，把此行數據寫入文件MKMATLAB.txt
       continue
    end
end
fclose(fidout);
transformation1=importdata('Trajectory1.txt');      % 將生成的MKMATLAB.txt文件導入工作空間，變量名為MK，實際上它不顯示出來 

fidin2=fopen('CameraTrajectory_room_PL-SVIO.txt');
fidout=fopen('Trajectory2.txt','w');
while ~feof(fidin2)                                      % 判斷是否為文件末尾               
    tline2=fgetl(fidin2);                                 % 從文件讀行   
    if double(tline2(1))>=48&&double(tline2(1))<=57       % 判斷首字符是否為數值
       fprintf(fidout,'%s\n\n',tline2);                  % 如果是數字行，把此行數據寫入文件MKMATLAB.txt
       continue
    end
end
fclose(fidout);
transformation2=importdata('Trajectory2.txt');      % 將生成的MKMATLAB.txt文件導入工作空間，變量名為MK，實際上它不顯示出來 

fidin3=fopen('groundtruth.txt');
fidout=fopen('Trajectory3.txt','w');
while ~feof(fidin3)                                      % 判斷是否為文件末尾               
    tline3=fgetl(fidin3);                                 % 從文件讀行   
    if double(tline3(1))>=48&&double(tline3(1))<=57       % 判斷首字符是否為數值
       fprintf(fidout,'%s\n\n',tline3);                  % 如果是數字行，把此行數據寫入文件MKMATLAB.txt
       continue
    end
end
fclose(fidout);
transformation3=importdata('Trajectory3.txt');      % 將生成的MKMATLAB.txt文件導入工作空間，變量名為MK，實際上它不顯示出來

plot3(transformation1(:, 2), transformation1(:, 3), transformation1(:, 4), 'bo-', transformation2(:, 2), transformation2(:, 3), transformation2(:, 4), 'ro-', transformation3(:, 2), transformation3(:, 3), transformation3(:, 4), 'go-','MarkerSize', 1);
grid on;
