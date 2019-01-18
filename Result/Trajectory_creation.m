clear all;
clc;

fidin1=fopen('CameraTrajectory_room_ORB.txt');
fidout=fopen('Trajectory1.txt','w');
while ~feof(fidin1)                                      % �P�_�O�_����󥽧�               
    tline=fgetl(fidin1);                                 % �q���Ū��   
    if double(tline(1))>=48&&double(tline(1))<=57       % �P�_���r�ŬO�_���ƭ�
       fprintf(fidout,'%s\n\n',tline);                  % �p�G�O�Ʀr��A�⦹��ƾڼg�J���MKMATLAB.txt
       continue
    end
end
fclose(fidout);
transformation1=importdata('Trajectory1.txt');      % �N�ͦ���MKMATLAB.txt���ɤJ�u�@�Ŷ��A�ܶq�W��MK�A��ڤW������ܥX�� 

fidin2=fopen('CameraTrajectory_room_PL-SVIO.txt');
fidout=fopen('Trajectory2.txt','w');
while ~feof(fidin2)                                      % �P�_�O�_����󥽧�               
    tline2=fgetl(fidin2);                                 % �q���Ū��   
    if double(tline2(1))>=48&&double(tline2(1))<=57       % �P�_���r�ŬO�_���ƭ�
       fprintf(fidout,'%s\n\n',tline2);                  % �p�G�O�Ʀr��A�⦹��ƾڼg�J���MKMATLAB.txt
       continue
    end
end
fclose(fidout);
transformation2=importdata('Trajectory2.txt');      % �N�ͦ���MKMATLAB.txt���ɤJ�u�@�Ŷ��A�ܶq�W��MK�A��ڤW������ܥX�� 

fidin3=fopen('groundtruth.txt');
fidout=fopen('Trajectory3.txt','w');
while ~feof(fidin3)                                      % �P�_�O�_����󥽧�               
    tline3=fgetl(fidin3);                                 % �q���Ū��   
    if double(tline3(1))>=48&&double(tline3(1))<=57       % �P�_���r�ŬO�_���ƭ�
       fprintf(fidout,'%s\n\n',tline3);                  % �p�G�O�Ʀr��A�⦹��ƾڼg�J���MKMATLAB.txt
       continue
    end
end
fclose(fidout);
transformation3=importdata('Trajectory3.txt');      % �N�ͦ���MKMATLAB.txt���ɤJ�u�@�Ŷ��A�ܶq�W��MK�A��ڤW������ܥX��

plot3(transformation1(:, 2), transformation1(:, 3), transformation1(:, 4), 'bo-', transformation2(:, 2), transformation2(:, 3), transformation2(:, 4), 'ro-', transformation3(:, 2), transformation3(:, 3), transformation3(:, 4), 'go-','MarkerSize', 1);
grid on;
