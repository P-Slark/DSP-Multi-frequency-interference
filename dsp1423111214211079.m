clear all;close all;clc;
file='zmr_jams.wav';
[x,fs]=audioread(file);
xl=x(:,1);
N1=length(xl);
n=4410; 
num=floor((N1-n)/n); %��֡����
s=0;
y=0;
Y=zeros();
for i=1:num
xx1=xl((i-1)*n+1:(i-1)*n+n);
N=length(xx1);
k=0:N-1;
%t=k/fs;
% t=0:1/fs:(n-1)/fs;   %��������źŵĳ���
xf=fft(xx1,N);    %����Ҷ�任  
f=k*fs/N;
y=y+xf;
yf=10*log10(abs(y));
U=xf;
U(201)=0;
U(251)=0;
U(301)=0;
U(4111)=0;
U(4161)=0;
U(4211)=0;
u=ifft(U,N);
y2=fft(u,N);
% f=0:fs/n:fs*(n-1)/n;          %�ó�Ƶ�� 
s=s+y2;
Y=[Y;u];
end
figure(1)
subplot(2,1,1),plot(x)
xlabel('ʱ��(s)');
ylabel('����');
title('��������ǰʱ����ͼ������ ���£�'); 
subplot(2,1,2),plot(f,yf)
xlabel('Ƶ��(HZ)');
ylabel('����(dB)');
axis auto;
title('��������ǰ�ź�Ƶ��ͼ')
figure(2)
subplot(2,1,1);  
plot(real(Y))   
xlabel('ʱ��(s)');
ylabel('����');
axis auto;
title('�������ƺ�ʱ����ͼ������ ���£�'); 
subplot(2,1,2); 
plot(f,10*log10(abs(s))); 
xlabel('Ƶ��(HZ)');
ylabel('����(dB)');
axis auto;
title('�������ƺ��ź�Ƶ��ͼ')
sound(real(Y),fs);
audiowrite('zmr.wav',real(Y),fs)