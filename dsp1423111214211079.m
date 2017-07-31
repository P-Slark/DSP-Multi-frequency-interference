clear all;close all;clc;
file='zmr_jams.wav';
[x,fs]=audioread(file);
xl=x(:,1);
N1=length(xl);
n=4410; 
num=floor((N1-n)/n); %分帧数量
s=0;
y=0;
Y=zeros();
for i=1:num
xx1=xl((i-1)*n+1:(i-1)*n+n);
N=length(xx1);
k=0:N-1;
%t=k/fs;
% t=0:1/fs:(n-1)/fs;   %求出语音信号的长度
xf=fft(xx1,N);    %傅里叶变换  
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
% f=0:fs/n:fs*(n-1)/n;          %得出频点 
s=s+y2;
Y=[Y;u];
end
figure(1)
subplot(2,1,1),plot(x)
xlabel('时间(s)');
ylabel('幅度');
title('干扰抑制前时域波形图（陶阳 杨媛）'); 
subplot(2,1,2),plot(f,yf)
xlabel('频率(HZ)');
ylabel('幅度(dB)');
axis auto;
title('干扰抑制前信号频谱图')
figure(2)
subplot(2,1,1);  
plot(real(Y))   
xlabel('时间(s)');
ylabel('幅度');
axis auto;
title('干扰抑制后时域波形图（陶阳 杨媛）'); 
subplot(2,1,2); 
plot(f,10*log10(abs(s))); 
xlabel('频率(HZ)');
ylabel('幅度(dB)');
axis auto;
title('干扰抑制后信号频谱图')
sound(real(Y),fs);
audiowrite('zmr.wav',real(Y),fs)