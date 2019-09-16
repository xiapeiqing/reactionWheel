function analyzeBLDCangle(LogFile)
%%
close all;
addpath D:\code\RadioLoc\matlab;
if nargin == 0
    LogFile = 'ctrl_vsPWMreadval.txt';
end
[~,LogStr]=system(['grep -v "#" ' LogFile]);
% motor #, PWM Reading value, Ctrl value
rawdata = sscanf(LogStr,'m%dR:%dCtrl:%d\n',[3 inf]);
rawdata(:,find(abs(rawdata(2,:))<0.1)) = [];
rawdata(:,find(abs(rawdata(3,:))<0.1)) = [];
figure;
plot(rawdata(3,:),rawdata(2,:),'.-');
xlabel('ctrl value');
ylabel('ctrl value');

uniquemotor = unique(rawdata(1,:));
for motorii = uniquemotor
    ctrl_vsPWMreadval = rawdata(3:-1:2,find(rawdata(1,:)==motorii))';
    angleCtrl = unique(ctrl_vsPWMreadval(:,1));
%     angleCtrl = angleCtrl(1:end-1);
    PWMest = zeros(1,length(angleCtrl));
    for ii = 1:length(angleCtrl)
        if ii == 24
            tt = 1;
        end
        thisctrlval = angleCtrl(ii);
        thisPWM = ctrl_vsPWMreadval(find(ctrl_vsPWMreadval(:,1)==thisctrlval),2);
        centerPWM = thisPWM(thisPWM >= prctile(thisPWM,10) & thisPWM <= prctile(thisPWM,90));
        PWMest(ii) = mean(centerPWM);
    end
    PWMtoctrlCoeff = CMathHelper.LSE(PWMest',angleCtrl,2);
    ctrltoPWMCoeff = CMathHelper.LSE(angleCtrl,PWMest',2);
    figure;
    subplot(211);
    plot(angleCtrl,PWMest,'.-');
    hold on;
    plot(angleCtrl,ctrltoPWMCoeff(2)*angleCtrl+ctrltoPWMCoeff(1),'x-');
    legend('meas','fitted');
    xlabel('ctrl');
    ylabel('PWM reading');
    str = sprintf('motor%d,y=%9.8f*x+%6.5f',motorii,ctrltoPWMCoeff(2),ctrltoPWMCoeff(1));
    title(str);
    disp(str);
    subplot(212);
    plot(PWMest,angleCtrl,'.-');
    hold on;
    plot(PWMest,PWMtoctrlCoeff(2)*PWMest+PWMtoctrlCoeff(1),'x-');
    legend('meas','fitted');
    xlabel('PWM reading');
    ylabel('ctrl');
    str = sprintf('motor%d,y=%9.8f*x+%6.5f',motorii,PWMtoctrlCoeff(2),PWMtoctrlCoeff(1));
    title(str);
    disp(str);
end
%%
%%
if false
pwmSin=[128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, ...
170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, ...
211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, ...
241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, ...
250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, ...
255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, ...
255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, ...
250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, ...
240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, ...
247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, ...
253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, ...
255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, ...
253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, ...
245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, ...
225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, ...
185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, ...
143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, ...
101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, ...
49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, ...
14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, ...
2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, ...
3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, ...
15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, ...
7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, ...
1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, ...
8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, ...
35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, ...
86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124];
figure;
plot(pwmSin,'.-');

end




