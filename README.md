# VOR-Receiver
Matlab implementation of VOR Receiver in Aviation system (CODE GIVEN BELOW)

clc;
clear all;
close all;

final = 36;                  % Define number of iterations

for p = 1:final

% Input Parameters
fs_i = 24e6;                     % Initial Sampling Frequency 
fc_i = 108e6;                    % Navigational Carrier Frequency (108-118) = 10 2.4*10 = 24
res = 1024/30000;                % 34.13ms
t_i = 0:1/fs_i:res-(1/fs_i);     % Initial Time Period

% Signal Noise
dBm = -93;                       % Signal Power (dBm)
W = 10^((dBm-30)/10);            % Power in Watts
Vrms = sqrt(2*W);                % RMS Value of Voltage 

% Thermal Noise
k = 1.38064852e-23;              % Boltzmann Constant
T = 300;                         % Temperature (27*C)
B = 10e6;                        % Bandwidth
NF = 2.81;                       % Noise Figure
P = (k*T*B*NF);                  % Thermal Noise Power
P_dBm = 10*log10(P*1e3);         % Thermal Noise Power in dBm

noise = sqrt(P)*rand(size(t_i)); % Noise

% Perceived Frequency
fc = 12e6; % Local Oscillator Input Frequency

% Input Signals
% ----------------- Variable Signal ----------------- %   
fm = 30;            % Frequency of Message Signal
w30 = 2*pi*fm;      % Angular Velocity of Message Signal
Am = 0.3;           % Amplitude of Message Signal
Ac = 1;             % Amplitude of Carrier Signal
m = Am/Ac;          % Am/Ac (Modulation Index)
fdev = 480;         % Frequency Devation
Kf = fdev/Am;       % Frequency Sensitivity
beta = fdev/fm;     % FM Modulation Index
fc1 = 9960;         % Frequency of Sub-Carrier
w9960 = 2*pi*fc1;   % Angular Velocity of Sub-Carrier

phase_dev = p*(2*pi/final);         % input('Enter Phase Deviation'); [Radians]
phase_dev_deg = rad2deg(phase_dev); % Input Phase Deviation in Degrees

x1 = cos(w30*t_i + phase_dev); % Input Variable Signal

var_signal = Ac*cos(w9960*t_i + 2*pi*Kf*cumsum(x1)/fs_i); % Frequency Modulated Signal

% ----------------- Audio Signal ----------------- %
% ----------------- Voice Data ----------------- %
f_voice = 2500; % Frequency of Voice Data

% ----------------- Morse Code ----------------- %
encodedtext = '';
text = 'D' ; % input('Enter Characters to Encode \n','s' );
text = upper(text);
code = {'.-';'-...';'-.-.';'-..';'.';'..-.';'--.';'....';'..';'.---';'-.-';'.-..';'--';'-.';'---';'.--.';'--.-';'.-.';'...';'-';'..-';'...-';'.--';'-..-';'-.--';'--..';'.----';'..---';'...--';'....-';'.....';'-....';'--...';'---..';'----.';'-----';'_'};
characters = {'A';'B';'C';'D';'E';'F';'G';'H';'I';'J';'K';'L';'M';'N';'O';'P';'Q';'R';'S';'T';'U';'V';'W';'X';'Y';'Z';'1';'2';'3';'4';'5';'6';'7';'8';'9';'0';' '};

k = 0;
for x = 1:length(text)
    [~,index] = ismember(text(x), characters);
    if index > 0
        encodedtext = strcat(encodedtext,code{index});
        if(~strcmp(code{index},'_'))
        encodedtext = strcat(encodedtext,'~');
        end
    end
end

b = 0;
t_dot = 0:1/((res/10)*fs_i):1-(1/fs_i);
f_morse = 1020; % Frequency of Morse Code

n = length(encodedtext);
for i = 1:n
    if(encodedtext(i)=='.')
        b = horzcat(b,(1.*cos(2*pi*f_morse*t_dot)),(0.*cos(2*pi*f_morse*t_dot)));

    elseif(encodedtext(i)=='-')
        for j = 1:2
            b = horzcat(b,(1.*cos(2*pi*f_morse*t_dot))); 
        end
        b = horzcat(b,(0.*cos(2*pi*f_morse*t_dot)));

    elseif(encodedtext(i)=='_')
        for k = 1:4
            b = horzcat(b,(0.*cos(2*pi*f_morse*t_dot)));
        end

    elseif(encodedtext(i)=='~')
        t1 = 0:1/fs_i:(res-(length(b))/fs_i)-(1/fs_i); 
            b = horzcat(b,(0.*cos(2*pi*f_morse*t1)));

    end
end

audiotone = [b];

aud_signal = cos(2*pi*f_morse*t_i) + cos(2*pi*f_voice*t_i); % Input Audio Signal

% ----------------- Reference Signal ----------------- %
mes_signal = cos(w30*t_i);            % Input Reference Signal
ref_sig = (mes_signal + aud_signal);  % Input Reference + Audio Signal

message = ref_sig + var_signal;       % Input VOR Transmitted Signal

% ----------------- VOR Input Signal ----------------- %
VOR_Signal = (Ac + Am*message).*cos(2*pi*fc*t_i); % Amplitude Modulated VOR Input Signal
VOR_Input = VOR_Signal*Vrms + noise;              % VOR Input Signal with Noise

% Local Oscillator
flo_signal = cos(2*pi*fc*t_i);

% Mixer Output
fcm = flo_signal.*VOR_Signal;

%% FPGA
% LPF Decimation 1
load('lpf1.mat')
lpf1 = filter(lpf1,1,fcm);
d1 = decimate(lpf1,16);

fs_ld1 = fs_i/16;

% LPF Decimation 2
load('lpf2.mat')
lpf2 = filter(lpf2,1,d1);
d2 = decimate(lpf2,10); 

fs_ld2 = fs_ld1/10;

% LPF Decimation 3
load('lpf3.mat')
lpf3 = filter(lpf3,1,d2);
VOR_FPGA_Out = decimate(lpf3,5);

fs = fs_ld2/5;
t = 0:1/fs:res-(1/fs);

dsp_sig = VOR_FPGA_Out - mean(VOR_FPGA_Out);

l = length(dsp_sig);
f = -fs/2:fs/l:fs/2-(fs/l);

% for i = 1:1228
% fprintf("%.16f, ", VOR_FPGA_Out(i));
% if mod(i,50) == 0
%     fprintf("\n");
% end
% end

% Elimination of DC Component
m1 = dsp_sig - mean(dsp_sig); % Removal of DC by Subtracting Mean

%% DSP
% AM Extraction
load("test1.mat");            % LPF Filter (Least Square)
z1 = filter(test1, 1, m1);    % Reference Signal

% for i = 1:11
% fprintf("%f, ", test1(i));
% if mod(i,50) == 0
%     fprintf("\n");
% end
% end

% FM Demodulation
% Quadrature Demodulation
coswave = 2*cos(2*pi*9960*t);
sinwave = -2*sin(2*pi*9960*t);
a1 = m1.*(2*cos(2*pi*9960*t));   % Real Part 
a2 = m1.*(-2*sin(2*pi*9960*t));  % Complex Part

% for i = 1:1024
% fprintf("%f, ", coswave(i));
% if mod(i,50) == 0
%     fprintf("\n");
% end
% end

% for i = 1:1024
% fprintf("%f, ", sinwave(i));
% if mod(i,50) == 0
%     fprintf("\n");
% end
% end

load("test2.mat")        % LPF Filter (Least Square)
xS = filter(test2,1,a1); % In-Phase Output
xC = filter(test2,1,a2); % Quadrature Output

% for i = 1:11
% fprintf("%f, ", test2(i));
% if mod(i,50) == 0
%     fprintf("\n");
% end
% end

x_t = (xS + 1i*xC);  % Complex Signal

len = size(x_t,1);
if(len==1)
    x_t = x_t(:);
end

t = (0:1/fs:((size(x_t,1)-1)/fs))';
t = t(:,ones(1,size(x_t,2)));

z2 = (1/(2*pi*fdev))*[zeros(1,size(x_t,2)); diff(unwrap(angle(x_t)))*fs]; % Variable Signal

if(len == 1)
    z2 = z2'; 
end

l1 = cos(2*pi*30*t);
l2 = cos(2*pi*30*t + phase_dev);

% Phase Difference Calculation
x = z1(:); % Reference Signal
y = z2(:); % Variable Signal

xwin = hanning(length(x), 'periodic'); % Hanning Window Samples

% for i = 1:1024
% fprintf("%f, ", xwin(i));
% if mod(i,50) == 0
%     fprintf("\n");
% end
% end

X = fft(x.*xwin); 
Y = fft(y.*xwin);

[~, indx] = max(abs(X)); % Calculating Max Absolute Index Value of Signal
[~, indy] = max(abs(Y)); % Calculating Max Absolute Index Value of Signal

PhDiff = angle(Y(indy)) - angle(X(indx)); % Calculating Phase Difference between Max Index Values
% phase_deg = rad2deg(PhDiff);

% Scaling to 0 - 360 Degrees
if phase_dev < 189*pi/180
   phase_deg = rad2deg(PhDiff);
else
   phase_deg = rad2deg(PhDiff + 2*pi);
end

phase_deg = phase_deg - 0.828615806;

offset = phase_deg - phase_dev_deg; % Offset between Actual and Measured Phase Difference
fprintf('When Input is %.3f -> Output is %.3f, The Offset is %.3f\n', phase_dev_deg, phase_deg, offset);
% fprintf('%.3f\n', phase_deg);

% arr(p) = offset;       % Store squared values

end

% anglemean = mean(arr);
% anglemean_offset = anglemean - 0.828615806; 
% disp(anglemean_offset)

% ----------------- Audio Indicator ----------------- %
% LPF Decimation 4
load('lpf4.mat')
lpf4 = filter(lpf4,1,VOR_FPGA_Out);
d4 = decimate(lpf4,3);
aud_indicator = d4.';

fs_aud = fs/3;

% BPF (Morse Code)
load('M_BPF.mat')
morse_indicator = filter(M_BPF,1,aud_indicator);

% sound(morse_indicator,fs_aud);

% BPF (Audio Tone)
load('A_BPF.mat')
audio_tone = filter(A_BPF,1,aud_indicator);
