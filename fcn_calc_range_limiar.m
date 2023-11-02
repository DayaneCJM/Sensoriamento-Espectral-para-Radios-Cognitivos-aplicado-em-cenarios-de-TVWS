function [limiar Pd Pfa] = fcn_calc_range_limiar()

%% Levantamento de curvas de desempenho em sistemas reais
%% Parametros configurados na usrp
Duration = 30;
%Initial parameters
Canal_Sensoriado = 473.142857e6;
CenterFrequency = Canal_Sensoriado;
LocalOscillatorOffset = 0;
MasterClockRate = 100e6;
TransportDataType = 'int8';
DecimationRxInterpolationTxFactor = 16;
SamplesPerFrame  = 2^15; 
EnableBurstMode  = 1;
NumFramesInBurst = 1;
Gain = 5;

%Derivados
SampleRate = MasterClockRate/DecimationRxInterpolationTxFactor;
FrameDuration = SamplesPerFrame/SampleRate;
Iterations = floor(Duration/FrameDuration);


%% Criando objeto rx
rx = comm.SDRuReceiver('Platform','N200/N210/USRP2', ...
                'IPAddress', '192.168.10.2', ...
                'CenterFrequency',CenterFrequency, ...
                'LocalOscillatorOffset',LocalOscillatorOffset, ...
                'Gain', Gain,...
               'MasterClockRate',MasterClockRate, ...
               'DecimationFactor',DecimationRxInterpolationTxFactor, ...
               'TransportDataType',TransportDataType, ...
               'SamplesPerFrame',SamplesPerFrame, ...
               'EnableBurstMode',EnableBurstMode, ...
               'NumFramesInBurst',NumFramesInBurst, ...
               'OutputDataType','single');

%% Mean power noise  
Mean_power_of_noise = 4.7217*10.^-06;
%% Parametros para levantamento do sensoriamento
%Canal de RF
Channel = 37;
%Gain
Gain = 5;
%Amostras capturadas pelo RC
Nsamples = 100;
%Relação Sinal ruído considerada no levantamento de curva
SNRdB = 5;
%Eventos de Montecarlo para simulação
Eventos = 500;
%Pontos entre limares mínimos e máximo
Npontos = 8;
T_ED_H0 = zeros(1,Eventos);
T_ED_H1 = zeros(1,Eventos);

display('Calculando range de limiares para ROC');
uiwait(msgbox('Desligue o Transmissor','Warning','warn','modal'));
rx.Gain = Gain
for i=1:Eventos
    %% Monitorando canal sensoriado
    rx.CenterFrequency = 473.142857e6+(Channel-14)*6e6;
    [data, ~, overrun]  = step(rx);   
    T_ED_H0(i) = sum(abs(data(1:Nsamples)).^2);
    pause(0.01)   
i
end
figure; hist(T_ED_H0,100)

SNR_target = SNRdB;
SNR = 1000;
uiwait(msgbox('Ligue o transmissor e ajuste para a SNR configura','Warning','warn','modal'));
while (SNR - SNR_target > 0.1)
    [data, ~, ~]  = step(rx);
    T_ED(i) = sum(abs(data(1:Nsamples)).^2);
    pause(0.01)
    SNR_target
    SNR = 10*log10((sum(abs(data).^2)/32768)/Mean_power_of_noise)
end

fprintf('Sistema de recepção com SNR = %f',SNR);

for i=1:Eventos
    %% Monitorando canal sensoriado
    [data, ~, overrun]  = step(rx);
    T_ED(i) = sum(abs(data(1:Nsamples)).^2);
    pause(0.01)
    T_ED_H1(i) = T_ED(i);
    SNR = 10*log10((sum(abs(data).^2)/32768)/Mean_power_of_noise)   
i
end
figure; hist(T_ED_H1,100)
min(T_ED_H0)
max(T_ED_H1)
range_total= max(T_ED_H1)-min(T_ED_H0)

limiar = linspace((min(T_ED_H0)+0.05*range_total), (max(T_ED_H1)-0.05*range_total),Npontos);

%% Calculando probabilidades de detecção e falso alarme para um dada SNR
Pfa = zeros(1,length(limiar));
Pd = zeros(1,length(limiar));
for i = 1 : length(limiar)
    Pfa(i) = sum(T_ED_H0>limiar(i))/length(T_ED_H0)
    Pd(i) = sum(T_ED_H1>limiar(i))/length(T_ED_H1)
       
end

plot(Pfa,Pd,'--b+','linewidth',1);%
title('Normal ROC - P_{D} por P_{FA} ');
xlabel('P_{FA}');
ylabel('P_{D}');
%legend('SNR = -10dB');
hold on;
grid on;
axis([0 1 0 1]);

