function  [T_ED, T_ED_H0, T_ED_H1] = usrp_n210_receiver_data()
%% %% Recepção de um sinal OFDM Real
Canal_Sensoriado = 473.142857e6;
Channel_min = 37;
Channel_max = 38;

%% Parametros configurados na usrp
Duration = 30;
%Initial parameters
CenterFrequency = Canal_Sensoriado;
LocalOscillatorOffset = 0;
MasterClockRate = 100e6;
TransportDataType = 'int8';
DecimationRxInterpolationTxFactor = 16;
SamplesPerFrame  = 2^15; 
EnableBurstMode  = 1;
NumFramesInBurst = 1;
Gain = 30;

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


%% Criando espectro
specAn = dsp.SpectrumAnalyzer('SampleRate',SampleRate,'SpectralAverages',1);

    

%% Configura tx/rx gain
% rx.Gain =10;% G= 0 a 38
% 
% rx.CenterFrequency = Canal_Sensoriado ;

%% Transmite Repeate 
Channel = Channel_min;
Eventos = 1000;
T_ED = zeros(1,Eventos);
T_ED_H0 = zeros(1,Eventos/2);
T_ED_H1 = zeros(1,Eventos/2);
for i=1:Eventos
    %% Monitorando canal sensoriado
    rx.CenterFrequency = 473.142857e6+(Channel-14)*6e6;
    [data, ~, overrun]  = step(rx);
    rxOut = complex(data);
    specAn(rxOut)
    T_ED(i) = sum(abs(data).^2)/length(data);
    pause(0.01)
     %% Incrementanto Canal
     if Channel == Channel_max
         Channel = Channel_min;
         T_ED_H0(i) = T_ED(i);
     else
         Channel = Channel + 1;
         T_ED_H1(i) = T_ED(i);
     end
     
end

figure; hist(T_ED,100)
figure; hist(T_ED_H0,100)
figure; hist(T_ED_H1,100)