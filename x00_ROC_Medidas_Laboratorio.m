clear all;
close all;
clc;

%% C�digo para levantamente de desempenho do sistema com r�dio definido por software
%% Data de atualizacao: 12/07/2023
%% Vers�o usada para levantamento de medi��es e valida��es te�ricas
%% Parametros configurados na usrp
%Frequencia central do canal inicial em Hz
CenterFrequency = 473.142857e6;
%Offset na frequ�ncia central do canal em Hz
LocalOscillatorOffset = 0;
%Clock base da USRP
MasterClockRate = 100e6;
%Tipo de dado capturada 
TransportDataType = 'int8';
%Fator de decima��o/interpola��o
DecimationRxInterpolationTxFactor = 16;
%N�mero de amostras por busrt
SamplesPerFrame  = 2^15;
%Habilita modo burst
EnableBurstMode  = 1;
%Numero de bursts
NumFramesInBurst = 1;
%Ganho inicial
Gain = 0;
%Derivados
SampleRate = MasterClockRate/DecimationRxInterpolationTxFactor;
FrameDuration = SamplesPerFrame/SampleRate;

%% Criando objeto que far� a recep��o dos dados Via USRP N210
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
%Cria objeto de analizador de espectro, caso seja necess�rio visualizar o espectro          
specAn = dsp.SpectrumAnalyzer('SampleRate',SampleRate,'SpectralAverages',20);

%% Parametros para levantamento do sensoriamento
%Canal de a ser avaliado RF 14 ao 51
Channel = 37;
%Gain do AGC recepcao 30 dB padr�o 
Gain = 30;
%Amostras capturadas pelo RC
Nsamples = 100;
%Rela��o Sinal ru�do considerada no levantamento de curva
SNRdB = -5;
%Eventos de Montecarlo para simula��o
Eventos = 1500;
%Pontos entre limares m�nimos e m�ximo
Npontos = 100;
%Inicializa variaveis que far�o armazenamento de T
T_ED_H0 = zeros(1,Eventos);
T_ED_H1 = zeros(1,Eventos);
T_ED_H0_full = zeros(1,Eventos);

%% Inicio do levantamento de desempenho sob Hip�tese H_0 (Tx desligado)
display('Calculando curva ROC');
uiwait(msgbox('Desligue o Transmissor','Warning','warn','modal'));
%Seta ganho configurado para levantamento de desempenho
rx.Gain = Gain;
% Seta frequ�ncia do canal
rx.CenterFrequency = 473.142857e6+(Channel-14)*6e6; 
for i=1:Eventos
    % Captura amostras
    [data, ~, overrun]  = step(rx);  
    % Calcula vari�vel de decis�o T_ED cos as N_samples sob hip�tese H0
    T_ED_H0(i) = sum(abs(data(1:Nsamples)).^2);
    % Calcula vari�vel energia como todas amostra capturadas no burst (Vari�ncia mais est�vel)
    T_ED_H0_full(i) = sum(abs(data).^2);
    pause(0.01)   
i
end
%Plota histograma da vari�vel de decis�o T|H_0
figure(1); 
hist(T_ED_H0,linspace(min(T_ED_H0),max(T_ED_H0),100))
%Calcula a vari�ncia m�dia do ru�do
Mean_power_of_noise = mean(T_ED_H0_full)/2^15
%Relacao sinal ru�do alvo na simulacao
SNR_target = SNRdB;
SNR = 1000;
uiwait(msgbox('Ligue o transmissor e ajuste para a SNR configura','Warning','warn','modal'));
while (abs(SNR - SNR_target) > 0.05)
    %Apenas captura amostras e monitora SNR para ver se est� dentro da alvo
    [data, ~, ~]  = step(rx);
    pause(0.01)
    SNR = 10*log10((sum(abs(data).^2)/32768)/Mean_power_of_noise-1);
    fprintf('SNR alvo:%f dB // SNR atual:%f dB\n',SNR_target, SNR)
end

fprintf('Sistema de recep��o com SNR = %f',SNR);

%% Inicio do levantamento de desempenho sob Hip�tese H_1 (Tx ligado)
for i=1:Eventos
    %Captura amostras
    [data, ~, overrun]  = step(rx);
    %Calcula vari�vel de decis�o 
    pause(0.01)
    T_ED_H1(i) = sum(abs(data(1:Nsamples)).^2);
    %Caclula SNR instantanea
    SNR = 10*log10((sum(abs(data).^2)/32768)/Mean_power_of_noise-1);
    fprintf('SNR atual:%f dB\n', SNR)
    rxOut = complex(data);
    %Para ver espectro, basta descomentar linha abaixo
    %specAn(rxOut)
i
end
%Plota histograma de T_ED|H_1
figure(2); hist(T_ED_H1,linspace(min(T_ED_H1),max(T_ED_H1),100))
%Defini��o do limiar para levantamento de curva ROC
limiar = linspace((min(T_ED_H0)),(max(T_ED_H1)),Npontos);

%% Calculando probabilidades de detec��o e falso alarme para um dada SNR
Pfa = zeros(1,length(limiar));
Pd = zeros(1,length(limiar));
for i = 1 : length(limiar)
    Pfa(i) = sum(T_ED_H0>limiar(i))/length(T_ED_H0);
    Pd(i) = sum(T_ED_H1>limiar(i))/length(T_ED_H1);       
end

%% Plota gr�fico de desempenho
figure(3)
hold on;
plot(Pfa,Pd,'--ro','linewidth',2);%
xlabel('P_{FA}');
ylabel('P_{D}');
%legend('SNR = -10dB');
hold on;
grid on;
axis([0 1 0 1]);
%Mostra SNR
SNRonSimulation = 10*log10(mean(T_ED_H1)/mean(T_ED_H0)-1)
%Armazena vetor para ROCs
Vec_ROC_pd_pfa = [Pfa' Pd'];
 %% Gerando histogramas finais
[amp_H0 x_axes_H0]=hist(T_ED_H0,linspace(min(T_ED_H0),max(T_ED_H1),100));
[amp_H1 x_axes_H1]=hist(T_ED_H1,linspace(min(T_ED_H0),max(T_ED_H1),100));
figure(5)
bar(x_axes_H0,amp_H0,'b');
hold on;
bar(x_axes_H1,amp_H1,'r');
xlabel('Variavel T');
ylabel('PDF');
legend('Histograma H0', 'Histograma H1');

%% Salvamento de arquivos
Pd_Pfa_Limiar =[Pd', Pfa',limiar'];
Histograma = [x_axes_H0' amp_H0' x_axes_H1' amp_H1']
scenario = strcat('Gain_',num2str(Gain),'_Nsamples_',num2str(Nsamples),'_SNR_db_',num2str(SNRdB),'_Eventos_',num2str(Eventos),'SNRonSimulation',num2str(SNRonSimulation))
dlmwrite(strcat(scenario,'_Pd_Pfa_Limiar.dat'),Pd_Pfa_Limiar,'delimiter',' ')
dlmwrite(strcat(scenario,'_Histograma.dat'),Histograma,'delimiter',' ')

%% Simulacoes para comparaca��o com a pr�tica
%% Simula��o de sensoriamento espectral Energy detection com valida��o simula��o/te�ria
%Validei express�es te�ricas de Pd e Pfa por meio de gera��o de ROCs sim e teo
%N�mero de Amostras coletadas
N = Nsamples;
%SNR m�dia em dB
SNRdB = SNRdB; % Implementado
%Desvio padr�o do Ru�do
sig_w = 1;
%"Desvio padr�o" do Sinal sig_s.^2 = pot sinal
sig_s = 0; %Implementado indiretamente pela SNR
% N�mero de Eventos de Monte Carlo
Me =100000;
% N�mero de pontos da ROC
Npts = Npontos;
% 0- AWGN, 1 Rayleig
Channel = 0;
%Convers�o da SNRdB para linear
SNR =10.^(SNRdB/10);
%% Loop Montecarlo
for i = 1:Me
   if i<=Me/2
        n = sig_w*1/sqrt(2)*(randn(1,N) +1i*randn(1,N));
        y = n;
        T_H0(i) = sum(abs(y).^2);
   else
        %ru�do Awgn com o desvio padr�o com o desvio padr�o unit�rio
        n = sig_w*1/sqrt(2)*(randn(1,N) +1i*randn(1,N));
        %Sinal te transmiss�o com a pot�ncia configurada
        x = sqrt(SNR)*1/sqrt(2)*(randn(1,N) +1i*randn(1,N));
        %Ganho complexo aleat�rio do canal Rayleigh pot�ncia unit�ria
        if Channel == 1
            h = 1/sqrt(2)*(randn(1,1) +1i*randn(1,1));
        else
            h = 1;
        end
        %Sinal recebido
        y = h*x+n;
        T_H1(i-Me/2) = sum(abs(y).^2);
   end 
     
end

%% Geracao ROC
limiar = linspace(min(T_H0), max(T_H1),Npts);
% Calculando probabilidades de detec��o e falso alarme para um dada SNR
Pfa = zeros(1,length(limiar));
Pd = zeros(1,length(limiar));
for i = 1 : length(limiar)
    Pfa(i) = sum(T_H0>limiar(i))/length(T_H0);
    Pd(i) = sum(T_H1>limiar(i))/length(T_H1);
end
%Armazena vetor simulado AWGN
Vec_ROC_pd_pfa = [Vec_ROC_pd_pfa Pfa' Pd'];
SNRonSimulation = 10*log10(mean(T_H1)/mean(T_H0)-1)
% Plota ROC simulada
figure(3);
hold on;
plot(Pfa,Pd,'gs','linewidth',2);
Pfa_teo = qfunc((limiar - N)/sqrt(N));
if Channel == 0
    Pd_teo = qfunc((limiar - N*(SNR + 1))/sqrt(N*(SNR + 1).^2));
    plot(Pfa_teo,Pd_teo,'--g','linewidth',2)
else
    % Usando fr_Rayleigh DEU CERTO % Quando quiser a te�rica � s� faser assim para as outras
    p00 = 1;
    fr_Rayleigh = @(p,r) 1/p(1)*2*(r/p(1)) .*exp(-(r/p(1)).^2);
    dr = 0.001;
    r = 0:dr:10;
    Unit_energy = sum(fr_Rayleigh(p00,r))*dr
    Thre = limiar;
    for i = 1: length(Thre)

        f = @(r) fr_Rayleigh(p00,r).*qfunc((Thre(i) - N*(r.^2*SNR + 1))./sqrt(N*(r.^2*SNR + 1).^2));
        Pd_teo(i) = integral(f,0,inf);
        % Unit_area(i)=sum(1/snrlin(i)*exp(-snr_inst/snrlin(i))*deltasnr)
    end    
    plot(Pfa_teo,Pd_teo,'b','linewidth',2)
end

Vec_ROC_pd_pfa = [Vec_ROC_pd_pfa Pfa_teo' Pd_teo'];

% 0- AWGN, 1 Rayleig
Channel = 1;
%Convers�o da SNRdB para linear
SNR =10.^(SNRdB/10);
%% Loop Montecarlo
for i = 1:Me
   if i<=Me/2
        n = sig_w*1/sqrt(2)*(randn(1,N) +1i*randn(1,N));
        y = n;
        T_H0(i) = sum(abs(y).^2);
   else
        %ru�do Awgn com o desvio padr�o com o desvio padr�o unit�rio
        n = sig_w*1/sqrt(2)*(randn(1,N) +1i*randn(1,N));
        %Sinal te transmiss�o com a pot�ncia configurada
        x = sqrt(SNR)*1/sqrt(2)*(randn(1,N) +1i*randn(1,N));
        %Ganho complexo aleat�rio do canal Rayleigh pot�ncia unit�ria
        if Channel == 1
            h = 1/sqrt(2)*(randn(1,1) +1i*randn(1,1));
        else
            h = 1;
        end
        %Sinal recebido
        y = h*x+n;
        T_H1(i-Me/2) = sum(abs(y).^2);
   end 
     
end

%% Geracao ROC
limiar = linspace(min(T_H0), max(T_H1),Npts);
% Calculando probabilidades de detec��o e falso alarme para um dada SNR
Pfa = zeros(1,length(limiar));
Pd = zeros(1,length(limiar));
for i = 1 : length(limiar)
    Pfa(i) = sum(T_H0>limiar(i))/length(T_H0);
    Pd(i) = sum(T_H1>limiar(i))/length(T_H1);
end
Vec_ROC_pd_pfa = [Vec_ROC_pd_pfa Pfa' Pd'];
SNRonSimulation = 10*log10(mean(T_H1)/mean(T_H0)-1)
% Plota ROC simulada
figure(3);
hold on;
plot(Pfa,Pd,'bd','linewidth',2)
Pfa_teo = qfunc((limiar - N)/sqrt(N));
if Channel == 0
    Pd_teo = qfunc((limiar - N*(SNR + 1))/sqrt(N*(SNR + 1).^2));
    plot(Pfa_teo,Pd_teo,'k')
else
    % Usando fr_Rayleigh DEU CERTO % Quando quiser a te�rica � s� faser assim para as outras
    p00 = 1;
    fr_Rayleigh = @(p,r) 1/p(1)*2*(r/p(1)) .*exp(-(r/p(1)).^2);
    dr = 0.001;
    r = 0:dr:10;
    Unit_energy = sum(fr_Rayleigh(p00,r))*dr
    Thre = limiar;
    for i = 1: length(Thre)
        f = @(r) fr_Rayleigh(p00,r).*qfunc((Thre(i) - N*(r.^2*SNR + 1))./sqrt(N*(r.^2*SNR + 1).^2));
        Pd_teo(i) = integral(f,0,inf);
        % Unit_area(i)=sum(1/snrlin(i)*exp(-snr_inst/snrlin(i))*deltasnr)
    end    
    plot(Pfa_teo,Pd_teo,'--b','linewidth',2) 
end
legend('Medida RDS','Simulada AWGN','Te�rica AWGN','Simulada Rayleigh', 'Te�rica Rayleigh')

%% Gravacao das curvas em arquivo
Vec_ROC_pd_pfa = [Vec_ROC_pd_pfa Pfa_teo' Pd_teo'];
%Salva .dats e figura final
dlmwrite(strcat(scenario,'_ROC_Medida_Sim_Teo.dat'),Vec_ROC_pd_pfa,'delimiter',' ')
savefig(strcat(scenario,'_Grafico.fig'))


