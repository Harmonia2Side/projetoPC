% =========================================================================
% 
% C�digo para simula��o de uma transmiss�o de um conjunto de dados MODBUS
% por meio dos protocolos RS485 e RS232.
% 
% =========================================================================

%% Inicializa��o 
clc, close all, clear all


%% .................  Parametriza��o do Sistema  ...........................
Rb = 10e6;          % Taxa de transmiss�o do sistema (10 x 10^6 = 10^7) em bits/segundo
Tb = 1/Rb;          % Dura��o de um bit
M = 2;              % N�vel de Modulu��o (Mapeamento para codifica��o digital)
m = log2(M);        % bits / s�mbolo
Rs = Rb/m;          % Taxa de s�mbolos (sps - taxa s�mbolos por segundo ou baud rate)
Ts = 1/Rs;          % Dura��o no tempo de cada s�mbolo
nsamp = 4;          % Fator de reamostragem
snr = 3;            % Rela��o sinal ru�do (dado em 3db / Dobro da potencia)
nData = 16*1024;    % N�mero de bits a serem transmitidos = 16kb
% nData = 16;       % N�mero de bits a serem transmitidos = 16 bits
Tsim = nData*Tb;    % Tempo total da simula��o = nbits * tempo_bit

% .................  C�lculo de vari�veis iniciais  ......................
t = linspace(0,Ts*((nData-1)*nsamp), nData*nsamp).';


%% .................  Transmiss�o  ......................................... 

%Gera��o de dados aleat�rios
data_in = randi([0 M-1], nData, 1);

% % Convers�o de bits em pulsos
% x_pulse = pammod(data_in, M); 

x_pulse = data_in; % Sem modula��o

% Conforma��o de pulso (Pulse Shaping) - Super Amostragem (Upsampling)
x_up = rectpulse(x_pulse, nsamp); % Filtro Retangular

signal = [t x_up];

%% -----------------  Canal  -----------------------------------------------
% % Adicionando Ru�do
% x_up_noise = awgn(x_up,snr,'measured');

% -------------------------------------------------------------------------

%% ................. Recep��o ..............................................

% % Filtragem do ru�do AWGN do sinal
% x_up_filtrado = x_up_noise;
% len = length(x_up_noise);
% t_hold = 0; % Defini��o do threshold (limiar) do sinal 

% % Codifica��o NRZ Polar
% for i = 1:len
%     if (x_up_noise(i)>t_hold)
%        x_up_filtrado(i) = 1;
%     else 
%         x_up_filtrado(i) = -1;
%     end
% end

% % ! Receber x_up_filtrado do simulink !
% out = sim("simulacao_485.slx",Tsim)

% Repete em uma amostra o �ltimo valor (restri��o do simulink)
x_up_filtrado = [out.x_up_filtrado ; out.x_up_filtrado(end)]

% Deconforma��o de pulso
x_dp = intdump(x_up_filtrado, nsamp);

% % Convers�o de pulsos em bits
% data_out = pamdemod(x_dp, M);

data_out = x_dp % Sem modula��o

% Calculo do BER do dado transmitido
bits_errados = 0;
i = 1;
while i <= length(data_in)
    if (data_in(i) ~= data_out(i))
        bits_errados = bits_errados + 1;
    end
    i = i + 1;
end
format shortEng
BER = (bits_errados / length(data_in)) % Calculo de BER




%% +++++++++++++++++++  Plotagens  +++++++++++++++++++++++++++++++++++++++++

% Exibi��o do pulso no canal de transmissao
figure;
plot(x_up,'LineWidth',0.5);
hold all;
plot(x_up_noise),xlim([0 200]), ylim([-2 2]), title('Canal - (Pulso + AWGN)');

% Exibi��o dos pulsos no sistema de comunica��o
figure;
subplot(3,1,1), plot(x_up), xlim([0 200]), ylim([-1.5 1.5]), title('Transmiss�o');
subplot(3,1,2), hold on, plot(x_up),plot(x_up_noise), xlim([0 200]), ylim([-1.5 1.5]), title('Canal - (Pulso + AWGN)');
subplot(3,1,3), plot(x_up_filtrado,'Color','green'), xlim([0 200]), ylim([-1.5 1.5]), title('Recep��o');


% Plotagem do Diagrama de Olho da sa�da do canal
eyediagram(x_up_noise(1:end/10000), 3)




