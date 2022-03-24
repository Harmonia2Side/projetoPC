% =========================================================================
% 
% Código para simulação de uma transmissão de um conjunto de dados MODBUS
% por meio dos protocolos RS485 e RS232.
% 
% =========================================================================

%% Inicialização 
clc, close all, clear all


%% .................  Parametrização do Sistema  ...........................
Rb = 10e6;          % Taxa de transmissão do sistema (10 x 10^6 = 10^7) em bits/segundo
Tb = 1/Rb;          % Duração de um bit
M = 2;              % Nível de Modulução (Mapeamento para codificação digital)
m = log2(M);        % bits / símbolo
Rs = Rb/m;          % Taxa de símbolos (sps - taxa símbolos por segundo ou baud rate)
Ts = 1/Rs;          % Duração no tempo de cada símbolo
nsamp = 4;          % Fator de reamostragem
snr = 3;            % Relação sinal ruído (dado em 3db / Dobro da potencia)
nData = 16*1024;    % Número de bits a serem transmitidos = 16kb
% nData = 16;       % Número de bits a serem transmitidos = 16 bits
Tsim = nData*Tb;    % Tempo total da simulação = nbits * tempo_bit

% .................  Cálculo de variáveis iniciais  ......................
t = linspace(0,Ts*((nData-1)*nsamp), nData*nsamp).';


%% .................  Transmissão  ......................................... 

%Geração de dados aleatórios
data_in = randi([0 M-1], nData, 1);

% % Conversão de bits em pulsos
% x_pulse = pammod(data_in, M); 

x_pulse = data_in; % Sem modulação

% Conformação de pulso (Pulse Shaping) - Super Amostragem (Upsampling)
x_up = rectpulse(x_pulse, nsamp); % Filtro Retangular

signal = [t x_up];

%% -----------------  Canal  -----------------------------------------------
% % Adicionando Ruído
% x_up_noise = awgn(x_up,snr,'measured');

% -------------------------------------------------------------------------

%% ................. Recepção ..............................................

% % Filtragem do ruído AWGN do sinal
% x_up_filtrado = x_up_noise;
% len = length(x_up_noise);
% t_hold = 0; % Definição do threshold (limiar) do sinal 

% % Codificação NRZ Polar
% for i = 1:len
%     if (x_up_noise(i)>t_hold)
%        x_up_filtrado(i) = 1;
%     else 
%         x_up_filtrado(i) = -1;
%     end
% end

% % ! Receber x_up_filtrado do simulink !
% out = sim("simulacao_485.slx",Tsim)

% Repete em uma amostra o último valor (restrição do simulink)
x_up_filtrado = [out.x_up_filtrado ; out.x_up_filtrado(end)]

% Deconformação de pulso
x_dp = intdump(x_up_filtrado, nsamp);

% % Conversão de pulsos em bits
% data_out = pamdemod(x_dp, M);

data_out = x_dp % Sem modulação

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

% Exibição do pulso no canal de transmissao
figure;
plot(x_up,'LineWidth',0.5);
hold all;
plot(x_up_noise),xlim([0 200]), ylim([-2 2]), title('Canal - (Pulso + AWGN)');

% Exibição dos pulsos no sistema de comunicação
figure;
subplot(3,1,1), plot(x_up), xlim([0 200]), ylim([-1.5 1.5]), title('Transmissão');
subplot(3,1,2), hold on, plot(x_up),plot(x_up_noise), xlim([0 200]), ylim([-1.5 1.5]), title('Canal - (Pulso + AWGN)');
subplot(3,1,3), plot(x_up_filtrado,'Color','green'), xlim([0 200]), ylim([-1.5 1.5]), title('Recepção');


% Plotagem do Diagrama de Olho da saída do canal
eyediagram(x_up_noise(1:end/10000), 3)




